#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include "sensor_msgs/msg/joint_state.hpp"
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "jagv_interfaces/srv/auto_move.hpp"
#include "jagv_interfaces/srv/motion_state_control.hpp"

#include "jaka_kargo_planner/JAKAZuRobot.h"
#include "jaka_kargo_planner/jkerr.h"
#include "jaka_kargo_planner/jktypes.h"

#include <array>
#include <string>
#include <vector>
#include <unordered_map>
#include <limits>
#include <mutex>
#include <chrono>
#include <map>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <thread>
#include <pthread.h>
#include <algorithm>
#include <csignal>

using namespace std;

JAKAZuRobot robot;

// Map error codes to messages
map<int, string> mapErr = {
    {2,   "ERR_FUCTION_CALL_ERROR"},
    {-1,  "ERR_INVALID_HANDLER"},
    {-2,  "ERR_INVALID_PARAMETER"},
    {-3,  "ERR_COMMUNICATION_ERR"},
    {-4,  "ERR_KINE_INVERSE_ERR"},
    {-5,  "ERR_EMERGENCY_PRESSED"},
    {-6,  "ERR_NOT_POWERED"},
    {-7,  "ERR_NOT_ENABLED"},
    {-8,  "ERR_DISABLE_SERVOMODE"},
    {-9,  "ERR_NOT_OFF_ENABLE"},
    {-10, "ERR_PROGRAM_IS_RUNNING"},
    {-11, "ERR_CANNOT_OPEN_FILE"},
    {-12, "ERR_MOTION_ABNORMAL"}
};

using Follow = control_msgs::action::FollowJointTrajectory;
using GoalHandle = rclcpp_action::ServerGoalHandle<Follow>;

rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub;
rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr agv_odom_sub;
rclcpp::Client<jagv_interfaces::srv::AutoMove>::SharedPtr agv_auto_move_client;
rclcpp::Client<jagv_interfaces::srv::MotionStateControl>::SharedPtr agv_motion_ctrl_client;

// Global EDG broadcast IP used by edg_init calls
static string edg_init_ip = "255.255.255.255";

/// Make EDG broadcast IP by replacing last IPv4 octet with 255.
static string make_edg_bcast(const string &ip)
{
    auto last_oct = ip.find_last_of('.');
    if (last_oct == string::npos) {
        return ip;
    }
    return ip.substr(0, last_oct + 1) + "255";
}

static double servo_period_sec = 0.002;

// Guards
static mutex g_arm_mtx[2];   // 0 = LEFT, 1 = RIGHT
static mutex g_ext_mtx;      // external axis group
static mutex g_agv_goal_mtx;
static mutex g_agv_odom_mtx;

struct AgvOdomState
{
    bool valid{false};
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
};

static AgvOdomState g_agv_odom;

// ----- index maps -----
struct IndexMapFull {
    array<int, 7> L{};
    array<int, 7> R{};
    array<int, 4> B{};
    int agv_x{-1};
    int agv_y{-1};
    int agv_yaw{-1};
    bool ok{false};
};

struct IndexMapExt {
    array<int, 4> B{};
    bool ok{false};
};

struct IndexMapAgv
{
    int x{-1};
    int y{-1};
    int yaw{-1};
    bool ok{false};
};

static IndexMapFull build_index_map_full(const vector<string>& names)
{
    IndexMapFull m;
    unordered_map<string, int> idx;
    for (int i = 0; i < (int)names.size(); ++i) idx[names[i]] = i;

    const char* Lnames[7] = {"arm_lj1","arm_lj2","arm_lj3","arm_lj4","arm_lj5","arm_lj6","arm_lj7"};
    const char* Rnames[7] = {"arm_rj1","arm_rj2","arm_rj3","arm_rj4","arm_rj5","arm_rj6","arm_rj7"};
    const char* Bnames[4] = {"body_j1","body_j2","body_j3","body_j4"};

    for (int i = 0; i < 7; ++i) {
        if (!idx.count(Lnames[i]) || !idx.count(Rnames[i])) {
            m.ok = false;
            return m;
        }
        m.L[i] = idx[Lnames[i]];
        m.R[i] = idx[Rnames[i]];
    }
    for (int i = 0; i < 4; ++i) {
        if (!idx.count(Bnames[i])) {
            m.ok = false;
            return m;
        }
        m.B[i] = idx[Bnames[i]];
    }
    if (!idx.count("agv_x") || !idx.count("agv_y") || !idx.count("agv_yaw")) {
        m.ok = false;
        return m;
    }
    m.agv_x = idx["agv_x"];
    m.agv_y = idx["agv_y"];
    m.agv_yaw = idx["agv_yaw"];

    m.ok = true;
    return m;
}

static bool build_index_map_single(const vector<string>& names, const char* prefix, array<int,7>& out)
{
    unordered_map<string,int> idx;
    for (int i = 0; i < (int)names.size(); ++i) idx[names[i]] = i;

    for (int i = 0; i < 7; ++i) {
        string jn = string(prefix) + to_string(i + 1);
        auto it = idx.find(jn);
        if (it == idx.end()) return false;
        out[i] = it->second;
    }
    return true;
}

static IndexMapExt build_index_map_ext(const vector<string>& names)
{
    IndexMapExt m;
    unordered_map<string, int> idx;
    for (int i = 0; i < (int)names.size(); ++i) idx[names[i]] = i;

    const char* Bnames[4] = {"body_j1","body_j2","body_j3","body_j4"};

    for (int i = 0; i < 4; ++i) {
        if (!idx.count(Bnames[i])) {
            m.ok = false;
            return m;
        }
        m.B[i] = idx[Bnames[i]];
    }

    m.ok = true;
    return m;
}

static IndexMapAgv build_index_map_agv(const vector<string>& names)
{
    IndexMapAgv m;
    unordered_map<string, int> idx;
    for (int i = 0; i < (int)names.size(); ++i) idx[names[i]] = i;

    if (!idx.count("agv_x") || !idx.count("agv_y") || !idx.count("agv_yaw")) {
        m.ok = false;
        return m;
    }

    m.x = idx["agv_x"];
    m.y = idx["agv_y"];
    m.yaw = idx["agv_yaw"];
    m.ok = true;
    return m;
}

// ----- helpers -----

static inline unsigned int steps_from_dt(double dt_sec)
{
    if (dt_sec <= 0.0) return 1u;
    const double s = dt_sec / servo_period_sec;
    unsigned int k = (unsigned int)llround(s);
    return k == 0 ? 1u : k;
}

static double max_abs_err(const JointValue &a, const JointValue &b)
{
    double m = 0.0;
    for (int i = 0; i < 7; ++i) {
        double e = abs(a.jVal[i] - b.jVal[i]);
        if (e > m) m = e;
    }
    return m;
}


static void disable_arm_servos()
{
    robot.servo_move_enable(FALSE, 1, 0);
    robot.servo_move_enable(FALSE, 1, 1);
}

static bool enable_all_ext_axes()
{
    errno_t r0 = robot.enable_ext(0);
    errno_t r1 = robot.enable_ext(1);
    errno_t r2 = robot.enable_ext(2);
    errno_t r3 = robot.enable_ext(3);
    return (r0 == ERR_SUCC) && (r1 == ERR_SUCC) && (r2 == ERR_SUCC) && (r3 == ERR_SUCC);
}

static void disable_all_ext_axes()
{
    robot.disable_ext(0);
    robot.disable_ext(1);
    robot.disable_ext(2);
    robot.disable_ext(3);
}

static bool stop_all_ext_axes()
{
    MultiMovInfoList stop_cmd{};
    stop_cmd.count = 4;

    for (int i = 0; i < 4; ++i) {
        stop_cmd.info[i].motion_unit_type = 1;
        stop_cmd.info[i].motion_unit_id   = i;
        stop_cmd.info[i].move_type        = JOINT_MOVE;
        stop_cmd.info[i].move_mode        = STOP;
    }

    errno_t ret = robot.multi_mov_with_ext(&stop_cmd, false);
    return ret == ERR_SUCC;
}

static bool read_ext_feedback(array<double,4>& out_pos)
{
    ExtAxisStatusList status_list{};
    errno_t ret = robot.get_ext_status(&status_list);
    if (ret != ERR_SUCC) {
        return false;
    }
    if (status_list.count < 4) {
        return false;
    }

    for (int i = 0; i < 4; ++i) {
        out_pos[i] = status_list.status[i].pos_fdb;  // mm for joint1, deg for joint2-4
    }
    return true;
}

static bool ext_target_reached(const array<double,4>& current_pos, const array<double,4>& target_pos, double ext_lin_tol, double ext_deg_tol)
{
    // joint 0: mm, joints 1..3: deg
    if (abs(current_pos[0] - target_pos[0]) > ext_lin_tol) return false;
    for (int i = 1; i < 4; ++i) {
        if (abs(current_pos[i] - target_pos[i]) > ext_deg_tol) return false;
    }
    return true;
}

static array<double,4> extract_ext_target_sdk_units(
    const trajectory_msgs::msg::JointTrajectoryPoint& pt,
    const array<int,4>& mapB)
{
    array<double,4> ext_target{};
    ext_target[0] = pt.positions[ mapB[0] ] * 1000.0;       // m -> mm
    ext_target[1] = pt.positions[ mapB[1] ] * 180.0 / M_PI; // rad -> deg
    ext_target[2] = pt.positions[ mapB[2] ] * 180.0 / M_PI;
    ext_target[3] = pt.positions[ mapB[3] ] * 180.0 / M_PI;
    return ext_target;
}

static errno_t send_ext_target_nonblocking(const array<double,4>& ext_target, double vel, double acc)
{
    MultiMovInfoList cmd{};
    cmd.count = 4;

    for (int i = 0; i < 4; ++i) {
        cmd.info[i].motion_unit_type = 1;
        cmd.info[i].motion_unit_id   = i;
        cmd.info[i].move_type        = JOINT_MOVE;
        cmd.info[i].move_mode        = ABS;
        cmd.info[i].movej_info.end_pos[0] = ext_target[i];
        cmd.info[i].movej_info.j_vel = vel;
        cmd.info[i].movej_info.j_acc = acc;
        cmd.info[i].movej_info.j_jerk = 0.0;
        cmd.info[i].movej_info.blend_tol = 0.0;
    }

    return robot.multi_mov_with_ext(&cmd, false);
}

static double yaw_from_quat(double x, double y, double z, double w)
{
    const double siny_cosp = 2.0 * (w * z + x * y);
    const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    return atan2(siny_cosp, cosy_cosp);
}

static double normalize_angle(double a)
{
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

static void agv_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    lock_guard<mutex> lk(g_agv_odom_mtx);
    g_agv_odom.valid = true;
    g_agv_odom.x = msg->pose.pose.position.x;
    g_agv_odom.y = msg->pose.pose.position.y;
    g_agv_odom.yaw = yaw_from_quat(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    // RCLCPP_INFO(rclcpp::get_logger("kargo_moveit_server"),
    //             "AGV odom received: x=%.3f y=%.3f, yaw=%.3f",
    //             g_agv_odom.x,
    //             g_agv_odom.y,
    //             g_agv_odom.yaw);
}

static bool get_latest_agv_pose(double &x, double &y, double &yaw)
{
    lock_guard<mutex> lk(g_agv_odom_mtx);
    if (!g_agv_odom.valid) return false;
    x = g_agv_odom.x;
    y = g_agv_odom.y;
    yaw = g_agv_odom.yaw;
    return true;
}

static geometry_msgs::msg::Pose2D point_to_pose2d(const trajectory_msgs::msg::JointTrajectoryPoint& pt, const IndexMapAgv& map)
{
    geometry_msgs::msg::Pose2D p;
    p.x = pt.positions[map.x];
    p.y = pt.positions[map.y];
    p.theta = pt.positions[map.yaw];
    return p;
}

static bool stop_agv_motion()
{
    if (!agv_motion_ctrl_client) return false;
    if (!agv_motion_ctrl_client->wait_for_service(chrono::seconds(1))) return false;

    auto req = make_shared<jagv_interfaces::srv::MotionStateControl::Request>();
    req->motion_state_code = 3;  // stop

    auto fut = agv_motion_ctrl_client->async_send_request(req);
    if (fut.wait_for(chrono::seconds(2)) != future_status::ready) {
        return false;
    }

    auto resp = fut.get();
    return resp && (resp->ret_code == 0);
}


static bool trajectory_has_full_velocities(const trajectory_msgs::msg::JointTrajectory& traj)
{
    if (traj.points.size() < 2) return false;

    const size_t n = traj.joint_names.size();
    for (const auto& pt : traj.points) {
        if (pt.velocities.size() != n) {
            return false;
        }
    }
    return true;
}

static double point_time_sec(const trajectory_msgs::msg::JointTrajectoryPoint& pt)
{
    return static_cast<double>(pt.time_from_start.sec) +
           1e-9 * static_cast<double>(pt.time_from_start.nanosec);
}

static JointValue interpolate_single_arm_sample(
    const trajectory_msgs::msg::JointTrajectory& traj,
    const array<int,7>& mapSingle,
    double t_sec,
    size_t& seg_idx)
{
    JointValue out{};

    const auto& pts = traj.points;
    const size_t n = pts.size();

    if (n == 0) {
        return out;
    }

    if (n == 1 || t_sec <= point_time_sec(pts.front())) {
        for (int j = 0; j < 7; ++j) {
            out.jVal[j] = pts.front().positions[mapSingle[j]];
        }
        return out;
    }

    if (t_sec >= point_time_sec(pts.back())) {
        for (int j = 0; j < 7; ++j) {
            out.jVal[j] = pts.back().positions[mapSingle[j]];
        }
        return out;
    }

    while (seg_idx + 1 < n && point_time_sec(pts[seg_idx + 1]) < t_sec) {
        ++seg_idx;
    }

    const auto& p0 = pts[seg_idx];
    const auto& p1 = pts[seg_idx + 1];

    const double t0 = point_time_sec(p0);
    const double t1 = point_time_sec(p1);
    const double h  = t1 - t0;

    if (h <= 1e-12) {
        for (int j = 0; j < 7; ++j) {
            out.jVal[j] = p1.positions[mapSingle[j]];
        }
        return out;
    }

    const double s = (t_sec - t0) / h;

    const bool have_vel =
        (p0.velocities.size() == traj.joint_names.size()) &&
        (p1.velocities.size() == traj.joint_names.size());

    if (have_vel) {
        // Cubic Hermite interpolation
        const double h00 =  2.0*s*s*s - 3.0*s*s + 1.0;
        const double h10 =      s*s*s - 2.0*s*s + s;
        const double h01 = -2.0*s*s*s + 3.0*s*s;
        const double h11 =      s*s*s -     s*s;

        for (int j = 0; j < 7; ++j) {
            const int idx = mapSingle[j];
            const double q0 = p0.positions[idx];
            const double q1 = p1.positions[idx];
            const double v0 = p0.velocities[idx];
            const double v1 = p1.velocities[idx];

            out.jVal[j] = h00 * q0 + h10 * h * v0 + h01 * q1 + h11 * h * v1;
        }
    } else {
        // Linear interpolation fallback
        for (int j = 0; j < 7; ++j) {
            const int idx = mapSingle[j];
            const double q0 = p0.positions[idx];
            const double q1 = p1.positions[idx];
            out.jVal[j] = q0 + s * (q1 - q0);
        }
    }

    return out;
}

struct FullArmSample
{
    JointValue jl{};
    JointValue jr{};
};

static FullArmSample interpolate_full_robot_dual_arm_sample(
    const trajectory_msgs::msg::JointTrajectory& traj,
    const IndexMapFull& mapFull,
    double t_sec,
    size_t& seg_idx)
{
    FullArmSample out{};

    const auto& pts = traj.points;
    const size_t n = pts.size();

    if (n == 0) {
        return out;
    }

    if (n == 1 || t_sec <= point_time_sec(pts.front())) {
        for (int j = 0; j < 7; ++j) {
            out.jl.jVal[j] = pts.front().positions[mapFull.L[j]];
            out.jr.jVal[j] = pts.front().positions[mapFull.R[j]];
        }
        return out;
    }

    if (t_sec >= point_time_sec(pts.back())) {
        for (int j = 0; j < 7; ++j) {
            out.jl.jVal[j] = pts.back().positions[mapFull.L[j]];
            out.jr.jVal[j] = pts.back().positions[mapFull.R[j]];
        }
        return out;
    }

    while (seg_idx + 1 < n && point_time_sec(pts[seg_idx + 1]) < t_sec) {
        ++seg_idx;
    }

    const auto& p0 = pts[seg_idx];
    const auto& p1 = pts[seg_idx + 1];

    const double t0 = point_time_sec(p0);
    const double t1 = point_time_sec(p1);
    const double h  = t1 - t0;

    if (h <= 1e-12) {
        for (int j = 0; j < 7; ++j) {
            out.jl.jVal[j] = p1.positions[mapFull.L[j]];
            out.jr.jVal[j] = p1.positions[mapFull.R[j]];
        }
        return out;
    }

    const double s = (t_sec - t0) / h;

    const bool have_vel =
        (p0.velocities.size() == traj.joint_names.size()) &&
        (p1.velocities.size() == traj.joint_names.size());

    if (have_vel) {
        // Cubic Hermite interpolation
        const double h00 =  2.0*s*s*s - 3.0*s*s + 1.0;
        const double h10 =      s*s*s - 2.0*s*s + s;
        const double h01 = -2.0*s*s*s + 3.0*s*s;
        const double h11 =      s*s*s -     s*s;

        for (int j = 0; j < 7; ++j) {
            const int idxL = mapFull.L[j];
            const int idxR = mapFull.R[j];

            const double q0L = p0.positions[idxL];
            const double q1L = p1.positions[idxL];
            const double v0L = p0.velocities[idxL];
            const double v1L = p1.velocities[idxL];

            const double q0R = p0.positions[idxR];
            const double q1R = p1.positions[idxR];
            const double v0R = p0.velocities[idxR];
            const double v1R = p1.velocities[idxR];

            out.jl.jVal[j] = h00 * q0L + h10 * h * v0L + h01 * q1L + h11 * h * v1L;
            out.jr.jVal[j] = h00 * q0R + h10 * h * v0R + h01 * q1R + h11 * h * v1R;
        }
    } else {
        // Linear interpolation fallback
        for (int j = 0; j < 7; ++j) {
            const int idxL = mapFull.L[j];
            const int idxR = mapFull.R[j];

            const double q0L = p0.positions[idxL];
            const double q1L = p1.positions[idxL];

            const double q0R = p0.positions[idxR];
            const double q1R = p1.positions[idxR];

            out.jl.jVal[j] = q0L + s * (q1L - q0L);
            out.jr.jVal[j] = q0R + s * (q1R - q0R);
        }
    }

    return out;
}


// ----- execute full body: 21 joints -----
void execute_full_robot_goal(const shared_ptr<GoalHandle> gh,
                             rclcpp::Node::SharedPtr node,
                             double ext_vel,
                             double ext_acc,
                             const string &agv_map_name,
                             double agv_linear_speed,
                             double agv_angular_speed,
                             double agv_dece_distance,
                             double agv_stop_distance)
{
    auto goal = gh->get_goal();
    const auto& traj = goal->trajectory;

    if (traj.joint_names.size() != 21) {
        RCLCPP_ERROR(node->get_logger(), "Expected 21 joint names for full-robot, got %zu", traj.joint_names.size());
        gh->abort(make_shared<Follow::Result>());
        return;
    }
    if (traj.points.empty()) {
        RCLCPP_ERROR(node->get_logger(), "Trajectory has no points");
        gh->abort(make_shared<Follow::Result>());
        return;
    }

    auto mapFull = build_index_map_full(traj.joint_names);
    if (!mapFull.ok) {
        RCLCPP_ERROR(node->get_logger(), "Incorrect joint names for full-robot");
        gh->abort(make_shared<Follow::Result>());
        return;
    }

    // lock all resources for full body execution
    lock(g_arm_mtx[0], g_arm_mtx[1], g_ext_mtx, g_agv_goal_mtx);
    unique_lock<mutex> lk0(g_arm_mtx[0], adopt_lock);
    unique_lock<mutex> lk1(g_arm_mtx[1], adopt_lock);
    unique_lock<mutex> lkE(g_ext_mtx, adopt_lock);
    unique_lock<mutex> lkA(g_agv_goal_mtx, adopt_lock);

    // robot.servo_move_use_none_filter(0);
    // robot.servo_move_use_none_filter(1);

    sched_param sch;
    sch.sched_priority = 90;
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &sch);

    if (robot.servo_move_enable(TRUE, 1, 0) != 0 || robot.servo_move_enable(TRUE, 1, 1) != 0) {
        RCLCPP_ERROR(node->get_logger(), "servo_move_enable failed");
        gh->abort(make_shared<Follow::Result>());
        return;
    }

    if (!enable_all_ext_axes()) {
        RCLCPP_ERROR(node->get_logger(), "enable_all_ext_axes failed");
        disable_arm_servos();
        disable_all_ext_axes();
        gh->abort(make_shared<Follow::Result>());
        return;
    }

    if (!agv_auto_move_client || !agv_auto_move_client->wait_for_service(chrono::seconds(2))) {
        RCLCPP_ERROR(node->get_logger(), "AGV auto move service not available");
        disable_arm_servos();
        // disable_all_ext_axes();
        gh->abort(make_shared<Follow::Result>());
        return;
    }

    double sx, sy, syaw;
    if (!get_latest_agv_pose(sx, sy, syaw)) {
        RCLCPP_ERROR(node->get_logger(), "No AGV odom received yet on global_nav_odom");
        disable_arm_servos();
        // disable_all_ext_axes();
        gh->abort(make_shared<Follow::Result>());
        return;
    }

    RCLCPP_INFO(node->get_logger(), "Received trajectory with %zu points (full-robot).", traj.points.size());

    const auto& last_pt = traj.points.back();
    if (last_pt.positions.size() != 21) {
        RCLCPP_ERROR(node->get_logger(), "Last point has %zu positions (need 21)", last_pt.positions.size());
        disable_arm_servos();
        // disable_all_ext_axes();
        gh->abort(make_shared<Follow::Result>());
        return;
    }

    // Start error check for both arms
    JointValue fbL0{}, fbR0{};
    robot.edg_get_stat(0, &fbL0, nullptr);
    robot.edg_get_stat(1, &fbR0, nullptr);
    const auto& first_pt = traj.points.front();
    double max_start_err_l = 0.0;
    double max_start_err_r = 0.0;
    for (int j = 0; j < 7; ++j) {
        max_start_err_l = max(max_start_err_l, abs(fbL0.jVal[j] - first_pt.positions[mapFull.L[j]]));
        max_start_err_r = max(max_start_err_r, abs(fbR0.jVal[j] - first_pt.positions[mapFull.R[j]]));
    }
    if (max_start_err_l > 0.01 || max_start_err_r > 0.01) {
        RCLCPP_ERROR(node->get_logger(), "Full-robot trajectory start too far from actual state, "
                     "errL=%.6f rad, errR=%.6f rad", max_start_err_l, max_start_err_r);
        disable_arm_servos();
        // disable_all_ext_axes();
        gh->abort(make_shared<Follow::Result>());
        return;
    }

    const bool use_hermite = trajectory_has_full_velocities(traj);
    RCLCPP_INFO(node->get_logger(),
                "Full-robot trajectory points=%zu, joint_names=%zu, interpolation=%s",
                traj.points.size(),
                traj.joint_names.size(),
                use_hermite ? "Hermite" : "Linear");

    // AGV auto_move request
    int agv_move_mode = 0;
    int agv_move_type = 0;

    auto agv_req = make_shared<jagv_interfaces::srv::AutoMove::Request>();
    agv_req->map_name = agv_map_name;

    agv_req->start_pose.x = sx;
    agv_req->start_pose.y = sy;
    agv_req->start_pose.theta = syaw;

    agv_req->end_pose.x = last_pt.positions[mapFull.agv_x];
    agv_req->end_pose.y = last_pt.positions[mapFull.agv_y];
    agv_req->end_pose.theta = last_pt.positions[mapFull.agv_yaw];

    agv_req->path_coords.clear();
    // if (traj.points.size() > 1) {
    //     for (size_t i = 0; i + 1 < traj.points.size(); ++i) {
    //         const auto& pt = traj.points[i];
    //         if (pt.positions.size() != 21) {
    //             RCLCPP_ERROR(node->get_logger(), "Point %zu has %zu positions (need 21)", i, pt.positions.size());
    //             disable_arm_servos();
    //             disable_all_ext_axes();
    //             gh->abort(make_shared<Follow::Result>());
    //             return;
    //         }

    //         geometry_msgs::msg::Pose2D p;
    //         p.x = pt.positions[mapFull.agv_x];
    //         p.y = pt.positions[mapFull.agv_y];
    //         p.theta = pt.positions[mapFull.agv_yaw];
    //         agv_req->path_coords.push_back(p);
    //     }
    // }

    agv_req->linear_speed = agv_linear_speed;
    agv_req->angular_speed = agv_angular_speed;
    agv_req->dece_distance = agv_dece_distance;
    agv_req->stop_distance = agv_stop_distance;
    agv_req->move_mode = agv_move_mode;
    agv_req->move_type = agv_move_type;

    RCLCPP_INFO(node->get_logger(),
                "Sending AGV auto move: start=(%.3f, %.3f, %.3f), end=(%.3f, %.3f, %.3f), path_coords=%zu",
                agv_req->start_pose.x, agv_req->start_pose.y, agv_req->start_pose.theta,
                agv_req->end_pose.x, agv_req->end_pose.y, agv_req->end_pose.theta,
                agv_req->path_coords.size());

    auto agv_future = agv_auto_move_client->async_send_request(agv_req);

    while (agv_future.wait_for(chrono::milliseconds(50)) != future_status::ready) {
        if (gh->is_canceling()) {
            stop_agv_motion();
            disable_arm_servos();
            disable_all_ext_axes();
            gh->canceled(make_shared<Follow::Result>());
            return;
        }
    }

    auto agv_resp = agv_future.get();
    if (!agv_resp || agv_resp->ret_code != 0) {
        RCLCPP_ERROR(node->get_logger(), "AGV auto move failed, ret_code=%d", agv_resp ? agv_resp->ret_code : -9999);
        disable_arm_servos();
        // disable_all_ext_axes();
        gh->abort(make_shared<Follow::Result>());
        return;
    }
    
    // external axis final target (non-blocking)
    const auto ext_target = extract_ext_target_sdk_units(last_pt, mapFull.B);

    for (int i = 0; i < 4; ++i) {
        if (!isfinite(ext_target[i])) {
            RCLCPP_ERROR(node->get_logger(), "Ext target[%d] is not finite", i);
            disable_arm_servos();
            // disable_all_ext_axes();
            gh->abort(make_shared<Follow::Result>());
            return;
        }
    }
    RCLCPP_INFO(node->get_logger(),
                "Ext target to send: j1=%.3f mm, j2=%.3f deg, j3=%.3f deg, j4=%.3f deg, vel=%.3f acc=%.3f",
                ext_target[0], ext_target[1], ext_target[2], ext_target[3], ext_vel, ext_acc);

    errno_t ext_ret = send_ext_target_nonblocking(ext_target, ext_vel, ext_acc);
    if (ext_ret != ERR_SUCC) {
        RCLCPP_ERROR(node->get_logger(),
                    "Failed to send ext-axis target, sdk_ret=%d", ext_ret);
        disable_arm_servos();
        // disable_all_ext_axes();
        gh->abort(make_shared<Follow::Result>());
        return;
    }

    RCLCPP_INFO(node->get_logger(),
                "Ext target sent: %.3f mm, %.3f deg, %.3f deg, %.3f deg",
                ext_target[0], ext_target[1], ext_target[2], ext_target[3]);

    // Dense EDG streaming at 2 ms
    const double total_t = point_time_sec(last_pt);
    const unsigned int step = 1;
    const size_t num_samples = static_cast<size_t>(ceil(total_t / servo_period_sec));
    auto start_time = chrono::steady_clock::now();
    size_t seg_idx = 0;

    for (size_t i = 0; i < traj.points.size(); ++i) {
        const auto& pt = traj.points[i];
        if (pt.positions.size() != 21) {
            RCLCPP_ERROR(node->get_logger(), "Point %zu has %zu positions (need 21)", i, pt.positions.size());
            robot.motion_abort();
            stop_all_ext_axes();
            disable_arm_servos();
            // disable_all_ext_axes();
            gh->abort(make_shared<Follow::Result>());
            return;
        }
    }

    for (size_t k = 0; k <= num_samples; ++k)
    {
        if (gh->is_canceling()) {
            robot.motion_abort();
            stop_all_ext_axes();
            disable_arm_servos();
            // disable_all_ext_axes();
            gh->canceled(make_shared<Follow::Result>());
            return;
        }

        double t = k * servo_period_sec;
        if (t > total_t) {
            t = total_t;
        }

        auto wake_time = start_time + chrono::duration_cast<chrono::steady_clock::duration>(chrono::duration<double>(t));
        this_thread::sleep_until(wake_time);

        FullArmSample sample = interpolate_full_robot_dual_arm_sample(traj, mapFull, t, seg_idx);

        int retL = robot.edg_servo_j(0u, &sample.jl, MoveMode::ABS, step);
        int retR = robot.edg_servo_j(1u, &sample.jr, MoveMode::ABS, step);
        if (retL != 0 || retR != 0) {
            RCLCPP_ERROR(node->get_logger(), "edg_servo_j failed: L=%s R=%s",
                         mapErr[retL].c_str(), mapErr[retR].c_str());
            robot.motion_abort();
            stop_all_ext_axes();
            disable_arm_servos();
            // disable_all_ext_axes();
            gh->abort(make_shared<Follow::Result>());
            return;
        }

        int send_ret = robot.edg_send();
        if (send_ret != 0) {
            RCLCPP_WARN(node->get_logger(), "edg_send failed %d", send_ret);
        }

        // Debug
        // RCLCPP_INFO(node->get_logger(),
        //             "full dense sample k=%zu t=%.6f step=%u seg_idx=%zu",
        //             k, t, step, seg_idx);
        if (k % 100 == 0 || k == num_samples) {
            RCLCPP_INFO(node->get_logger(),
                        "dense sample k=%zu/%zu t=%.6f seg_idx=%zu",
                        k, num_samples, t, seg_idx);
        }
    }

    // ---------------- Wait until all subsystems reach target ----------------
    JointValue jl_target{}, jr_target{};
    for (int j = 0; j < 7; ++j) jl_target.jVal[j] = last_pt.positions[ mapFull.L[j] ];
    for (int j = 0; j < 7; ++j) jr_target.jVal[j] = last_pt.positions[ mapFull.R[j] ];

    const double agv_target_x = last_pt.positions[mapFull.agv_x];
    const double agv_target_y = last_pt.positions[mapFull.agv_y];
    const double agv_target_yaw = last_pt.positions[mapFull.agv_yaw];

    const double traj_end_time = (double)last_pt.time_from_start.sec
                               + 1e-9 * (double)last_pt.time_from_start.nanosec;
    const int extra_ms_margin = 5000;
    auto deadline = chrono::steady_clock::now()
                  + chrono::milliseconds((int)(traj_end_time * 1000.0) + extra_ms_margin);

    const double arm_rad_tol = 0.01;
    const double ext_lin_tol = 0.5;
    const double ext_deg_tol = 0.5;
    const double agv_xy_tol = 0.05;
    const double agv_yaw_tol = 0.05;

    RCLCPP_INFO(node->get_logger(), "Waiting for full-robot to reach final point (end_time=%.3f s).", traj_end_time);

    int inerr[2] = {0, 0};
    BOOL in_col = FALSE;
    JointValue fbL{}, fbR{};
    array<double,4> ext_fb{};
    double agv_x, agv_y, agv_yaw;

    while (rclcpp::ok()) {
        if (gh->is_canceling()) {
            robot.motion_abort();
            stop_agv_motion();
            stop_all_ext_axes();
            disable_arm_servos();
            // disable_all_ext_axes();
            gh->canceled(make_shared<Follow::Result>());
            return;
        }

        robot.edg_recv();

        robot.is_in_collision(&in_col);
        robot.robot_is_in_error(inerr);
        if (in_col || inerr[0] || inerr[1]) {
            RCLCPP_WARN(node->get_logger(), "Abort: collision=%d errL=%d errR=%d", in_col, inerr[0], inerr[1]);
            robot.motion_abort();
            stop_agv_motion();
            stop_all_ext_axes();
            disable_arm_servos();
            // disable_all_ext_axes();
            gh->abort(make_shared<Follow::Result>());
            return;
        }

        const bool okL = (robot.edg_get_stat(0, &fbL, nullptr) == 0);
        const bool okR = (robot.edg_get_stat(1, &fbR, nullptr) == 0);
        const bool okE = read_ext_feedback(ext_fb);
        const bool okA = get_latest_agv_pose(agv_x, agv_y, agv_yaw);

        const double errL = okL ? max_abs_err(fbL, jl_target) : 1e6;
        const double errR = okR ? max_abs_err(fbR, jr_target) : 1e6;
        const double err_agv_pos = okA ? hypot(agv_x - agv_target_x, agv_y - agv_target_y) : 1e6;
        const double err_agv_yaw = okA ? abs(normalize_angle(agv_yaw - agv_target_yaw)) : 1e6;
        const bool arms_ok = okL && okR && (errL <= arm_rad_tol) && (errR <= arm_rad_tol);
        const bool ext_ok = okE && ext_target_reached(ext_fb, ext_target, ext_lin_tol, ext_deg_tol);
        const bool agv_ok = okA && (err_agv_pos <= agv_xy_tol) && (err_agv_yaw <= agv_yaw_tol);

        bool reached = arms_ok && ext_ok && agv_ok;

        if (reached) {
            RCLCPP_INFO(node->get_logger(), "==============Full-robot motion reached the target position==============");
            break;
        }

        if (chrono::steady_clock::now() >= deadline) {
            robot.motion_abort();
            stop_agv_motion();
            stop_all_ext_axes();
            disable_arm_servos();
            // disable_all_ext_axes();
            RCLCPP_WARN(node->get_logger(),
                        "Timeout waiting for full-robot target position (errL=%.4f errR=%.4f ext_ok=%d)",
                        errL, errR, (int)ext_ok);
            gh->abort(make_shared<Follow::Result>());
            return;
        }

        RCLCPP_INFO(node->get_logger(), 
              "Whether the robot has reached the target position: %d", (reached));

        rclcpp::sleep_for(chrono::milliseconds(50));
    }

    disable_arm_servos();
    // disable_all_ext_axes();
    gh->succeed(make_shared<Follow::Result>());
}

// ----- execute single arm: 7 joints -----
void execute_single_arm_goal(const shared_ptr<GoalHandle> gh, rclcpp::Node::SharedPtr node, int kind)
{
    auto goal = gh->get_goal();
    const auto& traj = goal->trajectory;

    if (traj.joint_names.size() != 7) {
        RCLCPP_ERROR(node->get_logger(), "Expected 7 joint names for single-arm, got %zu", traj.joint_names.size());
        gh->abort(make_shared<Follow::Result>());
        return;
    }
    if (traj.points.empty()) {
        RCLCPP_ERROR(node->get_logger(), "Trajectory has no points");
        gh->abort(make_shared<Follow::Result>());
        return;
    }

    array<int,7> mapSingle{};
    const char* pref = (kind == 0) ? "arm_lj" : "arm_rj";
    if (!build_index_map_single(traj.joint_names, pref, mapSingle)) {
        RCLCPP_ERROR(node->get_logger(), "Incorrect joint names for %s arm", (kind == 0 ? "LEFT" : "RIGHT"));
        gh->abort(make_shared<Follow::Result>());
        return;
    }

    const unsigned char arm = (kind == 0) ? 0u : 1u;
    const unsigned char other = (arm == 0) ? 1u : 0u;

    lock(g_arm_mtx[0], g_arm_mtx[1]);
    unique_lock<mutex> lk0(g_arm_mtx[0], adopt_lock);
    unique_lock<mutex> lk1(g_arm_mtx[1], adopt_lock);

    // robot.servo_move_use_none_filter(0);
    // robot.servo_move_use_none_filter(1);

    sched_param sch;
    sch.sched_priority = 90;
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &sch);

    if (robot.servo_move_enable(TRUE, 1, 0) != 0 || robot.servo_move_enable(TRUE, 1, 1) != 0) {
        RCLCPP_ERROR(node->get_logger(), "servo_move_enable failed");
        gh->abort(make_shared<Follow::Result>());
        return;
    }

    RCLCPP_INFO(node->get_logger(), "Received trajectory with %zu points (single-arm).", traj.points.size());

    JointValue other_hold{};
    if (robot.edg_get_stat(other, &other_hold, nullptr) != 0) {
        RCLCPP_WARN(node->get_logger(), "edg_get_stat failed when reading other arm");
        disable_arm_servos();
        gh->abort(make_shared<Follow::Result>());
        return;
    }

    RCLCPP_INFO(node->get_logger(), "Other arm (%u) read for hold: j1=%.2f°, j2=%.2f°, j3=%.2f°, j4=%.2f°, j5=%.2f°, j6=%.2f°, j7=%.2f°,",
                other, other_hold.jVal[0] * 180.0 / M_PI, other_hold.jVal[1] * 180.0 / M_PI, 
              other_hold.jVal[2] * 180.0 / M_PI, other_hold.jVal[3] * 180.0 / M_PI, 
            other_hold.jVal[4] * 180.0 / M_PI, other_hold.jVal[5] * 180.0 / M_PI, other_hold.jVal[6] * 180.0 / M_PI);

    JointValue fbA{};
    robot.edg_get_stat(arm, &fbA, nullptr);
    const auto& first_pt = traj.points.front();
    double max_start_err_a = 0.0;
    for (int j = 0; j < 7; ++j) {
        max_start_err_a = max(max_start_err_a, abs(fbA.jVal[j] - first_pt.positions[mapSingle[j]]));
    }
    if (max_start_err_a > 0.01) {
        RCLCPP_ERROR(node->get_logger(),"Single-robot trajectory start too far from actual state, error: arm=%.6f rad", max_start_err_a);
        disable_arm_servos();
        gh->abort(make_shared<Follow::Result>());
        return;
    }

    const bool use_hermite = trajectory_has_full_velocities(traj);
    RCLCPP_INFO(node->get_logger(),
                "Trajectory points=%zu, joint_names=%zu, interpolation=%s",
                traj.points.size(),
                traj.joint_names.size(),
                use_hermite ? "Hermite" : "Linear");

    const double total_t = point_time_sec(traj.points.back());
    const unsigned int step = 1;   // clean design: one EDG cycle per sample

    auto start_time = chrono::steady_clock::now();
    size_t seg_idx = 0;

    // Number of dense samples at 2 ms spacing
    const size_t num_samples = static_cast<size_t>(ceil(total_t / servo_period_sec));

    for (size_t k = 0; k <= num_samples; ++k)
    {
        if (gh->is_canceling()) {
            robot.motion_abort();
            disable_arm_servos();
            gh->canceled(make_shared<Follow::Result>());
            return;
        }

        double t = k * servo_period_sec;
        if (t > total_t) {
            t = total_t;
        }

        // Sleep until this sample's absolute execution time
        auto wake_time = start_time + chrono::duration_cast<chrono::steady_clock::duration>(chrono::duration<double>(t));

        this_thread::sleep_until(wake_time);

        JointValue j = interpolate_single_arm_sample(traj, mapSingle, t, seg_idx);

        int retArm, retOther;
        if (arm == 1) {
            retOther = robot.edg_servo_j(0, &other_hold, MoveMode::ABS, step);
            retArm   = robot.edg_servo_j(1, &j,        MoveMode::ABS, step);
        } else {
            retArm   = robot.edg_servo_j(0, &j,        MoveMode::ABS, step);
            retOther = robot.edg_servo_j(1, &other_hold, MoveMode::ABS, step);
        }

        if (retArm != 0 || retOther != 0) {
            RCLCPP_ERROR(node->get_logger(),
                        "edg_servo_j failed: arm=%s other=%s",
                        mapErr[retArm].c_str(),
                        mapErr[retOther].c_str());
            disable_arm_servos();
            gh->abort(make_shared<Follow::Result>());
            return;
        }

        int send_ret = robot.edg_send();
        if (send_ret != 0) {
            RCLCPP_WARN(node->get_logger(), "edg_send failed %d", send_ret);
        }

        // Debug
        // RCLCPP_INFO(node->get_logger(),
        //             "dense sample k=%zu t=%.6f step=%u, seg_idx=%zu",
        //             k, t, step, seg_idx);

        if (k % 100 == 0 || k == num_samples) {
            RCLCPP_INFO(node->get_logger(),
                        "dense sample k=%zu/%zu t=%.6f seg_idx=%zu",
                        k, num_samples, t, seg_idx);
        }
    }

    JointValue j_target{}, other_target{};
    const auto &last_pt = traj.points.back();
    for (int j = 0; j < 7; ++j) j_target.jVal[j] = last_pt.positions[ mapSingle[j] ];
    other_target = other_hold;

    const double traj_end_time = (double)last_pt.time_from_start.sec
                               + 1e-9 * (double)last_pt.time_from_start.nanosec;
    const int extra_ms_margin = 5000;
    auto deadline = chrono::steady_clock::now()
                  + chrono::milliseconds((int)(traj_end_time * 1000.0) + extra_ms_margin);

    const double arm_rad_tol = 0.01;

    RCLCPP_INFO(node->get_logger(),
                "Waiting for arm %u to reach final point (end_time=%.3f s, tol=%.4f rad).",
                arm, traj_end_time, arm_rad_tol);

    int inerr[2] = {0,0};
    BOOL in_col = FALSE;
    JointValue fbArm{};

    while (rclcpp::ok()) {
        if (gh->is_canceling()) {
            robot.motion_abort();
            disable_arm_servos();
            gh->canceled(make_shared<Follow::Result>());
            return;
        }

        robot.edg_recv();

        robot.is_in_collision(&in_col);
        robot.robot_is_in_error(inerr);
        if (in_col || inerr[arm] || inerr[other]) {
            RCLCPP_WARN(node->get_logger(), "Abort: collision=%d errArm=%d errOther=%d", in_col, inerr[arm], inerr[other]);
            disable_arm_servos();
            gh->abort(make_shared<Follow::Result>());
            return;
        }

        const bool okA = (robot.edg_get_stat(arm, &fbArm, nullptr) == 0);
    
        const double errArm = okA ? max_abs_err(fbArm, j_target) : 1e6;

        const bool reached = (okA && errArm <= arm_rad_tol);
        if (reached) {
            RCLCPP_INFO(node->get_logger(), "==============Single-arm Motion reached the target position for arm %u==============", arm);
            break;
        }

        if (chrono::steady_clock::now() >= deadline) {
            disable_arm_servos();
            RCLCPP_WARN(node->get_logger(), "Timeout waiting for single-arm target position (err=%.4f)", errArm);
            gh->abort(make_shared<Follow::Result>());
            return;
        }

        RCLCPP_INFO(node->get_logger(), 
              "Whether the robot has reached the target position: %d, errorArm: %.3f", reached, errArm);

        rclcpp::sleep_for(chrono::milliseconds(100));
    }

    disable_arm_servos();
    gh->succeed(make_shared<Follow::Result>());
}

// ----- execute external axis: 4 joints -----
void execute_ext_axis_goal(const shared_ptr<GoalHandle> gh, rclcpp::Node::SharedPtr node, double ext_vel, double ext_acc)
{
    auto goal = gh->get_goal();
    const auto& traj = goal->trajectory;

    if (traj.joint_names.size() != 4) {
        RCLCPP_ERROR(node->get_logger(), "Expected 4 joint names for ext-axis, got %zu", traj.joint_names.size());
        gh->abort(make_shared<Follow::Result>());
        return;
    }

    if (traj.points.empty()) {
        RCLCPP_ERROR(node->get_logger(), "Trajectory has no points");
        gh->abort(make_shared<Follow::Result>());
        return;
    }

    auto mapExt = build_index_map_ext(traj.joint_names);
    if (!mapExt.ok) {
        RCLCPP_ERROR(node->get_logger(), "Incorrect joint names for ext-axis");
        gh->abort(make_shared<Follow::Result>());
        return;
    }

    // lock(g_ext_mtx);
    unique_lock<mutex> lkE(g_ext_mtx);

    if (!enable_all_ext_axes()) {
        RCLCPP_ERROR(node->get_logger(), "enable_all_ext_axes failed");
        disable_all_ext_axes();
        gh->abort(make_shared<Follow::Result>());
        return;
    }

    RCLCPP_INFO(node->get_logger(), "Received trajectory with %zu points (ext-axis).", traj.points.size());

    const auto& last_pt = traj.points.back();
    if (last_pt.positions.size() != 4) { 
        RCLCPP_ERROR(node->get_logger(), "Last point has %zu positions (need 4)", last_pt.positions.size()); 
        // disable_all_ext_axes(); 
        gh->abort(make_shared<Follow::Result>()); 
        return; 
    }

    const auto ext_target = extract_ext_target_sdk_units(last_pt, mapExt.B);

    errno_t ext_ret = send_ext_target_nonblocking(ext_target, ext_vel, ext_acc);
    if (ext_ret != ERR_SUCC) {
        RCLCPP_ERROR(node->get_logger(),
                    "Failed to send ext-axis target, sdk_ret=%d", ext_ret);
        disable_arm_servos();
        // disable_all_ext_axes();
        gh->abort(make_shared<Follow::Result>());
        return;
    }

    RCLCPP_INFO(node->get_logger(),
                "Ext target sent: %.3f mm, %.3f deg, %.3f deg, %.3f deg",
                ext_target[0], ext_target[1], ext_target[2], ext_target[3]);

    const double traj_end_time = (double)last_pt.time_from_start.sec 
                               + 1e-9 * (double)last_pt.time_from_start.nanosec;

    const int extra_ms_margin = 5000;
    auto deadline = chrono::steady_clock::now()
                  + chrono::milliseconds((int)(traj_end_time * 1000.0) + extra_ms_margin);

    const double ext_lin_tol = 0.5;
    const double ext_deg_tol = 0.5;

    RCLCPP_INFO(node->get_logger(), "Waiting for ext-axis to reach final point (end_time=%.3f s).", traj_end_time);

    array<double,4> ext_fb{};

    while (rclcpp::ok()) {
        if (gh->is_canceling()) {
            stop_all_ext_axes();
            // disable_all_ext_axes();
            gh->canceled(make_shared<Follow::Result>());
            return;
        }

        const bool okE = read_ext_feedback(ext_fb);
        const bool reached = okE && ext_target_reached(ext_fb, ext_target, ext_lin_tol, ext_deg_tol);

        if (reached) {
            RCLCPP_INFO(node->get_logger(),
                        "==============Ext-axis motion reached the target position==============");
            break;
        }

        if (chrono::steady_clock::now() >= deadline) {
            stop_all_ext_axes();
            // disable_all_ext_axes();
            RCLCPP_WARN(node->get_logger(),
                        "Timeout waiting for ext-axis target position");
            gh->abort(make_shared<Follow::Result>());
            return;
        }

        RCLCPP_INFO(node->get_logger(), 
              "Whether the robot has reached the target position: %d", (reached));

        rclcpp::sleep_for(chrono::milliseconds(50));
    }

    stop_all_ext_axes();
    // disable_all_ext_axes();
    gh->succeed(make_shared<Follow::Result>());
}

void execute_agv_goal(const shared_ptr<GoalHandle> gh,
                      rclcpp::Node::SharedPtr node,
                      const string &agv_map_name,
                      double agv_linear_speed,
                      double agv_angular_speed,
                      double agv_dece_distance,
                      double agv_stop_distance)
{
    auto goal = gh->get_goal();
    const auto &traj = goal->trajectory;

    if (traj.joint_names.size() != 3) {
        RCLCPP_ERROR(node->get_logger(), "Expected 3 AGV joint names, got %zu", traj.joint_names.size());
        gh->abort(make_shared<Follow::Result>());
        return;
    }

    if (traj.points.empty()) {
        RCLCPP_ERROR(node->get_logger(), "AGV trajectory has no points");
        gh->abort(make_shared<Follow::Result>());
        return;
    }

    auto mapAgv = build_index_map_agv(traj.joint_names);
    if (!mapAgv.ok) {
        RCLCPP_ERROR(node->get_logger(), "AGV joint names must contain agv_x, agv_y, agv_yaw");
        gh->abort(make_shared<Follow::Result>());
        return;
    }

    unique_lock<mutex> lk(g_agv_goal_mtx);

    if (!agv_auto_move_client || !agv_auto_move_client->wait_for_service(chrono::seconds(2))) {
        RCLCPP_ERROR(node->get_logger(), "AGV auto move service not available");
        gh->abort(make_shared<Follow::Result>());
        return;
    }

    double sx, sy, syaw;
    if (!get_latest_agv_pose(sx, sy, syaw)) {
        RCLCPP_ERROR(node->get_logger(), "No AGV odom received yet on global_nav_odom");
        gh->abort(make_shared<Follow::Result>());
        return;
    }

    const auto &last_pt = traj.points.back();
    if (last_pt.positions.size() < 3) {
        RCLCPP_ERROR(node->get_logger(), "Last AGV point has fewer than 3 positions");
        gh->abort(make_shared<Follow::Result>());
        return;
    }

    int agv_move_mode = 0;
    int agv_move_type = 0;

    auto req = make_shared<jagv_interfaces::srv::AutoMove::Request>();
    req->map_name = agv_map_name;

    req->start_pose.x = sx;
    req->start_pose.y = sy;
    req->start_pose.theta = syaw;

    req->end_pose = point_to_pose2d(last_pt, mapAgv);

    req->path_coords.clear();
    // if (traj.points.size() > 1) {
    //     for (size_t i = 0; i + 1 < traj.points.size(); ++i) {
    //         if (traj.points[i].positions.size() < 3) {
    //             RCLCPP_ERROR(node->get_logger(), "AGV point %zu has fewer than 3 positions", i);
    //             gh->abort(make_shared<Follow::Result>());
    //             return;
    //         }
    //         req->path_coords.push_back(point_to_pose2d(traj.points[i], mapAgv));
    //     }
    // }

    req->linear_speed = agv_linear_speed;
    req->angular_speed = agv_angular_speed;
    req->dece_distance = agv_dece_distance;
    req->stop_distance = agv_stop_distance;
    req->move_mode = agv_move_mode;
    req->move_type = agv_move_type;

    RCLCPP_INFO(node->get_logger(),
                "Sending AGV auto move: start=(%.3f, %.3f, %.3f), end=(%.3f, %.3f, %.3f), path_coords=%zu",
                req->start_pose.x, req->start_pose.y, req->start_pose.theta,
                req->end_pose.x, req->end_pose.y, req->end_pose.theta,
                req->path_coords.size());

    auto future = agv_auto_move_client->async_send_request(req);

    while (future.wait_for(chrono::milliseconds(50)) != future_status::ready) {
        if (gh->is_canceling()) {
            stop_agv_motion();
            gh->canceled(make_shared<Follow::Result>());
            return;
        }
    }

    auto resp = future.get();
    if (!resp || resp->ret_code != 0) {
        RCLCPP_ERROR(node->get_logger(), "AGV auto move failed, ret_code=%d, ret_msg='%s'", resp ? resp->ret_code : -9999, resp->ret_msg.c_str());
        gh->abort(make_shared<Follow::Result>());
        return;
    }

    const double agv_target_x = last_pt.positions[mapAgv.x];
    const double agv_target_y = last_pt.positions[mapAgv.y];
    const double agv_target_yaw = last_pt.positions[mapAgv.yaw];

    const double traj_end_time = (double)last_pt.time_from_start.sec 
                               + 1e-9 * (double)last_pt.time_from_start.nanosec;
    const int extra_ms_margin = 5000;
    auto deadline = chrono::steady_clock::now() +
                    chrono::milliseconds((int)(traj_end_time * 1000.0) + extra_ms_margin);

    const double agv_xy_tol = 0.05;
    const double agv_yaw_tol = 0.05;

    RCLCPP_INFO(node->get_logger(), "Waiting for AGV to reach final point (end_time=%.3f s).", traj_end_time);
    
    double agv_x, agv_y, agv_yaw;

    while (rclcpp::ok()) {
        if (gh->is_canceling()) {
            stop_agv_motion();
            gh->canceled(make_shared<Follow::Result>());
            return;
        }

        const bool okA = get_latest_agv_pose(agv_x, agv_y, agv_yaw);
        const double err_agv_pos = okA ? hypot(agv_x - agv_target_x, agv_y - agv_target_y) : 1e6;
        const double err_agv_yaw = okA ? abs(normalize_angle(agv_yaw - agv_target_yaw)) : 1e6;
        const bool reached = okA && (err_agv_pos <= agv_xy_tol) && (err_agv_yaw <= agv_yaw_tol);

        auto feedback = make_shared<Follow::Feedback>();
        feedback->joint_names = traj.joint_names;
        feedback->actual.positions = {agv_x, agv_y, agv_yaw};
        gh->publish_feedback(feedback);

        if (reached) {
            RCLCPP_INFO(node->get_logger(),
                        "AGV reached target. err_agv_pos=%.4f m err_agv_yaw=%.4f rad",
                        err_agv_pos, err_agv_yaw);
            gh->succeed(make_shared<Follow::Result>());
            return;
        }

        if (chrono::steady_clock::now() >= deadline) {
            RCLCPP_WARN(node->get_logger(),
                        "Timeout waiting for AGV target. err_agv_pos=%.4f m err_agv_yaw=%.4f rad",
                        err_agv_pos, err_agv_yaw);
            gh->abort(make_shared<Follow::Result>());
            return;
        }

        rclcpp::sleep_for(chrono::milliseconds(100));
    }

    gh->abort(make_shared<Follow::Result>());
}

// ----- joint states: publish arms + body in one message -----
void joint_states_callback(rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr &joint_states_pub)
{
    sensor_msgs::msg::JointState joint_msg;

    JointValue jl{}, jr{};
    const bool okL = robot.edg_get_stat(0, &jl, nullptr) == 0;
    const bool okR = robot.edg_get_stat(1, &jr, nullptr) == 0;

    array<double,4> ext_fb{};
    const bool okE = read_ext_feedback(ext_fb);

    double agv_x = numeric_limits<double>::quiet_NaN();
    double agv_y = numeric_limits<double>::quiet_NaN();
    double agv_yaw = numeric_limits<double>::quiet_NaN();
    const bool okAGV = get_latest_agv_pose(agv_x, agv_y, agv_yaw);

    joint_msg.name.reserve(21);
    joint_msg.position.reserve(21);

    for (int i = 0; i < 7; ++i) {
        joint_msg.position.push_back(okL ? jl.jVal[i] : numeric_limits<double>::quiet_NaN());
        joint_msg.name.push_back("arm_lj" + to_string(i + 1));
    }

    for (int i = 0; i < 7; ++i) {
        joint_msg.position.push_back(okR ? jr.jVal[i] : numeric_limits<double>::quiet_NaN());
        joint_msg.name.push_back("arm_rj" + to_string(i + 1));
    }

    for (int i = 0; i < 4; ++i) {
        if (i == 0) {
            joint_msg.position.push_back(okE ? (ext_fb[i] / 1000.0) : numeric_limits<double>::quiet_NaN());  // mm -> m
        } else {
            joint_msg.position.push_back(okE ? (ext_fb[i] * M_PI / 180.0) : numeric_limits<double>::quiet_NaN());    // deg -> rad
        }
        joint_msg.name.push_back("body_j" + to_string(i + 1));
    }

    joint_msg.position.push_back(okAGV ? agv_x : numeric_limits<double>::quiet_NaN());
    joint_msg.name.push_back("agv_x");

    joint_msg.position.push_back(okAGV ? agv_y : numeric_limits<double>::quiet_NaN());
    joint_msg.name.push_back("agv_y");

    joint_msg.position.push_back(okAGV ? agv_yaw : numeric_limits<double>::quiet_NaN());
    joint_msg.name.push_back("agv_yaw");

    joint_msg.header.stamp = rclcpp::Clock().now();
    joint_states_pub->publish(joint_msg);
}

void sigintHandler(int /*sig*/)
{
    rclcpp::shutdown();
}

// ----- main -----
int main(int argc, char** argv)
{
    setlocale(LC_ALL, "");
    rclcpp::init(argc, argv);
    signal(SIGINT, sigintHandler);

    auto node = rclcpp::Node::make_shared("kargo_moveit_server");

    string default_ip = "127.0.0.1";
    string robot_ip = node->declare_parameter("ip", default_ip);

    edg_init_ip = make_edg_bcast(robot_ip);
    RCLCPP_INFO(node->get_logger(), "EDG init IP set to: %s", edg_init_ip.c_str());

    double ext_vel = node->declare_parameter("ext_vel", 50.0);
    double ext_acc = node->declare_parameter("ext_acc", 50.0);

    string agv_ns = node->declare_parameter("agv_ns", "/JAGV_O_01");
    string agv_map_name = node->declare_parameter("agv_map_name", "testmap");

    double agv_linear_speed = node->declare_parameter("agv_linear_speed", 0.5);
    double agv_angular_speed = node->declare_parameter("agv_angular_speed", 0.2);
    double agv_dece_distance = node->declare_parameter("agv_dece_distance", 1.0);
    double agv_stop_distance = node->declare_parameter("agv_stop_distance", 0.2);

    // Connect
    int ret = robot.login_in(robot_ip.c_str());
    if (ret != 0) 
    { 
        RCLCPP_FATAL(node->get_logger(), "login_in failed: %s", mapErr[ret].c_str()); 
        return -1; 
    }
    rclcpp::Rate rate(125);

    // ensure arms are out of servo mode at startup
    robot.servo_move_enable(FALSE, 1, 0);
    robot.servo_move_enable(FALSE, 1, 1);
    rclcpp::sleep_for(chrono::milliseconds(500));

    robot.servo_move_use_joint_LPF(0.5, -1);

    // Power on
    ret = robot.power_on();
    if (ret != 0) 
    { 
        RCLCPP_FATAL(node->get_logger(), "power_on failed: %s", mapErr[ret].c_str()); 
        return -1; 
    }
    rclcpp::sleep_for(chrono::seconds(10));

    ret = robot.enable_robot();
    if (ret != 0)
     { 
        RCLCPP_FATAL(node->get_logger(), "enable_robot failed: %s", mapErr[ret].c_str()); 
        return -1; 
    }
    rclcpp::sleep_for(chrono::seconds(5));

    robot.edg_init(1, edg_init_ip.c_str());

    joint_states_pub = node->create_publisher<sensor_msgs::msg::JointState>("/kargo_joint_states", 10);
    agv_odom_sub = node->create_subscription<nav_msgs::msg::Odometry>(agv_ns + "/global_nav_odom", 10, agv_odom_callback);

    agv_auto_move_client = node->create_client<jagv_interfaces::srv::AutoMove>(agv_ns + "/agv_auto_move");
    agv_motion_ctrl_client = node->create_client<jagv_interfaces::srv::MotionStateControl>(agv_ns + "/motion_state_control");

    // full body server: 21 joints
    auto full_server = rclcpp_action::create_server<Follow>(
        node,
        "/jaka_kargo_full_robot_controller/follow_joint_trajectory",
        [](const rclcpp_action::GoalUUID&, shared_ptr<const Follow::Goal>) {
            RCLCPP_INFO(rclcpp::get_logger("kargo_moveit_server"), "Received full_robot goal request");
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [](shared_ptr<GoalHandle>) {
            RCLCPP_INFO(rclcpp::get_logger("kargo_moveit_server"), "Received full_robot cancel request");
            return rclcpp_action::CancelResponse::ACCEPT;
        },
        [node, ext_vel, ext_acc,
        agv_map_name, agv_linear_speed, agv_angular_speed,
        agv_dece_distance, agv_stop_distance](shared_ptr<GoalHandle> gh) {
            thread([node, gh, ext_vel, ext_acc,
                        agv_map_name, agv_linear_speed, agv_angular_speed,
                        agv_dece_distance, agv_stop_distance]() {
                RCLCPP_INFO(rclcpp::get_logger("kargo_moveit_server"), "Executing full_robot goal");
                execute_full_robot_goal(gh, node,
                                        ext_vel, ext_acc,
                                        agv_map_name,
                                        agv_linear_speed,
                                        agv_angular_speed,
                                        agv_dece_distance,
                                        agv_stop_distance);
            }).detach();
        }
    );

    // left arm only server: 7 joints
    auto left_server = rclcpp_action::create_server<Follow>(
        node,
        "/jaka_kargo_arm_l_controller/follow_joint_trajectory",
        [](const rclcpp_action::GoalUUID&, shared_ptr<const Follow::Goal>) {
            RCLCPP_INFO(rclcpp::get_logger("kargo_moveit_server"), "Received arm_l goal request");
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [](shared_ptr<GoalHandle>) {
            RCLCPP_INFO(rclcpp::get_logger("kargo_moveit_server"), "Received arm_l cancel request");
            return rclcpp_action::CancelResponse::ACCEPT;
        },
        [node](shared_ptr<GoalHandle> gh) {
            thread([node, gh]() {
                RCLCPP_INFO(rclcpp::get_logger("kargo_moveit_server"), "Executing arm_l goal");
                execute_single_arm_goal(gh, node, 0);
            }).detach();
        }
    );

    // right arm only server: 7 joints
    auto right_server = rclcpp_action::create_server<Follow>(
        node,
        "/jaka_kargo_arm_r_controller/follow_joint_trajectory",
        [](const rclcpp_action::GoalUUID&, shared_ptr<const Follow::Goal>) {
            RCLCPP_INFO(rclcpp::get_logger("kargo_moveit_server"), "Received arm_r goal request");
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [](shared_ptr<GoalHandle>) {
            RCLCPP_INFO(rclcpp::get_logger("kargo_moveit_server"), "Received arm_r cancel request");
            return rclcpp_action::CancelResponse::ACCEPT;
        },
        [node](shared_ptr<GoalHandle> gh) {
            thread([node, gh]() {
                RCLCPP_INFO(rclcpp::get_logger("kargo_moveit_server"), "Executing arm_r goal");
                execute_single_arm_goal(gh, node, 1);
            }).detach();
        }
    );
    // ext-axis only server: 4 joints
    auto ext_axis_server = rclcpp_action::create_server<Follow>(
        node,
        "/jaka_kargo_body_controller/follow_joint_trajectory",
        [](const rclcpp_action::GoalUUID&, shared_ptr<const Follow::Goal>) {
            RCLCPP_INFO(rclcpp::get_logger("kargo_moveit_server"), "Received ext_axis goal request");
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [](shared_ptr<GoalHandle>) {
            RCLCPP_INFO(rclcpp::get_logger("kargo_moveit_server"), "Received ext_axis cancel request");
            return rclcpp_action::CancelResponse::ACCEPT;
        },
        [node, ext_vel, ext_acc](shared_ptr<GoalHandle> gh) {
            thread([node, gh, ext_vel, ext_acc]() {
                RCLCPP_INFO(rclcpp::get_logger("kargo_moveit_server"), "Executing ext_axis goal");
                execute_ext_axis_goal(gh, node, ext_vel, ext_acc);
            }).detach();
        }
    );

    // agv only server: 3 joints
    auto agv_server = rclcpp_action::create_server<Follow>(
        node,
        "/jaka_kargo_agv_controller/follow_joint_trajectory",
        [](const rclcpp_action::GoalUUID&, shared_ptr<const Follow::Goal>) {
            RCLCPP_INFO(rclcpp::get_logger("kargo_moveit_server"), "Received AGV goal request");
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [](shared_ptr<GoalHandle>) {
            RCLCPP_INFO(rclcpp::get_logger("kargo_moveit_server"), "Received AGV cancel request");
            return rclcpp_action::CancelResponse::ACCEPT;
        },
        [node, agv_map_name, agv_linear_speed, agv_angular_speed,
        agv_dece_distance, agv_stop_distance](shared_ptr<GoalHandle> gh) {
            thread([node, gh, agv_map_name, agv_linear_speed, agv_angular_speed,
                        agv_dece_distance, agv_stop_distance]() {
                RCLCPP_INFO(rclcpp::get_logger("kargo_moveit_server"), "Executing AGV goal");
                execute_agv_goal(gh, node,
                                agv_map_name,
                                agv_linear_speed,
                                agv_angular_speed,
                                agv_dece_distance,
                                agv_stop_distance);
            }).detach();
        }
    );

    RCLCPP_INFO(rclcpp::get_logger("kargo_moveit_server"),
                "==================Kargo MoveIt Server Start==================");

    while (rclcpp::ok()) {
        joint_states_callback(joint_states_pub);
        rate.sleep();
        rclcpp::spin_some(node);
    }

    // cleanup on shutdown
    disable_arm_servos();
    stop_all_ext_axes();
    // disable_all_ext_axes();
    robot.login_out();

    rclcpp::shutdown();
    return 0;
}