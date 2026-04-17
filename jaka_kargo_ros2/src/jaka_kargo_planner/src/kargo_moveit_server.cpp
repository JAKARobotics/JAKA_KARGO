#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include "sensor_msgs/msg/joint_state.hpp"
#include <trajectory_msgs/msg/joint_trajectory.hpp>

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

// Global EDG broadcast IP used by edg_init calls
static std::string edg_init_ip = "255.255.255.255";

/// Make EDG broadcast IP by replacing last IPv4 octet with 255.
static std::string make_edg_bcast(const std::string &ip)
{
    auto last_oct = ip.find_last_of('.');
    if (last_oct == std::string::npos) {
        return ip;
    }
    return ip.substr(0, last_oct + 1) + "255";
}

static double servo_period_sec = 0.002;

// Guards
static mutex g_arm_mtx[2];   // 0 = LEFT, 1 = RIGHT
static mutex g_ext_mtx;      // external axis group

// ----- index maps -----
struct IndexMapFull {
    array<int, 7> L{};
    array<int, 7> R{};
    array<int, 4> B{};
    bool ok{false};
};

struct IndexMapExt {
    array<int, 4> B{};
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
        double e = std::abs(a.jVal[i] - b.jVal[i]);
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

static bool ext_target_reached(const array<double,4>& current_pos, const array<double,4>& target_pos)
{
    // joint 0: mm, joints 1..3: deg
    if (std::abs(current_pos[0] - target_pos[0]) > 0.5) return false;
    for (int i = 1; i < 4; ++i) {
        if (std::abs(current_pos[i] - target_pos[i]) > 0.5) return false;
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
    const std::array<int,7>& mapSingle,
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

// ----- execute full body: 18 joints -----
void execute_full_robot_goal(const shared_ptr<GoalHandle> gh, rclcpp::Node::SharedPtr node, double ext_vel, double ext_acc)
{
    auto goal = gh->get_goal();
    const auto& traj = goal->trajectory;

    if (traj.joint_names.size() != 18) {
        RCLCPP_ERROR(node->get_logger(), "Expected 18 joint names for full-robot, got %zu", traj.joint_names.size());
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
    lock(g_arm_mtx[0], g_arm_mtx[1], g_ext_mtx);
    unique_lock<mutex> lk0(g_arm_mtx[0], adopt_lock);
    unique_lock<mutex> lk1(g_arm_mtx[1], adopt_lock);
    unique_lock<mutex> lkE(g_ext_mtx, adopt_lock);

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

    RCLCPP_INFO(node->get_logger(), "Received trajectory with %zu points (full-robot).", traj.points.size());

    const auto& last_pt = traj.points.back();
    if (last_pt.positions.size() != 18) {
        RCLCPP_ERROR(node->get_logger(), "Last point has %zu positions (need 18)", last_pt.positions.size());
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

    // external axis final target (non-blocking, starts near the same time as arm motion)
    const auto ext_target = extract_ext_target_sdk_units(last_pt, mapFull.B);

    for (int i = 0; i < 4; ++i) {
        if (!std::isfinite(ext_target[i])) {
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
    const size_t num_samples = static_cast<size_t>(std::ceil(total_t / servo_period_sec));

    auto start_time = std::chrono::steady_clock::now();
    size_t seg_idx = 0;

    for (size_t i = 0; i < traj.points.size(); ++i) {
        const auto& pt = traj.points[i];
        if (pt.positions.size() != 18) {
            RCLCPP_ERROR(node->get_logger(), "Point %zu has %zu positions (need 18)", i, pt.positions.size());
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
        RCLCPP_INFO(node->get_logger(),
                    "full dense sample k=%zu t=%.6f step=%u seg_idx=%zu",
                    k, t, step, seg_idx);
    }

    JointValue jl_target{}, jr_target{};
    for (int j = 0; j < 7; ++j) jl_target.jVal[j] = last_pt.positions[ mapFull.L[j] ];
    for (int j = 0; j < 7; ++j) jr_target.jVal[j] = last_pt.positions[ mapFull.R[j] ];

    const double traj_end_time = (double)last_pt.time_from_start.sec
                               + 1e-9 * (double)last_pt.time_from_start.nanosec;
    const int extra_ms_margin = 5000;
    auto deadline = chrono::steady_clock::now()
                  + chrono::milliseconds((int)(traj_end_time * 1000.0) + extra_ms_margin);

    const double tol_rad = 0.01;

    RCLCPP_INFO(node->get_logger(),
                "Waiting for full-robot to reach final point (end_time=%.3f s, tol=%.4f rad).",
                traj_end_time, tol_rad);

    int inerr[2] = {0, 0};
    BOOL in_col = FALSE;
    JointValue fbL{}, fbR{};
    array<double,4> ext_fb{};

    while (rclcpp::ok()) {
        if (gh->is_canceling()) {
            robot.motion_abort();
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
            stop_all_ext_axes();
            disable_arm_servos();
            // disable_all_ext_axes();
            gh->abort(make_shared<Follow::Result>());
            return;
        }

        const bool okL = (robot.edg_get_stat(0, &fbL, nullptr) == 0);
        const bool okR = (robot.edg_get_stat(1, &fbR, nullptr) == 0);
        const bool okE = read_ext_feedback(ext_fb);

        const double errL = okL ? max_abs_err(fbL, jl_target) : 1e6;
        const double errR = okR ? max_abs_err(fbR, jr_target) : 1e6;
        const bool arms_ok = okL && okR && (errL <= tol_rad) && (errR <= tol_rad);
        const bool ext_ok = okE && ext_target_reached(ext_fb, ext_target);
        bool reached = arms_ok && ext_ok;

        if (reached) {
            RCLCPP_INFO(node->get_logger(), "==============Full-robot motion reached the target position==============");
            break;
        }

        if (chrono::steady_clock::now() >= deadline) {
            robot.motion_abort();
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
        max_start_err_a = std::max(max_start_err_a, std::abs(fbA.jVal[j] - first_pt.positions[mapSingle[j]]));
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

    // double last_t = 0.0;
    // auto next_time = chrono::steady_clock::now();
    // auto start_time = std::chrono::steady_clock::now();

    const double total_t = point_time_sec(traj.points.back());
    const unsigned int step = 1;   // clean design: one EDG cycle per sample

    auto start_time = std::chrono::steady_clock::now();
    size_t seg_idx = 0;

    // Number of dense samples at 2 ms spacing
    const size_t num_samples =
        static_cast<size_t>(std::ceil(total_t / servo_period_sec));

    for (size_t k = 0; k <= num_samples; ++k)
    {
        if (gh->is_canceling()) {
            robot.motion_abort();
            disable_arm_servos();
            gh->canceled(std::make_shared<Follow::Result>());
            return;
        }

        double t = k * servo_period_sec;
        if (t > total_t) {
            t = total_t;
        }

        // Sleep until this sample's absolute execution time
        auto wake_time =
            start_time +
            std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<double>(t));

        std::this_thread::sleep_until(wake_time);

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
            gh->abort(std::make_shared<Follow::Result>());
            return;
        }

        int send_ret = robot.edg_send();
        if (send_ret != 0) {
            RCLCPP_WARN(node->get_logger(), "edg_send failed %d", send_ret);
        }

        // Debug
        RCLCPP_INFO(node->get_logger(),
                    "dense sample k=%zu t=%.6f step=%u, seg_idx=%zu",
                    k, t, step, seg_idx);
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

    const double tol_rad = 0.01;

    RCLCPP_INFO(node->get_logger(),
                "Waiting for arm %u to reach final point (end_time=%.3f s, tol=%.4f rad).",
                arm, traj_end_time, tol_rad);

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

        const bool reached = (okA && errArm <= tol_rad);
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

    if (!send_ext_target_nonblocking(ext_target, ext_vel, ext_acc)) {
        RCLCPP_ERROR(node->get_logger(), "Failed to send ext-axis target");
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

    RCLCPP_INFO(node->get_logger(),
                "Waiting for ext-axis to reach final point (end_time=%.3f s).",
                traj_end_time);

    array<double,4> ext_fb{};

    while (rclcpp::ok()) {
        if (gh->is_canceling()) {
            stop_all_ext_axes();
            // disable_all_ext_axes();
            gh->canceled(make_shared<Follow::Result>());
            return;
        }

        const bool okE = read_ext_feedback(ext_fb);
        const bool reached = okE && ext_target_reached(ext_fb, ext_target);

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

// ----- joint states: publish arms + body in one message -----
void joint_states_callback(rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr &joint_states_pub)
{
    sensor_msgs::msg::JointState joint_msg;

    JointValue jl{}, jr{};
    const bool okL = robot.edg_get_stat(0, &jl, nullptr) == 0;
    const bool okR = robot.edg_get_stat(1, &jr, nullptr) == 0;

    array<double,4> ext_fb{};
    const bool okE = read_ext_feedback(ext_fb);

    joint_msg.name.reserve(18);
    joint_msg.position.reserve(18);

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

    joint_states_pub = node->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

    // full body server: 18 joints
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
        [node, ext_vel, ext_acc](shared_ptr<GoalHandle> gh) {
            std::thread([node, gh, ext_vel, ext_acc]() {
                RCLCPP_INFO(rclcpp::get_logger("kargo_moveit_server"), "Executing full_robot goal");
                execute_full_robot_goal(gh, node, ext_vel, ext_acc);
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
            std::thread([node, gh]() {
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
            std::thread([node, gh]() {
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
            std::thread([node, gh, ext_vel, ext_acc]() {
                RCLCPP_INFO(rclcpp::get_logger("kargo_moveit_server"), "Executing ext_axis goal");
                execute_ext_axis_goal(gh, node, ext_vel, ext_acc);
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