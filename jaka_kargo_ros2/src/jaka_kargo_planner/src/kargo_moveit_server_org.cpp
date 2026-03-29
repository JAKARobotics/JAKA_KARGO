#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include "rclcpp_action/rclcpp_action.hpp"

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
using namespace std;
// using namespace chrono_literals;

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
        // not an IPv4-like string — return unchanged (fallback)
        return ip;
    }
    return ip.substr(0, last_oct + 1) + "255";
}

// --- Per-arm EDG access guards
static mutex g_arm_mtx[2]; // 0=LEFT, 1=RIGHT

// --- Build index map for dual arm (14-joints) goals
struct IndexMapDual  {
  // indices into goal.trajectory.joint_names for left/right joints
  array<int,7> L{};
  array<int,7> R{};
  bool ok{false};
};

static IndexMapDual build_index_map_dual(const vector<string>& names)
{
  IndexMapDual m;
  unordered_map<string,int> idx;
  for (int i=0;i<(int)names.size();++i) idx[names[i]] = i;

  const char* Lnames[7] = {"arm_lj1","arm_lj2","arm_lj3","arm_lj4","arm_lj5","arm_lj6","arm_lj7"};
  const char* Rnames[7] = {"arm_rj1","arm_rj2","arm_rj3","arm_rj4","arm_rj5","arm_rj6","arm_rj7"};

  for (int i=0;i<7;i++) {
    if (!idx.count(Lnames[i]) || !idx.count(Rnames[i])) {
      m.ok = false; return m;
    }
    m.L[i] = idx[Lnames[i]];
    m.R[i] = idx[Rnames[i]];
  }
  m.ok = true;
  return m;
}

// --- Build index map for a single arm (7) goals
static bool build_index_map_single(const vector<string>& names,
                                const char* prefix, array<int,7>& out) {
  unordered_map<string,int> idx;
  for (int i=0;i<(int)names.size();++i) idx[names[i]] = i;
  for (int i=0;i<7;i++) {
    string jn = string(prefix) + to_string(i+1); // e.g. "arm_lj3"
    auto it = idx.find(jn); if (it==idx.end()) return false;
    out[i] = it->second;
  }
  return true;
}


/* ---------- FollowJointTrajectory (14 joints) → EDG servo for both arms ---------- */
static inline unsigned int steps_from_dt(double dt_sec, double servo_period=0.001)
{
  if (dt_sec <= 0.0) return 1u;
  const double s = dt_sec / servo_period;
  unsigned int k = (unsigned int)llround(s);
  return k == 0 ? 1u : k;
}

// kind: -1 FULL, 0 LEFT, 1 RIGHT
void execute_goal(const shared_ptr<GoalHandle> gh, rclcpp::Node::SharedPtr node, int kind)
{
  auto goal = gh->get_goal();
  const auto& traj = goal->trajectory;

  if ((kind==-1 && traj.joint_names.size()!=14) ||
      (kind!=-1  && traj.joint_names.size()!=7)) {
    RCLCPP_ERROR(node->get_logger(),
      "Unexpected joint_names size: %zu", traj.joint_names.size());
    gh->abort(make_shared<Follow::Result>());
    return;
  }
  if (traj.points.empty()) {
    RCLCPP_ERROR(node->get_logger(), "Trajectory has no points");
    gh->abort(make_shared<Follow::Result>());
    return;
  }

  // Build name→index maps
  IndexMapDual mapDual {}; array<int,7> mapSingle{};
  if (kind==-1) {
    mapDual = build_index_map_dual(traj.joint_names);
    if (!mapDual.ok) { 
      RCLCPP_ERROR(node->get_logger(), "Incorrect joint names for dual-robot"); 
      gh->abort(make_shared<Follow::Result>()); 
      return; 
    }
  } else {
    const char* pref = (kind==0) ? "arm_lj" : "arm_rj";
    if (!build_index_map_single(traj.joint_names, pref, mapSingle)) {
      RCLCPP_ERROR(node->get_logger(), "Incorrect joint names for %s arm", (kind==0?"LEFT":"RIGHT"));
      gh->abort(make_shared<Follow::Result>()); 
      return;
    }
  }

  // Helper to compute max abs error
  auto max_abs_err = [](const JointValue &a, const JointValue &b) {
    double m = 0.0;
    for (int i = 0; i < 7; ++i) {
      double e = abs(a.jVal[i] - b.jVal[i]);
      if (e > m) m = e;
    }
    return m;
  };

  // Dual-arm (full-robot) case
  if (kind==-1) {
    lock(g_arm_mtx[0], g_arm_mtx[1]);
    unique_lock<mutex> lk0(g_arm_mtx[0], adopt_lock);
    unique_lock<mutex> lk1(g_arm_mtx[1], adopt_lock);

    robot.servo_move_use_none_filter(0);
    robot.servo_move_use_none_filter(1);
    sched_param sch;
    sch.sched_priority = 90;
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &sch);

    // Enable EDG servo for both arms
    if (robot.servo_move_enable(TRUE, 1, 0) != 0 || robot.servo_move_enable(TRUE, 1, 1) != 0) {
      RCLCPP_ERROR(node->get_logger(), "servo_move_enable failed");
      gh->abort(make_shared<Follow::Result>());
      return;
    }


    // Debug: how many points and mapping indices
    RCLCPP_INFO(node->get_logger(), "Received trajectory with %zu points (dual-arm, 14 joints).",
                traj.points.size());
    // Print joint_names mapping
    string map_s = "Mapping: ";
    for (int j=0;j<7;j++) {
      map_s += "L" + to_string(j+1) + "=" + to_string(mapDual.L[j]) + " ";
    }
    for (int j=0;j<7;j++) {
      map_s += "R" + to_string(j+1) + "=" + to_string(mapDual.R[j]) + " ";
    }
    RCLCPP_DEBUG(node->get_logger(), "%s", map_s.c_str());

    // Execute all points
    double last_t = 0.0;
    auto next_time = chrono::steady_clock::now();
    for (size_t i=0; i<traj.points.size(); ++i)
    {
      // Allow cancel
      if (gh->is_canceling()) {
        robot.motion_abort();
        robot.servo_move_enable(FALSE, 1, 0);
        robot.servo_move_enable(FALSE, 1, 1);
        gh->canceled(make_shared<Follow::Result>());
        return;
      }

      const auto& pt = traj.points[i];
      if (pt.positions.size() != 14) {
        RCLCPP_ERROR(node->get_logger(), "Point %zu has %zu positions (need 14)", i, pt.positions.size());
        robot.servo_move_enable(FALSE, 1, 0);
        robot.servo_move_enable(FALSE, 1, 1);
        gh->abort(make_shared<Follow::Result>());
        return;
      }

      // Split positions → per-arm JointValue (radians)
      JointValue jl{}, jr{};
      for (int j=0;j<7;j++) jl.jVal[j] = pt.positions[ mapDual.L[j] ];
      for (int j=0;j<7;j++) jr.jVal[j] = pt.positions[ mapDual.R[j] ];

      // Compute step count from dt
      const double t = (double)pt.time_from_start.sec + 1e-9 * (double)pt.time_from_start.nanosec;
      const double dt = (i==0) ? 0.0 : max(0.0, t - last_t);
      last_t = t;
      const unsigned int step = steps_from_dt(dt, 0.001);

      // Debug: print index, t, dt, step and left/right joint values
      {
        ostringstream ss;
        ss << "pt[" << i << "]: t=" << fixed << setprecision(4) << t
           << " dt=" << dt << " step=" << step << "\n  L(rad)=[";
        for (int k=0;k<7;k++) { ss << setprecision(4) << jl.jVal[k]; if(k<6) ss << ", "; }
        ss << "]\n  L(deg)=[";
        for (int k=0; k<7; k++) { ss << setprecision(2) << (jl.jVal[k] * 180.0 / M_PI); if (k<6) ss << ", "; }
        ss << "]\n  R(rad)=[";
        for (int k=0;k<7;k++) { ss << setprecision(4) << jr.jVal[k]; if(k<6) ss << ", "; }
        ss << "]\n  R(deg)=[";
        for (int k=0; k<7; k++) { ss << setprecision(2) << (jr.jVal[k] * 180.0 / M_PI); if (k<6) ss << ", "; }
        ss << "]";
        RCLCPP_DEBUG(node->get_logger(), "%s", ss.str().c_str());
      }

      // Send LEFT then RIGHT in the same callback tick (minimize skew)
      int retL = robot.edg_servo_j(0u, &jl, MoveMode::ABS, step);
      int retR = robot.edg_servo_j(1u, &jr, MoveMode::ABS, step);
      if (retL != 0 || retR != 0) {
        RCLCPP_ERROR(node->get_logger(), "edg_servo_j failed: L=%s R=%s",
                    mapErr[retL].c_str(), mapErr[retR].c_str());
        robot.servo_move_enable(FALSE, 1, 0);
        robot.servo_move_enable(FALSE, 1, 1);
        gh->abort(make_shared<Follow::Result>());
        return;
      }
      if (retL == 0 && retR == 0) {
        RCLCPP_DEBUG(node->get_logger(), "edg_servo_j SUCCEEED!");
      }

      // Flush queued EDG commands to the controller
      int send_ret = robot.edg_send();
      if (send_ret != 0) {
        RCLCPP_WARN(node->get_logger(), "edg_send failed %d", send_ret);
      } else {
        RCLCPP_DEBUG(node->get_logger(), "edg_send OK (pt %zu)", i);
      }

      next_time += chrono::milliseconds(1);
      this_thread::sleep_until(next_time);
    }

    JointValue jl_target{}, jr_target{};
    {
      const auto &last_pt = traj.points.back();
      for (int j = 0; j < 7; ++j) {
        jl_target.jVal[j] = last_pt.positions[ mapDual.L[j] ];
        jr_target.jVal[j] = last_pt.positions[ mapDual.R[j] ];
      }
    }

    // compute deadline from last point time + margin 
    const double traj_end_time = (double)traj.points.back().time_from_start.sec
                              + 1e-9 * (double)traj.points.back().time_from_start.nanosec;
    const int extra_ms_margin = 2000; // 2s margin, tune as needed
    auto deadline = chrono::steady_clock::now()
                  + chrono::milliseconds((int)(traj_end_time * 1000.0) + extra_ms_margin);

    // tolerance for “close enough” (radians). ~0.01 rad ≈ 0.57 degrees
    const double tol_rad = 0.01;

    RCLCPP_INFO(node->get_logger(),
                "Waiting for motion to reach final point (end_time=%.3f s, tol=%.4f rad).",
                traj_end_time, tol_rad);

    int inerr[2] = {0,0};
    BOOL in_col = FALSE;
    JointValue fbL{}, fbR{};

    while (rclcpp::ok()) {
      // let EDG recv update internal state
      // robot.edg_recv();

      // check for protective stop / errors first
      robot.is_in_collision(&in_col);
      robot.robot_is_in_error(inerr);
      if (in_col || inerr[0] || inerr[1]) {
        RCLCPP_WARN(node->get_logger(), "Abort: collision=%d errL=%d errR=%d", in_col, inerr[0], inerr[1]);
        robot.servo_move_enable(FALSE, 1, 0);
        robot.servo_move_enable(FALSE, 1, 1);
        gh->abort(make_shared<Follow::Result>()); 
        return;
      }

      const bool okL = (robot.edg_get_stat(0, &fbL, nullptr) == 0);
      const bool okR = (robot.edg_get_stat(1, &fbR, nullptr) == 0);

      // compute absolute max errors
      const double errL = okL ? max_abs_err(fbL, jl_target) : 1e6;
      const double errR = okR ? max_abs_err(fbR, jr_target) : 1e6;

      // log full joint feedback in degrees (all 7 joints per arm)
      {
        ostringstream ss;
        ss << fixed << setprecision(2);
        ss << "Feedback err (rad): L=" << errL << " R=" << errR << " | L(deg)=[";
        if (okL) {
          for (int jj = 0; jj < 7; ++jj) {
            ss << (fbL.jVal[jj] * 180.0 / M_PI);
            if (jj < 6) ss << ", ";
          }
        } else {
          ss << "NaN, NaN, NaN, NaN, NaN, NaN, NaN";
        }
        ss << "] R(deg)=[";
        if (okR) {
          for (int jj = 0; jj < 7; ++jj) {
            ss << (fbR.jVal[jj] * 180.0 / M_PI);
            if (jj < 6) ss << ", ";
          }
        } else {
          ss << "NaN, NaN, NaN, NaN, NaN, NaN, NaN";
        }
        ss << "]";

        RCLCPP_INFO(node->get_logger(), "%s", ss.str().c_str());
      }

      const bool L_ok = okL && errL <= tol_rad;
      const bool R_ok = okR && errR <= tol_rad;
      bool reached = L_ok && R_ok;

      if (reached) {
        RCLCPP_INFO(node->get_logger(), "==============Motion reached the target position (within tol)==============");
        break;
      }

      // Timeout
      if (chrono::steady_clock::now() >= deadline) {
        robot.servo_move_enable(FALSE, 1, 0);
        robot.servo_move_enable(FALSE, 1, 1);
        RCLCPP_WARN(node->get_logger(), "Timeout waiting for the target position (errL=%.4f errR=%.4f)", errL, errR);
        gh->abort(make_shared<Follow::Result>());
        return;
      }

      RCLCPP_INFO(node->get_logger(), 
              "Whether the robot has reached the target position: %d", (reached));

      rclcpp::sleep_for(chrono::milliseconds(50));
    }

    // shutdown servo mode for both arms
    robot.servo_move_enable(FALSE, 1, 0);
    robot.servo_move_enable(FALSE, 1, 1);

    // final check for errors to decide success/abort
    (inerr[0]||inerr[1]) ? gh->abort(make_shared<Follow::Result>())
                              : gh->succeed(make_shared<Follow::Result>());
    return;

  }

  // Single-arm (left-arm only or right-arm only)
  else {
    const unsigned char arm = (kind==0)? 0u : 1u;
    const unsigned char other = (arm==0) ? 1u : 0u;

    // lock_guard<mutex> lk(g_arm_mtx[arm]);
    lock(g_arm_mtx[0], g_arm_mtx[1]);
    unique_lock<mutex> lk0(g_arm_mtx[0], adopt_lock);
    unique_lock<mutex> lk1(g_arm_mtx[1], adopt_lock);

    robot.servo_move_use_none_filter();
    sched_param sch;
    sch.sched_priority = 90;
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &sch);

    // Enable EDG servo for both arms (to send the *other* arm a hold command)
    if (robot.servo_move_enable(TRUE, 1, 0) != 0 || robot.servo_move_enable(TRUE, 1, 1) != 0) {
      RCLCPP_ERROR(node->get_logger(), "servo_move_enable failed");
      gh->abort(make_shared<Follow::Result>());
      return;
    }

    // Debug: how many points and mapping indices
    RCLCPP_INFO(node->get_logger(), "Received trajectory with %zu points (single-arm, 7 joints).",
                traj.points.size());
    // Print joint_names mapping
    string map_s = "Mapping: ";
    for (int j=0;j<7;j++) {
      map_s += "Arm" + to_string(arm) + " - j"+ to_string(j+1) + "=" + to_string(mapSingle[j]) + " ";
    }
    RCLCPP_DEBUG(node->get_logger(), "%s", map_s.c_str());

    // Read current joints of the OTHER arm so we can keep it at its current position
    JointValue other_hold{};
    if (robot.edg_get_stat(other, &other_hold, nullptr) != 0) {
      RCLCPP_WARN(node->get_logger(), "edg_get_stat failed when reading other arm; aborting");
      // disable servos we enabled
      robot.servo_move_enable(FALSE, 1, 0);
      robot.servo_move_enable(FALSE, 1, 1);
      gh->abort(make_shared<Follow::Result>());
      return;
    }
    RCLCPP_INFO(node->get_logger(), "Other arm (%u) read for hold: j1=%.2f°, j2=%.2f°, j3=%.2f°, j4=%.2f°, j5=%.2f°, j6=%.2f°, j7=%.2f°,",
                other, other_hold.jVal[0] * 180.0 / M_PI, other_hold.jVal[1] * 180.0 / M_PI, 
              other_hold.jVal[2] * 180.0 / M_PI, other_hold.jVal[3] * 180.0 / M_PI, 
            other_hold.jVal[4] * 180.0 / M_PI, other_hold.jVal[5] * 180.0 / M_PI, other_hold.jVal[6] * 180.0 / M_PI);


    // Execute all points
    double last_t = 0.0;
    auto next_time = chrono::steady_clock::now();
    for (size_t i=0; i<traj.points.size(); ++i)
    {
      // Allow cancel
      if (gh->is_canceling()) {
        robot.motion_abort();
        robot.servo_move_enable(FALSE, 1, 0);
        robot.servo_move_enable(FALSE, 1, 1);
        gh->canceled(make_shared<Follow::Result>());
        return;
      }

      const auto& pt = traj.points[i];
      if (pt.positions.size() != 7) {
        RCLCPP_ERROR(node->get_logger(), "Point %zu has %zu positions (need 7)", i, pt.positions.size());
        robot.servo_move_enable(FALSE, 1, 0);
        robot.servo_move_enable(FALSE, 1, 1);
        gh->abort(make_shared<Follow::Result>());
        return;
      } 

      JointValue j{}; 
      for (int k=0;k<7;k++) j.jVal[k] = pt.positions[ mapSingle[k] ];
      const double t = (double)pt.time_from_start.sec + 1e-9*(double)pt.time_from_start.nanosec;
      const double dt = (i==0) ? 0.0 : max(0.0, t - last_t);
      last_t = t;
      const unsigned int step = steps_from_dt(dt, 0.001);

      // Debug print: rad + deg for this point
      {
        ostringstream ss;
        ss << "pt[" << i << "]: t=" << fixed << setprecision(4) << t
           << " dt=" << dt << " step=" << step << "\n  ARM" << (int)arm << "(rad)=[";
        for (int kk=0; kk<7; ++kk) { ss << setprecision(4) << j.jVal[kk]; if (kk<6) ss << ", "; }
        ss << "]\n  ARM" << (int)arm << "(deg)=[";
        for (int kk=0; kk<7; ++kk) { ss << setprecision(2) << (j.jVal[kk] * 180.0 / M_PI); if (kk<6) ss << ", "; }
        ss << "]";
        RCLCPP_DEBUG(node->get_logger(), "%s", ss.str().c_str());
      }

      // ensure we always call edg_servo_j(0, ...) then edg_servo_j(1, ...)
      int retArm; int retOther;
      if (arm == 1) {
        retOther = robot.edg_servo_j(0, &other_hold, MoveMode::ABS, step);
        retArm = robot.edg_servo_j(1, &j, MoveMode::ABS, step);
      } else {
        retArm = robot.edg_servo_j(0, &j, MoveMode::ABS, step);
        retOther = robot.edg_servo_j(1, &other_hold, MoveMode::ABS, step);
      }
      if (retArm!=0 || retOther!=0) { 
        RCLCPP_ERROR(node->get_logger(), "edg_servo_j failed: arm=%s other=%s",
                    mapErr[retArm].c_str(), mapErr[retOther].c_str());
        robot.servo_move_enable(FALSE, 1, 0); 
        robot.servo_move_enable(FALSE, 1, 1);
        gh->abort(make_shared<Follow::Result>()); 
        return; 
      } 

      // Flush queued EDG commands to the controller
      int send_ret = robot.edg_send();
      if (send_ret != 0) {
        RCLCPP_WARN(node->get_logger(), "edg_send failed %d", send_ret);
      } else {
        RCLCPP_DEBUG(node->get_logger(), "edg_send OK (pt %zu)", i);
      }

      // keep cadence close to servo period (1 ms) to avoid flooding
      next_time += chrono::milliseconds(1);
      this_thread::sleep_until(next_time);
    }

    // Build target joint sets: chosen arm from last point; other arm = other_hold (we kept it)
    JointValue j_target{}, other_target{};
    {
      const auto &last_pt = traj.points.back();
      for (int j = 0; j < 7; ++j) j_target.jVal[j] = last_pt.positions[ mapSingle[j] ];
      other_target = other_hold;
    }   
    
    // compute deadline from last point time + margin
    const double traj_end_time = (double)traj.points.back().time_from_start.sec
                              + 1e-9 * (double)traj.points.back().time_from_start.nanosec;
    const int extra_ms_margin = 2000; // 2s margin
    auto deadline = chrono::steady_clock::now()
                  + chrono::milliseconds((int)(traj_end_time * 1000.0) + extra_ms_margin);

    // tolerance for “close enough” (radians). ~0.01 rad ≈ 0.57 degrees
    const double tol_rad = 0.01;

    RCLCPP_INFO(node->get_logger(),
                "Waiting for arm %u to reach final point (end_time=%.3f s, tol=%.4f rad).",
                arm, traj_end_time, tol_rad);


    int inerr[2] = {0,0};
    BOOL in_col = FALSE;
    JointValue fbArm{}, fbOther{};
    while (rclcpp::ok()) {
      // refresh EDG internal state
      robot.edg_recv();

      // check protective stop / errors
      robot.is_in_collision(&in_col);
      robot.robot_is_in_error(inerr);
      if (in_col || inerr[arm] || inerr[other]) {
        RCLCPP_WARN(node->get_logger(), "Abort: collision=%d errArm=%d errOther=%d", in_col, inerr[arm], inerr[other]);
        robot.servo_move_enable(FALSE, 1, 0);
        robot.servo_move_enable(FALSE, 1, 1);
        gh->abort(make_shared<Follow::Result>());
        return;
      }

      const bool okA = (robot.edg_get_stat(arm, &fbArm, nullptr) == 0);
      const bool okO = (robot.edg_get_stat(other, &fbOther, nullptr) == 0);

      // compute absolute max error (across all joints) for the requested arm
      const double errArm = okA ? max_abs_err(fbArm, j_target) : 1e6;
      const double errOther = okO ? max_abs_err(fbOther, other_target) : 1e6;

      // log full joint feedback in degrees (all 7 joints per arm) -- concise format
      {
        ostringstream ss;
        ss << fixed << setprecision(2);
        ss << "Err(rad): arm=" << errArm << " other=" << errOther << " | ARM" << (int)arm << "(deg)=[";
        if (okA) {
          for (int jj = 0; jj < 7; ++jj) { ss << (fbArm.jVal[jj] * 180.0 / M_PI); if (jj<6) ss << ", "; }
        } else ss << "NaN, NaN, NaN, NaN, NaN, NaN, NaN";
        ss << "] ARM" << (int)other << "(deg)=[";
        if (okO) {
          for (int jj = 0; jj < 7; ++jj) { ss << (fbOther.jVal[jj] * 180.0 / M_PI); if (jj<6) ss << ", "; }
        } else ss << "NaN, NaN, NaN, NaN, NaN, NaN, NaN";
        ss << "]";
        RCLCPP_INFO(node->get_logger(), "%s", ss.str().c_str());
      }

      // decide success: only require the requested arm to be within tolerance
      const bool reached = (okA && errArm <= tol_rad);
      if (reached) {
        RCLCPP_INFO(node->get_logger(), "==============Motion reached the target position for arm %u (within tol)==============", arm);
        break;
      }

      // Timeout
      if (chrono::steady_clock::now() >= deadline) {
        robot.servo_move_enable(FALSE, 1, 0);
        robot.servo_move_enable(FALSE, 1, 1);
        RCLCPP_WARN(node->get_logger(), "Timeout waiting for the target position (errArm=%.4f)", errArm);
        gh->abort(make_shared<Follow::Result>());
        return;
      }

      RCLCPP_INFO(node->get_logger(), 
              "Whether the robot has reached the target position: %d", (reached));

      rclcpp::sleep_for(chrono::milliseconds(100));
    }

    // shutdown servo mode for both arms
    robot.servo_move_enable(FALSE, 1, 0);
    robot.servo_move_enable(FALSE, 1, 1);

    // final check for errors to decide success/abort
    (inerr[0]||inerr[1]) ? gh->abort(make_shared<Follow::Result>())
                        : gh->succeed(make_shared<Follow::Result>());
    return;
  }
}


/* ---------------- joint states: publish both arms in one message ---------------- */
void joint_states_callback(rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr &joint_states_pub)
{
    sensor_msgs::msg::JointState joint_msg;
    JointValue jl{}, jr{};
    const bool okL = robot.edg_get_stat(0, &jl, nullptr) == 0;
    const bool okR = robot.edg_get_stat(1, &jr, nullptr) == 0;  

    joint_msg.name.reserve(14);
    joint_msg.position.reserve(14);
    for (int i=0;i<7;i++) {
        joint_msg.position.push_back(okL ? jl.jVal[i] : numeric_limits<double>::quiet_NaN());
        joint_msg.name.push_back("arm_lj" + to_string(i+1));
    }
    for (int i=0;i<7;i++) {
        joint_msg.position.push_back(okR ? jr.jVal[i] : numeric_limits<double>::quiet_NaN());
        joint_msg.name.push_back("arm_rj" + to_string(i+1));
    }
    joint_msg.header.stamp = rclcpp::Clock().now();
    joint_states_pub->publish(joint_msg);
}

void sigintHandler(int /*sig*/) {
    rclcpp::shutdown();
}

/* -------------------- main -------------------- */
int main(int argc, char** argv)
{
    setlocale(LC_ALL, "");
    rclcpp::init(argc, argv);
    signal(SIGINT, sigintHandler);
    auto node = rclcpp::Node::make_shared("kargo_moveit_server");

    // Read parameters
    string default_ip = "127.0.0.1";
    string robot_ip = node->declare_parameter("ip", default_ip);

    edg_init_ip = make_edg_bcast(robot_ip);
    RCLCPP_INFO(node->get_logger(), "EDG init IP set to: %s", edg_init_ip.c_str());

    // Connect to robot
    robot.login_in(robot_ip.c_str());
    rclcpp::Rate rate(125);

    // Make sure both arms are out of servo mode at startup
    robot.servo_move_enable(FALSE, 1, 0);
    robot.servo_move_enable(FALSE, 1, 1);
    rclcpp::sleep_for(chrono::milliseconds(500));

    // Filter param
    robot.servo_move_use_joint_LPF(0.5, -1);

    // Power on + enable
    robot.power_on();
    rclcpp::sleep_for(chrono::seconds(8));
    robot.enable_robot();
    rclcpp::sleep_for(chrono::seconds(4));

    // Global call for edg_init
    robot.edg_init(1, edg_init_ip.c_str());
    
    // Publishers
    joint_states_pub = node->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

    // Dual-arm controller action server (14 joints)
    auto dual_server = rclcpp_action::create_server<Follow>(
        node,
        "/jaka_kargo_full_robot_controller/follow_joint_trajectory",
        // goal callback
        [](const rclcpp_action::GoalUUID&,
        shared_ptr<const Follow::Goal>) {
            RCLCPP_INFO(rclcpp::get_logger("kargo_moveit_server"), "Received full_robot goal request");
            // (void)goal; // Avoid unused parameter warning
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        // cancel callback
        [](shared_ptr<GoalHandle>) {
            RCLCPP_INFO(rclcpp::get_logger("kargo_moveit_server"), "Received full_robot cancel request");
            return rclcpp_action::CancelResponse::ACCEPT;
        },
        // execute callback
        [node](shared_ptr<GoalHandle> gh) {
            RCLCPP_INFO(rclcpp::get_logger("kargo_moveit_server"), "Executing full_robot goal");
            execute_goal(gh, node, -1);
        }
    );

    // Left-arm only controller action server (7 joints)
    auto left_server = rclcpp_action::create_server<Follow>(
        node,
        "/jaka_kargo_arm_l_controller/follow_joint_trajectory",
        // goal callback
        [](const rclcpp_action::GoalUUID&,
        shared_ptr<const Follow::Goal>) {
            RCLCPP_INFO(rclcpp::get_logger("kargo_moveit_server"), "Received arm_l goal request");
            // (void)goal; // Avoid unused parameter warning
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        // cancel callback
        [](shared_ptr<GoalHandle>) {
            RCLCPP_INFO(rclcpp::get_logger("kargo_moveit_server"), "Received arm_l cancel request");
            return rclcpp_action::CancelResponse::ACCEPT;
        },
        // execute callback
        [node](shared_ptr<GoalHandle> gh) {
            RCLCPP_INFO(rclcpp::get_logger("kargo_moveit_server"), "Executing arm_l goal");
            execute_goal(gh, node, 0);
        }
    );

    // Right-arm only controller action server (7 joints)
    auto right_server = rclcpp_action::create_server<Follow>(
        node,
        "/jaka_kargo_arm_r_controller/follow_joint_trajectory",
        // goal callback
        [](const rclcpp_action::GoalUUID&,
        shared_ptr<const Follow::Goal>) {
            RCLCPP_INFO(rclcpp::get_logger("kargo_moveit_server"), "Received arm_r goal request");
            // (void)goal; // Avoid unused parameter warning
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        // cancel callback
        [](shared_ptr<GoalHandle>) {
            RCLCPP_INFO(rclcpp::get_logger("kargo_moveit_server"), "Received arm_r cancel request");
            return rclcpp_action::CancelResponse::ACCEPT;
        },
        // execute callback
        [node](shared_ptr<GoalHandle> gh) {
            RCLCPP_INFO(rclcpp::get_logger("kargo_moveit_server"), "Executing arm_r goal");
            execute_goal(gh, node, 1);
        }
    );

    RCLCPP_INFO(rclcpp::get_logger("kargo_moveit_server"), "==================Kargo Moveit Start==================");

    while (rclcpp::ok()) {
        joint_states_callback(joint_states_pub);   // LEFT+RIGHT in one message
        rate.sleep();
        rclcpp::spin_some(node);
    }
    rclcpp::shutdown();
    return 0;
}
