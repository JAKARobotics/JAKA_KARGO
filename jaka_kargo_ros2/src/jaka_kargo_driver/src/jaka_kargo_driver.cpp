#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "jaka_kargo_msgs/msg/robot_states.hpp"
#include "jaka_kargo_msgs/msg/di_info.hpp"
#include "jaka_kargo_msgs/msg/multi_move_unit.hpp"
#include "jaka_kargo_msgs/srv/move.hpp"  
#include "jaka_kargo_msgs/srv/servo_move_enable.hpp"  
#include "jaka_kargo_msgs/srv/servo_move.hpp"
#include "jaka_kargo_msgs/srv/get_default_base.hpp" 
#include "jaka_kargo_msgs/srv/set_tool_offset.hpp" 
#include "jaka_kargo_msgs/srv/get_tool_offset.hpp" 
#include "jaka_kargo_msgs/srv/set_tool_payload.hpp" 
#include "jaka_kargo_msgs/srv/get_tool_payload.hpp" 
#include "jaka_kargo_msgs/srv/drag_mode.hpp" 
#include "jaka_kargo_msgs/srv/set_collision_level.hpp" 
#include "jaka_kargo_msgs/srv/get_collision_level.hpp" 
#include "jaka_kargo_msgs/srv/get_fk.hpp" 
#include "jaka_kargo_msgs/srv/get_ik.hpp" 
#include "jaka_kargo_msgs/srv/clear_error.hpp"
#include "jaka_kargo_msgs/srv/get_sdk_version.hpp"
#include "jaka_kargo_msgs/srv/set_debug_mode.hpp"   
#include "jaka_kargo_msgs/srv/get_dh_params.hpp"
#include "jaka_kargo_msgs/srv/ext_enable.hpp"
#include "jaka_kargo_msgs/srv/jog_ext.hpp"
#include "jaka_kargo_msgs/srv/multi_move_ext.hpp"
// #include "jaka_kargo_msgs/srv/get_ext_status.hpp"

#include "jaka_kargo_driver/JAKAZuRobot.h"
#include "jaka_kargo_driver/jkerr.h"
#include "jaka_kargo_driver/jktypes.h"

#include <action_msgs/msg/goal_status_array.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <string>
#include <map>
#include <chrono>
#include <thread>
#include <memory>
#include <array>
using namespace std;

const double PI = 3.1415926;

JAKAZuRobot robot;
//SDK interface return status
map<int, string>mapErr = {
    {2,"ERR_FUCTION_CALL_ERROR"},
    {-1,"ERR_INVALID_HANDLER"},
    {-2,"ERR_INVALID_PARAMETER"},
    {-3,"ERR_COMMUNICATION_ERR"},
    {-4,"ERR_KINE_INVERSE_ERR"},
    {-5,"ERR_EMERGENCY_PRESSED"},
    {-6,"ERR_NOT_POWERED"},
    {-7,"ERR_NOT_ENABLED"},
    {-8,"ERR_DISABLE_SERVOMODE"},
    {-9,"ERR_NOT_OFF_ENABLE"},
    {-10,"ERR_PROGRAM_IS_RUNNING"},
    {-11,"ERR_CANNOT_OPEN_FILE"},
    {-12,"ERR_MOTION_ABNORMAL"}
};


// Declare publishers
rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr tool_position_pub;
rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_position_pub;
rclcpp::Publisher<jaka_kargo_msgs::msg::RobotStates>::SharedPtr robot_states_pub;

// Global EDG broadcast IP used by edg_init calls
static std::string edg_init_ip = "255.255.255.255";

/// Make EDG broadcast IP by replacing last IPv4 octet with 255.
/// If input doesn't contain a '.' (not IPv4), returns input unchanged.
static std::string make_edg_bcast(const std::string &ip)
{
    auto last_oct = ip.find_last_of('.');
    if (last_oct == std::string::npos) {
        // not an IPv4-like string — return unchanged (fallback)
        return ip;
    }
    return ip.substr(0, last_oct + 1) + "255";
}


bool linear_move_callback(
    const shared_ptr<jaka_kargo_msgs::srv::Move::Request> req,
    shared_ptr<jaka_kargo_msgs::srv::Move::Response> res)
{
    // Validate robot index
    if (req->robot_index != -1 && req->robot_index != 0 && req->robot_index != 1) {
        res->ret = -1;
        res->message = "robot_index must be -1 (DUAL), 0 (LEFT), or 1 (RIGHT)";
        return false;
    }

    // Expect [x,y,z,rx,ry,rz]
    if (req->pose_left.size() != 6 || req->pose_right.size() != 6) {
        res->ret = -1;
        res->message = "pose_left and pose_right must both have 6 elements [x,y,z,rx,ry,rz].";
        return false;
    }

    // Validate modes
    if ((req->mode_left  != 0 && req->mode_left  != 1) ||
        (req->mode_right != 0 && req->mode_right != 1)) {
        res->ret = -1;
        res->message = "mode_left/mode_right must be 0 (ABS) or 1 (INCR)";
        return false;
    }

    // Ensure EDG servo mode is OFF on both arms (prevents preemption)
    robot.servo_move_enable(FALSE, 1, 0);
    robot.servo_move_enable(FALSE, 1, 1);
    this_thread::sleep_for(chrono::milliseconds(50));

    // Build CartesianPose targets
    CartesianPose goals[2];

    // LEFT pose [x,y,z,rx,ry,rz] (positions in millimeters, angles in radians)
    goals[0].tran.x = static_cast<double>(req->pose_left[0]);
    goals[0].tran.y = static_cast<double>(req->pose_left[1]);
    goals[0].tran.z = static_cast<double>(req->pose_left[2]);
    goals[0].rpy.rx = static_cast<double>(req->pose_left[3]);
    goals[0].rpy.ry = static_cast<double>(req->pose_left[4]);
    goals[0].rpy.rz = static_cast<double>(req->pose_left[5]);

    // RIGHT pose [x,y,z,rx,ry,rz] (positions in millimeters, angles in radians)
    goals[1].tran.x = static_cast<double>(req->pose_right[0]);
    goals[1].tran.y = static_cast<double>(req->pose_right[1]);
    goals[1].tran.z = static_cast<double>(req->pose_right[2]);
    goals[1].rpy.rx = static_cast<double>(req->pose_right[3]);
    goals[1].rpy.ry = static_cast<double>(req->pose_right[4]);
    goals[1].rpy.rz = static_cast<double>(req->pose_right[5]);

    // Fill unused arm from feedback if user requested single-arm (0 or 1)
    if (req->robot_index == 0) {
        // LEFT-only requested → get RIGHT current TCP and copy into goals[1]
        CartesianPose cur_right{};
        const int rret = robot.edg_get_stat(1u, nullptr, &cur_right);
        if (rret != 0) {
            RCLCPP_WARN(rclcpp::get_logger("linear_move_callback"), "edg_get_stat(1) failed (%d). Cannot fill RIGHT; aborting.", rret);
            res->ret = 0;
            res->message = "edg_get_stat failed when filling unused arm";
            return false;
        }
        // convert rpy from degrees -> radians 
        cur_right.rpy.rx *= M_PI / 180.0;
        cur_right.rpy.ry *= M_PI / 180.0;
        cur_right.rpy.rz *= M_PI / 180.0;
        goals[1] = cur_right;
        RCLCPP_INFO(rclcpp::get_logger("linear_move_callback"), "Filled unused RIGHT goal from feedback.");
    } else if (req->robot_index == 1) {
        // RIGHT-only requested → get LEFT current TCP and copy into goals[0]
        CartesianPose cur_left{};
        const int rret = robot.edg_get_stat(0u, nullptr, &cur_left);
        if (rret != 0) {
            RCLCPP_WARN(rclcpp::get_logger("linear_move_callback"), "edg_get_stat(0) failed (%d). Cannot fill LEFT; aborting.", rret);
            res->ret = 0;
            res->message = "edg_get_stat failed when filling unused arm";
            return false;
        }
        // convert rpy from degrees -> radians
        cur_left.rpy.rx *= M_PI / 180.0;
        cur_left.rpy.ry *= M_PI / 180.0;
        cur_left.rpy.rz *= M_PI / 180.0;        
        goals[0] = cur_left;
        RCLCPP_INFO(rclcpp::get_logger("linear_move_callback"), "Filled unused LEFT goal from feedback.");
    }

    // Per-arm move modes (0=ABS, 1=INCR)
    MoveMode modes[2];
    modes[0] = (req->mode_left  == 1) ? MoveMode::INCR : MoveMode::ABS;
    modes[1] = (req->mode_right == 1) ? MoveMode::INCR : MoveMode::ABS;

    // Per-arm speeds/accels
    double vels[2] = {
        static_cast<double>(req->vel_left),
        static_cast<double>(req->vel_right)
    };
    double accs[2] = {
        static_cast<double>(req->acc_left),
        static_cast<double>(req->acc_right)
    };

    // Blocking flag
    const BOOL is_block = req->is_block ? TRUE : FALSE;

    // Log exactly what we will send (positions mm and RPY in rad/deg)
    auto pose_to_string = [&](const CartesianPose &p) {
        ostringstream s;
        s << fixed << setprecision(3);
        s << "tran(mm)=[" << p.tran.x << ", " << p.tran.y << ", " << p.tran.z << "] ";
        s << "rpy(rad)=[" << setprecision(4) << p.rpy.rx << ", " << p.rpy.ry << ", " << p.rpy.rz << "] ";
        s << setprecision(2) << "rpy(deg)=[" << (p.rpy.rx * 180.0 / M_PI) << ", " << (p.rpy.ry * 180.0 / M_PI) << ", " << (p.rpy.rz * 180.0 / M_PI) << "]";
        return s.str();
    };

    RCLCPP_INFO(rclcpp::get_logger("linear_move_callback"), "robot_index=%d is_block=%d modes(L,R)=(%d,%d) vel(L,R)=(%.3f,%.3f) acc(L,R)=(%.3f,%.3f)",
                req->robot_index, req->is_block ? 1 : 0, (int)modes[0], (int)modes[1],
                vels[0], vels[1], accs[0], accs[1]);

    RCLCPP_INFO(rclcpp::get_logger("linear_move_callback"), "LEFT  goal:  %s", pose_to_string(goals[0]).c_str());
    RCLCPP_INFO(rclcpp::get_logger("linear_move_callback"), "RIGHT goal:  %s", pose_to_string(goals[1]).c_str());

    // Call dual-arm SDK   
    const int ret = robot.robot_run_multi_movl(req->robot_index, modes, is_block, goals, vels, accs);
    RCLCPP_INFO(rclcpp::get_logger("linear_move_callback"), "robot_run_multi_movl returned %d (%s)", ret, (ret==0?"OK":"ERR"));

    if (ret == 0) {
        res->ret = 1;
        res->message = "robot_run_multi_movl executed";
        return true;
    } else {
        res->ret = 0;
        res->message = string("error occurred: ") + mapErr[ret]; 
        return false;
    }
}


bool joint_move_callback(
  const shared_ptr<jaka_kargo_msgs::srv::Move::Request> req,
  shared_ptr<jaka_kargo_msgs::srv::Move::Response> res)
{
    // Validate robot index
    if (req->robot_index != -1 && req->robot_index != 0 && req->robot_index != 1) {
        res->ret = -1;
        res->message = "robot_index must be -1 (DUAL), 0 (LEFT), or 1 (RIGHT)";
        return false;
    }

    // Expect [x,y,z,rx,ry,rz]
    if (req->pose_left.size() != 7 || req->pose_right.size() != 7) {
        res->ret = -1;
        res->message = "pose_left and pose_right must both have 7 joint values [j0, ..., j6].";
        return false;
    }

    // Validate modes
    if ((req->mode_left  != 0 && req->mode_left  != 1) ||
        (req->mode_right != 0 && req->mode_right != 1)) {
        res->ret = -1;
        res->message = "mode_left/mode_right must be 0 (ABS) or 1 (INCR)";
        return false;
    }

    // Ensure EDG servo mode is OFF on both arms (prevents preemption)
    robot.servo_move_enable(FALSE, 1, 0);
    robot.servo_move_enable(FALSE, 1, 1);
    this_thread::sleep_for(chrono::milliseconds(50));

    // Build per-arm JointValue targets 
    JointValue joints[2];
    for (int i = 0; i < 7; ++i) {
        joints[0].jVal[i] = static_cast<double>(req->pose_left[i]);
        joints[1].jVal[i] = static_cast<double>(req->pose_right[i]);
    }

    // Fill the *unused* arm with its current joints
    if (req->robot_index == 0) {             // LEFT only: copy RIGHT from feedback
        JointValue jr{}; robot.edg_get_stat(1, &jr, nullptr);
        for (int i = 0; i < 7; ++i) joints[1].jVal[i] = jr.jVal[i];
    } else if (req->robot_index == 1) {      // RIGHT only: copy LEFT from feedback
        JointValue jl{}; robot.edg_get_stat(0, &jl, nullptr);
        for (int i = 0; i < 7; ++i) joints[0].jVal[i] = jl.jVal[i];
    }

    // Build per-arm MoveMode array (0=ABS, 1=INCR)
    MoveMode modes[2];
    modes[0] = (req->mode_left  == 1) ? MoveMode::INCR : MoveMode::ABS;
    modes[1] = (req->mode_right == 1) ? MoveMode::INCR : MoveMode::ABS;

    // Per-arm joint speeds/accels (rad/s, rad/s^2) 
    double vels[2] = {
        static_cast<double>(req->vel_left),
        static_cast<double>(req->vel_right)
    };
    double accs[2] = {
        static_cast<double>(req->acc_left),
        static_cast<double>(req->acc_right)
    };

    // Blocking flag
    BOOL is_block = req->is_block ? TRUE : FALSE;

    // Log exactly what we’ll send (inline string building)
    ostringstream Lrad, Ldeg, Rrad, Rdeg;
    Lrad<<fixed<<setprecision(4)<<"["; Ldeg<<fixed<<setprecision(1)<<"[";
    for (int i=0;i<7;i++){ if(i){Lrad<<", "; Ldeg<<", ";} Lrad<<joints[0].jVal[i]; Ldeg<<(joints[0].jVal[i]*180.0/M_PI); }
    Lrad<<"]"; Ldeg<<"]";
    Rrad<<fixed<<setprecision(4)<<"["; Rdeg<<fixed<<setprecision(1)<<"[";
    for (int i=0;i<7;i++){ if(i){Rrad<<", "; Rdeg<<", ";} Rrad<<joints[1].jVal[i]; Rdeg<<(joints[1].jVal[i]*180.0/M_PI); }
    Rrad<<"]"; Rdeg<<"]";
    RCLCPP_INFO(rclcpp::get_logger("joint_move_callback"), "robot_index=%d is_block=%d modes(L,R)=(%d,%d) vel(L,R)=(%.3f,%.3f) acc(L,R)=(%.3f,%.3f)",
                req->robot_index, (int)is_block, (int)modes[0], (int)modes[1], vels[0], vels[1], accs[0], accs[1]);
    RCLCPP_INFO(rclcpp::get_logger("joint_move_callback"), "LEFT  cmd (rad): %s", Lrad.str().c_str());
    RCLCPP_INFO(rclcpp::get_logger("joint_move_callback"), "LEFT  cmd (deg): %s", Ldeg.str().c_str());
    RCLCPP_INFO(rclcpp::get_logger("joint_move_callback"), "RIGHT cmd (rad): %s", Rrad.str().c_str());
    RCLCPP_INFO(rclcpp::get_logger("joint_move_callback"), "RIGHT cmd (deg): %s", Rdeg.str().c_str());

    // Call dual-arm SDK
    int ret = robot.robot_run_multi_movj(req->robot_index, modes, is_block, joints, vels, accs);
    RCLCPP_INFO(rclcpp::get_logger("joint_move_callback"), "robot_run_multi_movj returned %d (%s)", ret, (ret==0?"OK":"ERR"));

    // Map result
    if (ret == 0) {
        res->ret = 1;
        res->message = "robot_run_multi_movj executed";
        return true;
    } else {
        res->ret = 0;
        res->message = string("error occurred: ") + mapErr[ret]; 
        return false;
    }
}


bool servo_move_enable_callback(
    const shared_ptr<jaka_kargo_msgs::srv::ServoMoveEnable::Request> req,
    shared_ptr<jaka_kargo_msgs::srv::ServoMoveEnable::Response> res)
{
    // Validate robot index: -1 (both), 0 (left), 1 (right)
    const int robindex = static_cast<int>(req->robot_index);
    if (robindex != -1 && robindex != 0 && robindex != 1) {
        res->ret = -1;
        res->message = "robot_index must be -1 (DUAL), 0 (LEFT), or 1 (RIGHT)";
        return false;
    }

    const BOOL enable = req->enable ? TRUE : FALSE;
    const BOOL is_block = req->is_block ? TRUE : FALSE;

    // Call dual-arm SDK 
    const int ret = robot.servo_move_enable(enable, is_block, robindex);

    if (ret == 0) {
        res->ret = 1;
        res->message = "servo_move_enable executed";
        return true;
    } else {
        res->ret = 0;
        res->message = string("error occurred: ") + mapErr[ret];
        return false;
    }
}


bool edg_servo_p_callback(
    const shared_ptr<jaka_kargo_msgs::srv::ServoMove::Request> req,
    shared_ptr<jaka_kargo_msgs::srv::ServoMove::Response> res)
{
    RCLCPP_INFO(rclcpp::get_logger("edg_servo_p"), "Service called with robot_index=%d", req->robot_index);

    // Validate robot index for EDG (this is per-arm, not -1/DUAL)
    if (req->robot_index != 0 && req->robot_index != 1) {
        res->ret = 0;
        res->message = "robot_index must be 0 (LEFT) or 1 (RIGHT) for edg_servo_p";
        return false;
    }

    // Expect [x,y,z,rx,ry,rz]
    if (req->pose.size() != 6) {
        res->ret = 0; res->message = "pose must have 6 elements for edg_servo_p";
        return false;
    }

    // Build CartesianPose from RPY array 
    CartesianPose cp;
    cp.tran.x = static_cast<double>(req->pose[0]);
    cp.tran.y = static_cast<double>(req->pose[1]);
    cp.tran.z = static_cast<double>(req->pose[2]);
    cp.rpy.rx = static_cast<double>(req->pose[3]);  // radians
    cp.rpy.ry = static_cast<double>(req->pose[4]);  // radians
    cp.rpy.rz = static_cast<double>(req->pose[5]);  // radians

    // // Move mode (0=ABS, 1=INCR)
    // MoveMode mode = (req->move_mode == 1) ? MoveMode::INCR : MoveMode::ABS;

    // // step_num: default to 1 if 0 comes in
    // unsigned int step = (req->step_num == 0u) ? 1u
    //                                             : static_cast<unsigned int>(req->step_num);

    // Log what we're about to send (positions in mm, rpy both rad & deg)
    {
        ostringstream s;
        s << "edg_servo_p: arm=" << req->robot_index
          << " pos(mm)=[" << cp.tran.x << "," << cp.tran.y << "," << cp.tran.z << "]"
          << " rpy(rad)=[" << cp.rpy.rx << "," << cp.rpy.ry << "," << cp.rpy.rz << "]"
          << " rpy(deg)=[" << (cp.rpy.rx * 180.0 / M_PI) << "," << (cp.rpy.ry * 180.0 / M_PI) << "," << (cp.rpy.rz * 180.0 / M_PI) << "]";
        //   << " mode=" << (mode==MoveMode::ABS? "ABS":"INCR")
        //   << " step=" << step;
        RCLCPP_DEBUG(rclcpp::get_logger("edg_servo_p_callback"), "%s", s.str().c_str());
    }

    robot.servo_move_use_none_filter(0);
    robot.servo_move_use_none_filter(1);
    sched_param sch;
    sch.sched_priority = 90;
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &sch);

    // Call EDG servo_p 
    const int ret = robot.edg_servo_p(
        static_cast<unsigned char>(req->robot_index), &cp, MoveMode::INCR);
    if (ret != 0) {
        res->ret = 0;
        res->message = string("error occurred: ") + mapErr[ret];
        RCLCPP_ERROR(rclcpp::get_logger("edg_servo_p"), "Servo move failed with error: %s", mapErr[ret].c_str());
        return false;
    }

    errno_t send_ret = robot.edg_send();
    if (send_ret != 0) {
        res->ret = 0;
        res->message = string("error occurred: ") + mapErr[ret];
        RCLCPP_ERROR(rclcpp::get_logger("edg_servo_p_callback"),
                    "edg_send failed with error: %s", mapErr[send_ret].c_str());
    }

    rclcpp::sleep_for(chrono::milliseconds(8)); 

    // Success
    res->ret = 1;
    res->message = "edg_servo_p executed";
    RCLCPP_INFO(rclcpp::get_logger("edg_servo_p"), "Servo move completed successfully");
    return true;
}


bool edg_servo_j_callback(
    const shared_ptr<jaka_kargo_msgs::srv::ServoMove::Request> req,
    shared_ptr<jaka_kargo_msgs::srv::ServoMove::Response> res)
{
    // Validate per-arm EDG robot index
    if (req->robot_index != 0 && req->robot_index != 1) {
        res->ret = 0;
        res->message = "robot_index must be 0 (LEFT) or 1 (RIGHT) for edg_servo_j";
        return false;
    }

    // Expect 7 joint values [j0, ..., j6]
    if (req->pose.size() != 7) {
        res->ret = 0; res->message = "pose must have 7 joint values for edg_servo_j";
        return false;
    }

    // Build JointValue (SDK expects radians)
    JointValue jv;
    for (int i = 0; i < 7; ++i) {
        jv.jVal[i] = static_cast<double>(req->pose[i]);
    }

    // // Move mode (0=ABS, 1=INCR) 
    // MoveMode mode = (req->move_mode == 1) ? MoveMode::INCR : MoveMode::ABS;

    // // step_num guard: EDG expects >=1
    // unsigned int step = (req->step_num == 0) ? 1 : req->step_num;

    robot.servo_move_use_none_filter(0);
    robot.servo_move_use_none_filter(1);
    sched_param sch;
    sch.sched_priority = 90;
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &sch);

    // Call EDG servo_j
    // const int ret = robot.edg_servo_j(static_cast<unsigned char>(req->robot_index), &jv, mode, step);
    const int ret = robot.edg_servo_j(static_cast<unsigned char>(req->robot_index), &jv, MoveMode::INCR);
    if (ret != 0) {
        res->ret = 0;
        res->message = string("error occurred: ") + mapErr[ret];
        RCLCPP_ERROR(rclcpp::get_logger("edg_servo_j"), "Servo move failed with error: %s", mapErr[ret].c_str());
        return false;
    }
    
    errno_t send_ret = robot.edg_send();
    if (send_ret != 0) {
        res->ret = 0;
        res->message = string("error occurred: ") + mapErr[ret];
        RCLCPP_ERROR(rclcpp::get_logger("edg_servo_j_callback"),
                    "edg_send failed with error: %s", mapErr[send_ret].c_str());
    }

    rclcpp::sleep_for(chrono::milliseconds(8)); 

    // Success
    res->ret = 1;
    res->message = "edg_servo_j executed";
    RCLCPP_INFO(rclcpp::get_logger("edg_servo_j"), "Servo move completed successfully");
    return true;
}


bool stop_move_callback(
    const shared_ptr<std_srvs::srv::Empty::Request> /*req*/,
    shared_ptr<std_srvs::srv::Empty::Response> /*res*/)
{
    // Controller-wide stop (affects current motion on either/both arms)
    const int ret = robot.motion_abort();

    if (ret == 0) {
        RCLCPP_INFO(rclcpp::get_logger("stop_move_callback"), "motion_abort executed");
        return true;
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("stop_move_callback"),
                    "motion_abort failed: %s", mapErr[ret].c_str());
        return false;
    }
}


bool get_default_base_callback(
    const shared_ptr<jaka_kargo_msgs::srv::GetDefaultBase::Request> req,
    shared_ptr<jaka_kargo_msgs::srv::GetDefaultBase::Response> res)
{
    // Per-SDK: only per-arm (0/1). No -1/DUAL here.
    if (req->robot_index != 0 && req->robot_index != 1) {
        res->ret = 0;
        res->message = "robot_index must be 0 (LEFT) or 1 (RIGHT)";
        return false;
    }

    CartesianPose base{};
    const int ret = robot.robot_get_default_base(static_cast<int>(req->robot_index), &base);

    if (ret == 0) {
        res->ret = 1;
        // SDK returns x/y/z in mm, rpy in radians 
        res->base_rpy = {
            static_cast<float>(base.tran.x),
            static_cast<float>(base.tran.y),
            static_cast<float>(base.tran.z),
            static_cast<float>(base.rpy.rx),
            static_cast<float>(base.rpy.ry),
            static_cast<float>(base.rpy.rz)
        };
        res->message = "robot_get_default_base executed";
        return true;
    } else {
        res->ret = 0;
        res->base_rpy = {9999,9999,9999,9999,9999,9999};  
        res->message = string("robot_get_default_base failed: ") + mapErr[ret];
        return false;
    }
}


bool set_tool_offset_callback(
    const shared_ptr<jaka_kargo_msgs::srv::SetToolOffset::Request> req,
    shared_ptr<jaka_kargo_msgs::srv::SetToolOffset::Response> res)
{
    // Validate robot_index
    if (req->robot_index != 0 && req->robot_index != 1) {
        res->ret = -1;
        res->message = "robot_index must be 0 (LEFT), or 1 (RIGHT)";
        return false;
    }

    // Validate pose length
    if (req->pose_rpy.size() != 6) {
        res->ret = -1;
        res->message = "pose_rpy must have 6 elements: [x,y,z,rx,ry,rz]";
        return false;
    }

    // Build TCP (SDK expects RPY in radians)
    CartesianPose tcp{};
    tcp.tran.x = static_cast<double>(req->pose_rpy[0]);  // mm
    tcp.tran.y = static_cast<double>(req->pose_rpy[1]);  // mm
    tcp.tran.z = static_cast<double>(req->pose_rpy[2]);  // mm
    tcp.rpy.rx = static_cast<double>(req->pose_rpy[3]);  // rad
    tcp.rpy.ry = static_cast<double>(req->pose_rpy[4]);  // rad
    tcp.rpy.rz = static_cast<double>(req->pose_rpy[5]);  // rad

    const int ret = robot.robot_set_tool_offset(static_cast<int>(req->robot_index), tcp);

    if (ret == 0) {
        res->ret = 1;
        res->message = "robot_set_tool_offset executed";
        return true;
    } else {
        res->ret = 0;
        res->message = string("error occurred: ") + mapErr[ret];
        return false;
    }
}


bool get_tool_offset_callback(
    const shared_ptr<jaka_kargo_msgs::srv::GetToolOffset::Request> req,
    shared_ptr<jaka_kargo_msgs::srv::GetToolOffset::Response> res)
{
    // Validate robot_index
    if (req->robot_index != 0 && req->robot_index != 1) {
        res->ret = -1;
        res->message = "robot_index must be 0 (LEFT), or 1 (RIGHT)";
        return false;
    }

    CartesianPose tcp{};
    const int ret = robot.robot_get_tool_offset(static_cast<int>(req->robot_index), &tcp);

    if (ret == 0) {
        res->ret = 1;
        // SDK returns mm for x/y/z, radians for rpy
        res->pose_rpy = {
            static_cast<float>(tcp.tran.x),
            static_cast<float>(tcp.tran.y),
            static_cast<float>(tcp.tran.z),
            static_cast<float>(tcp.rpy.rx),
            static_cast<float>(tcp.rpy.ry),
            static_cast<float>(tcp.rpy.rz)
        };
        res->message = "robot_get_tool_offset executed";
        return true;
    } else {
        res->ret = 0;
        res->pose_rpy = {9999,9999,9999,9999,9999,9999}; 
        res->message = string("error occurred: ") + mapErr[ret];
        return false;
    }
}


bool set_tool_payload_callback(
    const shared_ptr<jaka_kargo_msgs::srv::SetToolPayload::Request> req,
    shared_ptr<jaka_kargo_msgs::srv::SetToolPayload::Response> res)
{
    // robot_index: only per-arm (no -1)
    if (req->robot_index != 0 && req->robot_index != 1) {
        res->ret = -1;
        res->message = "robot_index must be 0 (LEFT) or 1 (RIGHT)";
        return false;
    }
    // basic sanity
    if (!isfinite(req->mass) || req->mass < 0.0f ||
        !isfinite(req->xc)   || !isfinite(req->yc) || !isfinite(req->zc)) {
        res->ret = -1;
        res->message = "mass must be >=0 and all centroid values finite";
        return false;
    }

    PayLoad payload{};
    payload.mass       = static_cast<double>(req->mass);  // kg
    payload.centroid.x = static_cast<double>(req->xc);    // mm 
    payload.centroid.y = static_cast<double>(req->yc);    // mm 
    payload.centroid.z = static_cast<double>(req->zc);    // mm 

    const int ret = robot.robot_set_tool_payload(static_cast<int>(req->robot_index), &payload);

    if (ret == 0) {
        res->ret = 1;
        res->message = "robot_set_tool_payload executed";
        return true;
    } else {
        res->ret = 0;
        res->message = string("error occurred: ") + mapErr[ret];
        return false;
    }
}


bool get_tool_payload_callback(
    const shared_ptr<jaka_kargo_msgs::srv::GetToolPayload::Request> req,
    shared_ptr<jaka_kargo_msgs::srv::GetToolPayload::Response> res)
{
    // robot_index: only per-arm (no -1)
    if (req->robot_index != 0 && req->robot_index != 1) {
        res->ret = -1;
        res->message = "robot_index must be 0 (LEFT) or 1 (RIGHT)";
        return false;
    }

    // SDK returns payload for BOTH arms at once
    PayLoad payloads[2]{};
    const int ret = robot.robot_get_tool_payload(payloads);

    if (ret == 0) {
        res->ret = 1;
        const PayLoad& p = payloads[req->robot_index];
        res->mass = static_cast<float>(p.mass);         // kg
        res->xc   = static_cast<float>(p.centroid.x);   // mm
        res->yc   = static_cast<float>(p.centroid.y);   // mm
        res->zc   = static_cast<float>(p.centroid.z);   // mm
        res->message = "robot_get_tool_payload executed";
        return true;
    } else {
        res->ret = 0;
        res->message = string("error occurred: ") + mapErr[ret];
        return false;
    }
}


bool drag_mode_callback(
    const shared_ptr<jaka_kargo_msgs::srv::DragMode::Request> req,
    shared_ptr<jaka_kargo_msgs::srv::DragMode::Response> res)
{
    if (req->robot_index != -1 && req->robot_index != 0 && req->robot_index != 1) {
        res->ret = -1;
        res->message = "robot_index must be -1 (DUAL), 0 (LEFT), or 1 (RIGHT)";
        return false;
    }

    const BOOL enable = req->enable ? TRUE : FALSE;
    const int ret = robot.drag_mode_enable(static_cast<int>(req->robot_index), enable);

    if (ret == 0) {
        res->ret = 1;
        res->message = "drag_mode_enable executed";
        return true;
    } else {
        res->ret = 0;
        res->message = string("error occurred: ") + mapErr[ret];
        return false;
    }
}


bool set_collision_level_callback(
    const shared_ptr<jaka_kargo_msgs::srv::SetCollisionLevel::Request> req,
    shared_ptr<jaka_kargo_msgs::srv::SetCollisionLevel::Response> res)
{
    // robot_index is per-arm only (no -1)
    if (req->robot_index != 0 && req->robot_index != 1) {
        res->ret = -1;
        res->message = "robot_index must be 0 (LEFT) or 1 (RIGHT)";
        return false;
    }

    // Map enable/value → level [0..5]
    int level = 0;
    if(req->is_enable == 0)
        {
            level = 0;
        }
    else
        {
            const int v = max(0, static_cast<int>(req->value));
            if      (v <= 25) level = 1;
            else if (v <= 50) level = 2;
            else if (v <= 75) level = 3;
            else if (v <=100) level = 4;
            else                 level = 5;
        }

    const int ret = robot.set_collision_level(static_cast<int>(req->robot_index), level);

    if (ret == 0) {
        res->ret = 1;
        res->message = "Collision level " + to_string(level) + " set";
        return true;
    } else {
        res->ret = 0;
        res->message = string("error occurred: ") + mapErr[ret];
        return false;
    }
}


bool get_collision_level_callback(
    const shared_ptr<jaka_kargo_msgs::srv::GetCollisionLevel::Request> req,
    shared_ptr<jaka_kargo_msgs::srv::GetCollisionLevel::Response> res)
{
    // robot_index is per-arm only (no -1)
    if (req->robot_index != 0 && req->robot_index != 1) {
        res->ret = -1;
        res->level = -1;
        res->message = "robot_index must be 0 (LEFT) or 1 (RIGHT)";
        return false;
    }

    int level = -1;
    const int ret = robot.get_collision_level(static_cast<int>(req->robot_index), &level);

    if (ret == 0) {
        res->ret = 1;
        res->level = static_cast<int16_t>(level);  // expected 0..5
        res->message = "Collision level: " + to_string(level);
        return true;
    } else {
        res->ret = 0;
        res->message = string("error occurred: ") + mapErr[ret];
        return false;
    }
}


bool get_fk_callback(
    const shared_ptr<jaka_kargo_msgs::srv::GetFK::Request> req,
    shared_ptr<jaka_kargo_msgs::srv::GetFK::Response> res)
{
    // Require per-arm robot_index
    if (req->robot_index != 0 && req->robot_index != 1) {
        res->cartesian_pose = {9999,9999,9999,9999,9999,9999};
        res->message = "robot_index must be 0 (LEFT) or 1 (RIGHT)";
        return false;
    }

    // Expect 7 joints (radians)
    if (req->joints.size() != 7) {
        res->cartesian_pose = {9999,9999,9999,9999,9999,9999};
        res->message = "joints must have 6 or 7 elements (radians)";
        return false;
    }

    JointValue joint_pose{};     // SDK wants radians
    for (int i = 0; i < 7; ++i) {
        joint_pose.jVal[i] = static_cast<double>(req->joints[i]);
    }

    CartesianPose cartesian_pose{};
    const int ret = robot.kine_forward(static_cast<int>(req->robot_index),
                                        &joint_pose, &cartesian_pose);

    if (ret == 0) {
        // SDK returns x,y,z as-is (mm) and rpy already converted back to radians.
        res->cartesian_pose.clear();
        res->cartesian_pose.push_back(static_cast<float>(cartesian_pose.tran.x));
        res->cartesian_pose.push_back(static_cast<float>(cartesian_pose.tran.y));
        res->cartesian_pose.push_back(static_cast<float>(cartesian_pose.tran.z));
        res->cartesian_pose.push_back(static_cast<float>(cartesian_pose.rpy.rx));
        res->cartesian_pose.push_back(static_cast<float>(cartesian_pose.rpy.ry));
        res->cartesian_pose.push_back(static_cast<float>(cartesian_pose.rpy.rz));
        res->message = "kine_forward executed";
        return true;
    } else {
        res->cartesian_pose = {9999,9999,9999,9999,9999,9999};
        res->message = string("error occurred: ") + mapErr[ret];
        return false;
    }
}


bool get_ik_callback(
    const shared_ptr<jaka_kargo_msgs::srv::GetIK::Request> req,
    shared_ptr<jaka_kargo_msgs::srv::GetIK::Response> res)
{
    // Validate robot_index (no DUAL in kine_inverse)
    if (req->robot_index != 0 && req->robot_index != 1) {
        res->joints = {9999,9999,9999,9999,9999,9999,9999};
        res->message = "robot_index must be 0 (LEFT) or 1 (RIGHT)";
        return false;
    }

    // Expect pose length 6 (mm + rad)
    if (req->cartesian_pose.size() != 6) {
        res->joints = {9999,9999,9999,9999,9999,9999,9999};
        res->message = "cartesian_pose must be [x,y,z,rx,ry,rz]";
        return false;
    }

    // Expect 7 ref_joints (radians)
    if (req->ref_joints.size() != 7) {
        res->joints = {9999,9999,9999,9999,9999,9999,9999};
        res->message = "ref_joints must have 6 or 7 elements (radians)";
        return false;
    }

    JointValue ref{};     // radians
    for (int i = 0; i < 7; ++i)
        ref.jVal[i] = static_cast<double>(req->ref_joints[i]);

    CartesianPose target{};  // mm + radians
    target.tran.x = static_cast<double>(req->cartesian_pose[0]);
    target.tran.y = static_cast<double>(req->cartesian_pose[1]);
    target.tran.z = static_cast<double>(req->cartesian_pose[2]);
    target.rpy.rx = static_cast<double>(req->cartesian_pose[3]);
    target.rpy.ry = static_cast<double>(req->cartesian_pose[4]);
    target.rpy.rz = static_cast<double>(req->cartesian_pose[5]);

    JointValue out{};
    const int ret = robot.kine_inverse(static_cast<int>(req->robot_index), &ref, &target, &out);

    if (ret == 0) {
        // Return 7 joint values in radians
        res->joints.clear();
        for (int i = 0; i < 7; ++i) res->joints.push_back(static_cast<float>(out.jVal[i]));
        res->message = "kine_inverse executed";
        return true;
    } else {
        res->joints = {9999,9999,9999,9999,9999,9999,9999};
        res->message = string("error occurred: ") + mapErr[ret];
        return false;
    }
}


bool clear_error_callback(
    const shared_ptr<jaka_kargo_msgs::srv::ClearError::Request> /*request*/,
    shared_ptr<jaka_kargo_msgs::srv::ClearError::Response> res)
{
    const int ret = robot.clear_error();  // calls collision_recover sdk interface

    if (ret == 0) {
        res->ret = 1;
        res->message = "clear_error executed";
        return true;
    } else {
        res->ret = 0;
        res->message = string("error occurred: ") + mapErr[ret];
        return false;
    }
}


bool get_sdk_version_callback(
    const shared_ptr<jaka_kargo_msgs::srv::GetSdkVersion::Request> /*request*/,
    shared_ptr<jaka_kargo_msgs::srv::GetSdkVersion::Response> res)
{
    char buf[256] = {0};                     // enough for JAKA_CPP_SDK_INFO
    const int ret = robot.get_sdk_version(buf);

    if (ret == 0) {
        RCLCPP_INFO(rclcpp::get_logger("get_sdk_version_callback"), "SDK: %s", buf);
        res->ret = 1;
        res->version = string(buf);
        res->message = "get_sdk_version executed";
        return true;
    } else {
        res->ret = 0;
        res->version.clear();
        res->message = string("error occurred: ") + mapErr[ret];
        return false;
    }
}


bool set_debug_mode_callback(
    const shared_ptr<jaka_kargo_msgs::srv::SetDebugMode::Request> req,
    shared_ptr<jaka_kargo_msgs::srv::SetDebugMode::Response> res)
{
    const BOOL mode = req->enable ? TRUE : FALSE;
    const int ret = robot.set_debug_mode(mode);

    if (ret == 0) {
        res->ret = 1;
        res->message = "set_debug_mode executed";
        return true;
    } else {
        res->ret = 0;
        res->message = string("error occurred: ") + mapErr[ret];
        return false;
    }
}


bool get_dh_params_callback(
    const shared_ptr<jaka_kargo_msgs::srv::GetDHParams::Request> req,
    shared_ptr<jaka_kargo_msgs::srv::GetDHParams::Response> res)
{
    // Per-arm only: 0 (LEFT) or 1 (RIGHT)
    if (req->robot_index != 0 && req->robot_index != 1) {
        res->ret = -1;
        res->message = "robot_index must be 0 (LEFT) or 1 (RIGHT)";
        return false;
    }

    // SDK returns DH params for BOTH arms at once
    DHParam dh[2]{};  // [0]=LEFT, [1]=RIGHT
    const int ret = robot.robot_get_multi_robot_dh(dh);

    if (ret == 0) {
        res->ret = 1;
        const DHParam& P = dh[req->robot_index];
        // Resize response arrays to controller joint count
        res->alpha.resize(JAKA_ROBOT_MAX_JOINT);
        res->a.resize(JAKA_ROBOT_MAX_JOINT);
        res->d.resize(JAKA_ROBOT_MAX_JOINT);
        res->joint_homeoff.resize(JAKA_ROBOT_MAX_JOINT);
        for (int i = 0; i < JAKA_ROBOT_MAX_JOINT; ++i) {
            res->alpha[i]        = static_cast<float>(P.alpha[i]);       // rad (typ.)
            res->a[i]            = static_cast<float>(P.a[i]);           // mm (typ.)
            res->d[i]            = static_cast<float>(P.d[i]);           // mm (typ.)
            res->joint_homeoff[i]= static_cast<float>(P.joint_homeoff[i]); // rad (typ.)
        }
        res->message = "robot_get_multi_robot_dh executed";
        return true;
    } else {
        res->ret = 0;
        res->message = string("robot_get_multi_robot_dh failed: ") + mapErr[ret];
        return false;
    }
}


bool ext_enable_callback(
    const shared_ptr<jaka_kargo_msgs::srv::ExtEnable::Request> req,
    shared_ptr<jaka_kargo_msgs::srv::ExtEnable::Response> res)
{
    if (req->ext_id < -1 || req->ext_id > 3) {
        res->ret = -1;
        res->message = "ext_id must be -1 or 0..3";
        return false;
    }

    int ret = req->enable ? robot.enable_ext(req->ext_id)
                          : robot.disable_ext(req->ext_id);

    if (ret == 0) {
        res->ret = 1;
        res->message = req->enable ? "enable_ext executed" : "disable_ext executed";
        return true;
    } else {
        res->ret = 0;
        res->message = string("error occurred: ") + mapErr[ret];
        return false;
    }
}


bool jog_ext_callback(
    const shared_ptr<jaka_kargo_msgs::srv::JogExt::Request> req,
    shared_ptr<jaka_kargo_msgs::srv::JogExt::Response> res)
{
    if (req->ext_id < 0 || req->ext_id > 3) {
        res->ret = -1;
        res->message = "ext_id must be 0..3";
        return false;
    }

    if (req->is_abs < 0 || req->is_abs > 2) {
        res->ret = -1;
        res->message = "is_abs must be 0 (ABS), 1 (REL user), or 2 (REL tool)";
        return false;
    }

    if (!isfinite(req->vel) || !isfinite(req->step)) {
        res->ret = -1;
        res->message = "vel and step must be finite";
        return false;
    }

    const int ret = robot.jog_ext(req->ext_id, req->is_abs, req->vel, req->step);

    if (ret == 0) {
        res->ret = 1;
        res->message = "jog_ext executed";
        return true;
    } else {
        res->ret = 0;
        res->message = string("error occurred: ") + mapErr[ret];
        return false;
    }
}


bool multi_move_ext_callback(
    const shared_ptr<jaka_kargo_msgs::srv::MultiMoveExt::Request> req,
    shared_ptr<jaka_kargo_msgs::srv::MultiMoveExt::Response> res)
{
    const size_t n = req->moves.size();

    if (n == 0) {
        res->ret = -1;
        res->message = "moves cannot be empty";
        return false;
    }

    if (n > MAX_EXT_CNT) {
        res->ret = -1;
        res->message = "too many moves in request";
        return false;
    }

    MultiMovInfoList info_list{};
    info_list.count = static_cast<int>(n);

    for (size_t i = 0; i < n; ++i) {
        const auto &src = req->moves[i];
        auto &dst = info_list.info[i];

        if (src.motion_unit_type != 0 && src.motion_unit_type != 1) {
            res->ret = -1;
            res->message = "motion_unit_type must be 0 (robot) or 1 (ext)";
            return false;
        }

        if (src.move_type < 0 || src.move_type > 2) {
            res->ret = -1;
            res->message = "move_type must be 0 (JOINT), 1 (LINEAR), or 2 (CIRCULAR)";
            return false;
        }

        if (src.move_mode < 0 || src.move_mode > 3) {
            res->ret = -1;
            res->message = "move_mode must be 0 (ABS), 1 (INCR), 2 (CONTINUE), or 3 (STOP)";
            return false;
        }

        // ext axis only supports JOINT_MOVE here
        if (src.motion_unit_type == 1 && src.move_type != JOINT_MOVE) {
            res->ret = -1;
            res->message = "external axis motion_unit_type=1 only supports move_type=JOINT_MOVE";
            return false;
        }

        dst.motion_unit_type = src.motion_unit_type;
        dst.motion_unit_id   = src.motion_unit_id;
        dst.move_type        = static_cast<MoveType>(src.move_type);
        dst.move_mode        = static_cast<MoveMode>(src.move_mode);

        // zero-init joint array
        for (int j = 0; j < MAX_AXIS; ++j) {
            dst.movej_info.end_pos[j] = 0.0;
        }

        if (dst.move_type == JOINT_MOVE) {
            if (src.motion_unit_type == 1) {
                // ext axis: use only first value
                if (src.movej_end_pos.empty()) {
                    res->ret = -1;
                    res->message = "ext JOINT_MOVE requires movej_end_pos[0]";
                    return false;
                }
                dst.movej_info.end_pos[0] = src.movej_end_pos[0];
            } else {
                // robot arm: copy all provided joints
                if (src.movej_end_pos.empty() || src.movej_end_pos.size() > MAX_AXIS) {
                    res->ret = -1;
                    res->message = "robot JOINT_MOVE requires 1..MAX_AXIS joint values";
                    return false;
                }
                for (size_t j = 0; j < src.movej_end_pos.size(); ++j) {
                    dst.movej_info.end_pos[j] = src.movej_end_pos[j];
                }
            }

            dst.movej_info.j_vel     = src.movej_j_vel;
            dst.movej_info.j_acc     = src.movej_j_acc;
            dst.movej_info.j_jerk    = src.movej_j_jerk;
            dst.movej_info.blend_tol = src.movej_blend_tol;
        }
        else if (dst.move_type == LINEAR_MOVE) {
            if (src.motion_unit_type != 0) {
                res->ret = -1;
                res->message = "LINEAR_MOVE is only valid for robot motion_unit_type=0";
                return false;
            }

            dst.movel_info.end_pos.tran.x = src.movel_end_pos[0];
            dst.movel_info.end_pos.tran.y = src.movel_end_pos[1];
            dst.movel_info.end_pos.tran.z = src.movel_end_pos[2];
            dst.movel_info.end_pos.rpy.rx = src.movel_end_pos[3];
            dst.movel_info.end_pos.rpy.ry = src.movel_end_pos[4];
            dst.movel_info.end_pos.rpy.rz = src.movel_end_pos[5];

            dst.movel_info.vel       = src.movel_vel;
            dst.movel_info.acc       = src.movel_acc;
            dst.movel_info.jerk      = src.movel_jerk;
            dst.movel_info.ori_vel   = src.movel_ori_vel;
            dst.movel_info.ori_acc   = src.movel_ori_acc;
            dst.movel_info.ori_jerk  = src.movel_ori_jerk;
            dst.movel_info.blend_tol = src.movel_blend_tol;
        }
        else if (dst.move_type == CIRCULAR_MOVE) {
            if (src.motion_unit_type != 0) {
                res->ret = -1;
                res->message = "CIRCULAR_MOVE is only valid for robot motion_unit_type=0";
                return false;
            }

            dst.movec_info.mid_pos.tran.x = src.movec_mid_pos[0];
            dst.movec_info.mid_pos.tran.y = src.movec_mid_pos[1];
            dst.movec_info.mid_pos.tran.z = src.movec_mid_pos[2];
            dst.movec_info.mid_pos.rpy.rx = src.movec_mid_pos[3];
            dst.movec_info.mid_pos.rpy.ry = src.movec_mid_pos[4];
            dst.movec_info.mid_pos.rpy.rz = src.movec_mid_pos[5];

            dst.movec_info.end_pos.tran.x = src.movec_end_pos[0];
            dst.movec_info.end_pos.tran.y = src.movec_end_pos[1];
            dst.movec_info.end_pos.tran.z = src.movec_end_pos[2];
            dst.movec_info.end_pos.rpy.rx = src.movec_end_pos[3];
            dst.movec_info.end_pos.rpy.ry = src.movec_end_pos[4];
            dst.movec_info.end_pos.rpy.rz = src.movec_end_pos[5];

            dst.movec_info.vel         = src.movec_vel;
            dst.movec_info.acc         = src.movec_acc;
            dst.movec_info.jerk        = src.movec_jerk;
            dst.movec_info.ori_vel     = src.movec_ori_vel;
            dst.movec_info.ori_acc     = src.movec_ori_acc;
            dst.movec_info.ori_jerk    = src.movec_ori_jerk;
            dst.movec_info.blend_tol   = src.movec_blend_tol;
            dst.movec_info.circle_cnt  = src.movec_circle_cnt;
            dst.movec_info.circle_mode = src.movec_circle_mode;
        }
    }

    DI_Info di_info{};
    DI_Info *di_ptr = nullptr;
    if (req->use_di_info) {
        di_info.io_type = static_cast<IOType>(req->di_info.io_type);
        di_info.index   = req->di_info.index;
        di_info.value   = req->di_info.value ? TRUE : FALSE;
        di_info.submod  = req->di_info.submod;
        di_ptr = &di_info;
    }

    const BOOL is_block = req->is_block ? TRUE : FALSE;
    const int ret = robot.multi_mov_with_ext(&info_list, is_block, di_ptr, req->planner_type);

    if (ret == 0) {
        res->ret = 1;
        res->message = "multi_mov_with_ext executed";
        return true;
    } else {
        res->ret = 0;
        res->message = string("error occurred: ") + mapErr[ret];
        return false;
    }
}


// bool get_ext_status_callback(
//     const shared_ptr<jaka_kargo_msgs::srv::GetExtStatus::Request> req,
//     shared_ptr<jaka_kargo_msgs::srv::GetExtStatus::Response> res)
// {
//     if (req->ext_id < -1 || req->ext_id > 3) {
//         res->ret = -1;
//         res->message = "ext_id must be -1 or 0..3";
//         return false;
//     }

//     ExtAxisStatusList status_list{};
//     const int ret = robot.get_ext_status(&status_list, req->ext_id);

//     if (ret != 0) {
//         res->ret = 0;
//         res->message = string("error occurred: ") + mapErr[ret];
//         res->count = 0;
//         return false;
//     }

//     res->ret = 1;
//     res->message = "get_ext_status executed";
//     res->count = status_list.count;

//     res->is_powered.clear();
//     res->is_powering.clear();
//     res->is_enabled.clear();
//     res->is_enabling.clear();
//     res->is_inpos.clear();
//     res->is_on_limit.clear();
//     res->pos_cmd.clear();
//     res->pos_fdb.clear();

//     for (int i = 0; i < status_list.count; ++i) {
//         res->is_powered.push_back(status_list.status[i].is_powered);
//         res->is_powering.push_back(status_list.status[i].is_powering);
//         res->is_enabled.push_back(status_list.status[i].is_enabled);
//         res->is_enabling.push_back(status_list.status[i].is_enabling);
//         res->is_inpos.push_back(status_list.status[i].is_inpos);
//         res->is_on_limit.push_back(status_list.status[i].is_on_limit);
//         res->pos_cmd.push_back(status_list.status[i].pos_cmd);
//         res->pos_fdb.push_back(status_list.status[i].pos_fdb);
//     }

//     return true;
// }


// TCP pose via EDG (per arm). Publishes position (mm) + RPY (deg in angular fields).
void tool_position_callback(
    const rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr& tool_position_pub)
{
    // Check if publisher is valid
    if (!tool_position_pub)
    {
        RCLCPP_ERROR(rclcpp::get_logger("tool_position_callback"), "TCP publisher is not initialized!");
        return;
    }

    for (int robot_index = 0; robot_index < 2; ++robot_index) {
        geometry_msgs::msg::TwistStamped tool_position;
        CartesianPose tcp_position{};

        const int ret = robot.edg_get_stat(static_cast<unsigned char>(robot_index), nullptr, &tcp_position);
        if (ret != 0) 
        { 
            RCLCPP_ERROR(rclcpp::get_logger("tool_position_callback"), "edg_get_stat failed: %s", mapErr[ret].c_str()); 
            return; 
        }

        tool_position.twist.linear.x  = tcp_position.tran.x;                    // mm
        tool_position.twist.linear.y  = tcp_position.tran.y;                    // mm
        tool_position.twist.linear.z  = tcp_position.tran.z;                    // mm
        tool_position.twist.angular.x = tcp_position.rpy.rx;                    // deg 
        tool_position.twist.angular.y = tcp_position.rpy.ry;                    // deg 
        tool_position.twist.angular.z = tcp_position.rpy.rz;                    // deg 
        tool_position.header.frame_id = (robot_index ==0) ? "left_tcp" : "right_tcp";
        tool_position.header.stamp = rclcpp::Clock().now();
        tool_position_pub->publish(tool_position);
    }
}


// Joint state for full-robot (two arms + external-axis)
void joint_position_callback(
    const rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr& joint_position_pub)
{
    // Check if publisher is valid
    if (!joint_position_pub) 
    { 
        RCLCPP_ERROR(rclcpp::get_logger("joint_position_callback"), "Joint publisher not initialized"); 
        return; 
    }

    sensor_msgs::msg::JointState joint_position;
    JointValue joint_pos{};

    // Arms
    for (int robot_index = 0; robot_index < 2; ++robot_index) {
        const int ret = robot.edg_get_stat(static_cast<unsigned char>(robot_index), &joint_pos, nullptr);
        if (ret != 0) 
        { 
            RCLCPP_ERROR(rclcpp::get_logger("joint_position_callback"), "edg_get_stat failed: %s", mapErr[ret].c_str()); 
            continue; 
        }

        const char* prefix = (robot_index == 0) ? "left_joint_" : "right_joint_";
        for (int i = 0; i < 7; ++i) {                        
            joint_position.position.push_back(joint_pos.jVal[i]);               // radians
            joint_position.name.push_back(string(prefix) + to_string(i+1));
        }
    }

    // External-axis
    ExtAxisStatusList ext_status{};
    const int ext_ret = robot.get_ext_status(&ext_status, -1);
    if (ext_ret != 0)
    {
        RCLCPP_ERROR(rclcpp::get_logger("joint_position_callback"), "get_ext_status failed: %s", mapErr[ext_ret].c_str());
    }
    else if (ext_status.count < 4)
    {
        RCLCPP_ERROR(rclcpp::get_logger("joint_position_callback"), "get_ext_status couldn't read states of all 4 external-axis");
    }
    else
    {
        for (int i = 0; i < 4; ++i) {
            joint_position.name.push_back("ext_joint_" + to_string(i + 1));
            if (i == 0) {
                joint_position.position.push_back(ext_status.status[i].pos_fdb / 1000.0);  // mm -> m
            } else {
                joint_position.position.push_back(ext_status.status[i].pos_fdb * M_PI / 180.0);  // deg -> rad
            }
        }
    }

    joint_position.header.stamp = rclcpp::Clock().now();
    joint_position_pub->publish(joint_position);
}


void robot_states_callback(const rclcpp::Publisher<jaka_kargo_msgs::msg::RobotStates>::SharedPtr& robot_states_pub)
{
    if (!robot_states_pub) {
        RCLCPP_ERROR(rclcpp::get_logger("robot_states_callback"), "Robot states publisher is not initialized!");
        return;
    }

    jaka_kargo_msgs::msg::RobotStates robot_states{};
    bool ok = true;

    // Power/servo_enable/estop come from get_robot_state (aggregated)
    RobotState st{};
    int ret = robot.get_robot_state(&st);
    if (ret != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("robot_states_callback"),
                    "get_robot_state failed: %s", mapErr[ret].c_str());
        ok = false;

    }
    robot_states.power_state = st.poweredOn;
    robot_states.servo_state = st.servoEnabled;
    robot_states.estoped = st.estoped;

    // Collision (protective stop flag)
    BOOL in_col = FALSE;
    ret = robot.is_in_collision(&in_col);
    if (ret != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("robot_states_callback"),
                    "is_in_collision failed: %s", mapErr[ret].c_str());
        ok = false;
    }
    robot_states.collision_state = in_col;

    // In-position per arm (0/1 each)
    int in_pos[2] = {0, 0};
    ret = robot.robot_is_inpos(in_pos);
    if (ret != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("robot_states_callback"),
                    "robot_is_inpos failed: %s", mapErr[ret].c_str());
        ok = false;
    }
    robot_states.inpos_left = (in_pos[0] != 0);
    robot_states.inpos_right = (in_pos[1] != 0);

    // Drag mode per arm (0/1 each)
    BOOL in_drag[2] = {0, 0};
    ret = robot.is_in_drag_mode(in_drag);
    if (ret != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("robot_states_callback"),
                    "is_in_drag_mode failed: %d (%s)", ret, mapErr[ret].c_str());
        ok = false;
    }
    robot_states.drag_left = (in_drag[0] != 0);
    robot_states.drag_right = (in_drag[1] != 0);

    // Error per arm (0/1 each)
    int in_error[2] = {0, 0};
    ret = robot.robot_is_in_error(in_error);
    if (ret != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("robot_states_callback"),
                    "robot_is_in_error failed: %s", mapErr[ret].c_str());
        ok = false;
    }
    robot_states.inerror_left = (in_error[0] != 0);
    robot_states.inerror_right = (in_error[1] != 0);

    // External axis states
    ExtAxisStatusList ext_status{};
    ret = robot.get_ext_status(&ext_status, -1);
    if (ret != 0)
    {
        RCLCPP_ERROR(rclcpp::get_logger("robot_states_callback"), 
                    "get_ext_status failed: %s", mapErr[ret].c_str());
        ok = false;
    }
    else if (ext_status.count < 4)
    {
        RCLCPP_ERROR(rclcpp::get_logger("robot_states_callback"), 
                    "get_ext_status couldn't read states of all 4 external-axis");
        ok = false;
    } else {
        robot_states.ext_is_powered.clear();
        robot_states.ext_is_powering.clear();
        robot_states.ext_is_enabled.clear();
        robot_states.ext_is_enabling.clear();
        robot_states.ext_is_inpos.clear();
        robot_states.ext_is_on_limit.clear();

        for (int i = 0; i < 4; ++i) {
            robot_states.ext_is_powered.push_back(ext_status.status[i].is_powered);
            robot_states.ext_is_powering.push_back(ext_status.status[i].is_powering);
            robot_states.ext_is_enabled.push_back(ext_status.status[i].is_enabled);
            robot_states.ext_is_enabling.push_back(ext_status.status[i].is_enabling);
            robot_states.ext_is_inpos.push_back(ext_status.status[i].is_inpos);
            robot_states.ext_is_on_limit.push_back(ext_status.status[i].is_on_limit);
        }
    }

    // Only publish if all reads succeeded
    if (!ok) {
        RCLCPP_WARN(rclcpp::get_logger("robot_states_callback"), "Robot state read incomplete this tick — skipping publish.");
        return;
    }

    robot_states_pub->publish(robot_states);
}


void get_connect_state()
{
    RobotState st{};
    while (rclcpp::ok())
    {
        // Controller-wide health check
        const int ret = robot.get_robot_state(&st);

        if (ret != 0) 
        {
            // Skip this tick entirely — no reliable state
            RCLCPP_ERROR(rclcpp::get_logger("get_connect_state"),
                        "Connection error or get_robot_state failed: %s", mapErr[ret].c_str());
        } 
        
        else 
        {
            // Connection OK → publish robot states
            robot_states_callback(robot_states_pub);
            
            // EDG stream health check (per arm)
            const int ret_edg_left  = robot.edg_get_stat(0, nullptr, nullptr);
            const int ret_edg_right = robot.edg_get_stat(1, nullptr, nullptr);
            const bool edg_ok = (ret_edg_left == 0) && (ret_edg_right == 0);

            if (!edg_ok) 
            {
                // At least one EDG stream is down — don’t publish EDG-based topics this tick
                RCLCPP_ERROR(rclcpp::get_logger("get_connect_state"),
                            "Connection error or edg_get_stat failed: %s", mapErr[ret].c_str());
            }
            else 
            {
                // EDG streams OK → publish TCP position and joints position from EDG
                tool_position_callback(tool_position_pub);
                joint_position_callback(joint_position_pub);
            }

        }

        rclcpp::sleep_for(chrono::milliseconds(100)); 
    }
}


int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("jaka_kargo_driver");

    // Params
    string default_ip = "127.0.0.1";
    string robot_ip = node->declare_parameter("ip", default_ip);

    edg_init_ip = make_edg_bcast(robot_ip);
    RCLCPP_INFO(node->get_logger(), "EDG init IP set to: %s", edg_init_ip.c_str());

    // Connect
    int ret = robot.login_in(robot_ip.c_str());
    if (ret != 0) 
    { 
        RCLCPP_FATAL(node->get_logger(), "login_in failed: %s", mapErr[ret].c_str()); 
        return -1; 
    }

    // robot.set_status_data_update_time_interval(100); // ms 
    robot.set_block_wait_timeout(120);               // seconds

    // Power on
    ret = robot.power_on();
    if (ret != 0) 
    { 
        RCLCPP_FATAL(node->get_logger(), "power_on failed: %s", mapErr[ret].c_str()); 
        return -1; 
    }

    // Wait until poweredOn 
    {
        RobotState st{};
        auto deadline = chrono::steady_clock::now() + 10s;
        do {
        ret = robot.get_robot_state(&st);
        if (ret == 0 && st.poweredOn) break;
        this_thread::sleep_for(200ms);
        } while (chrono::steady_clock::now() < deadline);
        if (!(ret == 0)) 
        { 
            RCLCPP_FATAL(node->get_logger(), "get_robot_state failed: %s", mapErr[ret].c_str()); 
            return -1; 
        }
        if (!st.poweredOn) 
        { 
            RCLCPP_FATAL(node->get_logger(), "Controller did not power on in time"); 
            return -1; 
        }
    }

    // Enable
    ret = robot.enable_robot();
    if (ret != 0)
     { 
        RCLCPP_FATAL(node->get_logger(), "enable_robot failed: %s", mapErr[ret].c_str()); 
        return -1; 
    }

    // Confirm servoEnabled
    {
        RobotState st{};
        auto deadline = chrono::steady_clock::now() + 5s;
        do {
        ret = robot.get_robot_state(&st);
        if (ret == 0 && st.servoEnabled) break;
        this_thread::sleep_for(100ms);
        } while (chrono::steady_clock::now() < deadline);
        if (ret != 0) 
        { 
            RCLCPP_FATAL(node->get_logger(), "get_robot_state failed: %s", mapErr[ret].c_str()); 
            return -1; 
        }
        if (!st.servoEnabled) 
        { 
            RCLCPP_FATAL(node->get_logger(), "Robot not servo-enabled in time"); 
            return -1; 
        }
    }

    // Global call for edg_init
    robot.edg_init(1, edg_init_ip.c_str());

    // Configure controller behavior on SDK link loss
    {
        const float timeout_ms = 1000.0f;  // start safe; tighten to 200–500 ms for EDG servo loops
        const ProcessType on_net_fail = ProcessType::MOT_ABORT;  

        int ret = robot.set_network_exception_handle(timeout_ms, on_net_fail);
        if (ret != 0) {
            RCLCPP_WARN(node->get_logger(),
            "set_network_exception_handle failed: %s (controller will use its default behavior)",
            mapErr[ret].c_str());
        } else {
            RCLCPP_INFO(node->get_logger(),
            "Network fail-safe set: %.0f ms → %d (abort/stop)", timeout_ms, (int)on_net_fail);
        }
    }

    //--------Services------------//
    auto linear_move_service = node->create_service<jaka_kargo_msgs::srv::Move>("/jaka_kargo_driver/linear_move", &linear_move_callback);
    auto joint_move_service = node->create_service<jaka_kargo_msgs::srv::Move>("/jaka_kargo_driver/joint_move", &joint_move_callback);
    auto servo_move_enable_service = node->create_service<jaka_kargo_msgs::srv::ServoMoveEnable>("/jaka_kargo_driver/servo_move_enable", &servo_move_enable_callback);
    auto edg_servo_p_service = node->create_service<jaka_kargo_msgs::srv::ServoMove>("/jaka_kargo_driver/edg_servo_p", &edg_servo_p_callback);
    auto edg_servo_j_service = node->create_service<jaka_kargo_msgs::srv::ServoMove>("/jaka_kargo_driver/edg_servo_j", &edg_servo_j_callback);
    auto stop_move_service = node->create_service<std_srvs::srv::Empty>("/jaka_kargo_driver/stop_move", &stop_move_callback);
    auto get_default_base_service = node->create_service<jaka_kargo_msgs::srv::GetDefaultBase>("/jaka_kargo_driver/get_default_base", &get_default_base_callback);
    auto set_tool_offset_service = node->create_service<jaka_kargo_msgs::srv::SetToolOffset>("/jaka_kargo_driver/set_tool_offset", &set_tool_offset_callback);
    auto get_tool_offset_service = node->create_service<jaka_kargo_msgs::srv::GetToolOffset>("/jaka_kargo_driver/get_tool_offset", &get_tool_offset_callback);
    auto set_tool_payload_service = node->create_service<jaka_kargo_msgs::srv::SetToolPayload>("/jaka_kargo_driver/set_tool_payload", &set_tool_payload_callback);
    auto get_tool_payload_service = node->create_service<jaka_kargo_msgs::srv::GetToolPayload>("/jaka_kargo_driver/get_tool_payload", &get_tool_payload_callback);
    auto drag_move_service = node->create_service<jaka_kargo_msgs::srv::DragMode>("/jaka_kargo_driver/drag_mode", &drag_mode_callback);
    auto set_collision_level_service = node->create_service<jaka_kargo_msgs::srv::SetCollisionLevel>("/jaka_kargo_driver/set_collision_level", &set_collision_level_callback);
    auto get_collision_level_service = node->create_service<jaka_kargo_msgs::srv::GetCollisionLevel>("/jaka_kargo_driver/get_collision_level", &get_collision_level_callback);
    auto get_fk_service = node->create_service<jaka_kargo_msgs::srv::GetFK>("/jaka_kargo_driver/get_fk", &get_fk_callback);
    auto get_ik_service = node->create_service<jaka_kargo_msgs::srv::GetIK>("/jaka_kargo_driver/get_ik", &get_ik_callback);
    auto clear_error_service = node->create_service<jaka_kargo_msgs::srv::ClearError>("/jaka_kargo_driver/clear_error", &clear_error_callback);
    auto get_sdk_version_service = node->create_service<jaka_kargo_msgs::srv::GetSdkVersion>("/jaka_kargo_driver/get_sdk_version", &get_sdk_version_callback);
    auto set_debug_mode_service = node->create_service<jaka_kargo_msgs::srv::SetDebugMode>("/jaka_kargo_driver/set_debug_mode", &set_debug_mode_callback);
    auto get_dh_params_service = node->create_service<jaka_kargo_msgs::srv::GetDHParams>("/jaka_kargo_driver/get_dh_params", &get_dh_params_callback);
    auto ext_enable_service = node->create_service<jaka_kargo_msgs::srv::ExtEnable>("/jaka_kargo_driver/ext_enable", &ext_enable_callback);
    auto jog_ext_service = node->create_service<jaka_kargo_msgs::srv::JogExt>("/jaka_kargo_driver/jog_ext", &jog_ext_callback);
    auto multi_move_ext_service = node->create_service<jaka_kargo_msgs::srv::MultiMoveExt>("/jaka_kargo_driver/multi_move_ext", &multi_move_ext_callback);
    // auto get_ext_status_service = node->create_service<jaka_kargo_msgs::srv::GetExtStatus>("/jaka_kargo_driver/get_ext_status", &get_ext_status_callback);

    // //3.1 End position pose status information reporting
    tool_position_pub = node->create_publisher<geometry_msgs::msg::TwistStamped>("/jaka_kargo_driver/tool_position", 10);
    // //3.2 Joint status information reporting
    joint_position_pub = node->create_publisher<sensor_msgs::msg::JointState>("/jaka_kargo_driver/joint_position", 10);
    // //3.3 Report robot event status information
    robot_states_pub = node->create_publisher<jaka_kargo_msgs::msg::RobotStates>("/jaka_kargo_driver/robot_states", 10);

    // Monitor network connection status (runs in background thread)
    thread conn_state_thread(get_connect_state);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "start");

    rclcpp::spin(node);
    // Ensure thread is joined before shutting down the node
    if (conn_state_thread.joinable()) {
        conn_state_thread.join();
    }

    rclcpp::shutdown();
    return 0;
}