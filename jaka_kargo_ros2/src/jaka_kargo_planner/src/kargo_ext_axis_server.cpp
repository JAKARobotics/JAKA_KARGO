#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/empty.hpp"

#include "JAKAZuRobot.h"

#include <mutex>
#include <thread>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <csignal>
#include <sstream>

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

class KargoExtAxisServer : public rclcpp::Node
{
public:
    using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
    using GoalHandleFollowJointTrajectory = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;

    bool stop_ext_motion();

    KargoExtAxisServer()
        : Node("kargo_ext_axis_server")
    {
        robot_ip_      = this->declare_parameter<std::string>("robot_ip", "10.5.5.100");
        edg_broadcast_ = this->declare_parameter<std::string>("edg_broadcast", "10.5.5.255");

        vel_ = this->declare_parameter<double>("vel", 80.0);
        acc_ = this->declare_parameter<double>("acc", 80.0);


        if (!init_robot()) {
            throw std::runtime_error("Failed to initialize Kargo external axis robot.");
        }

        action_server_ = rclcpp_action::create_server<FollowJointTrajectory>(
            this,
            "/jaka_kargo_ext_axis_controller/follow_joint_trajectory",
            std::bind(&KargoExtAxisServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&KargoExtAxisServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&KargoExtAxisServer::handle_accepted, this, std::placeholders::_1));

        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

        stop_srv_ = this->create_service<std_srvs::srv::Empty>(
            "kargo_ext_axis/stop",
            std::bind(&KargoExtAxisServer::stopService, this, _1, _2));

        poll_thread_ = std::thread([this]() {
            rclcpp::Rate rate(5); // 5 Hz
            while (rclcpp::ok() && running_) {
                this->pollStatus();
                rate.sleep();
            }
        });

        RCLCPP_INFO(this->get_logger(), "Kargo external axis FollowJointTrajectory action server ready.");
    }

    ~KargoExtAxisServer()
    {
        running_ = false;
        if (poll_thread_.joinable()) {
            poll_thread_.join();
        }

        std::lock_guard<std::mutex> lock(mtx_);
        stop_ext_motion();
        robot.disable_ext(0);
        robot.disable_ext(1);
        robot.disable_ext(2);
        robot.disable_ext(3);
        robot.login_out();
    }

    // void stop_ext_motion_public()
    // {
    //     std::lock_guard<std::mutex> lock(mtx_);
    //     stop_ext_motion();
    // }

private:
    rclcpp_action::Server<FollowJointTrajectory>::SharedPtr action_server_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_srv_;

    std::thread poll_thread_;
    std::mutex mtx_;
    bool running_{true};

    JAKAZuRobot robot;

    std::string robot_ip_;
    std::string edg_broadcast_;

    double vel_{80.0};
    double acc_{80.0};

    bool init_robot()
    {
        std::lock_guard<std::mutex> lock(mtx_);

        errno_t ret = robot.login_in(robot_ip_.c_str());
        if (ret != ERR_SUCC) {
            RCLCPP_ERROR(this->get_logger(), "login_in failed. ret=%d", ret);
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "login_in success.");

        robot.edg_init(1, edg_broadcast_.c_str());
        RCLCPP_INFO(this->get_logger(), "edg_init done.");

        ret = robot.power_on();
        if (ret != ERR_SUCC) {
            RCLCPP_ERROR(this->get_logger(), "power_on failed. ret=%d", ret);
            return false;
        }
        std::this_thread::sleep_for(1000ms);

        ret = robot.enable_robot();
        if (ret != ERR_SUCC) {
            RCLCPP_ERROR(this->get_logger(), "enable_robot failed. ret=%d", ret);
            return false;
        }
        std::this_thread::sleep_for(1000ms);

        return true;
    }

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const FollowJointTrajectory::Goal> goal)
    {
        (void)uuid;

        if (!goal) {
            RCLCPP_WARN(this->get_logger(), "Rejected null goal.");
            return rclcpp_action::GoalResponse::REJECT;
        }

        if (goal->trajectory.points.empty()) {
            RCLCPP_WARN(this->get_logger(), "Rejected empty trajectory.");
            return rclcpp_action::GoalResponse::REJECT;
        }

        const auto &pt = goal->trajectory.points.back();
        if (pt.positions.size() < 4) {
            RCLCPP_WARN(this->get_logger(), "Rejected trajectory. Need 4 joint values, got %zu.", pt.positions.size());
            return rclcpp_action::GoalResponse::REJECT;
        }

        RCLCPP_INFO(this->get_logger(), "Received new 4-axis trajectory goal.");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle)
    {
        (void)goal_handle;
        RCLCPP_WARN(this->get_logger(), "Received request to cancel goal.");

        {
            std::lock_guard<std::mutex> lock(mtx_);
            if (!stop_ext_motion()) {
                RCLCPP_WARN(this->get_logger(), "Failed to stop ext axis.");
            }
        }

        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle)
    {
        std::thread{std::bind(&KargoExtAxisServer::execute, this, goal_handle)}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle)
    {
        const auto goal = goal_handle->get_goal();
        auto result = std::make_shared<FollowJointTrajectory::Result>();

        const auto &pt = goal->trajectory.points.back();

        if (pt.positions.size() < 4) {
            RCLCPP_ERROR(this->get_logger(), "Trajectory point has fewer than 4 joints.");
            result->error_code = -1;
            goal_handle->abort(result);
            return;
        }

        RCLCPP_INFO(this->get_logger(),
                    "Final target trajectory points: %.6f %.6f %.6f %.6f",
                    pt.positions[0], pt.positions[1], pt.positions[2], pt.positions[3]);
        
        // Convert ROS units to SDK units
        std::vector<double> converted_pos(4, 0.0);
        converted_pos[0] = pt.positions[0] * 1000.0;        // joint 0: prismatic, m -> mm
        converted_pos[1] = pt.positions[1] * 180.0 / M_PI;  // joint 1..3: revolute, rad -> deg
        converted_pos[2] = pt.positions[2] * 180.0 / M_PI;
        converted_pos[3] = pt.positions[3] * 180.0 / M_PI;

        RCLCPP_INFO(this->get_logger(),
                    "Sending converted trajectory: %.3f mm, %.3f deg, %.3f deg, %.3f deg",
                    converted_pos[0], converted_pos[1], converted_pos[2], converted_pos[3]);

        {
            std::lock_guard<std::mutex> lock(mtx_);
            errno_t ret_0 = robot.enable_ext(0);
            errno_t ret_1 = robot.enable_ext(1);
            errno_t ret_2 = robot.enable_ext(2);
            errno_t ret_3 = robot.enable_ext(3);
            if  ((ret_0 != ERR_SUCC) || (ret_1 != ERR_SUCC) || (ret_2 != ERR_SUCC) || (ret_3 != ERR_SUCC)) {
                RCLCPP_ERROR(this->get_logger(), "enable_ext failed before motion.");
                result->error_code = -10;
                goal_handle->abort(result);
                return;
            }

            MultiMovInfoList cmd{};
            cmd.count = 4;

            for (int i = 0; i < 4; ++i) {
                cmd.info[i].motion_unit_type = 1;     // external axis
                cmd.info[i].motion_unit_id   = i;     // 0,1,2,3 for the 4 ext joints
                cmd.info[i].move_type        = JOINT_MOVE;
                cmd.info[i].move_mode        = ABS;
                cmd.info[i].movej_info.end_pos[0] = converted_pos[i];
                cmd.info[i].movej_info.j_vel = vel_;
                cmd.info[i].movej_info.j_acc = acc_;
                cmd.info[i].movej_info.j_jerk = 0.0;
                cmd.info[i].movej_info.blend_tol = 0.0;
            }

            errno_t ret = robot.multi_mov_with_ext(&cmd, false);
            if (ret != ERR_SUCC) {
                RCLCPP_ERROR(this->get_logger(), "multi_mov_with_ext failed. ret=%d", ret);
                result->error_code = -2;
                goal_handle->abort(result);
                return;
            }
            RCLCPP_INFO(this->get_logger(), "multi_mov_with_ext success.");
        }

        if (!wait_until_reached(converted_pos)) {
            RCLCPP_ERROR(this->get_logger(), "Timeout waiting for external axis joints to reach target.");
            result->error_code = -3;
            goal_handle->abort(result);
            return;
        }

        auto feedback = std::make_shared<FollowJointTrajectory::Feedback>();
        feedback->joint_names = goal->trajectory.joint_names;
        feedback->actual.positions = pt.positions;
        goal_handle->publish_feedback(feedback);

        if (goal_handle->is_canceling()) {
            result->error_code = -4;
            goal_handle->canceled(result);
            RCLCPP_WARN(this->get_logger(), "Goal canceled.");
            return;
        }
        {
            std::lock_guard<std::mutex> lock(mtx_);
            robot.disable_ext(0);
            robot.disable_ext(1);
            robot.disable_ext(2);
            robot.disable_ext(3);
        }

        RCLCPP_INFO(this->get_logger(), "Trajectory execution completed.");
        result->error_code = 0;
        goal_handle->succeed(result);
    }

    void pollStatus()
    {
        std::vector<double> positions = read_current_positions();

        if (positions.size() < 4) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 5000,
                "Failed to read 4 external axis joint positions.");
            return;
        }

        sensor_msgs::msg::JointState msg;
        msg.header.stamp = this->now();

        for (int i = 0; i < 4; ++i) {
            msg.name.push_back("body_j" + std::to_string(i + 1));

            double pos_converted = 0.0;
            if (i == 0) {
                // prismatic: mm -> m
                pos_converted = positions[i] / 1000.0;
            } else {
                // revolute: deg -> rad
                pos_converted = positions[i] * M_PI / 180.0;
            }

            msg.position.push_back(pos_converted);
        }

        RCLCPP_DEBUG(this->get_logger(),
                     "Publishing joint states: %.6f %.6f %.6f %.6f",
                     msg.position[0], msg.position[1], msg.position[2], msg.position[3]);
        
        joint_state_pub_->publish(msg);
    }

    // bool stop_ext_motion()
    // {
    //     // Best-effort stop for all 4 ext joints
    //     MultiMovInfoList stop_cmd{};
    //     stop_cmd.count = 4;

    //     for (int i = 0; i < 4; ++i) {
    //         stop_cmd.info[i].motion_unit_type = 1;
    //         stop_cmd.info[i].motion_unit_id   = i;
    //         stop_cmd.info[i].move_type        = JOINT_MOVE;
    //         stop_cmd.info[i].move_mode        = STOP;
    //     }

    //     errno_t ret = robot.multi_mov_with_ext(&stop_cmd, true);
    //     if (ret != ERR_SUCC) {
    //         RCLCPP_ERROR(this->get_logger(), "STOP command failed. ret=%d", ret);
    //         return false;
    //     }

    //     RCLCPP_WARN(this->get_logger(), "STOP command sent to 4 external axis joints.");
    //     return true;
    // }

    bool wait_until_reached(const std::vector<double>& target_positions)
    {
        const auto start = std::chrono::steady_clock::now();

        while (rclcpp::ok()) {
            std::vector<double> current_pos = read_current_positions();

            if (current_pos.size() < 4) {
                RCLCPP_WARN(this->get_logger(), "Could not read current positions while waiting.");
                return false;
            }

            bool all_close = true;
            for (size_t i = 0; i < 4; i++) {
                if (std::abs(current_pos[i] - target_positions[i]) > 0.5) {
                    all_close = false;
                    break;
                }
            }

            if (all_close) {
                return true;
            }

            if ((std::chrono::steady_clock::now() - start) >
                std::chrono::duration<double>(10)) // seconds
                {
                return false;
            }

            std::this_thread::sleep_for(200ms);
        }

        return false;
    }

    std::vector<double> read_current_positions()
    {
        std::lock_guard<std::mutex> lock(mtx_);

        ExtAxisStatusList status_list{};
        errno_t ret = robot.get_ext_status(&status_list);
        if (ret != ERR_SUCC) {
            RCLCPP_ERROR_THROTTLE(
                this->get_logger(), *this->get_clock(), 3000,
                "get_ext_status failed. ret=%d", ret);
            return {};
        }

        if (status_list.count < 4) {
            RCLCPP_WARN(this->get_logger(),
                        "get_ext_status returned count=%d, expected at least 4.", status_list.count);
            return {};
        }

        std::vector<double> pos(4, 0.0);
        for (int i = 0; i < 4; ++i) {
            pos[i] = status_list.status[i].pos_fdb;
        }

        return pos;
    }

    void stopService(
        const std::shared_ptr<std_srvs::srv::Empty::Request> /*req*/,
        std::shared_ptr<std_srvs::srv::Empty::Response> /*res*/)
    {
        std::lock_guard<std::mutex> lock(mtx_);
        stop_ext_motion();
        RCLCPP_INFO(this->get_logger(), "Stop command sent.");
    }
};

std::shared_ptr<KargoExtAxisServer> kargo_ext_action_node = nullptr;

bool KargoExtAxisServer::stop_ext_motion()
{
    // Best-effort stop for all 4 ext joints
    MultiMovInfoList stop_cmd{};
    stop_cmd.count = 4;

    for (int i = 0; i < 4; ++i) {
        stop_cmd.info[i].motion_unit_type = 1;
        stop_cmd.info[i].motion_unit_id   = i;
        stop_cmd.info[i].move_type        = JOINT_MOVE;
        stop_cmd.info[i].move_mode        = STOP;
    }

    errno_t ret = robot.multi_mov_with_ext(&stop_cmd, false);
    if (ret != ERR_SUCC) {
        RCLCPP_ERROR(this->get_logger(), "STOP command failed. ret=%d", ret);
        return false;
    }

    RCLCPP_WARN(this->get_logger(), "STOP command sent to 4 external axis joints.");
    return true;
}

void sigintHandler(int /*sig*/)
{
    if (kargo_ext_action_node) {
        // kargo_ext_action_node->stop_ext_motion_public();
        kargo_ext_action_node->stop_ext_motion();
        RCLCPP_WARN(kargo_ext_action_node->get_logger(), "SIGINT received. Stopping Kargo external axis...");
    }
    rclcpp::shutdown();
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    signal(SIGINT, sigintHandler);

    try {
        kargo_ext_action_node = std::make_shared<KargoExtAxisServer>();
        rclcpp::spin(kargo_ext_action_node);
    } catch (const std::exception &e) {
        fprintf(stderr, "Failed to start kargo_ext_axis_server: %s\n", e.what());
    }

    rclcpp::shutdown();
    return 0;
}