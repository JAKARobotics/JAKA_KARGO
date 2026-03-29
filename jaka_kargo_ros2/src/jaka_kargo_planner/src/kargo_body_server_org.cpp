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

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

class KargoExtAxisServer : public rclcpp::Node
{
public:
    using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
    using GoalHandleFollowJointTrajectory = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;

    KargoExtAxisServer()
        : Node("kargo_ext_axis_server")
    {
        robot_ip_       = this->declare_parameter<std::string>("robot_ip", "10.5.5.100");
        edg_broadcast_  = this->declare_parameter<std::string>("edg_broadcast", "10.5.5.255");
        ext_id_         = this->declare_parameter<int>("ext_id", 0);
        joint_name_     = this->declare_parameter<std::string>("joint_name", "kargo_ext_axis");
        axis_type_      = this->declare_parameter<std::string>("axis_type", "prismatic"); // prismatic | revolute
        vel_            = this->declare_parameter<double>("vel", 80.0);
        acc_            = this->declare_parameter<double>("acc", 80.0);
        tolerance_      = this->declare_parameter<double>("tolerance", 0.5); // SDK units: mm or deg
        poll_rate_hz_   = this->declare_parameter<double>("poll_rate_hz", 5.0);
        wait_timeout_s_ = this->declare_parameter<double>("wait_timeout_s", 10.0);

        if (!init_robot()) {
            throw std::runtime_error("Failed to initialize Kargo robot/ext axis.");
        }

        action_server_ = rclcpp_action::create_server<FollowJointTrajectory>(
            this,
            "/jaka_kargo_ext_axis_controller/follow_joint_trajectory",
            std::bind(&KargoExtAxisServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&KargoExtAxisServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&KargoExtAxisServer::handle_accepted, this, std::placeholders::_1));

        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/joint_states", 10);

        stop_srv_ = this->create_service<std_srvs::srv::Empty>(
            "kargo_ext_axis/stop",
            std::bind(&KargoExtAxisServer::stopService, this, _1, _2));

        poll_thread_ = std::thread([this]() {
            rclcpp::Rate rate(poll_rate_hz_);
            while (rclcpp::ok() && running_) {
                this->pollStatus();
                rate.sleep();
            }
        });

        RCLCPP_INFO(this->get_logger(), "Kargo external axis FollowJointTrajectory action server ready!");
    }

    ~KargoExtAxisServer()
    {
        running_ = false;
        if (poll_thread_.joinable()) {
            poll_thread_.join();
        }

        std::lock_guard<std::mutex> lock(robot_mtx_);

        // Best-effort stop and cleanup
        stop_ext_motion();
        robot_.disable_ext(ext_id_);
        robot_.login_out();
    }

    void stop_ext_motion_public()
    {
        std::lock_guard<std::mutex> lock(robot_mtx_);
        stop_ext_motion();
    }

private:
    rclcpp_action::Server<FollowJointTrajectory>::SharedPtr action_server_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_srv_;

    std::thread poll_thread_;
    std::mutex robot_mtx_;
    bool running_{true};

    JAKAZuRobot robot_;

    std::string robot_ip_;
    std::string edg_broadcast_;
    int ext_id_{0};
    std::string joint_name_;
    std::string axis_type_;     // "prismatic" or "revolute"
    double vel_{80.0};
    double acc_{80.0};
    double tolerance_{0.5};     // mm if prismatic, deg if revolute
    double poll_rate_hz_{5.0};
    double wait_timeout_s_{10.0};

    bool init_robot()
    {
        std::lock_guard<std::mutex> lock(robot_mtx_);

        errno_t ret = robot_.login_in(robot_ip_.c_str());
        if (ret != ERR_SUCC) {
            RCLCPP_ERROR(this->get_logger(), "login_in failed. ret=%d", ret);
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "login_in success.");

        robot_.edg_init(1, edg_broadcast_.c_str());
        RCLCPP_INFO(this->get_logger(), "edg_init done.");

        ret = robot_.power_on();
        if (ret != ERR_SUCC) {
            RCLCPP_ERROR(this->get_logger(), "power_on failed. ret=%d", ret);
            return false;
        }
        std::this_thread::sleep_for(8000ms);

        ret = robot_.enable_robot();
        if (ret != ERR_SUCC) {
            RCLCPP_ERROR(this->get_logger(), "enable_robot failed. ret=%d", ret);
            return false;
        }
        std::this_thread::sleep_for(4000ms);

        ret = robot_.enable_ext(0);
        if (ret != ERR_SUCC) {
            RCLCPP_ERROR(this->get_logger(), "enable_ext failed. ret=%d", ret);
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "External axis enabled.");
        return true;
    }

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const FollowJointTrajectory::Goal> goal)
    {
        (void)uuid;

        if (!goal || goal->trajectory.points.empty()) {
            RCLCPP_WARN(this->get_logger(), "Rejected empty trajectory.");
            return rclcpp_action::GoalResponse::REJECT;
        }

        const auto &pt = goal->trajectory.points.back();
        if (pt.positions.empty()) {
            RCLCPP_WARN(this->get_logger(), "Rejected trajectory with empty positions.");
            return rclcpp_action::GoalResponse::REJECT;
        }

        RCLCPP_INFO(this->get_logger(), "Received new trajectory goal.");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle)
    {
        (void)goal_handle;
        RCLCPP_WARN(this->get_logger(), "Received request to cancel goal.");

        {
            std::lock_guard<std::mutex> lock(robot_mtx_);
            if (!stop_ext_motion()) {
                RCLCPP_WARN(this->get_logger(), "Failed to stop ext axis with STOP move mode.");
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

        if (pt.positions.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Trajectory point has no positions.");
            result->error_code = -1;
            goal_handle->abort(result);
            return;
        }

        const double ros_target = pt.positions[0];
        const double sdk_target = ros_to_sdk(ros_target);

        RCLCPP_INFO(this->get_logger(),
                    "Final target in ROS units: %.6f, converted to SDK units: %.3f",
                    ros_target, sdk_target);

        {
            std::lock_guard<std::mutex> lock(robot_mtx_);

            errno_t ret = robot_.enable_ext(ext_id_);
            if (ret != ERR_SUCC) {
                RCLCPP_ERROR(this->get_logger(), "enable_ext failed before motion. ret=%d", ret);
                result->error_code = -10;
                goal_handle->abort(result);
                return;
            }

            MultiMovInfoList cmd{};
            cmd.count = 1;
            cmd.info[0].motion_unit_type = 1;     // ext axis
            cmd.info[0].motion_unit_id   = ext_id_;
            cmd.info[0].move_type        = JOINT_MOVE;
            cmd.info[0].move_mode        = ABS;
            cmd.info[0].movej_info.end_pos[0] = sdk_target;
            cmd.info[0].movej_info.j_vel = vel_;
            cmd.info[0].movej_info.j_acc = acc_;
            cmd.info[0].movej_info.j_jerk = 0.0;
            cmd.info[0].movej_info.blend_tol = 0.0;

            ret = robot_.multi_mov_with_ext(&cmd, true);
            if (ret != ERR_SUCC) {
                RCLCPP_ERROR(this->get_logger(), "multi_mov_with_ext failed. ret=%d", ret);
                result->error_code = -2;
                goal_handle->abort(result);
                return;
            }
        }

        // Since SDK call was blocking, in many cases this may already be enough.
        // But we still verify final status.
        if (!wait_until_reached(sdk_target)) {
            RCLCPP_ERROR(this->get_logger(), "Timeout waiting for ext axis to reach target.");
            result->error_code = -3;
            goal_handle->abort(result);
            return;
        }

        auto feedback = std::make_shared<FollowJointTrajectory::Feedback>();
        feedback->joint_names = {joint_name_};
        feedback->actual.positions = {ros_target};
        goal_handle->publish_feedback(feedback);

        if (goal_handle->is_canceling()) {
            result->error_code = -4;
            goal_handle->canceled(result);
            RCLCPP_WARN(this->get_logger(), "Goal canceled.");
            return;
        }

        result->error_code = 0;
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Trajectory execution completed.");
    }

    void pollStatus()
    {
        ExtAxisStatus status{};
        if (!read_ext_status(status)) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 5000,
                "Failed to read external axis status.");
            return;
        }

        sensor_msgs::msg::JointState msg;
        msg.header.stamp = this->now();
        msg.name.push_back(joint_name_);
        msg.position.push_back(sdk_to_ros(status.pos_fdb));

        joint_state_pub_->publish(msg);

        RCLCPP_DEBUG(this->get_logger(),
                     "Ext axis status: pos_cmd=%.3f pos_fdb=%.3f inpos=%d enabled=%d",
                     status.pos_cmd, status.pos_fdb, status.is_inpos, status.is_enabled);
    }

    bool read_ext_status(ExtAxisStatus &out_status)
    {
        std::lock_guard<std::mutex> lock(robot_mtx_);

        ExtAxisStatusList list{};
        errno_t ret = robot_.get_ext_status(&list, ext_id_);
        if (ret != ERR_SUCC) {
            RCLCPP_ERROR_THROTTLE(
                this->get_logger(), *this->get_clock(), 3000,
                "get_ext_status failed. ret=%d", ret);
            return false;
        }

        // Depending on SDK behavior:
        // - some APIs may return one entry when ext_id is specified
        // - others may still return all entries
        if (list.count <= 0) {
            RCLCPP_WARN(this->get_logger(), "get_ext_status returned empty list.");
            return false;
        }

        // safest choice: if ext_id_ is within count use it, otherwise use first
        int idx = 0;
        if (ext_id_ >= 0 && ext_id_ < list.count) {
            idx = ext_id_;
        }

        out_status = list.status[idx];
        return true;
    }

    bool wait_until_reached(double target_sdk)
    {
        const auto start = std::chrono::steady_clock::now();

        while (rclcpp::ok()) {
            ExtAxisStatus st{};
            if (!read_ext_status(st)) {
                return false;
            }

            const double err = std::abs(st.pos_fdb - target_sdk);

            if (st.is_inpos || err <= tolerance_) {
                return true;
            }

            if ((std::chrono::steady_clock::now() - start) >
                std::chrono::duration<double>(wait_timeout_s_)) {
                return false;
            }

            std::this_thread::sleep_for(200ms);
        }

        return false;
    }

    bool stop_ext_motion()
    {
        MultiMovInfoList stop_cmd{};
        stop_cmd.count = 1;
        stop_cmd.info[0].motion_unit_type = 1;   // ext axis
        stop_cmd.info[0].motion_unit_id   = ext_id_;
        stop_cmd.info[0].move_type        = JOINT_MOVE;
        stop_cmd.info[0].move_mode        = STOP;

        errno_t ret = robot_.multi_mov_with_ext(&stop_cmd, false);
        if (ret != ERR_SUCC) {
            RCLCPP_ERROR(this->get_logger(), "STOP command failed. ret=%d", ret);
            return false;
        }

        RCLCPP_WARN(this->get_logger(), "STOP command sent to external axis.");
        return true;
    }

    double ros_to_sdk(double ros_pos) const
    {
        if (axis_type_ == "prismatic") {
            // ROS meters -> SDK mm
            return ros_pos * 1000.0;
        } else if (axis_type_ == "revolute") {
            // ROS radians -> SDK degrees
            return ros_pos * 180.0 / M_PI;
        } else {
            // fallback: pass-through
            return ros_pos;
        }
    }

    double sdk_to_ros(double sdk_pos) const
    {
        if (axis_type_ == "prismatic") {
            // SDK mm -> ROS meters
            return sdk_pos / 1000.0;
        } else if (axis_type_ == "revolute") {
            // SDK degrees -> ROS radians
            return sdk_pos * M_PI / 180.0;
        } else {
            // fallback: pass-through
            return sdk_pos;
        }
    }

    void stopService(
        const std::shared_ptr<std_srvs::srv::Empty::Request> /*req*/,
        std::shared_ptr<std_srvs::srv::Empty::Response> /*res*/)
    {
        std::lock_guard<std::mutex> lock(robot_mtx_);
        if (stop_ext_motion()) {
            RCLCPP_INFO(this->get_logger(), "Stop command sent.");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Stop command failed.");
        }
    }
};

std::shared_ptr<KargoExtAxisServer> g_kargo_ext_node = nullptr;

void sigintHandler(int /*sig*/)
{
    if (g_kargo_ext_node) {
        g_kargo_ext_node->stop_ext_motion_public();
        RCLCPP_WARN(g_kargo_ext_node->get_logger(), "SIGINT received, stopping Kargo external axis.");
    }
    rclcpp::shutdown();
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    signal(SIGINT, sigintHandler);

    try {
        g_kargo_ext_node = std::make_shared<KargoExtAxisServer>();
        rclcpp::spin(g_kargo_ext_node);
    } catch (const std::exception &e) {
        fprintf(stderr, "Failed to start kargo_ext_axis_server: %s\n", e.what());
    }

    rclcpp::shutdown();
    return 0;
}