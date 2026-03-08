#pragma once

#include <array>
#include <vector>
#include <memory>
#include <string>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "std_srvs/srv/trigger.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "tarea2/metrics_utils.hpp"
#include "tarea2/braitenberg_repulsion.hpp"
#include "tarea2/controllers.hpp"
#include "interfaces/action/execute_trajectory.hpp"

class BraitenbergTargetController : public rclcpp::Node
{
public:
    using ExecuteTrajectory = interfaces::action::ExecuteTrajectory;
    using GoalHandleExecuteTrajectory = rclcpp_action::ServerGoalHandle<ExecuteTrajectory>;

    BraitenbergTargetController();

private:
    // -------------------------
    // ACTION SERVER CALLBACKS
    // -------------------------
    rclcpp_action::GoalResponse handleGoal(
        const rclcpp_action::GoalUUID &,
        std::shared_ptr<const ExecuteTrajectory::Goal>);

    rclcpp_action::CancelResponse handleCancel(
        const std::shared_ptr<GoalHandleExecuteTrajectory> goal_handle);

    void handleAccepted(
        const std::shared_ptr<GoalHandleExecuteTrajectory> goal_handle);

    // -------------------------
    // SERVICE RESPONSE CALLBACK (ASÍNCRONO)
    // -------------------------
    void onTargetReceived(
        const std_srvs::srv::Trigger::Response::SharedPtr response);

    // -------------------------
    // ROS CALLBACKS
    // -------------------------
    void irCallback(size_t idx, sensor_msgs::msg::Range::SharedPtr msg);
    void robotPoseCallback(const geometry_msgs::msg::Pose::SharedPtr msg);
    void controlLoop();

    // -------------------------
    // UTIL
    // -------------------------
    std::unique_ptr<BraitenbergRepulsion> repulsion_;
    std::unique_ptr<BaseController> controller_;

    // -------------------------
    // ESTADO INTERNO
    // -------------------------
    double xt_, yt_;
    bool target_received_ = false;
    bool pose_received_   = false;
    double time_elapsed_ = 0.0;

    std::array<double,8> ir_ranges_;
    std::array<rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr,8> ir_subs_;

    geometry_msgs::msg::Pose robot_pose_;

    // -------------------------
    // ROS INTERFACES
    // -------------------------
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Action server
    rclcpp_action::Server<ExecuteTrajectory>::SharedPtr action_server_;
    std::shared_ptr<GoalHandleExecuteTrajectory> goal_handle_;

    // -------------------------
    // PARÁMETROS
    // -------------------------
    double kp_;
    int controller_type_;
    double wheel_radius_;
    double wheel_base_;
    double v0_;
    double vmin_;
    double vmax_;
    double wmax_;
    double max_detection_dist_;
    double max_range_default_;
    double control_hz_;
    double d_safe_ = 0.0;

    std::array<double,8> GL_;
    std::array<double,8> GR_;

    // -------------------------
    // MÉTRICAS
    // -------------------------
    std::unique_ptr<MetricsLogger> metrics_;
    
};
