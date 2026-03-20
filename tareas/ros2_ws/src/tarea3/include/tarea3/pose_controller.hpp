#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "std_srvs/srv/trigger.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "tarea3/metrics_utils.hpp"
#include "interfaces/action/execute_trajectory.hpp"

class PoseController : public rclcpp::Node
{
public:
    using ExecuteTrajectory = interfaces::action::ExecuteTrajectory;
    using GoalHandleExecuteTrajectory = rclcpp_action::ServerGoalHandle<ExecuteTrajectory>;

    PoseController();

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
    void robotPoseCallback(const geometry_msgs::msg::Pose::SharedPtr msg);
    void controlLoop();

    // -------------------------
    // ESTADO INTERNO
    // -------------------------
    double xt_, yt_;
    bool target_received_ = false;
    bool pose_received_   = false;

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
    double kp_, vmax_, wmax_;
    int controller_type_;

    // -------------------------
    // MÉTRICAS
    // -------------------------
    std::unique_ptr<MetricsLogger> metrics_;
    double time_elapsed_ = 0.0;
};
