#pragma once

#include <array>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "std_srvs/srv/trigger.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/range.hpp"   // ← NUEVO

#include "tarea2/metrics_utils.hpp"
#include "interfaces/action/execute_trajectory.hpp"

#include "tarea2/braitenberg.hpp"      // ← NUEVO (tu módulo puro)

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
    double kp_, vmax_, wmax_, ks_, k_tan_;
    int controller_type_;

    // Parámetros Braitenberg (NUEVO)
    double k_front_;   // freno reactivo
    double k_turn_;    // giro reactivo

    // -------------------------
    // SENSORES IR (NUEVO)
    // -------------------------
    std::array<double, 8> ir_ranges_;   // lecturas IR
    std::array<
        rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr,
        8
    > ir_subs_;                          // suscriptores IR

    // -------------------------
    // MÉTRICAS
    // -------------------------
    std::unique_ptr<MetricsLogger> metrics_;
    double time_elapsed_ = 0.0;

    // -------------------------
    // UTILIDAD
    // -------------------------
    double normalizeAngle(double a)
    {
        while (a > M_PI)  a -= 2.0 * M_PI;
        while (a < -M_PI) a += 2.0 * M_PI;
        return a;
    }
};
