#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "tarea1/metrics_utils.hpp"

class ICCController : public rclcpp::Node
{
public:
    ICCController();

    // Llamada al servicio desde fuera del constructor
    bool callTargetService(rclcpp::Node::SharedPtr node);

private:
    // Callbacks
    void robotPoseCallback(const geometry_msgs::msg::Pose::SharedPtr msg);
    void controlLoop();

    // Variables internas
    double xt_, yt_;
    bool target_received_ = false;
    bool pose_received_   = false;

    geometry_msgs::msg::Pose robot_pose_;

    // ROS interfaces
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Parámetros
    double kv_, vmax_, wmax_;
    int controller_type_;

    // Métricas
    std::unique_ptr<MetricsLogger> metrics_;
    double time_elapsed_ = 0.0;
};
