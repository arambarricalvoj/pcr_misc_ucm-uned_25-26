#pragma once

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"

class FormationController : public rclcpp::Node
{
public:
    FormationController();

private:
    // Callbacks de suscripción
    void masterCallback(const geometry_msgs::msg::Pose::SharedPtr msg);
    void slave1Callback(const geometry_msgs::msg::Pose::SharedPtr msg);
    void slave2Callback(const geometry_msgs::msg::Pose::SharedPtr msg);

    // Bucle principal de control
    void controlLoop();

    // Utilidades
    double extractYaw(const geometry_msgs::msg::Pose &pose);

    // Nueva función basada en el paper (ecuación 12)
    std::pair<double,double> getTargetPosition(
        double xL, double yL, double thetaL,
        double r, double beta_deg);

    // Control de movimiento
    void publishCmd(
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub,
        const geometry_msgs::msg::Pose &pose,
        double xd, double yd);

    // Últimas poses recibidas
    geometry_msgs::msg::Pose master_pose_;
    geometry_msgs::msg::Pose slave1_pose_;
    geometry_msgs::msg::Pose slave2_pose_;

    // Flags de recepción
    bool master_received_ = false;
    bool slave1_received_ = false;
    bool slave2_received_ = false;

    // Suscripciones
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr master_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr slave1_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr slave2_sub_;

    // Publicadores
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr slave1_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr slave2_pub_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // Ganancia de formación
    double kf_;
};
