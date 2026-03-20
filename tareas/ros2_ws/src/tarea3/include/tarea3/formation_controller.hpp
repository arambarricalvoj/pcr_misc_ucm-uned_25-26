#pragma once

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"

class FormationController : public rclcpp::Node
{
public:
    FormationController();

private:
    void masterCallback(const geometry_msgs::msg::Pose::SharedPtr msg);
    void slave1Callback(const geometry_msgs::msg::Pose::SharedPtr msg);
    void slave2Callback(const geometry_msgs::msg::Pose::SharedPtr msg);

    void controlLoop();

    double extractYaw(const geometry_msgs::msg::Pose &pose);
    std::pair<double,double> transform(double xm, double ym, double th, double dx, double dy);

    void publishCmd(
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub,
        const geometry_msgs::msg::Pose &pose,
        double xd, double yd);

    geometry_msgs::msg::Pose master_pose_;
    geometry_msgs::msg::Pose slave1_pose_;
    geometry_msgs::msg::Pose slave2_pose_;

    bool master_received_ = false;
    bool slave1_received_ = false;
    bool slave2_received_ = false;

    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr master_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr slave1_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr slave2_sub_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr slave1_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr slave2_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    double kf_;
};
