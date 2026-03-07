#pragma once

#include <array>
#include <vector>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "geometry_msgs/msg/twist.hpp"

class BraitenbergController : public rclcpp::Node
{
public:
    BraitenbergController();

private:
    void controlLoop();
    void irCallback(size_t idx, sensor_msgs::msg::Range::SharedPtr msg);

    // parámetros y estado
    double wheel_radius_;
    double wheel_base_;
    double v0_;
    double vmin_;
    double vmax_;
    double max_detection_dist_;
    double max_range_default_;
    double control_hz_;

    std::array<double,8> braitenbergL_;
    std::array<double,8> braitenbergR_;

    std::array<double,8> ir_ranges_;
    std::array<rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr,8> ir_subs_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};
