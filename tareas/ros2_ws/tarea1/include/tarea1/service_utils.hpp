#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

bool getTargetPosition(
    rclcpp::Node::SharedPtr node,
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client,
    double &xt, double &yt,
    rclcpp::Logger logger);
