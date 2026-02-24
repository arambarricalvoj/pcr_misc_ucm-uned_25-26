#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

// Lanza la petición asíncrona.
// El callback se ejecutará cuando llegue la respuesta.
void requestTargetPositionAsync(
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client,
    std::function<void(const std_srvs::srv::Trigger::Response::SharedPtr)> callback,
    rclcpp::Logger logger);
