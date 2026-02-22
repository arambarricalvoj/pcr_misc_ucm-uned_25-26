#include "tarea1/service_utils.hpp"

bool getTargetPosition(
    rclcpp::Node::SharedPtr node,
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client,
    double &xt, double &yt,
    rclcpp::Logger logger)
{
    using namespace std::chrono_literals;

    RCLCPP_INFO(logger, "Esperando servicio /get_target_pose...");

    while (!client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(logger, "Interrumpido mientras esperaba el servicio.");
            return false;
        }
        RCLCPP_INFO(logger, "Servicio no disponible, esperando...");
    }

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future = client->async_send_request(request);

    if (rclcpp::spin_until_future_complete(node, future)
        != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(logger, "No se recibió respuesta del servicio.");
        return false;
    }

    auto response = future.get();

    int parsed = sscanf(response->message.c_str(),
                        "{\"position\":{\"x\":%lf,\"y\":%lf",
                        &xt, &yt);

    if (parsed != 2) {
        RCLCPP_ERROR(logger, "No se pudo parsear la respuesta.");
        return false;
    }

    RCLCPP_INFO(logger, "Objetivo recibido: x=%.3f y=%.3f", xt, yt);
    return true;
}
