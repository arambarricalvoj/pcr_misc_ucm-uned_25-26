#include "tarea1/service_utils.hpp"

void requestTargetPositionAsync(
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client,
    std::function<void(const std_srvs::srv::Trigger::Response::SharedPtr)> callback,
    rclcpp::Logger logger)
{
    using namespace std::chrono_literals;

    RCLCPP_INFO(logger, "Esperando servicio /get_target_pose...");

    if (!client->wait_for_service(1s)) {
        RCLCPP_ERROR(logger, "Servicio no disponible.");
        callback(nullptr);
        return;
    }

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

    // Petición asíncrona
    client->async_send_request(
        request,
        [callback, logger](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future)
        {
            try {
                auto response = future.get();
                callback(response);
            }
            catch (...) {
                RCLCPP_ERROR(logger, "Error al recibir respuesta del servicio.");
                callback(nullptr);
            }
        }
    );
}
