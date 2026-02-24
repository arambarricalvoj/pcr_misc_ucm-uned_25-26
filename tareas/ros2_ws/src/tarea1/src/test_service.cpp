#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

#include <chrono>
#include <memory>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  // Crear nodo
  auto node = rclcpp::Node::make_shared("test_service_node");

  // Crear cliente del servicio
  auto client = node->create_client<std_srvs::srv::Trigger>("/get_target_pose");

  RCLCPP_INFO(node->get_logger(), "Esperando servicio /get_target_pose...");

  // Esperar a que el servicio esté disponible
  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(),
                   "Interrumpido mientras esperaba el servicio. Saliendo.");
      return 0;
    }
    RCLCPP_INFO(node->get_logger(),
                "Servicio no disponible, esperando otra vez...");
  }

  // Crear petición
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

  RCLCPP_INFO(node->get_logger(), "Llamando al servicio...");

  // Enviar petición
  auto future = client->async_send_request(request);

  // EXACTAMENTE como el ejemplo oficial:
  if (rclcpp::spin_until_future_complete(node, future)
      == rclcpp::FutureReturnCode::SUCCESS)
  {
    auto response = future.get();

    RCLCPP_INFO(node->get_logger(), "success = %s",
                response->success ? "true" : "false");

    RCLCPP_INFO(node->get_logger(), "message = %s",
                response->message.c_str());
  }
  else
  {
    RCLCPP_ERROR(node->get_logger(),
                 "Fallo al llamar al servicio /get_target_pose");
  }

  rclcpp::shutdown();
  return 0;
}
