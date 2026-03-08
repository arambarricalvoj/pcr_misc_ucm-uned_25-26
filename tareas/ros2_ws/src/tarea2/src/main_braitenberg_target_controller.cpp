#include "rclcpp/rclcpp.hpp"
#include "tarea2/braitenberg_target_controller.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<BraitenbergTargetController>();

    // node->callTargetService(node);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
