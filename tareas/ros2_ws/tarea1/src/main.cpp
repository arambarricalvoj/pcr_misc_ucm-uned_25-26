#include "rclcpp/rclcpp.hpp"
#include "tarea1/icc_controller.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ICCController>();

    node->callTargetService(node);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
