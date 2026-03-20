#include "rclcpp/rclcpp.hpp"
#include "tarea3/formation_controller.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<FormationController>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
