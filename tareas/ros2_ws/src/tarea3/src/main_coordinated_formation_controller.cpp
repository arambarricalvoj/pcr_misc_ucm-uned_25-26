#include "rclcpp/rclcpp.hpp"
#include "tarea3/coordinated_formation_controller.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<CoordinatedFormationController>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
