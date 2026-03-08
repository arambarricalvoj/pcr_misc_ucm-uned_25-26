#include "rclcpp/rclcpp.hpp"
#include "tarea1/pose_controller.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<PoseController>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
