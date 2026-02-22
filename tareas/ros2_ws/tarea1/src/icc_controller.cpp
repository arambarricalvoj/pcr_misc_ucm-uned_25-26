#include <memory>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_srvs/srv/trigger.hpp>

class ICCController : public rclcpp::Node
{
public:
    ICCController() : Node("icc_controller")
    {
        // Parámetros del controlador
        kv_ = declare_parameter("kv", 0.5);
        vmax_ = declare_parameter("vmax", 0.15);
        wmax_ = declare_parameter("wmax", 1.0);

        // Suscripción a la pose del robot
        robot_pose_sub_ = create_subscription<geometry_msgs::msg::Pose>(
            "/robot_pose", 10,
            std::bind(&ICCController::robotPoseCallback, this, std::placeholders::_1));

        // Publicador de velocidades
        cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Cliente del servicio del target
        target_client_ = create_client<std_srvs::srv::Trigger>("/get_target_pose");

        // Timer del controlador
        timer_ = create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&ICCController::controlLoop, this));
    }

private:
    geometry_msgs::msg::Pose robot_pose_;
    bool pose_received_ = false;

    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr robot_pose_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr target_client_;
    rclcpp::TimerBase::SharedPtr timer_;

    double kv_, vmax_, wmax_;

    void robotPoseCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        robot_pose_ = *msg;
        pose_received_ = true;
    }

    void controlLoop()
    {
        if (!pose_received_)
            return;

        // Llamar al servicio del target
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto future = target_client_->async_send_request(request);

        if (future.wait_for(std::chrono::milliseconds(10)) != std::future_status::ready)
            return;

        auto response = future.get();
        if (!response->success)
            return;

        // El servicio devuelve un string con un diccionario Python
        // Ejemplo: "{'position': {'x': 1.0, 'y': 2.0}}"
        double xt, yt;
        sscanf(response->message.c_str(),
               "{'position': {'x': %lf, 'y': %lf}}",
               &xt, &yt);

        // Pose del robot
        double xr = robot_pose_.position.x;
        double yr = robot_pose_.position.y;

        // Orientación del robot (convertir quaternion a yaw)
        double qw = robot_pose_.orientation.w;
        double qx = robot_pose_.orientation.x;
        double qy = robot_pose_.orientation.y;
        double qz = robot_pose_.orientation.z;

        double theta_r = std::atan2(2*(qw*qz + qx*qy),
                                    1 - 2*(qy*qy + qz*qz));

        // Distancia al objetivo
        double dx = xt - xr;
        double dy = yt - yr;
        double d = std::sqrt(dx*dx + dy*dy);

        // Ángulo deseado
        double theta_d = std::atan2(dy, dx);

        // Error angular normalizado
        double e_theta = theta_d - theta_r;
        e_theta = std::atan2(std::sin(e_theta), std::cos(e_theta));

        // Control lineal
        double v = kv_ * d;
        if (v > vmax_) v = vmax_;

        // Control angular ICC
        double w = wmax_ * std::sin(e_theta);

        // Publicar Twist
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = v;
        cmd.angular.z = w;
        cmd_pub_->publish(cmd);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ICCController>());
    rclcpp::shutdown();
    return 0;
}
