#include "tarea3/formation_controller.hpp"
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;


FormationController::FormationController()
: Node("formation_controller")
{
    kf_ = declare_parameter("kf", 1.0);

    master_sub_ = create_subscription<geometry_msgs::msg::Pose>(
        "/master_khepera_iv/robot_pose", 10,
        std::bind(&FormationController::masterCallback, this, std::placeholders::_1));

    slave1_sub_ = create_subscription<geometry_msgs::msg::Pose>(
        "/slave_1_khepera_iv/robot_pose", 10,
        std::bind(&FormationController::slave1Callback, this, std::placeholders::_1));

    slave2_sub_ = create_subscription<geometry_msgs::msg::Pose>(
        "/slave_2_khepera_iv/robot_pose", 10,
        std::bind(&FormationController::slave2Callback, this, std::placeholders::_1));

    slave1_pub_ = create_publisher<geometry_msgs::msg::Twist>(
        "/slave_1_khepera_iv/cmd_vel", 10);

    slave2_pub_ = create_publisher<geometry_msgs::msg::Twist>(
        "/slave_2_khepera_iv/cmd_vel", 10);

    timer_ = create_wall_timer(
        50ms,
        std::bind(&FormationController::controlLoop, this));
}

void FormationController::masterCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
{
    master_pose_ = *msg;
    master_received_ = true;
}

void FormationController::slave1Callback(const geometry_msgs::msg::Pose::SharedPtr msg)
{
    slave1_pose_ = *msg;
    slave1_received_ = true;
}

void FormationController::slave2Callback(const geometry_msgs::msg::Pose::SharedPtr msg)
{
    slave2_pose_ = *msg;
    slave2_received_ = true;
}

void FormationController::controlLoop()
{
    if (!master_received_ || !slave1_received_ || !slave2_received_) return;

    double xm = master_pose_.position.x;
    double ym = master_pose_.position.y;
    double th = extractYaw(master_pose_);

    auto [x1d, y1d] = transform(xm, ym, th, -0.00, -0.40);
    auto [x2d, y2d] = transform(xm, ym, th, -0.00, 0.40);

    publishCmd(slave1_pub_, slave1_pose_, x1d, y1d);
    publishCmd(slave2_pub_, slave2_pose_, x2d, y2d);
}

double FormationController::extractYaw(const geometry_msgs::msg::Pose &pose)
{
    double x = pose.orientation.x;
    double y = pose.orientation.y;
    double z = pose.orientation.z;
    double w = pose.orientation.w;

    return std::atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z));
}

std::pair<double,double> FormationController::transform(double xm, double ym, double th, double dx, double dy)
{
    double xd = xm + dx*std::cos(th) - dy*std::sin(th);
    double yd = ym + dx*std::sin(th) + dy*std::cos(th);
    return {xd, yd};
}

void FormationController::publishCmd(
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub,
    const geometry_msgs::msg::Pose &pose,
    double xd, double yd)
{
    double xs = pose.position.x;
    double ys = pose.position.y;

    double ex = xd - xs;
    double ey = yd - ys;

    double dist = std::sqrt(ex*ex + ey*ey);

    // Ángulo deseado hacia el punto objetivo
    double theta_d = std::atan2(ey, ex);

    // Orientación actual del esclavo
    double theta_s = extractYaw(pose);

    // Error angular
    double e_theta = theta_d - theta_s;

    // Normalizar a [-pi, pi]
    while (e_theta > M_PI)  e_theta -= 2*M_PI;
    while (e_theta < -M_PI) e_theta += 2*M_PI;

    geometry_msgs::msg::Twist cmd;

    // Control proporcional con límites
    cmd.linear.x  = std::clamp(kf_ * dist, -0.1, 0.1);
    cmd.angular.z = std::clamp(kf_ * e_theta, -1.0, 1.0);

    pub->publish(cmd);
}

