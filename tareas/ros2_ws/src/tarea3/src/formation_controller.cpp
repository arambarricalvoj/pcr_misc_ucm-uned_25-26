#include "tarea3/formation_controller.hpp"
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

FormationController::FormationController()
: Node("formation_controller")
{
    kf_ = declare_parameter("kf", 1.0);
    formation_type_ = declare_parameter("formation_type", "cross");

    // Suscripciones
    master_sub_ = create_subscription<geometry_msgs::msg::Pose>(
        "/master_khepera_iv/robot_pose", 10,
        std::bind(&FormationController::masterCallback, this, std::placeholders::_1));

    slave1_sub_ = create_subscription<geometry_msgs::msg::Pose>(
        "/slave_1_khepera_iv/robot_pose", 10,
        std::bind(&FormationController::slave1Callback, this, std::placeholders::_1));

    slave2_sub_ = create_subscription<geometry_msgs::msg::Pose>(
        "/slave_2_khepera_iv/robot_pose", 10,
        std::bind(&FormationController::slave2Callback, this, std::placeholders::_1));

    slave3_sub_ = create_subscription<geometry_msgs::msg::Pose>(
        "/slave_3_khepera_iv/robot_pose", 10,
        std::bind(&FormationController::slave3Callback, this, std::placeholders::_1));

    // Publicadores
    slave1_pub_ = create_publisher<geometry_msgs::msg::Twist>(
        "/slave_1_khepera_iv/cmd_vel", 10);

    slave2_pub_ = create_publisher<geometry_msgs::msg::Twist>(
        "/slave_2_khepera_iv/cmd_vel", 10);

    slave3_pub_ = create_publisher<geometry_msgs::msg::Twist>(
        "/slave_3_khepera_iv/cmd_vel", 10);


    // Timer
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

void FormationController::slave3Callback(const geometry_msgs::msg::Pose::SharedPtr msg)
{
    slave3_pose_ = *msg;
    slave3_received_ = true;
}


void FormationController::controlLoop()
{
    if (!master_received_ || !slave1_received_ || !slave2_received_ || !slave3_received_) return;

    double xL = master_pose_.position.x;
    double yL = master_pose_.position.y;
    double thetaL = extractYaw(master_pose_);

    // Obtener ángulos según la formación
    auto [beta1, beta2, beta3] = getAnglesForFormation();

    // Distancia fija
    double r = 0.40;
    double r3 = (formation_type_ == "line") ? 0.80 : 0.40;

    // Posiciones deseadas
    auto [x1d, y1d] = getTargetPosition(xL, yL, thetaL, r, beta1);
    auto [x2d, y2d] = getTargetPosition(xL, yL, thetaL, r, beta2);
    auto [x3d, y3d] = getTargetPosition(xL, yL, thetaL, r3, beta3);

    // Publicar comandos
    publishCmd(slave1_pub_, slave1_pose_, x1d, y1d);
    publishCmd(slave2_pub_, slave2_pose_, x2d, y2d);
    publishCmd(slave3_pub_, slave3_pose_, x3d, y3d);
}


std::pair<double,double> FormationController::getTargetPosition(
    double xL, double yL, double thetaL, double r, double beta_deg)
{
    double beta = beta_deg * M_PI / 180.0;

    double xp = xL + r * std::cos(beta + thetaL);
    double yp = yL + r * std::sin(beta + thetaL);

    return {xp, yp};
}

double FormationController::extractYaw(const geometry_msgs::msg::Pose &pose)
{
    double x = pose.orientation.x;
    double y = pose.orientation.y;
    double z = pose.orientation.z;
    double w = pose.orientation.w;

    return std::atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z));
}

std::tuple<double,double,double> FormationController::getAnglesForFormation()
{
    if (formation_type_ == "cross") {
        // T: izquierda, derecha, delante
        return { 90.0, -90.0, 180.0 };
    }
    else if (formation_type_ == "circle") {
        // Y: izquierda, derecha, detrás
        return { 120.0, -120.0, 180.0 };
    }
    else if (formation_type_ == "line") {
        // línea: izquierda, derecha, más derecha
        return { 90.0, -90.0, -90.0 };
    }

    // por defecto, cruz
    return { 90.0, -90.0, 0.0 };
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

    double theta_s = extractYaw(pose);
    double theta_L = extractYaw(master_pose_);   // orientación del líder

    geometry_msgs::msg::Twist cmd;

    // ---------------------------
    // 1) CONTROL DE POSICIÓN
    // ---------------------------
    if (dist > 0.05) {   // si aún no llegó al punto
        double theta_d = std::atan2(ey, ex);
        double e_theta = theta_d - theta_s;

        // normalizar
        while (e_theta > M_PI)  e_theta -= 2*M_PI;
        while (e_theta < -M_PI) e_theta += 2*M_PI;

        cmd.linear.x  = std::clamp(0.5 * dist, -0.12, 0.12);
        cmd.angular.z = std::clamp(1.5 * e_theta, -1.0, 1.0);

        pub->publish(cmd);
        return;
    }

    // ---------------------------
    // 2) CONTROL DE ORIENTACIÓN
    // ---------------------------
    double e_theta = theta_L - theta_s;

    while (e_theta > M_PI)  e_theta -= 2*M_PI;
    while (e_theta < -M_PI) e_theta += 2*M_PI;

    if (std::fabs(e_theta) < 0.05) {
        cmd.angular.z = 0.0;
    } else {
        cmd.angular.z = std::clamp(1.2 * e_theta, -0.8, 0.8);
    }

    cmd.linear.x = 0.0;
    pub->publish(cmd);
}

