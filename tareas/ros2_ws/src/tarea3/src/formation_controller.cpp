#include "tarea3/formation_controller.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>
#include <iostream>
#include <iomanip>
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

    std::string pkg_share = ament_index_cpp::get_package_share_directory("tarea3");
    std::filesystem::path pkg_path = std::filesystem::path(pkg_share)
        .parent_path().parent_path().parent_path().parent_path();

    std::string results_dir = (pkg_path / "src" / "tarea3" / "results").string() + "/";
    std::filesystem::create_directories(results_dir);

    logger_ = std::make_unique<FormationLogger>(results_dir + "formation_log.csv");
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

    time_elapsed_ += 0.05;
    xL = master_pose_.position.x;
    yL = master_pose_.position.y;
    thetaL = extractYaw(master_pose_);

    double x1 = slave1_pose_.position.x;
    double y1 = slave1_pose_.position.y;
    double th1 = extractYaw(slave1_pose_);

    double x2 = slave2_pose_.position.x;
    double y2 = slave2_pose_.position.y;
    double th2 = extractYaw(slave2_pose_);

    double x3 = slave3_pose_.position.x;
    double y3 = slave3_pose_.position.y;
    double th3 = extractYaw(slave3_pose_);

    logger_->log(time_elapsed_,
                xL, yL, thetaL,
                x1, y1, th1,
                x2, y2, th2,
                x3, y3, th3);
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

        //double v = 0.5 * dist * cos(e_theta);   // velocidad lineal
        //double w = 1.5 * e_theta;               // velocidad angular

        //if (v < 0)
        //    w = -w;

        //cmd.linear.x  = std::clamp(v, -0.12, 0.12);
        //cmd.angular.z = std::clamp(w, -1.0, 1.0);

        //cmd.linear.x  = std::clamp(0.5 * dist, -0.12, 0.12);
        cmd.linear.x = std::clamp(0.5 * dist * cos(e_theta), -0.12, 0.12);
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

