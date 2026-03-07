#pragma once

#include <array>
#include <vector>
#include <memory>
#include <string>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include "tarea2/metrics_utils.hpp"

class BraitenbergController : public rclcpp::Node
{
public:
    BraitenbergController();

    // No destructor explícito: guardado controlado desde controlLoop (save after parameter)

private:
    // callbacks
    void irCallback(size_t idx, sensor_msgs::msg::Range::SharedPtr msg);
    void robotPoseCallback(const geometry_msgs::msg::Pose::SharedPtr msg);
    void controlLoop();

    // util
    double computeMinDistanceToObstacles(double x, double y) const;

    // parámetros y estado
    double wheel_radius_;
    double wheel_base_;
    double v0_;
    double vmin_;
    double vmax_;
    double max_detection_dist_;
    double max_range_default_;
    double control_hz_;

    std::array<double,8> braitenbergL_;
    std::array<double,8> braitenbergR_;

    std::array<double,8> ir_ranges_;
    std::array<rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr,8> ir_subs_;

    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_sub_;
    geometry_msgs::msg::Pose robot_pose_;
    bool pose_received_ = false;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // métricas (clase original)
    std::unique_ptr<MetricsLogger> metrics_;

    // obstáculos: vector de {x,y,r} (opcional, puede quedar vacío)
    std::vector<std::array<double,3>> obstacles_;

    // tiempo acumulado (estilo ICCController)
    double time_elapsed_ = 0.0;

    // indicador para guardar métricas una sola vez tras save_after_seconds_
    bool metrics_saved_ = false;

    // distancia de seguridad (si la usas más adelante)
    double d_safe_ = 0.0;

    // limitación de aceleración (parámetros y estado)
    double max_accel_linear_  = 0.5; // m/s^2
    double max_accel_angular_ = 1.0; // rad/s^2
    double prev_v_ = 0.0;
    double prev_w_ = 0.0;

    // tiempo tras el cual se guarda el CSV (segundos)
    double save_after_seconds_;

    // nombre fichero
    std::string controller_name_;
};
