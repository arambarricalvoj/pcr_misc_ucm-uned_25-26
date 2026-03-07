// braitenberg_node.cpp
#include "tarea2/braitenberg_puro.hpp"

#include <algorithm>
#include <chrono>

using namespace std::chrono_literals;

BraitenbergController::BraitenbergController()
: Node("braitenberg_controller")
{
    // Declarar parámetros con valores por defecto
    this->declare_parameter<double>("wheel_radius", 0.021);
    this->declare_parameter<double>("wheel_base",   0.088);
    this->declare_parameter<double>("v0",           0.6);
    this->declare_parameter<double>("vmin",         0.12);
    this->declare_parameter<double>("vmax",         1.0);
    this->declare_parameter<double>("max_detection_dist", 0.02);
    this->declare_parameter<double>("max_range_default", 0.25);
    this->declare_parameter<double>("control_hz", 20.0);

    // Pesos: declarar como vector<double> para poder cargarlos desde YAML
    this->declare_parameter<std::vector<double>>("braitenbergL",
        std::vector<double>{0.0,0.0,0.0,0.0,-0.025,-0.0375,-0.05,-0.000625});
    this->declare_parameter<std::vector<double>>("braitenbergR",
        std::vector<double>{-0.000625,-0.05,-0.0375,-0.025,0.0,0.0,0.0,0.0});

    // Leer parámetros
    wheel_radius_ = this->get_parameter("wheel_radius").as_double();
    wheel_base_   = this->get_parameter("wheel_base").as_double();
    v0_           = this->get_parameter("v0").as_double();
    vmin_         = this->get_parameter("vmin").as_double();
    vmax_         = this->get_parameter("vmax").as_double();
    max_detection_dist_ = this->get_parameter("max_detection_dist").as_double();
    max_range_default_  = this->get_parameter("max_range_default").as_double();
    control_hz_ = this->get_parameter("control_hz").as_double();

    // Cargar pesos desde parámetros (asegurando tamaño 8)
    auto Lvec = this->get_parameter("braitenbergL").as_double_array();
    auto Rvec = this->get_parameter("braitenbergR").as_double_array();
    if (Lvec.size() != 8 || Rvec.size() != 8) {
        RCLCPP_WARN(this->get_logger(), "Los vectores de pesos deben tener 8 elementos. Usando valores por defecto si es necesario.");
    }
    for (size_t i = 0; i < 8; ++i) {
        braitenbergL_[i] = (i < Lvec.size()) ? Lvec[i] : 0.0;
        braitenbergR_[i] = (i < Rvec.size()) ? Rvec[i] : 0.0;
    }

    // Inicializar rangos IR
    ir_ranges_.fill(max_range_default_);

    // Suscriptores IR
    for (int i = 0; i < 8; ++i) {
        std::string topic = "/khepera/ir" + std::to_string(i+1);
        // usar lambda que llama a método miembro con índice
        ir_subs_[i] = this->create_subscription<sensor_msgs::msg::Range>(
            topic, 10,
            [this, i](sensor_msgs::msg::Range::SharedPtr msg) {
                this->irCallback(i, msg);
            }
        );
    }

    // Publicador cmd_vel
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // Timer de control
    auto period = std::chrono::duration<double>(1.0 / control_hz_);
    timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(period),
        std::bind(&BraitenbergController::controlLoop, this)
    );

    RCLCPP_INFO(this->get_logger(), "Braitenberg node inicializado (hz=%.1f).", control_hz_);
}

void BraitenbergController::irCallback(size_t idx, sensor_msgs::msg::Range::SharedPtr msg)
{
    double r = msg->range;
    if (r < msg->min_range) r = msg->min_range;
    if (r > msg->max_range) r = msg->max_range;
    ir_ranges_[idx] = r;
}

void BraitenbergController::controlLoop()
{
    // Normalizar lecturas y calcular detect[i] en [0,1]
    std::array<double,8> detect;
    for (int i = 0; i < 8; ++i) {
        double dist = ir_ranges_[i];
        if (std::isnan(dist) || dist <= 0.0) dist = max_range_default_;

        double dnorm = 0.0;
        if (dist < max_range_default_) {
            double d = dist;
            if (d < max_detection_dist_) d = max_detection_dist_;
            dnorm = 1.0 - ((d - max_detection_dist_) / (max_range_default_ - max_detection_dist_));
            dnorm = std::clamp(dnorm, 0.0, 1.0);
        } else {
            dnorm = 0.0;
        }
        detect[i] = dnorm;
    }

    // Calcular velocidades de rueda en m/s (acoplamiento cruzado)
    double vLeft_lin  = v0_;
    double vRight_lin = v0_;
    for (int i = 0; i < 8; ++i) {
        vLeft_lin  += braitenbergL_[i] * detect[i];
        vRight_lin += braitenbergR_[i] * detect[i];
    }

    // Clamp para evitar reversa y paradas bruscas: [vmin_, vmax_]
    vLeft_lin  = std::clamp(vLeft_lin,  vmin_, vmax_);
    vRight_lin = std::clamp(vRight_lin, vmin_, vmax_);

    // Convertir a velocidad lineal y angular del chasis
    double V = (vRight_lin + vLeft_lin) / 2.0;
    double W = (vRight_lin - vLeft_lin) / wheel_base_;

    // Publicar Twist
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = V;
    cmd.angular.z = W;
    cmd_pub_->publish(cmd);
}