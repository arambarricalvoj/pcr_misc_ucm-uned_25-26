#include <filesystem>
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "tarea1/icc_controller.hpp"
#include "tarea1/service_utils.hpp"
#include "tarea1/pose_utils.hpp"

using namespace std::chrono_literals;

ICCController::ICCController()
: Node("icc_controller")
{
    // Declaración de parámetros
    this->declare_parameter("kv", 0.5);
    this->declare_parameter("vmax", 0.15);
    this->declare_parameter("wmax", 1.0);
    this->declare_parameter("controller_type", 0);

    // Lectura
    kv_   = get_parameter("kv").as_double();
    vmax_ = get_parameter("vmax").as_double();
    wmax_ = get_parameter("wmax").as_double();
    controller_type_ = get_parameter("controller_type").as_int();

    // Nombre del controlador
    std::string controller_name =
        (controller_type_ == 0) ? "icc_reactivo" : "icc_geometrico";

    std::string kv_str = std::to_string(kv_);
    kv_str.erase(kv_str.find_last_not_of('0') + 1);
    if (kv_str.back() == '.') kv_str.pop_back();

    // Ruta al share del paquete (instalado)
    std::string pkg_share = ament_index_cpp::get_package_share_directory("tarea1");

    // Subir dos niveles: share/tarea1/ → install/tarea1/ → ros2_ws/src/tarea1/
    std::filesystem::path pkg_path = std::filesystem::path(pkg_share).parent_path().parent_path().parent_path().parent_path();

    // Ruta final: src/results/
    std::string results_dir = (pkg_path / "tarea1" / "results").string() + "/";

    // Crear carpeta si no existe
    std::filesystem::create_directories(results_dir);

    // Inicialización REAL del logger
    metrics_ = std::make_unique<MetricsLogger>(
        results_dir + controller_name + "_Kv" + kv_str,
        kv_, vmax_, wmax_, controller_type_
    );

    // Suscripción
    pose_sub_ = create_subscription<geometry_msgs::msg::Pose>(
        "/robot_pose", 10,
        std::bind(&ICCController::robotPoseCallback, this, std::placeholders::_1));

    // Publicador
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // Cliente del servicio
    client_ = create_client<std_srvs::srv::Trigger>("/get_target_pose");

    // Timer
    timer_ = create_wall_timer(
        50ms,
        std::bind(&ICCController::controlLoop, this));
}



bool ICCController::callTargetService(rclcpp::Node::SharedPtr node)
{
    bool ok = getTargetPosition(node, client_, xt_, yt_, get_logger());
    if (ok) target_received_ = true;
    return ok;
}

void ICCController::robotPoseCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
{
    robot_pose_ = *msg;
    pose_received_ = true;
}

void ICCController::controlLoop()
{
    if (!target_received_ || !pose_received_)
        return;

    // Tiempo acumulado
    time_elapsed_ += 0.05;

    // Pose del robot
    double xr, yr, theta_r;
    extractPose(robot_pose_, xr, yr, theta_r);

    // Distancia al objetivo
    double dx = xt_ - xr;
    double dy = yt_ - yr;
    double d = std::sqrt(dx*dx + dy*dy);

    // Ángulo deseado
    double theta_d = std::atan2(dy, dx);

    // Error angular normalizado
    double e_theta = normalizeAngle(theta_d - theta_r);

    // Control lineal
    double v = kv_ * d;
    if (v > vmax_) v = vmax_;

    double w = 0.0;

    // Selección del controlador
    if (controller_type_ == 0) {
        // Controlador ICC reactivo
        w = wmax_ * std::sin(e_theta);
    }
    else {
        // Controlador geométrico: arco perfecto
        double R = d / (2 * std::sin(e_theta));
        w = v / R;
    }

    // Registrar métricas acumuladas
    metrics_->updateMetrics(d, time_elapsed_);

    // Registrar TODA la información de esta iteración
    metrics_->addSample(
        time_elapsed_,
        xr, yr, theta_r,   // pose del robot
        xt_, yt_,          // objetivo
        d, e_theta,        // errores
        v, w               // señales de control
    );

    // Si ya hemos llegado
    if (d < 0.05) {
        RCLCPP_INFO(get_logger(), "Objetivo alcanzado. Guardando métricas y finalizando nodo.");

        // Parar robot
        cmd_pub_->publish(geometry_msgs::msg::Twist());

        // Guardar CSV completo
        metrics_->saveToCSV();

        rclcpp::shutdown();
        return;
    }

    // Publicar comando
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x  = v;
    cmd.angular.z = w;
    cmd_pub_->publish(cmd);
}

