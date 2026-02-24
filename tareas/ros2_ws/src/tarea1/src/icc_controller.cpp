#include <filesystem>
#include <cmath>

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "tarea1/icc_controller.hpp"
#include "tarea1/service_utils.hpp"
#include "tarea1/pose_utils.hpp"

using namespace std::chrono_literals;

ICCController::ICCController()
: Node("icc_controller")
{
    // Parámetros
    this->declare_parameter("kv", 0.5);
    this->declare_parameter("vmax", 0.15);
    this->declare_parameter("wmax", 1.0);
    this->declare_parameter("controller_type", 0);

    kv_   = get_parameter("kv").as_double();
    vmax_ = get_parameter("vmax").as_double();
    wmax_ = get_parameter("wmax").as_double();
    controller_type_ = get_parameter("controller_type").as_int();

    // Nombre del controlador
    std::string controller_name =
        (controller_type_ == 0) ? "icc_reactivo" : "icc_geometrico";

    // Formateo de Kv
    std::string kv_str = std::to_string(kv_);
    kv_str.erase(kv_str.find_last_not_of('0') + 1);
    if (kv_str.back() == '.') kv_str.pop_back();

    // Ruta al share
    std::string pkg_share = ament_index_cpp::get_package_share_directory("tarea1");
    std::filesystem::path pkg_path = std::filesystem::path(pkg_share)
        .parent_path().parent_path().parent_path().parent_path();

    std::string results_dir = (pkg_path / "src" / "tarea1" / "results").string() + "/";
    std::filesystem::create_directories(results_dir);

    metrics_ = std::make_unique<MetricsLogger>(
        results_dir + controller_name + "_Kv" + kv_str,
        kv_, vmax_, wmax_, controller_type_
    );

    // Suscriptor
    pose_sub_ = create_subscription<geometry_msgs::msg::Pose>(
        "/robot_pose", 10,
        std::bind(&ICCController::robotPoseCallback, this, std::placeholders::_1));

    // Publicador
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // Cliente del servicio
    client_ = create_client<std_srvs::srv::Trigger>("/get_target_pose");

    // Action server
    action_server_ = rclcpp_action::create_server<ExecuteTrajectory>(
        this,
        "/execute_trajectory",
        std::bind(&ICCController::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&ICCController::handleCancel, this, std::placeholders::_1),
        std::bind(&ICCController::handleAccepted, this, std::placeholders::_1)
    );

    // Timer del control
    timer_ = create_wall_timer(
        50ms,
        std::bind(&ICCController::controlLoop, this));
}

// ---------------------------------------------------------------------------
// ACTION SERVER CALLBACKS
// ---------------------------------------------------------------------------

rclcpp_action::GoalResponse ICCController::handleGoal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const ExecuteTrajectory::Goal>)
{
    RCLCPP_INFO(get_logger(), "Nueva meta recibida: ejecutar trayectoria");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ICCController::handleCancel(
    const std::shared_ptr<GoalHandleExecuteTrajectory>)
{
    RCLCPP_INFO(get_logger(), "Cancelando trayectoria");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void ICCController::handleAccepted(
    const std::shared_ptr<GoalHandleExecuteTrajectory> goal_handle)
{
    goal_handle_ = goal_handle;
    target_received_ = false;

    auto goal = goal_handle->get_goal();

    // Detectar si el target_pose está vacío (NaN)
    bool target_is_empty =
        std::isnan(goal->target_pose.position.x) ||
        std::isnan(goal->target_pose.position.y) ||
        std::isnan(goal->target_pose.position.z);

    if (target_is_empty)
    {
        RCLCPP_INFO(get_logger(), "No se recibió target → solicitándolo al servicio...");

        // Llamada ASÍNCRONA al servicio
        requestTargetPositionAsync(
            client_,
            std::bind(&ICCController::onTargetReceived, this, std::placeholders::_1),
            get_logger()
        );
    }
    else
    {
        RCLCPP_INFO(get_logger(), "Target recibido en la acción → usando target_pose directamente");

        // Guardar el target directamente (igual que hace onTargetReceived)
        xt_ = goal->target_pose.position.x;
        yt_ = goal->target_pose.position.y;

        target_received_ = true;

        // NO LLAMAR A executeControlLoop()
        // El timer controlLoop() se encargará automáticamente
    }
}



// ---------------------------------------------------------------------------
// CALLBACK DE RESPUESTA DEL SERVICIO (ASÍNCRONO)
// ---------------------------------------------------------------------------

void ICCController::onTargetReceived(
    const std_srvs::srv::Trigger::Response::SharedPtr response)
{
    if (!response) {
        RCLCPP_ERROR(get_logger(), "No se pudo obtener el target");
        auto result = std::make_shared<ExecuteTrajectory::Result>();
        result->success = false;
        result->message = "No se pudo obtener el target";
        goal_handle_->abort(result);
        return;
    }

    int parsed = sscanf(response->message.c_str(),
                        "{\"position\":{\"x\":%lf,\"y\":%lf",
                        &xt_, &yt_);

    if (parsed != 2) {
        RCLCPP_ERROR(get_logger(), "No se pudo parsear la respuesta del servicio.");
        auto result = std::make_shared<ExecuteTrajectory::Result>();
        result->success = false;
        result->message = "Respuesta inválida del servicio";
        goal_handle_->abort(result);
        return;
    }

    target_received_ = true;
    RCLCPP_INFO(get_logger(), "Objetivo recibido: x=%.3f y=%.3f", xt_, yt_);
}

// ---------------------------------------------------------------------------
// POSE CALLBACK
// ---------------------------------------------------------------------------

void ICCController::robotPoseCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
{
    robot_pose_ = *msg;
    pose_received_ = true;
}

// ---------------------------------------------------------------------------
// CONTROL LOOP
// ---------------------------------------------------------------------------

void ICCController::controlLoop()
{
    if (!goal_handle_) return;
    if (!pose_received_) return;
    if (!target_received_) return;

    auto feedback = std::make_shared<ExecuteTrajectory::Feedback>();

    time_elapsed_ += 0.05;

    double xr, yr, theta_r;
    extractPose(robot_pose_, xr, yr, theta_r);

    double dx = xt_ - xr;
    double dy = yt_ - yr;
    double d = std::sqrt(dx*dx + dy*dy);

    double theta_d = std::atan2(dy, dx);
    double e_theta = normalizeAngle(theta_d - theta_r);

    double v = kv_ * d;
    if (v > vmax_) v = vmax_;

    double w = 0.0;
    if (controller_type_ == 0)
        w = wmax_ * std::sin(e_theta);
    else {
        double R = d / (2 * std::sin(e_theta));
        w = v / R;
    }

    // Feedback
    feedback->distance = d;
    feedback->e_theta = e_theta;
    feedback->v = v;
    feedback->w = w;
    feedback->time_elapsed = time_elapsed_;

    goal_handle_->publish_feedback(feedback);

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

    // Llegada al objetivo
    if (d < 0.05) {
        RCLCPP_INFO(get_logger(), "Objetivo alcanzado");

        cmd_pub_->publish(geometry_msgs::msg::Twist());

        metrics_->saveToCSV();

        auto result = std::make_shared<ExecuteTrajectory::Result>();
        result->success = true;
        result->message = "Objetivo alcanzado";

        goal_handle_->succeed(result);
        goal_handle_.reset();
        return;
    }

    // Publicar comando
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = v;
    cmd.angular.z = w;
    cmd_pub_->publish(cmd);
}
