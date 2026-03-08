#include <filesystem>
#include <cmath>
#include <algorithm>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <limits>

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "tarea2/khepera_controller.hpp"
#include "tarea2/braitenberg_repulsion.hpp"
#include "tarea2/service_utils.hpp"
#include "tarea2/pose_utils.hpp"

using namespace std::chrono_literals;

BraitenbergTargetController::BraitenbergTargetController()
: Node("braitenberg_target_controller")
{
    // -------------------------
    // PARÁMETROS
    // -------------------------
    this->declare_parameter("kp", 0.5);
    this->declare_parameter("controller_type", 0);
    this->declare_parameter("wheel_radius", 0.021);
    this->declare_parameter("wheel_base",   0.088);
    this->declare_parameter("v0",           0.6);
    this->declare_parameter("vmin",         0.0);
    this->declare_parameter("vmax",         1.0);
    this->declare_parameter("wmax",         1.0);
    this->declare_parameter("max_detection_dist", 0.15);
    this->declare_parameter("max_range_default", 0.25);
    this->declare_parameter("control_hz", 20.0);
    this->declare_parameter("d_safe", 0.15);
    this->declare_parameter<std::vector<double>>("GL",
        std::vector<double>{0.0,0.0,0.0,0.0,-0.025,-0.0375,-0.05,-0.000625});
    this->declare_parameter<std::vector<double>>("GR",
        std::vector<double>{-0.000625,-0.05,-0.0375,-0.025,0.0,0.0,0.0,0.0});

    kp_   = this->get_parameter("kp").as_double();
    controller_type_ = this->get_parameter("controller_type").as_int();
    wheel_radius_ = this->get_parameter("wheel_radius").as_double();
    wheel_base_   = this->get_parameter("wheel_base").as_double();
    v0_           = this->get_parameter("v0").as_double();
    vmin_         = this->get_parameter("vmin").as_double();
    vmax_         = this->get_parameter("vmax").as_double();
    wmax_         = this->get_parameter("wmax").as_double();
    max_detection_dist_ = this->get_parameter("max_detection_dist").as_double();
    max_range_default_  = this->get_parameter("max_range_default").as_double();
    control_hz_ = this->get_parameter("control_hz").as_double();
    d_safe_ = this->get_parameter("d_safe").as_double();
    auto Lvec = this->get_parameter("GL").as_double_array();
    auto Rvec = this->get_parameter("GR").as_double_array();
    
    // -------------------------
    // FORMATEAR CORRECTAMENTE GANANCIAS
    // -------------------------
    for (size_t i = 0; i < 8; ++i) {
        GL_[i] = (i < Lvec.size()) ? Lvec[i] : 0.0;
        GR_[i] = (i < Rvec.size()) ? Rvec[i] : 0.0;
    }

    // -------------------------
    // INICIALIZACIONES
    // -------------------------
    ir_ranges_.fill(max_range_default_);

    // -------------------------
    // SUSCRIPCIONES
    // -------------------------
    pose_sub_ = create_subscription<geometry_msgs::msg::Pose>(
        "/robot_pose", 10,
        std::bind(&BraitenbergTargetController::robotPoseCallback, this, std::placeholders::_1));

    for (int i = 0; i < 8; ++i) {
        std::string topic = "/khepera/ir" + std::to_string(i+1);
        ir_subs_[i] = this->create_subscription<sensor_msgs::msg::Range>(
            topic, 10,
            [this, i](sensor_msgs::msg::Range::SharedPtr msg) {
                this->irCallback(i, msg);
            }
        );
    }

    // -------------------------
    // PUBLICADORES
    // -------------------------
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // -------------------------
    // SERVICIOS
    // -------------------------
    client_ = create_client<std_srvs::srv::Trigger>("/get_target_pose");
    action_server_ = rclcpp_action::create_server<ExecuteTrajectory>(
        this,
        "/execute_trajectory",
        std::bind(&BraitenbergTargetController::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&BraitenbergTargetController::handleCancel, this, std::placeholders::_1),
        std::bind(&BraitenbergTargetController::handleAccepted, this, std::placeholders::_1)
    );

    // -------------------------
    // TIMERS
    // -------------------------
    auto period = std::chrono::duration<double>(1.0 / control_hz_);
    timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(period),
        std::bind(&BraitenbergTargetController::controlLoop, this)
    );

    // -------------------------
    // OTROS
    // -------------------------
    std::string controller_name;
    switch (controller_type_) {
    case 0: controller_name = "blending_lineal"; break;
    case 1: controller_name = "arbitraje_prioritario"; break;
    case 2: controller_name = "campos_potenciales"; break;
    default: 
        RCLCPP_WARN(get_logger(), "controller_type_ inválido (%d). Usando blending_lineal.", controller_type_);
        controller_name = "blending_lineal"; 
        controller_type_ = 0; 
        break;
    }

    // Ruta al share
    std::string pkg_share = ament_index_cpp::get_package_share_directory("tarea2");
    std::filesystem::path pkg_path = std::filesystem::path(pkg_share)
        .parent_path().parent_path().parent_path().parent_path();
    std::string results_dir = (pkg_path / "src" / "tarea2" / "results").string() + "/";
    std::filesystem::create_directories(results_dir);

    // Instaciar módulo métricas
    metrics_ = std::make_unique<MetricsLogger>(
        results_dir + controller_name,
        kp_,        // double kp
        vmax_,      // double vmax
        wmax_,      // double wmax
        controller_type_ // int controller_type
    );

    repulsion_ = std::make_unique<BraitenbergRepulsion>(
        wheel_base_, v0_, vmin_, vmax_, max_range_default_, max_detection_dist_, GL_, GR_
    );

    controller_ = createController(controller_type_, kp_, vmax_, wmax_, 1e-3);


}

// ---------------------------------------------------------------------------
// ACTION SERVER CALLBACKS
// ---------------------------------------------------------------------------

rclcpp_action::GoalResponse BraitenbergTargetController::handleGoal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const ExecuteTrajectory::Goal>)
{
    RCLCPP_INFO(get_logger(), "Nueva meta recibida: ejecutar trayectoria");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse BraitenbergTargetController::handleCancel(
    const std::shared_ptr<GoalHandleExecuteTrajectory>)
{
    RCLCPP_INFO(get_logger(), "Cancelando trayectoria");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void BraitenbergTargetController::handleAccepted(
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
            std::bind(&BraitenbergTargetController::onTargetReceived, this, std::placeholders::_1),
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

void BraitenbergTargetController::onTargetReceived(
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

void BraitenbergTargetController::robotPoseCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
{
    robot_pose_ = *msg;
    pose_received_ = true;
}

// ---------------------------------------------------------------------------
// IR CALLBACK
// ---------------------------------------------------------------------------
void BraitenbergTargetController::irCallback(size_t idx, sensor_msgs::msg::Range::SharedPtr msg)
{
    double r = msg->range;
    if (r < msg->min_range) r = msg->min_range;
    if (r > msg->max_range) r = msg->max_range;
    ir_ranges_[idx] = r;
}

// ---------------------------------------------------------------------------
// UTIL
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// CONTROL LOOP
// ---------------------------------------------------------------------------
void BraitenbergTargetController::controlLoop()
{
    if (!goal_handle_) return;
    if (!pose_received_) return;
    if (!target_received_) return;
    if (!controller_) return;

    auto feedback = std::make_shared<ExecuteTrajectory::Feedback>();

    // dt según control_hz_ (protección contra 0)
    double hz = std::max(1.0, control_hz_);
    double dt = 1.0 / hz;
    time_elapsed_ += dt;

    // Extraer pose actual
    double xr, yr, theta_r;
    extractPose(robot_pose_, xr, yr, theta_r);

    // Errores geométricos hacia el objetivo
    double dx = xt_ - xr;
    double dy = yt_ - yr;
    double d = std::hypot(dx, dy);

    double theta_d = std::atan2(dy, dx);
    double e_theta = normalizeAngle(theta_d - theta_r);

    // --- Señal objetivo desde el módulo controller_ ---
    ControllerInput in{xr, yr, theta_r, xt_, yt_, dt};
    auto [v_goal, w_goal] = controller_->computeCommand(in);

    // Convertir objetivo a velocidades de rueda (espacio de ruedas)
    double vL_g = v_goal - (w_goal * wheel_base_ / 2.0);
    double vR_g = v_goal + (w_goal * wheel_base_ / 2.0);

    // --- Señal repulsiva desde BraitenbergRepulsion ---
    double vL_b = 0.0, vR_b = 0.0;
    double dmin = std::numeric_limits<double>::infinity();
    if (repulsion_) {
        auto p = repulsion_->computeWheelSpeeds(); // {vL, vR}
        vL_b = p.first;
        vR_b = p.second;
        dmin = repulsion_->getMinDetection();
        RCLCPP_INFO(get_logger(), "BRAIT: vl=%.3f vr=%.3f", vL_b, vR_b);
    }

    // --- Calcular alpha blending lineal ---
    // Usa d_safe_ y d_blend (puedes exponer d_blend_ como parámetro)
    double d_blend = std::max(d_safe_ + 0.10, max_detection_dist_);
    double alpha = 1.0;
    if (std::isfinite(dmin)) {
        if (dmin <= d_safe_) alpha = 0.0;
        else if (dmin >= d_blend) alpha = 1.0;
        else {
            double denom = d_blend - d_safe_;
            alpha = (denom > 1e-6) ? std::clamp((dmin - d_safe_) / denom, 0.0, 1.0) : 1.0;
        }
    }

    // --- Mezcla en espacio de ruedas ---
    double vL = alpha * vL_g + (1.0 - alpha) * vL_b;
    double vR = alpha * vR_g + (1.0 - alpha) * vR_b;

    // --- Convertir a V,W y saturar ---
    double V = (vR + vL) / 2.0;
    double W = (vR - vL) / wheel_base_;

    V = std::clamp(V, vmin_, vmax_);
    W = std::clamp(W, -wmax_, wmax_);

    // --- Feedback y métricas ---
    feedback->distance = d;
    feedback->e_theta = e_theta;
    feedback->v = V;
    feedback->w = W;
    feedback->time_elapsed = time_elapsed_;
    goal_handle_->publish_feedback(feedback);

    metrics_->updateMetrics(d, time_elapsed_);
    metrics_->addSample(
        time_elapsed_,
        xr, yr, theta_r,
        xt_, yt_,
        d, e_theta,
        V, W
    );

    // Llegada al objetivo
    if (d < 0.05) {
        RCLCPP_INFO(get_logger(), "Objetivo alcanzado");
        cmd_pub_->publish(geometry_msgs::msg::Twist()); // stop
        metrics_->saveToCSV();
        auto result = std::make_shared<ExecuteTrajectory::Result>();
        result->success = true;
        result->message = "Objetivo alcanzado";
        goal_handle_->succeed(result);
        goal_handle_.reset();
        return;
    }

    // Safety stop opcional si dmin extremadamente pequeño
    double d_critical = 0.02; // ajustar según robot
    if (dmin < d_critical) {
        RCLCPP_WARN(get_logger(), "Detección crítica (dmin=%.3f). Parada de emergencia.", dmin);
        cmd_pub_->publish(geometry_msgs::msg::Twist()); // stop
        return;
    }

    // Publicar comando
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = V;
    cmd.angular.z = W;
    cmd_pub_->publish(cmd);

    // Debug opcional
    // RCLCPP_DEBUG(get_logger(), "dmin=%.3f alpha=%.3f vL_g=%.3f vR_g=%.3f vL_b=%.3f vR_b=%.3f V=%.3f W=%.3f",
    //     dmin, alpha, vL_g, vR_g, vL_b, vR_b, V, W);
}


/*
void BraitenbergTargetController::controlLoop()
{
    if (!goal_handle_) return;
    if (!pose_received_) return;
    if (!target_received_) return;
    if (!controller_) return;

    auto feedback = std::make_shared<ExecuteTrajectory::Feedback>();

    // dt según control_hz_ (protección contra 0)
    double hz = std::max(1.0, control_hz_);
    double dt = 1.0 / hz;
    time_elapsed_ += dt;

    // Extraer pose actual
    double xr, yr, theta_r;
    extractPose(robot_pose_, xr, yr, theta_r);

    // Errores geométricos hacia el objetivo
    double dx = xt_ - xr;
    double dy = yt_ - yr;
    double d = std::hypot(dx, dy);

    double theta_d = std::atan2(dy, dx);
    double e_theta = normalizeAngle(theta_d - theta_r);

    // --- Señal objetivo desde el módulo controller_ ---
    ControllerInput in{xr, yr, theta_r, xt_, yt_, dt};
    auto [v_goal, w_goal] = controller_->computeCommand(in);

    // Opción A (recomendada): convertir objetivo directamente a V,W
    // Esto evita depender de velocidades de rueda si no quieres usarlas ahora.
    double V = std::clamp(v_goal, vmin_, vmax_);
    double W = std::clamp(w_goal, -wmax_, wmax_);

    // --- Feedback y métricas ---
    feedback->distance = d;
    feedback->e_theta = e_theta;
    feedback->v = V;
    feedback->w = W;
    feedback->time_elapsed = time_elapsed_;
    goal_handle_->publish_feedback(feedback);

    metrics_->updateMetrics(d, time_elapsed_);
    metrics_->addSample(
        time_elapsed_,
        xr, yr, theta_r,
        xt_, yt_,
        d, e_theta,
        V, W
    );

    // Llegada al objetivo
    if (d < 0.05) {
        RCLCPP_INFO(get_logger(), "Objetivo alcanzado");
        cmd_pub_->publish(geometry_msgs::msg::Twist()); // stop
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
    cmd.linear.x = V;
    cmd.angular.z = W;
    cmd_pub_->publish(cmd);

    // Debug opcional
    // RCLCPP_DEBUG(get_logger(), "d=%.3f e_theta=%.3f v_goal=%.3f w_goal=%.3f V=%.3f W=%.3f",
    //     d, e_theta, v_goal, w_goal, V, W);
}
*/
