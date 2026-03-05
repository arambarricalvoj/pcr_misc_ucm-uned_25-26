#include <filesystem>
#include <cmath>
#include <algorithm>

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "tarea2/campo_vectorial.hpp"
#include "tarea2/service_utils.hpp"
#include "tarea2/pose_utils.hpp"
#include "tarea2/braitenberg.hpp"
#include "tarea2/tang.hpp"

#include <iostream>
#include <iomanip>

using namespace std::chrono_literals;

ICCController::ICCController()
: Node("khepera_controller")
{
    // -------------------------
    // PARÁMETROS básicos
    // -------------------------
    this->declare_parameter("kp", 0.5);
    this->declare_parameter("vmax", 0.15);
    this->declare_parameter("wmax", 1.0);
    this->declare_parameter("controller_type", 0);

    this->declare_parameter("ks", 1.0);
    this->declare_parameter("k_front", 0.8);
    this->declare_parameter("k_turn", 1.2);

    kp_   = get_parameter("kp").as_double();
    vmax_ = get_parameter("vmax").as_double();
    wmax_ = get_parameter("wmax").as_double();
    controller_type_ = get_parameter("controller_type").as_int();

    ks_   = get_parameter("ks").as_double();
    k_front_ = get_parameter("k_front").as_double();
    k_turn_  = get_parameter("k_turn").as_double();

    this->declare_parameter("k_tan", 0.8);
    k_tan_ = get_parameter("k_tan").as_double();

    // -------------------------
    // Parámetros campo vectorial (valores por defecto)
    // -------------------------
    this->declare_parameter("k_att", 1.0);
    this->declare_parameter("k_rep", 0.9);
    this->declare_parameter("k_v", 0.6);
    this->declare_parameter("k_w", 2.0);
    this->declare_parameter("d_dead", 0.6);
    this->declare_parameter("d_min", 0.08);
    this->declare_parameter("F_max", 0.2);
    this->declare_parameter("filter_beta", 0.85);

    k_att_ = get_parameter("k_att").as_double();
    k_rep_ = get_parameter("k_rep").as_double();
    k_v_   = get_parameter("k_v").as_double();
    k_w_   = get_parameter("k_w").as_double();
    d_dead_ = get_parameter("d_dead").as_double();
    d_min_  = get_parameter("d_min").as_double();
    F_max_  = get_parameter("F_max").as_double();
    filter_beta_ = get_parameter("filter_beta").as_double();

    // -------------------------
    // RUTA MÉTRICAS (igual que en tu nodo original)
    // -------------------------
    std::string controller_name =
        (controller_type_ == 0) ? "icc_reactivo" : "icc_geometrico";

    std::string kp_str = std::to_string(kp_);
    kp_str.erase(kp_str.find_last_not_of('0') + 1);
    if (!kp_str.empty() && kp_str.back() == '.') kp_str.pop_back();

    std::string pkg_share = ament_index_cpp::get_package_share_directory("tarea2");
    std::filesystem::path pkg_path = std::filesystem::path(pkg_share)
        .parent_path().parent_path().parent_path().parent_path();

    std::string results_dir = (pkg_path / "src" / "tarea2" / "results").string() + "/";
    std::filesystem::create_directories(results_dir);

    metrics_ = std::make_unique<MetricsLogger>(
        results_dir + controller_name + "_Kp" + kp_str,
        kp_, vmax_, wmax_, controller_type_
    );

    // -------------------------
    // SUSCRIPTORES IR
    // -------------------------
    ir_ranges_.fill(0.25);  // valor por defecto (sin obstáculo)

    for (int i = 0; i < 8; ++i)
    {
        std::string topic = "/khepera/ir" + std::to_string(i+1);
        ir_subs_[i] = create_subscription<sensor_msgs::msg::Range>(
            topic, 10,
            [this, i](sensor_msgs::msg::Range::SharedPtr msg)
            {
                double r = msg->range;
                if (r < msg->min_range) r = msg->min_range;
                if (r > msg->max_range) r = msg->max_range;
                ir_ranges_[i] = r;
            }
        );
    }

    // -------------------------
    // SUSCRIPTOR POSE
    // -------------------------
    pose_sub_ = create_subscription<geometry_msgs::msg::Pose>(
        "/robot_pose", 10,
        std::bind(&ICCController::robotPoseCallback, this, std::placeholders::_1));

    // -------------------------
    // PUBLICADOR CMD_VEL
    // -------------------------
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // -------------------------
    // CLIENTE SERVICIO TARGET
    // -------------------------
    client_ = create_client<std_srvs::srv::Trigger>("/get_target_pose");

    // -------------------------
    // ACTION SERVER
    // -------------------------
    action_server_ = rclcpp_action::create_server<ExecuteTrajectory>(
        this,
        "/execute_trajectory",
        std::bind(&ICCController::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&ICCController::handleCancel, this, std::placeholders::_1),
        std::bind(&ICCController::handleAccepted, this, std::placeholders::_1)
    );

    // -------------------------
    // FILTRADO inicial
    // -------------------------
    rep_fx_filt_ = 0.0;
    rep_fy_filt_ = 0.0;

    // -------------------------
    // TIMER CONTROL
    // -------------------------
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

    bool target_is_empty =
        std::isnan(goal->target_pose.position.x) ||
        std::isnan(goal->target_pose.position.y) ||
        std::isnan(goal->target_pose.position.z);

    if (target_is_empty)
    {
        RCLCPP_INFO(get_logger(), "No se recibió target → solicitándolo al servicio...");

        requestTargetPositionAsync(
            client_,
            std::bind(&ICCController::onTargetReceived, this, std::placeholders::_1),
            get_logger()
        );
    }
    else
    {
        xt_ = goal->target_pose.position.x;
        yt_ = goal->target_pose.position.y;
        target_received_ = true;
    }
}

// ---------------------------------------------------------------------------
// CALLBACK SERVICIO
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
}

// ---------------------------------------------------------------------------
// CALLBACK POSE
// ---------------------------------------------------------------------------

void ICCController::robotPoseCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
{
    robot_pose_ = *msg;
    pose_received_ = true;
}

// ---------------------------------------------------------------------------
// CONTROL LOOP: campo vectorial (atracción + repulsión por sensor)
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

    double theta_to_goal = std::atan2(dy, dx);
    double e_theta_goal = normalizeAngle(theta_to_goal - theta_r);

    // velocidad base hacia objetivo (seguridad)
    double v_goal = kp_ * d;
    if (v_goal > vmax_) v_goal = vmax_;

    // ángulos sensores (orden: RL, L, FL, F, FR, R, RR, Rear)
    double sensor_angles[8] = {
        -3.0*M_PI/4.0, -M_PI/2.0, -M_PI/4.0, 0.0,
         M_PI/4.0,     M_PI/2.0,  3.0*M_PI/4.0, M_PI
    };

    // 1) Fuerza atractiva (normalizada)
    double Fa_x = 0.0, Fa_y = 0.0;
    if (d > 1e-6) {
        Fa_x = k_att_ * (dx / d);
        Fa_y = k_att_ * (dy / d);
    }

    // 2) Sumar repulsiones por sensor (en coordenadas del robot)
    double Fr_x = 0.0, Fr_y = 0.0;
    const double eps = 1e-6;
    for (int i = 0; i < 8; ++i) {
        double dist = ir_ranges_[i];
        if (dist > d_dead_) continue; // dead zone
        double s = (d_dead_ - dist) / (d_dead_ - d_min_ + eps);
        s = std::clamp(s, 0.0, 1.0);
        s = s * s; // curva suave
        // vector unitario en la dirección del sensor (apunta desde robot hacia obstáculo)
        double ux = std::cos(sensor_angles[i]);
        double uy = std::sin(sensor_angles[i]);
        // repulsión apunta desde obstáculo hacia robot -> invertimos ux,uy
        Fr_x += k_rep_ * s * (-ux);
        Fr_y += k_rep_ * s * (-uy);
    }

    // 3) Filtrado exponencial de la repulsión (suaviza saltos)
    rep_fx_filt_ = filter_beta_ * rep_fx_filt_ + (1.0 - filter_beta_) * Fr_x;
    rep_fy_filt_ = filter_beta_ * rep_fy_filt_ + (1.0 - filter_beta_) * Fr_y;

    // 4) Vector resultante (en coordenadas del mundo)
    double F_x = Fa_x + rep_fx_filt_;
    double F_y = Fa_y + rep_fy_filt_;

    // limitar magnitud para evitar reacciones extremas
    double F_norm = std::hypot(F_x, F_y);
    if (F_norm > F_max_) {
        F_x = F_x * (F_max_ / (F_norm + eps));
        F_y = F_y * (F_max_ / (F_norm + eps));
    }

    // 5) Dirección deseada y control
    double theta_d = std::atan2(F_y, F_x);
    double e_theta = normalizeAngle(theta_d - theta_r);

    // componente longitudinal del vector en eje del robot
    double fx_robot = F_x * std::cos(theta_r) + F_y * std::sin(theta_r);

    double v = k_v_ * std::max(0.0, fx_robot);
    double w = k_w_ * e_theta;

    // si estás muy cerca del objetivo, prioriza llegada
    if (d < 0.15) {
        v = std::min(v, v_goal);
    }

    // límites
    if (v < 0.0) v = 0.0;
    if (v > vmax_) v = vmax_;
    if (w > wmax_) w = wmax_;
    if (w < -wmax_) w = -wmax_;

    // Tangential escape (si lo tienes activado para casos de bloqueo)
    if (controller_type_ == 1) {
        // calcula indicadores A_front, A_left, A_right si tu compute_braitenberg los proporciona
        auto b = compute_braitenberg(ir_ranges_, k_front_, k_turn_);
        auto t = compute_tangential_escape(
            e_theta, d,
            b.A_front, b.A_left, b.A_right,
            k_tan_);

        if (t.active) {
            w += t.w_tang;
            v *= 0.5;
            w = std::clamp(w, -wmax_, wmax_);
        }
    }

    // -------------------------
    // FEEDBACK ACCIÓN
    // -------------------------
    feedback->distance = d;
    feedback->e_theta = e_theta;
    feedback->v = v;
    feedback->w = w;
    feedback->time_elapsed = time_elapsed_;

    goal_handle_->publish_feedback(feedback);

    // -------------------------
    // MÉTRICAS
    // -------------------------
    metrics_->updateMetrics(d, time_elapsed_);
    metrics_->addSample(
        time_elapsed_,
        xr, yr, theta_r,
        xt_, yt_,
        d, e_theta,
        v, w
    );

    // -------------------------
    // LLEGADA AL OBJETIVO
    // -------------------------
    if (d < 0.05) {
        cmd_pub_->publish(geometry_msgs::msg::Twist());

        metrics_->saveToCSV();

        auto result = std::make_shared<ExecuteTrajectory::Result>();
        result->success = true;
        result->message = "Objetivo alcanzado";

        goal_handle_->succeed(result);
        goal_handle_.reset();
        return;
    }

    // -------------------------
    // PUBLICAR VELOCIDAD
    // -------------------------
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = v;
    cmd.angular.z = w;
    cmd_pub_->publish(cmd);
}
