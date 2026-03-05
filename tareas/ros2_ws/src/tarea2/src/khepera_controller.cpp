#include <filesystem>
#include <cmath>

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "tarea2/khepera_controller.hpp"
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
    // PARÁMETROS
    // -------------------------
    this->declare_parameter("kp", 0.5);
    this->declare_parameter("vmax", 0.15);
    this->declare_parameter("wmax", 1.0);
    this->declare_parameter("controller_type", 0);

    // Parámetros Braitenberg
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
    // NOMBRE DEL CONTROLADOR
    // -------------------------
    std::string controller_name =
        (controller_type_ == 0) ? "icc_reactivo" : "icc_geometrico";

    std::string kp_str = std::to_string(kp_);
    kp_str.erase(kp_str.find_last_not_of('0') + 1);
    if (kp_str.back() == '.') kp_str.pop_back();

    // -------------------------
    // RUTA MÉTRICAS
    // -------------------------
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
    ir_ranges_.fill(0.25);  // sin obstáculo

    for (int i = 0; i < 8; ++i)
    {
        std::string topic = "/khepera/ir" + std::to_string(i+1);
        ir_subs_[i] = create_subscription<sensor_msgs::msg::Range>(
            topic, 10,
            [this, i](sensor_msgs::msg::Range::SharedPtr msg)
            {
                double r = msg->range;
                //if (r < msg->min_range) r = msg->min_range;
                if (r < msg->min_range) r = msg->min_range;
                if (r > msg->max_range) r = msg->max_range;
                ir_ranges_[i] = r;
                //RCLCPP_INFO(this->get_logger(), "ir%d, r = %.5f", 
                //    i, r);
                /*RCLCPP_INFO(this->get_logger(),
                    "IR (RearLeft, Left, FrontLeft, Front, FrontRight, Right, RearRight, Rear): %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f",
                    ir_ranges_[0], ir_ranges_[1], ir_ranges_[2], ir_ranges_[3],
                    ir_ranges_[4], ir_ranges_[5], ir_ranges_[6], ir_ranges_[7]);*/
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
// CONTROL LOOP (GEOMÉTRICO/ICC + BRAITENBERG)
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

    // -------------------------
    // CONTROL HACIA EL OBJETIVO
    // -------------------------
    double v_goal = kp_ * d;
    if (v_goal > vmax_) v_goal = vmax_;

    double w_goal = 0.0;

    // ICC reactivo
    w_goal = wmax_ * std::sin(e_theta);

    // -------------------------
    // BRAITENBERG (MÓDULO EXTERNO)
    // -------------------------
    auto b = compute_braitenberg(ir_ranges_, k_front_, k_turn_);

    double factor = std::clamp(d / ks_, 0.0, 1.0);
    double v = v_goal + factor * b.v_react;
    double w = w_goal + factor * b.w_react;

    if (controller_type_ == 1){
        auto t = compute_tangential_escape(
            e_theta, d,
            b.A_front, b.A_left, b.A_right,
            k_tan_);

        if (t.active) {
            w += t.w_tang;
        }
    }

    RCLCPP_INFO(this->get_logger(), "v_goal = %.3f, w_goal = %.3f", 
            v_goal, w_goal);
    
    RCLCPP_INFO(this->get_logger(), "b.v_react = %.3f, b.w_react = %.3f", 
            b.v_react, b.w_react);


    /*RCLCPP_INFO(this->get_logger(),
            "IR (RearLeft, Left, FrontLeft, Front, FrontRight, Right, RearRight, Rear): %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f",
            ir_ranges_[0], ir_ranges_[1], ir_ranges_[2], ir_ranges_[3],
            ir_ranges_[4], ir_ranges_[5], ir_ranges_[6], ir_ranges_[7]);*/

    RCLCPP_INFO(this->get_logger(),
            "IR (RearLeft): %.5f",
            ir_ranges_[0]);

    if (v < 0.0) v = 0.0;
    if (v > vmax_) v = vmax_;
    if (w > wmax_) w = wmax_;
    if (w < -wmax_) w = -wmax_;

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
