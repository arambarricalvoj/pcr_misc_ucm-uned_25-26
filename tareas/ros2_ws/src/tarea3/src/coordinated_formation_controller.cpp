#include "tarea3/coordinated_formation_controller.hpp"
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

CoordinatedFormationController::CoordinatedFormationController()
: Node("coordinated_formation_controller")
{
    // Parámetros YAML (coinciden con tu params.yaml)
    formation_type_         = declare_parameter<std::string>("formation_type", "cross");

    kx_                     = declare_parameter<double>("kx", 0.5);
    ky_                     = declare_parameter<double>("ky", 1.2);

    dist_stationary_thresh_ = declare_parameter<double>("dist_stationary_thresh", 0.03);
    dist_dynamic_thresh_    = declare_parameter<double>("dist_dynamic_thresh", 0.12);

    v_max_                  = declare_parameter<double>("v_max", 0.15);
    w_max_                  = declare_parameter<double>("w_max", 1.0);

    leader_v_eps_           = declare_parameter<double>("leader_v_eps", 0.01);
    leader_w_eps_           = declare_parameter<double>("leader_w_eps", 0.01);

    // Subscripciones
    master_pose_sub_ = create_subscription<geometry_msgs::msg::Pose>(
        "/master_khepera_iv/robot_pose", 10,
        std::bind(&CoordinatedFormationController::masterPoseCallback, this, std::placeholders::_1));

    master_cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "/master_khepera_iv/cmd_vel", 10,
        std::bind(&CoordinatedFormationController::masterCmdCallback, this, std::placeholders::_1));

    slave1_sub_ = create_subscription<geometry_msgs::msg::Pose>(
        "/slave_1_khepera_iv/robot_pose", 10,
        std::bind(&CoordinatedFormationController::slave1Callback, this, std::placeholders::_1));

    slave2_sub_ = create_subscription<geometry_msgs::msg::Pose>(
        "/slave_2_khepera_iv/robot_pose", 10,
        std::bind(&CoordinatedFormationController::slave2Callback, this, std::placeholders::_1));

    slave3_sub_ = create_subscription<geometry_msgs::msg::Pose>(
        "/slave_3_khepera_iv/robot_pose", 10,
        std::bind(&CoordinatedFormationController::slave3Callback, this, std::placeholders::_1));

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
        std::bind(&CoordinatedFormationController::controlLoop, this));
}

// ---------------- CALLBACKS ----------------
void CoordinatedFormationController::masterPoseCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
{
    master_pose_ = *msg;
    master_pose_received_ = true;
}

void CoordinatedFormationController::masterCmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    v_L_ = msg->linear.x;
    w_L_ = msg->angular.z;
    master_cmd_received_ = true;
}

void CoordinatedFormationController::slave1Callback(const geometry_msgs::msg::Pose::SharedPtr msg)
{
    slave1_pose_ = *msg;
    slave1_received_ = true;
}

void CoordinatedFormationController::slave2Callback(const geometry_msgs::msg::Pose::SharedPtr msg)
{
    slave2_pose_ = *msg;
    slave2_received_ = true;
}

void CoordinatedFormationController::slave3Callback(const geometry_msgs::msg::Pose::SharedPtr msg)
{
    slave3_pose_ = *msg;
    slave3_received_ = true;
}

// ---------------- CONTROL LOOP ----------------
void CoordinatedFormationController::controlLoop()
{
    if (!master_pose_received_ ||
        !slave1_received_ || !slave2_received_ || !slave3_received_)
        return;

    auto [p1, p2, p3] = getRelativePositions();

    auto [x1_rel, y1_rel] = p1;
    auto [x2_rel, y2_rel] = p2;
    auto [x3_rel, y3_rel] = p3;

    auto [x1d, y1d] = computeTarget(x1_rel, y1_rel);
    auto [x2d, y2d] = computeTarget(x2_rel, y2_rel);
    auto [x3d, y3d] = computeTarget(x3_rel, y3_rel);

    publishRigid(slave1_pub_, slave1_pose_, x1d, y1d, x1_rel, y1_rel);
    publishRigid(slave2_pub_, slave2_pose_, x2d, y2d, x2_rel, y2_rel);
    publishRigid(slave3_pub_, slave3_pose_, x3d, y3d, x3_rel, y3_rel);
}

// ---------------- FORMACIÓN ----------------
std::tuple<double,double,double>
CoordinatedFormationController::getAnglesForFormation()
{
    if (formation_type_ == "cross")
        return { 90.0, -90.0, 180.0 };
    else if (formation_type_ == "circle")
        return { 120.0, -120.0, 180.0 };
    else if (formation_type_ == "line")
        return { 90.0, -90.0, -90.0 };

    return { 90.0, -90.0, 0.0 };
}

std::tuple<
    std::pair<double,double>,
    std::pair<double,double>,
    std::pair<double,double>>
CoordinatedFormationController::getRelativePositions()
{
    auto [b1, b2, b3] = getAnglesForFormation();

    double r = 0.4;
    double r3 = (formation_type_ == "line") ? 0.8 : 0.4;

    auto toXY = [](double r, double b){
        double rad = b * M_PI / 180.0;
        return std::make_pair(r * std::cos(rad), r * std::sin(rad));
    };

    return { toXY(r,b1), toXY(r,b2), toXY(r3,b3) };
}

// ---------------- TARGET ----------------
std::pair<double,double>
CoordinatedFormationController::computeTarget(double x_rel, double y_rel)
{
    double xL = master_pose_.position.x;
    double yL = master_pose_.position.y;
    double thetaL = extractYaw(master_pose_);

    double xd = xL + std::cos(thetaL)*x_rel - std::sin(thetaL)*y_rel;
    double yd = yL + std::sin(thetaL)*x_rel + std::cos(thetaL)*y_rel;

    return {xd, yd};
}

// ---------------- UTILS ----------------
double CoordinatedFormationController::extractYaw(const geometry_msgs::msg::Pose &pose)
{
    double z = pose.orientation.z;
    double w = pose.orientation.w;
    return std::atan2(2*w*z, 1 - 2*z*z);
}

void CoordinatedFormationController::normalizeAngle(double &a)
{
    while (a > M_PI) a -= 2*M_PI;
    while (a < -M_PI) a += 2*M_PI;
}

// ---------------- CONTROL RÍGIDO + ESTACIONARIO ----------------
void CoordinatedFormationController::publishRigid(
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub,
    const geometry_msgs::msg::Pose &pose,
    double xd, double yd,
    double x_rel, double y_rel)
{
    double xs = pose.position.x;
    double ys = pose.position.y;

    double theta_s = extractYaw(pose);
    double theta_L = extractYaw(master_pose_);

    double ex = xd - xs;
    double ey = yd - ys;

    double dist = std::sqrt(ex*ex + ey*ey);

    double ex_r =  std::cos(theta_s)*ex + std::sin(theta_s)*ey;
    double ey_r = -std::sin(theta_s)*ex + std::cos(theta_s)*ey;

    geometry_msgs::msg::Twist cmd;

    // ⭐ CORRECCIÓN CLAVE:
    // Antes del primer cmd_vel → consideramos que el líder está parado
    bool leader_stopped =
        (!master_cmd_received_) ||
        (std::fabs(v_L_) < leader_v_eps_ && std::fabs(w_L_) < leader_w_eps_);

    // --------------------------
    // MODO ESTACIONARIO
    // --------------------------
    if (leader_stopped)
    {
        if (dist > dist_stationary_thresh_) {
            double theta_d = std::atan2(ey, ex);
            double e_theta2 = theta_d - theta_s;
            normalizeAngle(e_theta2);

            cmd.linear.x  = std::clamp(0.5 * dist, -v_max_, v_max_);
            cmd.angular.z = std::clamp(1.2 * e_theta2, -w_max_, w_max_);
        }
        else {
            double e_theta3 = theta_L - theta_s;
            normalizeAngle(e_theta3);

            cmd.linear.x = 0.0;
            cmd.angular.z = std::clamp(1.0 * e_theta3, -0.8, 0.8);
        }

        pub->publish(cmd);
        return;
    }

    // --------------------------
    // MODO RÍGIDO (DINÁMICO)
    // --------------------------
    double v_cmd;
    double w_cmd;

    if (dist > dist_dynamic_thresh_) {
        v_cmd = v_L_ + kx_ * ex_r;
        if (ex_r < 0.0)
            v_cmd = 0.0;

        w_cmd = w_L_ + ky_ * ey_r;
    } else {
        double theta_d = std::atan2(ey, ex);
        double e_theta_pos = theta_d - theta_s;
        normalizeAngle(e_theta_pos);

        double e_theta_L = theta_L - theta_s;
        normalizeAngle(e_theta_L);

        v_cmd = v_L_ + kx_ * ex_r;
        if (ex_r < 0.0)
            v_cmd = 0.0;

        w_cmd = w_L_ + ky_ * ey_r + 1.0 * e_theta_L;
    }

    cmd.linear.x  = std::clamp(v_cmd, -v_max_, v_max_);
    cmd.angular.z = std::clamp(w_cmd, -w_max_, w_max_);

    pub->publish(cmd);
}
