#include "tarea2/braitenberg_puro.hpp"

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <limits>

#include <cmath>

#include "ament_index_cpp/get_package_share_directory.hpp"

using namespace std::chrono_literals;

BraitenbergController::BraitenbergController()
: Node("braitenberg_controller")
{
    // Parámetros por defecto
    this->declare_parameter<double>("wheel_radius", 0.021);
    this->declare_parameter<double>("wheel_base",   0.088);
    this->declare_parameter<double>("v0",           0.6);
    this->declare_parameter<double>("vmin",         0.0);
    this->declare_parameter<double>("vmax",         1.0);
    //this->declare_parameter<double>("wmax",         1.0);
    this->declare_parameter<double>("max_detection_dist", 0.02);
    this->declare_parameter<double>("max_range_default", 0.25);
    this->declare_parameter<double>("control_hz", 20.0);

    // Pesos Braitenberg (vector cargable desde YAML)
    this->declare_parameter<std::vector<double>>("GL",
        std::vector<double>{0.0,0.0,0.0,0.0,-0.025,-0.0375,-0.05,-0.000625});
    this->declare_parameter<std::vector<double>>("GR",
        std::vector<double>{-0.000625,-0.05,-0.0375,-0.025,0.0,0.0,0.0,0.0});

    // Parámetro opcional de obstáculos (declararlo para evitar excepciones si no está en YAML)
    this->declare_parameter<std::vector<double>>("obstacles", std::vector<double>{});
    // d_safe opcional (no obligatorio para el CSV_ mínimo)
    this->declare_parameter<double>("d_safe", 0.05);

    // Parámetros nuevos: limitación de aceleración y tiempo para guardar métricas
    this->declare_parameter<double>("max_accel_linear", 0.5);   // m/s^2
    this->declare_parameter<double>("max_accel_angular", 1.0);  // rad/s^2
    this->declare_parameter<double>("save_after_seconds", 25.0);

    // Leer parámetros
    wheel_radius_ = this->get_parameter("wheel_radius").as_double();
    wheel_base_   = this->get_parameter("wheel_base").as_double();
    v0_           = this->get_parameter("v0").as_double();
    vmin_         = this->get_parameter("vmin").as_double();
    vmax_         = this->get_parameter("vmax").as_double();
    //wmax_         = this->get_parameter("wmax").as_double();
    max_detection_dist_ = this->get_parameter("max_detection_dist").as_double();
    max_range_default_  = this->get_parameter("max_range_default").as_double();
    control_hz_ = this->get_parameter("control_hz").as_double();

    auto Lvec = this->get_parameter("braitenbergL").as_double_array();
    auto Rvec = this->get_parameter("braitenbergR").as_double_array();

    d_safe_ = this->get_parameter("d_safe").as_double();

    max_accel_linear_  = this->get_parameter("max_accel_linear").as_double();
    max_accel_angular_ = this->get_parameter("max_accel_angular").as_double();
    save_after_seconds_ = this->get_parameter("save_after_seconds").as_double();

    for (size_t i = 0; i < 8; ++i) {
        braitenbergL_[i] = (i < Lvec.size()) ? Lvec[i] : 0.0;
        braitenbergR_[i] = (i < Rvec.size()) ? Rvec[i] : 0.0;
    }

    // Leer obstáculos (lista plana x,y,r,x,y,r,...). Si no hay, queda vacío.
    auto obs_vec = this->get_parameter("obstacles").as_double_array();
    for (size_t i = 0; i + 2 < obs_vec.size(); i += 3) {
        obstacles_.push_back({ obs_vec[i], obs_vec[i+1], obs_vec[i+2] });
    }

    // Inicializar rangos IR
    ir_ranges_.fill(max_range_default_);

    //prev_v_ = v0_;
    //prev_w_ = 0;

    // Suscriptores IR
    for (int i = 0; i < 8; ++i) {
        std::string topic = "/khepera/ir" + std::to_string(i+1);
        ir_subs_[i] = this->create_subscription<sensor_msgs::msg::Range>(
            topic, 10,
            [this, i](sensor_msgs::msg::Range::SharedPtr msg) {
                this->irCallback(i, msg);
            }
        );
    }

    // Suscriptor pose
    pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
        "/robot_pose", 10,
        std::bind(&BraitenbergController::robotPoseCallback, this, std::placeholders::_1)
    );

    // Publicador cmd_vel
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // Preparar ruta para métricas (igual que en tus otros controladores)
    std::string pkg_share = ament_index_cpp::get_package_share_directory("tarea2");
    std::filesystem::path pkg_path = std::filesystem::path(pkg_share)
        .parent_path().parent_path().parent_path().parent_path();
    std::string results_dir = (pkg_path / "src" / "tarea2" / "results").string() + "/";
    std::filesystem::create_directories(results_dir);

    controller_name_ = "braitenberg";
    std::ostringstream ss;
    ss << controller_name_;
    std::string fname = (std::filesystem::path(results_dir) / ss.str()).string();

    RCLCPP_INFO(this->get_logger(), "Metrics file base: %s", fname.c_str());
    RCLCPP_INFO(this->get_logger(), "max_accel_linear=%.3f max_accel_angular=%.3f save_after_seconds=%.1f",
                max_accel_linear_, max_accel_angular_, save_after_seconds_);

    // Crear MetricsLogger original (misma firma que usas en ICCController)
    metrics_ = std::make_unique<MetricsLogger>(
        fname,
        /*kp*/ 0.0, /*vmax*/ vmax_, /*wmax*/ 0.0, /*controller_type*/ 0
    );

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

void BraitenbergController::robotPoseCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
{
    robot_pose_ = *msg;
    pose_received_ = true;
}

double BraitenbergController::computeMinDistanceToObstacles(double x, double y) const
{
    if (obstacles_.empty()) return std::numeric_limits<double>::infinity();
    double dmin = std::numeric_limits<double>::infinity();
    for (const auto &o : obstacles_) {
        double dx = x - o[0];
        double dy = y - o[1];
        double dist_center = std::hypot(dx, dy);
        double dist_edge = dist_center - o[2];
        if (dist_edge < dmin) dmin = dist_edge;
    }
    return dmin;
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

    double dt = 1.0 / control_hz_;
    if (dt <= 0.0) dt = 0.05;

    // Clamp para evitar reversa y paradas bruscas: [vmin_, vmax_]
    vLeft_lin  = std::clamp(vLeft_lin,  vmin_, vmax_);
    vRight_lin = std::clamp(vRight_lin, vmin_, vmax_);

    // Convertir a velocidad lineal y angular del chasis
    double V_ = (vRight_lin + vLeft_lin) / 2.0;
    double W_ = (vRight_lin - vLeft_lin) / wheel_base_;

    // Acc
    /*double delta_v = V_ - prev_v_;
    double delta_w = W_ - prev_w_;
    if (abs(delta_v) > max_accel_linear_){
        V_ = prev_v_ + delta_v;
        prev_v_ = V_;
    }
    if (abs(delta_w > max_accel_angular_)){
        W_ = prev_w_ + delta_w;
        prev_w_ = W_;
    }*/

    // Publicar Twist
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = V_;
    cmd.angular.z = W_;
    cmd_pub_->publish(cmd);

    // Actualizar prev para la siguiente iteración
    //prev_v_ = V_;
    //prev_w_ = W_;

    // Preparar datos para métricas: solo si tenemos pose
    if (!pose_received_) {
        // Si no hay pose, no registramos la muestra (evita NaNs en pose)
        return;
    }

    double xr = robot_pose_.position.x;
    double yr = robot_pose_.position.y;

    // quaternion -> yaw
    double qx = robot_pose_.orientation.x;
    double qy = robot_pose_.orientation.y;
    double qz = robot_pose_.orientation.z;
    double qw = robot_pose_.orientation.w;
    double siny_cosp = 2.0 * (qw * qz + qx * qy);
    double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    double theta_r = std::atan2(siny_cosp, cosy_cosp);

    // tiempo (usamos acumulado estilo ICCController)
    time_elapsed_ += dt;
    double time = time_elapsed_;

    // Llamada mínima a MetricsLogger: guardamos time, pose y velocidades.
    // Rellenamos xt, yt, d, e_theta con NaN porque no los usamos ahora.
    if (metrics_) {
        metrics_->addSample(
            time,
            xr, yr, theta_r,
            std::numeric_limits<double>::quiet_NaN(), // xt
            std::numeric_limits<double>::quiet_NaN(), // yt
            std::numeric_limits<double>::quiet_NaN(), // d
            std::numeric_limits<double>::quiet_NaN(), // e_theta
            V_, W_
        );
    }

    // Guardar métricas una sola vez cuando haya pasado save_after_seconds_
    if (!metrics_saved_ && time_elapsed_ >= save_after_seconds_) {
        if (metrics_) {
            RCLCPP_INFO(this->get_logger(), "%.1f s elapsed — saving metrics CSV_ now.", save_after_seconds_);
            metrics_->saveToCSV();
            metrics_saved_ = true;
            RCLCPP_INFO(this->get_logger(), "Metrics saved to CSV_.");
        }
    }
}
