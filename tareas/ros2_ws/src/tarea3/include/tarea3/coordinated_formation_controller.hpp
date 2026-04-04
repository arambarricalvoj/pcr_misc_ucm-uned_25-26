#ifndef COORDINATED_FORMATION_CONTROLLER_HPP_
#define COORDINATED_FORMATION_CONTROLLER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include <tuple>
#include <string>

#include "formation_logger.hpp"

class CoordinatedFormationController : public rclcpp::Node
{
public:
    CoordinatedFormationController();

private:

    void masterPoseCallback(const geometry_msgs::msg::Pose::SharedPtr msg);
    void masterCmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

    void slave1Callback(const geometry_msgs::msg::Pose::SharedPtr msg);
    void slave2Callback(const geometry_msgs::msg::Pose::SharedPtr msg);
    void slave3Callback(const geometry_msgs::msg::Pose::SharedPtr msg);

    void controlLoop();

    std::tuple<double,double,double> getAnglesForFormation();

    std::tuple<
        std::pair<double,double>,
        std::pair<double,double>,
        std::pair<double,double>>
    getRelativePositions();

    std::pair<double,double> computeTarget(double x_rel, double y_rel);

    void publishRigid(
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub,
        const geometry_msgs::msg::Pose &pose,
        double xd, double yd,
        double x_rel, double y_rel);

    double extractYaw(const geometry_msgs::msg::Pose &pose);
    void normalizeAngle(double &a);

    // Variables
    std::unique_ptr<FormationLogger> logger_;
    double time_elapsed_ = 0.0;
    
    std::string formation_type_;

    geometry_msgs::msg::Pose master_pose_;
    geometry_msgs::msg::Pose slave1_pose_;
    geometry_msgs::msg::Pose slave2_pose_;
    geometry_msgs::msg::Pose slave3_pose_;

    bool master_pose_received_ = false;
    bool master_cmd_received_ = false;
    bool slave1_received_ = false;
    bool slave2_received_ = false;
    bool slave3_received_ = false;

    double v_L_ = 0.0;
    double w_L_ = 0.0;

    // Parámetros YAML (coinciden EXACTAMENTE con tu params.yaml)
    double kx_;
    double ky_;

    double dist_stationary_thresh_;
    double dist_dynamic_thresh_;

    double v_max_;
    double w_max_;

    double leader_v_eps_;
    double leader_w_eps_;

    // ROS2
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr master_pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr master_cmd_sub_;

    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr slave1_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr slave2_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr slave3_sub_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr slave1_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr slave2_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr slave3_pub_;

    rclcpp::TimerBase::SharedPtr timer_;
};

#endif
