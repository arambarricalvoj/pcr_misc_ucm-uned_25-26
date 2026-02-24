#pragma once

#include "geometry_msgs/msg/pose.hpp"
#include <cmath>

inline void extractPose(
    const geometry_msgs::msg::Pose &pose,
    double &x, double &y, double &yaw)
{
    x = pose.position.x;
    y = pose.position.y;

    double qw = pose.orientation.w;
    double qx = pose.orientation.x;
    double qy = pose.orientation.y;
    double qz = pose.orientation.z;

    yaw = std::atan2(2*(qw*qz + qx*qy),
                     1 - 2*(qy*qy + qz*qz));
}

inline double normalizeAngle(double a)
{
    return std::atan2(std::sin(a), std::cos(a));
}
