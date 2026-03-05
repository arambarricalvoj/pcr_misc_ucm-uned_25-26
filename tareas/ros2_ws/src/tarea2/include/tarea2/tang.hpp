#pragma once
#include <array>

struct TangentialOutput {
    double w_tang = 0.0;
    bool active = false;
};

TangentialOutput compute_tangential_escape(
    double e_theta,
    double d,
    double A_front,
    double A_left,
    double A_right,
    double k_tan
);
