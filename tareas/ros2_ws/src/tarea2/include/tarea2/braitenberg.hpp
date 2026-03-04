#pragma once
#include <array>

struct BraitenbergOutput {
    double v_react;
    double w_react;
};

BraitenbergOutput compute_braitenberg(
    const std::array<double,8>& ir,
    double k_front = 0.8,
    double k_turn  = 1.2
);
