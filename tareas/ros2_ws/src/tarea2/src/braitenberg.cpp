#include "tarea2/braitenberg.hpp"
#include <algorithm>
#include <iostream>
#include <iomanip>

static double activation(double r)
{
    double a = 1.0 - (r / 0.25);   // 0.25 = max_range IR
    return std::clamp(a, 0.0, 1.0);
}

BraitenbergOutput compute_braitenberg(
    const std::array<double,8>& ir,
    double k_front,
    double k_turn)
{
    // Mapeo:
    // 0: rear-left
    // 1: left
    // 2: front-left
    // 3: front
    // 4: front-right
    // 5: right
    // 6: rear-right
    // 7: rear

    double a_rear_l = activation(ir[0]); //rear_left
    double a_l  = activation(ir[1]); //l
    double a_fl = activation(ir[2]); //FL
    double a_f = activation(ir[3]); //F
    double a_fr = activation(ir[4]); //FR
    double a_r  = activation(ir[5]); //R
    double a_rear_r = activation(ir[6]); //rear_right
    double a_rear = activation(ir[7]); //rear

    double A_front = (a_fl + a_f + a_fr) / 3.0;
    double A_left  = (a_fl + a_rear_l + a_l) / 3.0;
    double A_right = (a_fr + a_rear_r + a_r) / 3.0;

    BraitenbergOutput out;
    out.v_react = -k_front * A_front;
    //out.w_react =  k_turn  * (A_right - A_left);
    out.w_react = k_turn * A_front;

    return out;
}
