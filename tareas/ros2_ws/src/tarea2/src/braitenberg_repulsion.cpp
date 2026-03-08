#include "tarea2/braitenberg_repulsion.hpp"
#include <cmath>
#include <algorithm>
#include <limits>

BraitenbergRepulsion::BraitenbergRepulsion(
    double wheel_base,
    double v0,
    double vmin,
    double vmax,
    double max_range_default,
    double max_detection_dist,
    const std::array<double,8> &L,
    const std::array<double,8> &R)
: ranges_()
, weightsL_(L)
, weightsR_(R)
, wheel_base_(wheel_base)
, v0_(v0)
, vmin_(vmin)
, vmax_(vmax)
, max_range_default_(max_range_default)
, max_detection_dist_(max_detection_dist)
{
    ranges_.fill(max_range_default_);
}

void BraitenbergRepulsion::updateRange(size_t idx, double range, double /*min_range*/, double /*max_range*/)
{
    if (idx >= ranges_.size()) return;
    double r = std::clamp(range, 0.0, max_range_default_);
    ranges_[idx] = r;
}

double BraitenbergRepulsion::sensorActivation(double range) const
{
    if (range <= 0.0) return 1.0;
    if (range >= max_range_default_) return 0.0;
    double x = (max_range_default_ - range) / max_range_default_; // 0..1
    return x * x; // curva cuadrática (más sensibilidad cerca)
}

std::pair<double,double> BraitenbergRepulsion::computeWheelSpeeds() const
{
    double vL = 0.0;
    double vR = 0.0;

    for (size_t i = 0; i < ranges_.size(); ++i) {
        double a = sensorActivation(ranges_[i]); // 0..1
        vL += weightsL_[i] * a * v0_;
        vR += weightsR_[i] * a * v0_;
    }

    auto saturate = [this](double v) {
        return std::clamp(v, vmin_, vmax_);
    };

    vL = saturate(vL);
    vR = saturate(vR);

    return {vL, vR};
}

double BraitenbergRepulsion::getMinDetection() const
{
    double dmin = std::numeric_limits<double>::infinity();
    for (double r : ranges_) {
        if (r < dmin) dmin = r;
    }
    return dmin;
}

void BraitenbergRepulsion::resetRanges()
{
    ranges_.fill(max_range_default_);
}

void BraitenbergRepulsion::setWeights(const std::array<double,8> &L, const std::array<double,8> &R)
{
    weightsL_ = L;
    weightsR_ = R;
}

std::array<double,8> BraitenbergRepulsion::getActivations() const
{
    std::array<double,8> acts;
    for (size_t i = 0; i < ranges_.size(); ++i) acts[i] = sensorActivation(ranges_[i]);
    return acts;
}