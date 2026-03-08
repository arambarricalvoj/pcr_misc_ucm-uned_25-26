#include "tarea2/braitenberg_repulsion.hpp"
#include <cmath>
#include <algorithm>
#include <limits>
#include <iostream>
#include <sstream>
#include <iomanip>

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
    std::cerr << "[BRAIT] v0="<<v0_<<" vmin="<<vmin_<<" vmax="<<vmax_<<" max_range_default="<<max_range_default_<<"\n";
for (size_t i=0;i<8;++i) std::cerr << "wL["<<i<<"]="<<weightsL_[i]<<" wR["<<i<<"]="<<weightsR_[i]<<"\n";

}

void BraitenbergRepulsion::updateRange(size_t idx, double range, double /*min_range*/, double /*max_range*/)
{
    if (idx >= ranges_.size()) return;
    double r = std::clamp(range, 0.02, max_range_default_);
    ranges_[idx] = r;
std::cerr << "[BRAIT] updateRange idx="<<idx<<" stored_range="<<ranges_[idx]<<"\n";

}

double BraitenbergRepulsion::sensorActivation(double range) const
{
    // Lecturas inválidas -> no detección
    if (!std::isfinite(range) || range <= 0.0) return 0.0;

    // Fuera del rango máximo del sensor -> no detección
    if (range >= max_range_default_) return 0.0;

    // Evitar división por cero si los parámetros están mal configurados
    double denom = max_range_default_ - max_detection_dist_;
    if (denom <= 0.0) return 0.0;

    // Normalización idéntica a tu detect[]
    double d = range;
    if (d < max_detection_dist_) d = max_detection_dist_;

    double dnorm = 1.0 - ((d - max_detection_dist_) / denom);
    dnorm = std::clamp(dnorm, 0.0, 1.0);

    return dnorm;
}


/*std::pair<double,double> BraitenbergRepulsion::computeWheelSpeeds() const
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
}*/
std::pair<double,double> BraitenbergRepulsion::computeWheelSpeeds() const
{
    double vL = 0.0;
    double vR = 0.0;

    for (size_t i = 0; i < ranges_.size(); ++i) {
        double a = sensorActivation(ranges_[i]); // 0..1
        double contribL = weightsL_[i] * a * v0_;
        double contribR = weightsR_[i] * a * v0_;
        vL += contribL;
        vR += contribR;

        std::cerr << std::fixed << std::setprecision(6)
                  << "[BRAIT ACT] i=" << i
                  << " range=" << ranges_[i]
                  << " act=" << a
                  << " wL=" << weightsL_[i]
                  << " wR=" << weightsR_[i]
                  << " cL=" << contribL
                  << " cR=" << contribR << "\n";
    }

    std::cerr << std::fixed << std::setprecision(6)
              << "[BRAIT RAW] vL_raw=" << vL << " vR_raw=" << vR << " v0=" << v0_ << "\n";

    // temporalmente comentar saturación para ver valores brutos
    // auto saturate = [this](double v) { return std::clamp(v, vmin_, vmax_); };
    // vL = saturate(vL);
    // vR = saturate(vR);

    std::cerr << std::fixed << std::setprecision(6)
              << "[BRAIT OUT] vL=" << vL << " vR=" << vR << "\n";

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