#include "tarea2/controllers.hpp"
#include <cmath>
#include <algorithm>

// ---------------- ReactiveController ----------------
ReactiveController::ReactiveController(double kp, double vmax, double wmax)
: kp_(kp), vmax_(vmax), wmax_(wmax) {}

std::pair<double,double> ReactiveController::computeCommand(const ControllerInput &in) {
  double dx = in.xt - in.xr;
  double dy = in.yt - in.yr;
  double d = std::hypot(dx, dy);
  double theta_d = std::atan2(dy, dx);
  double e_theta = theta_d - in.theta_r;
  // normalizar
  while (e_theta > M_PI) e_theta -= 2.0*M_PI;
  while (e_theta < -M_PI) e_theta += 2.0*M_PI;

  double v = kp_ * d;
  v = std::min(v, vmax_);
  double w = wmax_ * std::sin(e_theta);
  return {v, w};
}

// ---------------- GeometricController ----------------
GeometricController::GeometricController(double kp, double vmax, double wmax, double eps_sin)
: kp_(kp), vmax_(vmax), wmax_(wmax), eps_sin_(eps_sin) {}

std::pair<double,double> GeometricController::computeCommand(const ControllerInput &in) {
  double dx = in.xt - in.xr;
  double dy = in.yt - in.yr;
  double d = std::hypot(dx, dy);
  double theta_d = std::atan2(dy, dx);
  double e_theta = theta_d - in.theta_r;
  while (e_theta > M_PI) e_theta -= 2.0*M_PI;
  while (e_theta < -M_PI) e_theta += 2.0*M_PI;

  double v = kp_ * d;
  v = std::min(v, vmax_);

  double s = std::sin(e_theta);
  double w;
  if (std::abs(s) < eps_sin_) {
    w = wmax_ * s;
  } else {
    double R = d / (2.0 * s);
    if (std::isfinite(R) && std::abs(R) > 1e-6) w = v / R;
    else w = wmax_ * s;
  }
  return {v, w};
}

// ---------------- Factory ----------------
std::unique_ptr<BaseController> createController(int controller_type,
                                                 double kp,
                                                 double vmax,
                                                 double wmax,
                                                 double eps_sin) {
  if (controller_type == 0) {
    return std::make_unique<ReactiveController>(kp, vmax, wmax);
  } else {
    return std::make_unique<GeometricController>(kp, vmax, wmax, eps_sin);
  }
}