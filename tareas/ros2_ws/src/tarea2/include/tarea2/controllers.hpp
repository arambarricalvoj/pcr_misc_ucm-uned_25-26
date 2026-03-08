#pragma once

#include <utility>
#include <memory>

// Entrada para el controlador
struct ControllerInput {
  double xr;
  double yr;
  double theta_r;
  double xt;
  double yt;
  double dt;
};

// Clase base simple (definida en el mismo módulo)
class BaseController {
public:
  virtual ~BaseController() = default;
  // Devuelve {v, w}
  virtual std::pair<double,double> computeCommand(const ControllerInput &in) = 0;
  virtual void reset() = 0;
};

// Reactive controller (implementación simple)
class ReactiveController : public BaseController {
public:
  ReactiveController(double kp, double vmax, double wmax);
  std::pair<double,double> computeCommand(const ControllerInput &in) override;
  void reset() override {}
private:
  double kp_;
  double vmax_;
  double wmax_;
};

// Geometric controller (curvature based)
class GeometricController : public BaseController {
public:
  GeometricController(double kp, double vmax, double wmax, double eps_sin);
  std::pair<double,double> computeCommand(const ControllerInput &in) override;
  void reset() override {}
private:
  double kp_;
  double vmax_;
  double wmax_;
  double eps_sin_;
};

// Factory: crea el controlador según type (0 reactivo, 1 geométrico)
std::unique_ptr<BaseController> createController(int controller_type,
                                                 double kp,
                                                 double vmax,
                                                 double wmax,
                                                 double eps_sin = 1e-3);