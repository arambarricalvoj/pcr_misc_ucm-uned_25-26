#pragma once

#include <array>
#include <utility>
#include <cstddef>

class BraitenbergRepulsion
{
public:
  // Constructor
  BraitenbergRepulsion(
    double wheel_base,
    double v0,
    double vmin,
    double vmax,
    double max_range_default,
    double max_detection_dist,
    const std::array<double,8> &L,
    const std::array<double,8> &R);

  // Actualizar lectura de un sensor IR (índice 0..7)
  void updateRange(size_t idx, double range, double min_range, double max_range);

  // Calcular velocidades de rueda repulsivas {vL, vR} en m/s
  std::pair<double,double> computeWheelSpeeds() const;

  // Obtener la mínima distancia detectada por los sensores (m)
  double getMinDetection() const;

  // Reestablecer lecturas al valor por defecto
  void resetRanges();

  // Cambiar pesos en tiempo de ejecución
  void setWeights(const std::array<double,8> &L, const std::array<double,8> &R);

  // Obtener activaciones normalizadas por sensor (0..1)
  std::array<double,8> getActivations() const;

private:
  // Curva de activación del sensor (0..1)
  double sensorActivation(double range) const;

  std::array<double,8> ranges_;       // lecturas actuales (m)
  std::array<double,8> weightsL_;     // pesos Braitenberg izquierda
  std::array<double,8> weightsR_;     // pesos Braitenberg derecha

  double wheel_base_;
  double v0_;                         // factor de escala de velocidad
  double vmin_;
  double vmax_;
  double max_range_default_;
  double max_detection_dist_;
};