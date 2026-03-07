#pragma once

#include <string>
#include <vector>
#include <cmath>
#include <limits>

struct MetricsSample
{
    double time;
    double xr;
    double yr;
    double theta_r;
    double xt;
    double yt;
    double d;
    double e_theta;
    double v;
    double w;
    double d_min;
    int collision_flag;
};

struct Obstacle
{
    double x;
    double y;
    double r;
};

class MetricsLogger
{
public:
    MetricsLogger(const std::string &filename,
                  double kp, double vmax, double wmax, int controller_type,
                  const std::vector<Obstacle>& obstacles = {},
                  double xmin = NAN, double xmax = NAN, double ymin = NAN, double ymax = NAN,
                  double d_safe = 0.05, double collision_threshold = 0.0);

    ~MetricsLogger();

    // Actualiza métricas integrales; si dt <= 0 usa dt_ interno
    void updateMetrics(double error, double time, double dt = -1.0);

    // Añade una muestra; pasar dt real es opcional (si dt<=0 se usa dt_)
    void addSample(double time,
                   double xr, double yr, double theta_r,
                   double xt, double yt,
                   double d, double e_theta,
                   double v, double w,
                   double d_min,
                   double dt = -1.0);

    void saveToCSV();
    void clearSamples();

    // Getters
    const std::vector<MetricsSample>& samples() const { return samples_; }
    const std::vector<Obstacle>& obstacles() const { return obstacles_; }

    // Ajustes
    void setDt(double dt) { dt_ = dt; }
    void setDSafe(double d_safe) { d_safe_ = d_safe; }
    void setCollisionThreshold(double thr) { collision_threshold_ = thr; }

private:
    // métricas acumuladas
    double iae_  = 0.0;
    double ise_  = 0.0;
    double itae_ = 0.0;
    double itse_ = 0.0;

    // métricas auxiliares relacionadas con seguridad
    int collision_count_ = 0;
    double min_distance_ = std::numeric_limits<double>::infinity();
    double time_in_danger_ = 0.0;

    // paso de muestreo por defecto
    double dt_ = 0.05;

    std::string filename_;

    // Configuración del controlador
    double kp_   = 0.0;
    double vmax_ = 0.0;
    double wmax_ = 0.0;
    int controller_type_ = 0;

    // parámetros de seguridad
    double d_safe_ = 0.05;
    double collision_threshold_ = 0.0;

    // muestras y obstáculos
    std::vector<MetricsSample> samples_;
    std::vector<Obstacle> obstacles_;

    // límites del campo (opcional)
    double xmin_ = NAN;
    double xmax_ = NAN;
    double ymin_ = NAN;
    double ymax_ = NAN;
};
