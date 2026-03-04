#pragma once

#include <string>
#include <vector>

struct MetricsSample
{
    double time;
    double xr, yr, theta_r;
    double xt, yt;
    double d;
    double e_theta;
    double v;
    double w;
};

class MetricsLogger
{
public:
    MetricsLogger(const std::string &filename,
              double kp, double vmax, double wmax, int controller_type);

    void updateMetrics(double error, double time);
    void addSample(double time,
                   double xr, double yr, double theta_r,
                   double xt, double yt,
                   double d, double e_theta,
                   double v, double w);

    void saveToCSV();

private:
    double iae_  = 0.0;
    double ise_  = 0.0;
    double itae_ = 0.0;
    double itse_ = 0.0;

    double dt_ = 0.05;

    std::string filename_;

    // Configuración del controlador
    double kp_, vmax_, wmax_;
    int controller_type_;

    std::vector<MetricsSample> samples_;
};
