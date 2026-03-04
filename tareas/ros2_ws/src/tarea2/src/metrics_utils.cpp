#include "tarea2/metrics_utils.hpp"
#include <fstream>
#include <cmath>

MetricsLogger::MetricsLogger(const std::string &filename,
                             double kp, double vmax, double wmax, int controller_type)
    : kp_(kp), vmax_(vmax), wmax_(wmax), controller_type_(controller_type)
{
    filename_ = filename + ".csv";
}

void MetricsLogger::updateMetrics(double error, double time)
{
    iae_  += std::abs(error) * dt_;
    ise_  += error * error * dt_;
    itae_ += time * std::abs(error) * dt_;
    itse_ += time * error * error * dt_;
}

void MetricsLogger::addSample(double time,
                              double xr, double yr, double theta_r,
                              double xt, double yt,
                              double d, double e_theta,
                              double v, double w)
{
    samples_.push_back({time, xr, yr, theta_r, xt, yt, d, e_theta, v, w});
}

void MetricsLogger::saveToCSV()
{
    std::ofstream file(filename_);

    // Cabecera
    file << "time,xr,yr,theta_r,xt,yt,d,e_theta,v,w\n";

    // Datos por iteración
    for (const auto &s : samples_) {
        file << s.time << ","
             << s.xr << "," << s.yr << "," << s.theta_r << ","
             << s.xt << "," << s.yt << ","
             << s.d << "," << s.e_theta << ","
             << s.v << "," << s.w << "\n";
    }

    // Métricas finales
    file << "\nMetric,Value\n";
    file << "IAE,"  << iae_  << "\n";
    file << "ISE,"  << ise_  << "\n";
    file << "ITAE," << itae_ << "\n";
    file << "ITSE," << itse_ << "\n";

    // Configuración del controlador
    file << "\nParameter,Value\n";
    file << "kp,"   << kp_   << "\n";
    file << "vmax," << vmax_ << "\n";
    file << "wmax," << wmax_ << "\n";
    file << "controller_type," << controller_type_ << "\n";

    file.close();
}
