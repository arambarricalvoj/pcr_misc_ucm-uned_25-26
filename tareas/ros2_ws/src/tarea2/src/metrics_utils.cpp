#include "tarea2/metrics_utils.hpp"
#include <fstream>
#include <iomanip>
#include <sstream>

MetricsLogger::MetricsLogger(const std::string &filename,
                             double kp, double vmax, double wmax, int controller_type,
                             const std::vector<Obstacle>& obstacles,
                             double xmin, double xmax, double ymin, double ymax,
                             double d_safe, double collision_threshold)
    : kp_(kp), vmax_(vmax), wmax_(wmax), controller_type_(controller_type),
      obstacles_(obstacles), xmin_(xmin), xmax_(xmax), ymin_(ymin), ymax_(ymax),
      d_safe_(d_safe), collision_threshold_(collision_threshold)
{
    if (filename.size() >= 4 && filename.substr(filename.size()-4) == ".csv") {
        filename_ = filename;
    } else {
        filename_ = filename + ".csv";
    }
}

MetricsLogger::~MetricsLogger()
{
    // Guardar si hay muestras
    if (!samples_.empty()) {
        try { saveToCSV(); } catch (...) {}
    }
}

void MetricsLogger::updateMetrics(double error, double time, double dt)
{
    double use_dt = (dt > 0.0) ? dt : dt_;
    iae_  += std::abs(error) * use_dt;
    ise_  += error * error * use_dt;
    itae_ += time * std::abs(error) * use_dt;
    itse_ += time * error * error * use_dt;
}

void MetricsLogger::addSample(double time,
                              double xr, double yr, double theta_r,
                              double xt, double yt,
                              double d, double e_theta,
                              double v, double w,
                              double d_min,
                              double dt)
{
    MetricsSample s;
    s.time = time;
    s.xr = xr; s.yr = yr; s.theta_r = theta_r;
    s.xt = xt; s.yt = yt;
    s.d = d;
    s.e_theta = e_theta;
    s.v = v;
    s.w = w;
    s.d_min = d_min;
    s.collision_flag = (d_min <= collision_threshold_) ? 1 : 0;

    samples_.push_back(s);

    // actualizar métricas auxiliares
    if (s.collision_flag) collision_count_++;
    if (d_min < min_distance_) min_distance_ = d_min;

    // error definido como violación de distancia de seguridad
    double err = std::max(0.0, d_safe_ - d_min);

    double use_dt = (dt > 0.0) ? dt : dt_;
    iae_  += std::abs(err) * use_dt;
    ise_  += err * err * use_dt;
    itae_ += time * std::abs(err) * use_dt;
    itse_ += time * err * err * use_dt;

    if (err > 0.0) time_in_danger_ += use_dt;
}

void MetricsLogger::saveToCSV()
{
    std::ofstream file(filename_, std::ios::out);
    if (!file.is_open()) return;

    file << "# Metrics CSV\n";
    file << "# kp," << kp_ << ",vmax," << vmax_ << ",wmax," << wmax_ << ",controller_type," << controller_type_ << "\n";

    file << "# obstacles:";
    for (const auto &o : obstacles_) {
        file << " (" << o.x << "," << o.y << "," << o.r << ")";
    }
    file << "\n";

    if (!std::isnan(xmin_) && !std::isnan(xmax_) && !std::isnan(ymin_) && !std::isnan(ymax_)) {
        file << "# field_limits: xmin," << xmin_ << ",xmax," << xmax_ << ",ymin," << ymin_ << ",ymax," << ymax_ << "\n";
    }

    file << "# d_safe," << d_safe_ << ",collision_threshold," << collision_threshold_ << "\n";

    file << "time,xr,yr,theta_r,xt,yt,d,e_theta,v,w,d_min,collision_flag\n";
    file << std::fixed << std::setprecision(6);

    for (const auto &s : samples_) {
        file << s.time << ","
             << s.xr << "," << s.yr << "," << s.theta_r << ","
             << s.xt << "," << s.yt << ","
             << s.d << "," << s.e_theta << ","
             << s.v << "," << s.w << ","
             << s.d_min << "," << s.collision_flag << "\n";
    }

    file << "\n# Metrics\n";
    file << "# IAE,"  << iae_  << "\n";
    file << "# ISE,"  << ise_  << "\n";
    file << "# ITAE," << itae_ << "\n";
    file << "# ITSE," << itse_ << "\n";

    file << "# collision_count," << collision_count_ << "\n";
    file << "# min_distance," << min_distance_ << "\n";
    file << "# time_in_danger," << time_in_danger_ << "\n";

    file << "\n# Parameters\n";
    file << "# kp,"   << kp_   << "\n";
    file << "# vmax," << vmax_ << "\n";
    file << "# wmax," << wmax_ << "\n";
    file << "# controller_type," << controller_type_ << "\n";

    file.close();
}

void MetricsLogger::clearSamples()
{
    samples_.clear();
}
