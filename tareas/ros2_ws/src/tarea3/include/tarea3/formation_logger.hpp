#ifndef FORMATION_LOGGER_HPP_
#define FORMATION_LOGGER_HPP_

#include <fstream>
#include <string>

class FormationLogger {
public:
    FormationLogger(const std::string &filename)
    {
        file_.open(filename);
        file_ << "t,xL,yL,thetaL,x1,y1,theta1,x2,y2,theta2,x3,y3,theta3\n";
    }

    ~FormationLogger() {
        if (file_.is_open()) file_.close();
    }

    void log(double t,
             double xL, double yL, double thL,
             double x1, double y1, double th1,
             double x2, double y2, double th2,
             double x3, double y3, double th3)
    {
        file_ << t << ","
              << xL << "," << yL << "," << thL << ","
              << x1 << "," << y1 << "," << th1 << ","
              << x2 << "," << y2 << "," << th2 << ","
              << x3 << "," << y3 << "," << th3 << "\n";
    }

private:
    std::ofstream file_;
};

#endif
