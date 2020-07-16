#ifndef _CALCULATETOOL_H_
#define _CALCULATETOOL_H_

#include <math.h>
#include <sys/time.h>
#include <unistd.h>
#include <Eigen/Core>
#include <vector>

using namespace Eigen;

class CalculateTool{
  public:
    int factorial(int n);
    double power(double a, int k);
    double getCurrentTime();
    void getRx(double roll, Matrix4d &rotationX);
    void getRy(double pitch, Matrix4d &rotationY);
    void getRz(double yaw, Matrix4d &rotationZ);
    void RTmatrix(std::vector<double> orientation, std::vector<double> position, Matrix4d &translateMatrix);
    void transform(std::vector<double> coord, std::vector<double> orientation, 
                   std::vector<double> position, std::vector<double> &transformVector);
};

#endif