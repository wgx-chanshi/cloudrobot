#ifndef _KINEMATIC_SOLVER_H_
#define _KINEMATIC_SOLVER_H_

#include "calculateTool.h"
#include <math.h>
#include <stdlib.h>
#include <iostream>
#include <vector>

#define Length 0.378
#define Width  0.099
#define Coxa_Length 0.0671
#define Femur_Length 0.206
#define Tibia_Length 0.185

CalculateTool *calculateTool2;

class KinematicModel{
  public:
    KinematicModel();
    ~KinematicModel();

    double checkdomain(double D);
    void inverseSolve_R(std::vector<double> coord, double coxa, double femur, double tibia, std::vector<double> &angles);
    void inverseSolve_L(std::vector<double> coord, double coxa, double femur, double tibia, std::vector<double> &angles);
    void inverseKineSolver(std::vector<double> orn, std::vector<double> pos, std::vector<std::vector<double>> bodytoFeet,
                           std::vector<std::vector<double>> &jointAngles, std::vector<std::vector<double>> &_bodytoFeet);

    std::vector<double> bodytoFR0;
    std::vector<double> bodytoFL0;
    std::vector<double> bodytoBR0;
    std::vector<double> bodytoBL0;
};

#endif