#include "calculateTool.h"
#include <math.h>
#include <vector>
#include <iostream>

#define PI 3.14159265
#define N  9

CalculateTool *calculateTool;

class GaitPlanner{
  public:
    GaitPlanner();
    ~GaitPlanner();
    double binomialFactor(int n, int k);
    double bezier(double t, int k, double point);
    void calculateStance(double phi_st, double Velocity, double angle, std::vector<double> &stance);
    void calculateBezier_swing(double phi_sw, double Velocity, double angle, std::vector<double> &swing);
    void stepTrajectory(double phi, double Velocity, double angle, double Wrot, 
                        std::vector<double> centerToFoot, std::vector<double> &coord);
    void loop(double Velocity, double angle, double Wrot, double T, 
              std::vector<double> offset, std::vector<std::vector<double>> &bodytoFeet_);

    std::vector<std::vector<double>> bodytoFeet;
    double phi;
    double phiStance, phiSwing;
    double lastTime;
    double alpha;
    bool S;
    bool first;
};