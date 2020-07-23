/*
*this file is to build a gaitplanner to generate suitable gait
*
*/
#include "gaitPlanner.h"

GaitPlanner::GaitPlanner(){
  bodytoFeet.clear();
  bodytoFeet.resize(4);
  for(int i = 0; i < 4; i++){
    bodytoFeet[i].resize(3);
  }

  phi = 0.0;
  phiStance = 0.0;
  lastTime = 0.0;
  alpha = 0.0;
  S = false;
  first = true;
}

GaitPlanner::~GaitPlanner(){

}

double GaitPlanner::binomialFactor(int n, int k){
  int a,b,c;
  double d;
  a = calculateTool->factorial(n);
  b = calculateTool->factorial(k);
  c = calculateTool->factorial(n - k);
  d = double(a) / (double(b) * double(c));
  return d;
}

double GaitPlanner::bezier(double t, int k, double point){
  double result;
  double a,b;
  a = calculateTool->power(t, k);
  b = calculateTool->power(1-t, N-k);
  result = point * binomialFactor(N, k) * a * b;
  
  return result;
}

void GaitPlanner::calculateStance(double phi_st, double Velocity, double angle, std::vector<double> &stance){
  double c, s;
  double A, halfL, p_stance;

  c = cos(angle * PI / 180);
  s = sin(angle * PI / 180);

  A = 0.0;
  halfL = 0.1;
  p_stance = halfL * (1 - 2 * phi_st);

  stance[0] =  c * p_stance * fabs(Velocity);
  stance[1] = -s * p_stance * fabs(Velocity);
  stance[2] = -A * cos(PI / (2 * halfL) * p_stance);
}

void GaitPlanner::calculateBezier_swing(double phi_sw, double Velocity, double angle, std::vector<double> &swing){
  double c, s;
  double swingX, swingY, swingZ;
  double X[10];
  double Y[10];
  double Z[10];
  double listX[10] = {-0.05, -0.06, -0.07, -0.07, 0.0, 0.0, 0.07, 0.07, 0.06, 0.05};
  double listY[10] = {0.05, 0.06, 0.07, 0.07, 0.0, -0.0, -0.07, -0.07, -0.06, -0.05};
  double listZ[10] = {0.0, 0.0, 0.1, 0.1, 0.1, 0.12, 0.12, 0.12, 0.0, 0.0};

  c = cos(angle * PI / 180);
  s = sin(angle * PI / 180);

  for(int i = 0; i < 10; i++){
    X[i] = 2 * fabs(Velocity) * c * listX[i];
    Y[i] = 2 * fabs(Velocity) * s * listY[i];
    Z[i] = 0.3 * listZ[i];
  }

  swingX = 0.0;
  swingY = 0.0;
  swingZ = 0.0;
  for(int i = 0; i < 10; i++){  //sum all terms of the curve
    swingX = swingX + bezier(phi_sw, i, X[i]);
    swingY = swingY + bezier(phi_sw, i, Y[i]);
    swingZ = swingZ + bezier(phi_sw, i, Z[i]);
  }

  swing[0] = swingX;
  swing[1] = swingY;
  swing[2] = swingZ;
}

void GaitPlanner::stepTrajectory(double phi_step, double Velocity, double angle, double Wrot, 
                                    std::vector<double> centerToFoot, std::vector<double> &coord){
  double r;
  double footAngle, circleTrayectory;
  double stepOffset = 0.5;
  std::vector<double> step_long;
  std::vector<double> step_rot;
  step_long.clear();
  step_rot.clear();
  step_long.resize(3);
  step_rot.resize(3);

  if(phi_step > 1){
    phi_step = phi_step - 1;
  }

  r = sqrt(centerToFoot[0] * centerToFoot[0] + centerToFoot[1] * centerToFoot[1]);
  footAngle = atan2(centerToFoot[1], centerToFoot[0]);

  if(Wrot >= 0){
    circleTrayectory = 90 - (footAngle - alpha) * 180 / PI;
  }else{
    circleTrayectory = 270 - (footAngle - alpha) * 180 / PI;
  }


  if(phi_step <= stepOffset){ //stance phase
    phiStance = phi_step / stepOffset;
    calculateStance(phiStance, Velocity, angle, step_long);
    calculateStance(phiStance, Wrot, circleTrayectory, step_rot);
  }else{  //swing phase
    phiSwing = (phi_step - stepOffset) / (1 - stepOffset);
    calculateBezier_swing(phiSwing, Velocity, angle, step_long);
    calculateBezier_swing(phiSwing, Wrot, circleTrayectory, step_rot);
  }

  if(centerToFoot[1] > 0){
    if(step_rot[0] < 0){
      alpha = -atan2(sqrt(step_rot[0] * step_rot[0] + step_rot[1] * step_rot[1]), r);
    }else{
      alpha = atan2(sqrt(step_rot[0] * step_rot[0] + step_rot[1] * step_rot[1]), r);
    }
  }else{
    if(step_rot[0] < 0){
      alpha = atan2(sqrt(step_rot[0] * step_rot[0] + step_rot[1] * step_rot[1]), r);
    }else{
      alpha = -atan2(sqrt(step_rot[0] * step_rot[0] + step_rot[1] * step_rot[1]), r);
    }
  }

  coord[0] = step_long[0] + step_rot[0];
  coord[1] = step_long[1] + step_rot[1];
  coord[2] = step_long[2] + step_rot[2];
}

void GaitPlanner::loop(double Velocity, double angle, double Wrot, double T, 
                        std::vector<double> offset, std::vector<std::vector<double>> &bodytoFeet_){
  std::vector<double> step_coord;
  step_coord.clear();
  step_coord.resize(3);

  if(T <= 0.01){
    T = 0.01;
  }

  if(first){
    lastTime = calculateTool->getCurrentTime();
    first = false;
  }

  if(phi >= 0.99){
    lastTime = calculateTool->getCurrentTime();
  }

  phi = (calculateTool->getCurrentTime() - lastTime) / T;

  stepTrajectory(phi + offset[0], Velocity, angle, Wrot, bodytoFeet_[0], step_coord);
  bodytoFeet[0][0] = bodytoFeet_[0][0] + step_coord[0];
  bodytoFeet[0][1] = bodytoFeet_[0][1] + step_coord[1];
  bodytoFeet[0][2] = bodytoFeet_[0][2] + step_coord[2];
  // std::cout << "value is: " << bodytoFeet[0][0] << ", " << bodytoFeet[0][1] << ", " << bodytoFeet[0][2] << ", " << Wrot << std::endl;

  stepTrajectory(phi + offset[1], Velocity, angle, Wrot, bodytoFeet_[1], step_coord);
  bodytoFeet[1][0] = bodytoFeet_[1][0] + step_coord[0];
  bodytoFeet[1][1] = bodytoFeet_[1][1] + step_coord[1];
  bodytoFeet[1][2] = bodytoFeet_[1][2] + step_coord[2];

  stepTrajectory(phi + offset[2], Velocity, angle, Wrot, bodytoFeet_[2], step_coord);
  bodytoFeet[2][0] = bodytoFeet_[2][0] + step_coord[0];
  bodytoFeet[2][1] = bodytoFeet_[2][1] + step_coord[1];
  bodytoFeet[2][2] = bodytoFeet_[2][2] + step_coord[2];

  stepTrajectory(phi + offset[3], Velocity, angle, Wrot, bodytoFeet_[3], step_coord);
  bodytoFeet[3][0] = bodytoFeet_[3][0] + step_coord[0];
  bodytoFeet[3][1] = bodytoFeet_[3][1] + step_coord[1];
  bodytoFeet[3][2] = bodytoFeet_[3][2] + step_coord[2];

  
}
