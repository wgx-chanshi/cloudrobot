#include "kinematic_solver.h"

KinematicModel::KinematicModel(){
  bodytoFR0.clear();
  bodytoFR0.resize(3);
  bodytoFL0.clear();
  bodytoFL0.resize(3);
  bodytoBR0.clear();
  bodytoBR0.resize(3);
  bodytoBL0.clear();
  bodytoBL0.resize(3);

  bodytoFR0[0] = Length / 2;
  bodytoFR0[1] = -Width / 2;
  bodytoFR0[2] = 0.0;
  bodytoFL0[0] = Length / 2;
  bodytoFL0[1] = Width / 2;
  bodytoFL0[2] = 0.0;
  bodytoBR0[0] = -Length / 2;
  bodytoBR0[1] = -Width / 2;
  bodytoBR0[2] = 0.0;
  bodytoBL0[0] = -Length / 2;
  bodytoBL0[1] = Width / 2;
  bodytoBL0[2] = 0.0;

}

KinematicModel::~KinematicModel(){

}

double KinematicModel::checkdomain(double D){
  if(D > 1.0 || D < -1.0){
    std::cout << "______OUT OF DOMAIN_______!" << std::endl;
    if(D > 1.0){
      D = 0.99;
      return D;
    }else if(D < -1.0){
      D = -0.99;
      return D;
    }
  }else{
    return D;
  }
}

void KinematicModel::inverseSolve_R(std::vector<double> coord, double coxa, double femur, double tibia, std::vector<double> &angles){
  double D, gamma;
  double tetta, alpha;

  D = (coord[1] * coord[1] + (-coord[2]) * (-coord[2]) - coxa * coxa + (-coord[0]) * (-coord[0]) - femur * femur - tibia * tibia) / (2 * tibia * femur);
  D = checkdomain(D);

  gamma = atan2(-sqrt(1 - D * D), D);
  tetta = -atan2(coord[2], coord[1]) - atan2(sqrt(coord[1] * coord[1] + (-coord[2]) * (-coord[2]) - coxa * coxa), -coxa);
  alpha = atan2(-coord[0], sqrt(coord[1] * coord[1] + (-coord[2]) * (-coord[2]) - coxa * coxa)) - atan2(tibia * sin(gamma), femur + tibia * cos(gamma));
  angles[0] = -tetta;
  angles[1] = alpha;
  angles[2] = gamma;
}

void KinematicModel::inverseSolve_L(std::vector<double> coord, double coxa, double femur, double tibia, std::vector<double> &angles){
  double D, gamma;
  double tetta, alpha;

  D = (coord[1] * coord[1] + (-coord[2]) * (-coord[2]) - coxa * coxa + (-coord[0]) * (-coord[0]) - femur * femur - tibia * tibia) / (2 * tibia * femur);
  D = checkdomain(D);

  gamma = atan2(-sqrt(1 - D * D), D);
  tetta = -atan2(coord[2], coord[1]) - atan2(sqrt(coord[1] * coord[1] + (-coord[2]) * (-coord[2]) - coxa * coxa), coxa);
  alpha = atan2(-coord[0], sqrt(coord[1] * coord[1] + (-coord[2]) * (-coord[2]) - coxa * coxa)) - atan2(tibia * sin(gamma), femur + tibia * cos(gamma));
  angles[0] = -tetta;
  angles[1] = alpha;
  angles[2] = gamma;
}

void KinematicModel::inverseKineSolver(std::vector<double> orn, std::vector<double> pos, std::vector<std::vector<double>> bodytoFeet,
                                       std::vector<std::vector<double>> &jointAngles, std::vector<std::vector<double>> &_bodytoFeet){
  std::vector<double> bodytoFR4, bodytoFL4, bodytoBR4, bodytoBL4;
  std::vector<double> _bodytoFR0, _bodytoFL0, _bodytoBR0, _bodytoBL0;
  std::vector<double> FRcoord, FLcoord, BRcoord, BLcoord;
  std::vector<double> _FRcoord, _FLcoord, _BRcoord, _BLcoord;
  std::vector<double> undoOrn, undoPos;
  
  bodytoFR4.clear();
  bodytoFR4.resize(3);
  bodytoFL4.clear();
  bodytoFL4.resize(3);
  bodytoBR4.clear();
  bodytoBR4.resize(3);
  bodytoBL4.clear();
  bodytoBL4.resize(3);

  _bodytoFR0.clear();
  _bodytoFR0.resize(3);
  _bodytoFL0.clear();
  _bodytoFL0.resize(3);
  _bodytoBR0.clear();
  _bodytoBR0.resize(3);
  _bodytoBL0.clear();
  _bodytoBL0.resize(3);

  FRcoord.clear();
  FRcoord.resize(3);
  FLcoord.clear();
  FLcoord.resize(3);
  BRcoord.clear();
  BRcoord.resize(3);
  BLcoord.clear();
  BLcoord.resize(3);

  _FRcoord.clear();
  _FRcoord.resize(3);
  _FLcoord.clear();
  _FLcoord.resize(3);
  _BRcoord.clear();
  _BRcoord.resize(3);
  _BLcoord.clear();
  _BLcoord.resize(3);

  undoOrn.clear();
  undoOrn.resize(3);
  undoPos.clear();
  undoPos.resize(3);

  for(int i = 0; i < 3; i++){
    bodytoFR4[i] = bodytoFeet[0][i];
    bodytoFL4[i] = bodytoFeet[1][i];
    bodytoBR4[i] = bodytoFeet[2][i];
    bodytoBL4[i] = bodytoFeet[3][i];
  }

  calculateTool2->transform(bodytoFR0, orn, pos, _bodytoFR0);
  calculateTool2->transform(bodytoFL0, orn, pos, _bodytoFL0);
  calculateTool2->transform(bodytoBR0, orn, pos, _bodytoBR0);
  calculateTool2->transform(bodytoBL0, orn, pos, _bodytoBL0);

  for(int i = 0; i < 3; i++){
    FRcoord[i] = bodytoFR4[i] - _bodytoFR0[i];
    FLcoord[i] = bodytoFL4[i] - _bodytoFL0[i];
    BRcoord[i] = bodytoBR4[i] - _bodytoBR0[i];
    BLcoord[i] = bodytoBL4[i] - _bodytoBL0[i];
  }

  for(int i = 0; i < 3; i++){
    undoOrn[i] = -orn[i];
    undoPos[i] = -pos[i];
  }

  calculateTool2->transform(FRcoord, undoOrn, undoPos, _FRcoord);
  calculateTool2->transform(FLcoord, undoOrn, undoPos, _FLcoord);
  calculateTool2->transform(BRcoord, undoOrn, undoPos, _BRcoord);
  calculateTool2->transform(BLcoord, undoOrn, undoPos, _BLcoord);

  // _FRcoord[0] = 0.2873678054962195;
  // _FRcoord[1] = -0.0671;
  // _FRcoord[2] = 0.08387872195106044;

  // std::cout << "sadajka: " << _FRcoord[0] << ",  " << _FRcoord[1] << ",  " << _FRcoord[2] << ",  " << std::endl;

  inverseSolve_R(_FRcoord, Coxa_Length, Femur_Length, Tibia_Length, jointAngles[0]);
  inverseSolve_L(_FLcoord, Coxa_Length, Femur_Length, Tibia_Length, jointAngles[1]);
  inverseSolve_R(_BRcoord, Coxa_Length, Femur_Length, Tibia_Length, jointAngles[2]);
  inverseSolve_L(_BLcoord, Coxa_Length, Femur_Length, Tibia_Length, jointAngles[3]);

  // std::cout << "sadajka: " << jointAngles[0][0] << ",  " << jointAngles[0][1] << ",  " << jointAngles[0][2] << ",  " << std::endl;

  for(int i = 0; i < 3; i++){
    _bodytoFeet[0][i] = _bodytoFR0[i] + _FRcoord[i];
    _bodytoFeet[1][i] = _bodytoFL0[i] + _FLcoord[i];
    _bodytoFeet[2][i] = _bodytoBR0[i] + _BRcoord[i];
    _bodytoFeet[3][i] = _bodytoBL0[i] + _BLcoord[i];
  }

}

