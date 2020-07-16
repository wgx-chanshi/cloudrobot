#include <ros/ros.h>
#include <math.h>
#include <iostream> 
#include <string>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include "gaitPlanner.h"
#include "kinematic_solver.h"

#define PI 3.14159265
#define T 0.25
#define freq 200
#define Xdist 0.38
#define Ydist 0.2322
#define height 0.26

double rotSpeed = 0.0;
double gaitLength = 0.0;
double gaitYaw = 0.0;
double robotX = 0.0;
double robotY = 0.0;
double robotZ = 0.0;
double robotRoll = 0.0;
double robotPitch = 0.0;
double robotYaw = 0.0;
double compensateReal[12] = {-1, -1, -1, 1, -1, -1, -1, 1, 1, 1, 1, 1};
// double compensateSim[3] = {1, -1, -1};
std::string joint_name[12] = {"abduct_fl", "thigh_fl", "knee_fl", "abduct_hl", "thigh_hl", "knee_hl",
                             "abduct_fr", "thigh_fr", "knee_fr", "abduct_hr", "thigh_hr", "knee_hr"};
std::vector<double> pos;
std::vector<double> orn;
std::vector<double> offset;
std::vector<std::vector<double>> bodytoFeet0; //initial foot position
std::vector<std::vector<double>> jointAngles;
std::vector<std::vector<double>> _bodytoFeet;

GaitPlanner *gaitplanner;
KinematicModel *kinematicmodel;


void QuadrupedCtrl(){
  

}

void velCmdCallBack(const geometry_msgs::Twist& msg){
  double linearX, linearY;

  linearX = msg.linear.x;
  linearY = -msg.linear.y;
  rotSpeed = msg.angular.x * 0.25;
   
  if(fabs(linearX) < 0.1 && fabs(linearY) < 0.1){
    gaitLength = 0;
    gaitYaw = 0;
  }else{
    gaitLength = sqrt(linearX * linearX + linearY * linearY) * 0.5;
    gaitYaw = atan2(linearY, linearX) * 180 / PI;    //degrees
  }

}

void PoseCmdCallBack(const geometry_msgs::Twist& msg){
  robotRoll += 0.1 * msg.angular.x;
  if(robotRoll > PI/4){
    robotRoll = PI/4;
  }else if(robotRoll < -PI/4){
    robotRoll = -PI/4;
  }

  robotPitch += 0.1 * msg.angular.y;
  if(robotPitch > PI/4){
    robotPitch = PI/4;
  }else if(robotPitch < -PI/4){
    robotPitch = -PI/4;
  }

  robotYaw += 0.1 * msg.angular.z;
  if(robotYaw > PI/4){
    robotYaw = PI/4;
  }else if(robotYaw < -PI/4){
    robotYaw = -PI/4;
  }
  
}

int main(int argc, char **argv) {

  // std::vector<std::vector<double>> feet;

  // feet.clear();
  // feet.resize(4);
  // feet[0].resize(3);
  // feet[1].resize(3);
  // feet[2].resize(3);
  // feet[3].resize(3);

  // for(int i = 0; i < 4; i++){
  //   feet[i][0] = 0.19;
  //   feet[i][1] = 0.1161;
  //   feet[i][2] = -0.26;
  // }

  gaitplanner = new GaitPlanner();
  kinematicmodel = new KinematicModel();

  offset.clear();
  offset.resize(4);
  offset[0] = 0.5;
  offset[1] = 0.0;
  offset[2] = 0.0;
  offset[3] = 0.5;

  bodytoFeet0.clear();
  bodytoFeet0.resize(4);
  jointAngles.clear();
  jointAngles.resize(4);
  _bodytoFeet.clear();
  _bodytoFeet.resize(4);
  for(int i = 0; i < 4; i++){
    bodytoFeet0[i].resize(3);
    jointAngles[i].resize(3);
    _bodytoFeet[i].resize(3);
  }

  //initial foot positions
  bodytoFeet0[0][0] = Xdist/2;
  bodytoFeet0[0][1] = -Ydist/2;
  bodytoFeet0[0][2] = -height;
  bodytoFeet0[1][0] = Xdist/2;
  bodytoFeet0[1][1] = Ydist/2;
  bodytoFeet0[1][2] = -height;
  bodytoFeet0[2][0] = -Xdist/2;
  bodytoFeet0[2][1] = -Ydist/2;
  bodytoFeet0[2][2] = -height;
  bodytoFeet0[3][0] = -Xdist/2;
  bodytoFeet0[3][1] = Ydist/2;
  bodytoFeet0[3][2] = -height;



  pos.clear();
  pos.resize(3);
  pos[0] = robotX;
  pos[1] = robotY;
  pos[2] = robotZ;

  orn.clear();
  orn.resize(3);
  orn[0] = robotRoll;
  orn[1] = robotPitch;
  orn[2] = robotYaw;

  ros::init(argc, argv, "quadruped_robot");
  ros::NodeHandle n;

  ros::Rate loop_rate(freq);
  sensor_msgs::JointState joint_state;

  ros::Publisher pub_joint = n.advertise<sensor_msgs::JointState>("set_js", 1000);
  ros::Subscriber sub_vel = n.subscribe("cmd_vel", 1000, velCmdCallBack);
  ros::Subscriber sub_pos = n.subscribe("cmd_pose", 1000, PoseCmdCallBack);

  while (ros::ok()){
    joint_state.header.stamp = ros::Time::now();
    joint_state.name.resize(12);
    joint_state.position.resize(12);
    // std::cout << "gaitLength: " << gaitLength << " , " << "gaitYaw: " << gaitYaw << " , " << "rotSpeed: " << rotSpeed << " , " << "T: " << T << std::endl;
    gaitplanner->loop(gaitLength, gaitYaw, rotSpeed, T, offset, bodytoFeet0);

    kinematicmodel->inverseKineSolver(orn, pos, gaitplanner->bodytoFeet, jointAngles, _bodytoFeet);
    // kinematicmodel->inverseKineSolver(orn, pos, feet, jointAngles, _bodytoFeet);

    // for(int i = 0; i < 4; i++){
    //   for(int j = 0; j < 3; j++){
    //     std::cout << "the value is: " << gaitplanner->bodytoFeet[i][j] << ", ";
    //     if(i == 3 && j == 2){
    //       std::cout << std::endl;
    //     }
    //   }
    // }

    // for(int i = 0; i < 4; i++){
    //   for(int j = 0; j < 3; j++){
    //     std::cout << "the angle is: " << jointAngles[i][j] << ", ";
    //     if(i == 3 && j == 2){
    //       std::cout << std::endl;
    //     }
    //   }
    // }

    for(int i = 0; i < 4; i++){
      for(int j = 0; j < 3; j++){
        joint_state.name[3 * i + j] = joint_name[3 * i + j];
        joint_state.position[3 * i + j] = jointAngles[i][j];
        // std::cout << "the angle is :" << jointAngles[i][j] << std::endl;
      }  
    }

    pub_joint.publish(joint_state);
    ros::spinOnce();
    loop_rate.sleep();
  }


  return 0;
}