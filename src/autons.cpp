#include "robot-config.h"
#include "vex.h"
#include "PID.h"
#include "arm.h"
#include "functions.h"
#include "autons.h"

using namespace vex;


//red left side of the field//
void rightAuton() {
  drivePID(26, 0.15, 0.004, 0);
  drivePID(9, 0.15, 0.004, 0);
  turnPID(-45,0.35,0.004,0);
  drivePID(5, 0.15, 0.004, 0);
  drivePID(-35, 0.15, 0.004, 0);
  turnPID(44, 0.6, 0, 0);
  drivePID(-13, 0.15, 0.004, 0);
  drivePID(20, 0.15, 0.004, 0);



  // If Auton is marked WRONG, Click with 2 Fingers on the bulb and click "add AUTON to workspace".//
  }



void leftAuton() {
  drivePID(26, 0.15, 0.004, 0);
  drivePID(9, 0.15, 0.004, 0);
  turnPID(45,0.35,0.004,0);
  drivePID(7.5, 0.15, 0.004, 0);
  drivePID(-40, 0.15, 0.004, 0);
  turnPID(-50, 0.6, 0, 0);
  drivePID(7, 0.15, 0.004, 0);
  drivePID(-18, 0.15, 0.004, 0);
} 


// Void skills for going SOLO//




void skillsAuton() {
  drivePID(25, 0.2, 0.01, 0.3);
  turnPID(-90, 0.2, 0.01, 0.3);
  drivePID(18.3, 0.2, 0.01, 0.3);
  turnPID(-87, 0.2, 0.01, 0.3);
  drivePID(22.5, 0.2, 0.01, 0.3);
  drivePID(-23, 0.2, 0.01, 0.3); 
  drivePID(4,0.2,0.01,0.3);
  turnPID(-90,0.3,0.01,0.3);
  drivePID(6,0.2,0.01,0.3);
  turnPID(-72,0.2,0.01,0.3);
  drivePID(19,0.2,0.01,0.3);
  turnPID(20,0.2,0.01,0.3);
  //drivePID(50,0.2,0.01,0.3);
  //turnPID(90,0.2,0.01,0.3);
  //drivePID(6,0.2,0.01,0.3);
  //turnPID(-90,0.2,0.01,0.3);
  //drivePID(6,0.2,0.01,0.3);
  //drivePID(-19,0.2,0.01,0.3);
  //drivePID(4,0.2,0.01,0.3);
  //turnPID(90,0.2,0.01,0.3);
  //drivePID(60,0.2,0.01,0.3);
} 


  


//drivePID(26, 0.15, 0.004, 0);
  //drivePID(9, 0.15, 0.004, 0);
  //turnPID(-45,0.35,0.004,0);
  //drivePID(5, 0.15, 0.004, 0);
  //drivePID(-35, 0.15, 0.004, 0);
  //turnPID(44, 0.6, 0, 0);
  //drivePID(-13, 0.15, 0.004, 0);
  //drivePID(20, 0.15, 0.004, 0);
  //actual drivetrain VEX ports
  //port:1 - right back 
  //port:2 - right middle 
  //port:3 - right front 
  //port:4 - left back 
  //port:5 - left middle 
  //port:6 - left front 