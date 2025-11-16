/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       katherinezeng                                             */
/*    Created:      5/27/2025, 10:30:19 PM                                    */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/


#include "vex.h"
#include "robot-config.h"
#include "PID.h"
#include "autons.h"
#include "auton-selector.h"
#include "arm.h"
#include "functions.h"

using namespace vex;

// A global instance of competition
competition Competition;


// define your global instances of motors and other devices here

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...


  vexcodeInit();
  displayAutonSelector();
  
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  
  switch (getSelectedAuton()) {
    case 0:
      rightAuton();
      break;
    case 1:
      leftAuton();
      break;
    case 2:
      skillsAuton();
      break;
    default:
      Brain.Screen.print("No valid auton selected!");
      break;
  }
 


}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

bool inauton = false;

// Drive train functions in functions.cpp



  void checkDoubleClick();
int main();
// User control function
void usercontrol(void) {
  bool slowModeActive = false;

  while (true) {
    // ========== DRIVE CONTROL ========== //
    double fwd = Controller.Axis3.position(percentUnits::pct);
    double turn = Controller.Axis1.position(percentUnits::pct);
    double leftPower  = fwd + turn;
    double rightPower = fwd - turn;

    spinLeftDT(leftPower * 0.8);
    spinRightDT(rightPower * 0.8);
    spinLeftDT(leftPower * 0.7);
    spinRightDT(rightPower * 0.7);

    // ========== ARM CONTROL ========== //
   // if (Controller.ButtonR1.pressing()) {
     // Arm1.spin(forward, 75, percent);
     // Arm2.spin(forward, 75, percent);
    //} else if (Controller.ButtonR2.pressing()) {
     // Arm1.spin(reverse, 75, percent);
     // Arm2.spin(reverse, 75, percent);
    //} else {
      //Arm1.stop(hold);
     // Arm2.stop(hold);
    //}

    // ========== INTAKE ========== //
    //out put for the intake//
    // if (Controller.ButtonL1.pressing()) {
    // bottomIntakeMotor.spin(forward, 100, percent);
    //   middleIntakeMotor.spin(reverse, 100, percent);
    //   topIntakeMotor.spin(forward, 100, percent); 
    // wait(20, msec);
    // } 
   
    //   else if (Controller.ButtonL2.pressing()) {
    //  bottomIntakeMotor.spin(reverse, 100, percent);
    //   middleIntakeMotor.spin(forward, 100, percent);
    //   topIntakeMotor.spin(reverse, 100, percent); } 
  
    //   else if (Controller.ButtonR1.pressing()){
    //     bottomIntakeMotor.stop();
    //     middleIntakeMotor.stop();
    //     topIntakeMotor.stop();
    //   }
    // }

  
  
   
    
    // ========== COLOR SENSOR ========== //
    // color detectedColor = OpticalSensor.color();
    // if (detectedColor == color::blue) {
    //   Brain.Screen.printAt(50, 50, "Blue Detected!");
    // } else if (detectedColor == color::red) {
    //   Brain.Screen.printAt(50, 50, "Red Detected!");
    // } else {
    //   Brain.Screen.printAt(50, 50, "No specific color detected.");
    // }

    // double hue = OpticalSensor.hue();
    // double brightness = OpticalSensor.brightness();
    // Brain.Screen.printAt(50, 70, "Hue: %.2f", hue);
    // Brain.Screen.printAt(50, 90, "Brightness: %.2f", brightness);
    // Brain.Screen.clearScreen();

    // ========== LOOP DELAY ========== //
    wait(20, msec);
  }
}

//void driver controll(){
//doble forwards = Controller1.Axis2.position();
//doble turning = Controller1.Axis4}

  

// Entry point
int main() {
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  
  
  pre_auton();
  //Competition.test_auton();
  //Competition.test_driver();


  while (true) {
    wait(100, msec);
  }
}






