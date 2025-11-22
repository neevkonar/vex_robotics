/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       aland                                                     */
/*    Created:      3/12/2025, 8:09:57 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"

using namespace vex;


// START V5 MACROS
#define waitUntil(condition)                                                   \
  do {                                                                         \
    wait(5, msec);                                                             \
  } while (!(condition))

#define repeat(iterations)                                                     \
  for (int iterator = 0; iterator < iterations; iterator++)
// END V5 MACROS


// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here
brain Brain;
// Robot configuration code.
controller Controller1 = controller(primary);
motor intakeMotorA = motor(PORT5, ratio6_1, true);
motor intakeMotorB = motor(PORT2, ratio6_1, false);
motor_group intake = motor_group(intakeMotorA, intakeMotorB);

motor wall_StakeMotor = motor(PORT11, ratio18_1, false);

digital_out clamp = digital_out(Brain.ThreeWirePort.E);

motor leftMotorA = motor(PORT4, ratio6_1, true);
motor leftMotorB = motor(PORT1, ratio6_1, true);
motor leftMotorC = motor(PORT8, ratio6_1, false);
motor_group LeftDriveSmart = motor_group(leftMotorA, leftMotorB, leftMotorC);

motor rightMotorA = motor(PORT3, ratio6_1, false);
motor rightMotorB = motor(PORT7, ratio6_1, false);
motor rightMotorC = motor(PORT9, ratio6_1, true);
motor_group RightDriveSmart = motor_group(rightMotorA, rightMotorB, rightMotorC);

inertial DrivetrainInertial = inertial(PORT6);
smartdrive Drivetrain = smartdrive(LeftDriveSmart, RightDriveSmart, DrivetrainInertial, 299.24, 320, 40, mm, 0.4);

digital_out armThingy = digital_out(Brain.ThreeWirePort.D);


static void drivePID(double kp, double ki, double kd, double target, int timeout) 
{
  double lefterror = target;
  double plefterror = lefterror;
  double leftd = 0;
  double lefti = 0;
  double reallefti = 0;
  double plefttotal = 0;
  double lefttotal = 0;
  double leftsaturation = 0;
  double leftsign = 0;

  double righterror = target;
  double prighterror = righterror;
  double rightd = 0;
  double righti = 0;
  double realrighti = 0;
  double prighttotal = 0;
  double righttotal = 0;
  double rightsaturation = 0;
  double rightsign = 0;
  int    last_time;

  LeftDriveSmart.resetPosition();
  RightDriveSmart.resetPosition();

  last_time = time(NULL);
  while((fabs(lefterror) + fabs(righterror)) / 2 > 0.2) {
      if (timeout) {
        if ((time(NULL) - last_time)>timeout) {
          break;
        }
      }
      lefterror = target - LeftDriveSmart.position(turns) * 3.25 * M_PI * 0.6;
      righterror = target - RightDriveSmart.position(turns) * 3.25 * M_PI * 0.6;
      
      leftd = (lefterror - plefterror) * 40;
      rightd = (righterror - prighterror) * 40;
      
      lefttotal = lefterror * kp + lefti * ki - leftd * kd;
      righttotal = righterror * kp + righti * ki - rightd * kd;

      if(plefttotal == lefttotal) {
          leftsaturation = 0;
      }
      else {
          leftsaturation = 1;
      }
      if((plefttotal < 0) && (lefttotal < 0)) {
          leftsign = -1;
      }
      else {
          leftsign = 1;
      }

      if(prighttotal == righttotal) {
          rightsaturation = 0;
      }
      else {
          rightsaturation = 1;
      }
      if((plefttotal < 0) && (lefttotal < 0)) {
          rightsign = -1;
      }
      else {
          rightsign = 1;
      }

      LeftDriveSmart.spin(forward, lefttotal, pct);
      RightDriveSmart.spin(forward, righttotal, pct);

      if((leftsaturation == 1) || (leftsign == 1 )) {
          lefti = 0;
      }
      else {
         lefti = reallefti; 
      }
      if(fabs(lefterror) < 10) {
          reallefti += lefterror / 50;
      }
      
      if((rightsaturation == 1) || (rightsign == 1 )) {
          righti = 0;
      }
      else {
        righti = realrighti;  
      }
      if(fabs(righterror) < 10) {
          realrighti += righterror / 50;
      }

      plefterror = lefterror;
      prighterror = righterror;

      wait(20, msec);
  }

  LeftDriveSmart.stop(brake);
  RightDriveSmart.stop(brake);

  wait(100, msec);
}

#define max(a,b) (((a)>(b))? (a):(b))
#define min(a,b) (((a)<(b))? (a):(b))

static double geterror(double target) {
  if((max(target, DrivetrainInertial.heading()) - min(target, DrivetrainInertial.heading())) > 180) {
      if(min(target, DrivetrainInertial.heading()) == target) {
          return (360 - max(target, DrivetrainInertial.heading()) + min(target, DrivetrainInertial.heading()));
      }
      else {
          return -(360 - max(target, DrivetrainInertial.heading()) + min(target, DrivetrainInertial.heading()));
      }
  }
  else {
      return (target - DrivetrainInertial.heading());
  }
}

static void turnPID(double kp, double ki, double kd, double target, int timeout) {
  double error = geterror(target);
  double perror = error;
  double d = 0;
  double i = 0;
  double reali = 0;
  double ptotal  = 0;
  double total = 0;
  double saturation = 0;
  double sign = 0;
  int last_time = time(NULL);

  while(fabs(error) > 1 || (d > 3)) {
      if (timeout) {
        if ((time(NULL) - last_time) > timeout) {
            break;
        }
      }
      error = geterror(target);
      d = (error - perror) * 40;
      ptotal = total;
      total = error * kp + i * ki - d * kd;
      if(ptotal == total) {
          saturation = 0;
      }
      else {
          saturation = 1;
      }
      if((ptotal < 0) && (total < 0)) {
          sign = -1;
      }
      else {
          sign = 1;
      }
      
      LeftDriveSmart.spin(forward, total, pct);
      RightDriveSmart.spin(reverse, total, pct);

      if((saturation == 1) && (sign == 1)) {
          i = 0;
      }
      else {
          i = reali;
      }
      if(fabs(error) < 20) {
          i += error/50;
      }
      
      perror = error;

      wait(20, msec);
  }

  LeftDriveSmart.stop(brake);
  RightDriveSmart.stop(brake);

  wait(100, msec);
}

static void drive(vex::directionType dir, double target, int timeout) {
if (dir == forward) {            
      drivePID(1.8, 0.005, 0.75, target, timeout);
  }

  if(dir == reverse) {
        drivePID(1.8, 0.005, 0.75, -target, timeout);
  }
}



static void turn(double target, int timeout) {
  turnPID(0.42, 0.001, 0, target, timeout);
}

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/
bool vexcode_initial_drivetrain_calibration_completed = false;
void calibrateDrivetrain() {
  wait(200, msec);
  Brain.Screen.print("Calibrating");
  Brain.Screen.newLine();
  Brain.Screen.print("Inertial");
  DrivetrainInertial.calibrate();
  while (DrivetrainInertial.isCalibrating()) {
    wait(25, msec);
  }
  vexcode_initial_drivetrain_calibration_completed = true;
  // Clears the screen and returns the cursor to row 1, column 1.
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1, 1);
}

// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;
// define variables used for controlling motors based on controller inputs
bool Controller1LeftShoulderControlMotorsStopped = true;
bool DrivetrainLNeedsToBeStopped_Controller1 = true;
bool DrivetrainRNeedsToBeStopped_Controller1 = true;

// define a task that will handle monitoring inputs from Controller1
int rc_auto_loop_function_Controller1() {
  // process the controller input every 20 milliseconds
  // update the motors based on the input values
  while(true) {
    if(RemoteControlCodeEnabled) {
      // stop the motors if the brain is calibrating
      if (DrivetrainInertial.isCalibrating()) {
        LeftDriveSmart.stop();
        RightDriveSmart.stop();
        while (DrivetrainInertial.isCalibrating()) {
          wait(25, msec);
        }
      }
      
      // calculate the drivetrain motor velocities from the controller joystick axies
      // left = Axis3 + Axis1
      // right = Axis3 - Axis1
      int drivetrainLeftSideSpeed = Controller1.Axis3.position() + Controller1.Axis1.position();
      int drivetrainRightSideSpeed = Controller1.Axis3.position() - Controller1.Axis1.position();
      
      // check if the value is inside of the deadband range
      if (drivetrainLeftSideSpeed < 5 && drivetrainLeftSideSpeed > -5) {
        // check if the left motor has already been stopped
        if (DrivetrainLNeedsToBeStopped_Controller1) {
          // stop the left drive motor
          LeftDriveSmart.stop();
          // tell the code that the left motor has been stopped
          DrivetrainLNeedsToBeStopped_Controller1 = false;
        }
      } else {
        // reset the toggle so that the deadband code knows to stop the left motor nexttime the input is in the deadband range
        DrivetrainLNeedsToBeStopped_Controller1 = true;
      }
      // check if the value is inside of the deadband range
      if (drivetrainRightSideSpeed < 5 && drivetrainRightSideSpeed > -5) {
        // check if the right motor has already been stopped
        if (DrivetrainRNeedsToBeStopped_Controller1) {
          // stop the right drive motor
          RightDriveSmart.stop();
          // tell the code that the right motor has been stopped
          DrivetrainRNeedsToBeStopped_Controller1 = false;
        }
      } else {
        // reset the toggle so that the deadband code knows to stop the right motor next time the input is in the deadband range
        DrivetrainRNeedsToBeStopped_Controller1 = true;
      }
      
      // only tell the left drive motor to spin if the values are not in the deadband range
      if (DrivetrainLNeedsToBeStopped_Controller1) {
        LeftDriveSmart.setVelocity(drivetrainLeftSideSpeed, percent);
        LeftDriveSmart.spin(forward);
      }
      // only tell the right drive motor to spin if the values are not in the deadband range
      if (DrivetrainRNeedsToBeStopped_Controller1) {
        RightDriveSmart.setVelocity(drivetrainRightSideSpeed, percent);
        RightDriveSmart.spin(forward);
      }
      // check the ButtonL1/ButtonL2 status to control intake
      if (Controller1.ButtonL1.pressing()) {
        intake.spin(forward);
        Controller1LeftShoulderControlMotorsStopped = false;
      } else if (Controller1.ButtonL2.pressing()) {
        intake.spin(reverse);
        Controller1LeftShoulderControlMotorsStopped = false;
      } else if (!Controller1LeftShoulderControlMotorsStopped) {
        intake.stop();
        // set the toggle so that we don't constantly tell the motor to stop when the buttons are released
        Controller1LeftShoulderControlMotorsStopped = true;
      }
      
      if (Controller1.ButtonA.pressing()) {
        wall_StakeMotor.spin(forward);
        Controller1LeftShoulderControlMotorsStopped = false;
      } else if (Controller1.ButtonB.pressing()) {
        wall_StakeMotor.spin(reverse);
        Controller1LeftShoulderControlMotorsStopped = false;
      } else{
        wall_StakeMotor.stop();
      }
    }
    // wait before repeating the process
    wait(20, msec);
  }
  return 0;
}

task rc_auto_loop_task_Controller1(rc_auto_loop_function_Controller1);


void vexcodeInit() 
{

  // Calibrate the Drivetrain
  calibrateDrivetrain();
}


// User defined function
void myblockfunction_redPositiveAutonomous();
// User defined function
void myblockfunction_bluePositiveAutonomous();
// User defined function
void myblockfunction_Programing_skills();
// User defined function
void myblockfunction_Blue_right();
// User defined function
void myblockfunction_Red_right();
// User defined function
void myblockfunction_Red_left();
// User defined function
void myblockfunction_Blue_left();
// User defined function
void myblockfunction_autonSkills();

int Brain_precision = 0, Console_precision = 0, Controller1_precision = 0;

float myVariable, clamp_status, armThingy_status, auton_pick, game_start;

// Function to move the robot forward (time-based)
void goForward(double time, int speed) {
  LeftDriveSmart.spin(forward, speed, pct);
  RightDriveSmart.spin(forward, speed, pct);
  wait(time, sec); 
  LeftDriveSmart.stop();
  RightDriveSmart.stop(); 
}

void goBackward(double time, int speed) {
  LeftDriveSmart.spin(reverse, speed, pct);
  RightDriveSmart.spin(reverse, speed, pct);
  wait(time, sec); 
  LeftDriveSmart.stop();
  RightDriveSmart.stop(); 
}

// Function to turn the robot (time-based)
void turnLeft(double time, int speed) {
  LeftDriveSmart.spin(forward, speed, pct);
  RightDriveSmart.spin(reverse, speed, pct);
  wait(time, sec); 
  LeftDriveSmart.stop();
  RightDriveSmart.stop();
}

// Function to turn the robot (time-based)
void turnRight(double time, int speed) {
  LeftDriveSmart.spin(reverse, speed, pct);
  RightDriveSmart.spin(forward, speed, pct);
  wait(time, sec); 
  LeftDriveSmart.stop();
  RightDriveSmart.stop();
}

// Function to run the intake
void runIntake(int speed) {
  intake.spin(forward, speed, pct); 
}

void stopIntake() {
  intake.stop();
}


// User defined function
void myblockfunction_Programing_skills() {
  Drivetrain.setStopping(coast);
  intake.setVelocity(100.0, percent);
  Drivetrain.setDriveVelocity(50.0, percent);
  Drivetrain.setTurnVelocity(50.0, percent);
  clamp_status = 0.0;
  armThingy_status = 0.0;
  intake.spinFor(forward, 3.0, turns, true);
  Drivetrain.driveFor(forward, 8.0, inches, true);
  Drivetrain.turnFor(right, 80.0, degrees, true);
  Drivetrain.setDriveVelocity(20.0, percent);
  Drivetrain.setTurnVelocity(20.0, percent);
  Drivetrain.driveFor(reverse, 18.0, inches, true);
  wait(0.1, seconds);
  clamp.set(true);
  wait(0.5, seconds);
  Drivetrain.setDriveVelocity(50.0, percent);
  Drivetrain.setTurnVelocity(50.0, percent);
  Drivetrain.turnFor(right, 165.0, degrees, true);
  intake.spinFor(forward, 13.0, turns, false);
  Drivetrain.setDriveVelocity(30.0, percent);
  Drivetrain.setTurnVelocity(30.0, percent);
  Drivetrain.driveFor(forward, 24.0, inches, true);
  Drivetrain.driveFor(reverse, 8.0, inches, true);
  Drivetrain.turnFor(left, 72.0, degrees, true);
  Drivetrain.driveFor(forward, 12.0, inches, true);
  Drivetrain.turnFor(left, 135.0, degrees, true);
  Drivetrain.driveFor(reverse, 2.0, inches, true);
  Drivetrain.turnFor(left, 40.0, degrees, true);
  Drivetrain.driveFor(reverse, 2.0, inches, true);
  wait(0.1, seconds);
  clamp.set(false);
}

// User defined function
void myblockfunction_Blue_right() 
{
  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1, 1);
  Controller1.Screen.print("myblockfunction_Blue_right");

  LeftDriveSmart.setStopping(brake);
  RightDriveSmart.setStopping(brake);

  clamp_status = 0.0;
  armThingy_status = 0.0;

  drive(reverse, 29.5, 2);

  clamp.set(true);
  clamp_status = 1.0;
  wait(0.5, seconds);


  intake.spinFor(reverse, 2500.0, degrees, true);
  turn(-145.0, 2);

  intake.spinFor(reverse, 55.0, turns, false);

  drive(forward, 26.5, 6);

  turn(-30.0, 2);

  drive(forward, 32.0, 2);
  return;
}

// User defined function
void myblockfunction_Red_right() {
  Drivetrain.setStopping(coast);
  intake.setVelocity(100.0, percent);
  Drivetrain.setDriveVelocity(30.0, percent);
  Drivetrain.setTurnVelocity(30.0, percent);
  clamp_status = 0.0;
  armThingy_status = 0.0;
  Drivetrain.driveFor(reverse, 24.0, inches, true);
  clamp.set(true);
  clamp_status = 1.0;
  wait(1.0, seconds);
  intake.spinFor(forward, 700.0, degrees, true);
  Drivetrain.setDriveVelocity(50.0, percent);
  Drivetrain.setTurnVelocity(50.0, percent);
  Drivetrain.turnFor(left, 70.0, degrees, true);
  intake.spinFor(forward, 1900.0, degrees, false);
  Drivetrain.driveFor(forward, 24.0, inches, true);
  Drivetrain.driveFor(reverse, 24.0, inches, true);
  Drivetrain.turnFor(left, 95.0, degrees, true);
  Drivetrain.driveFor(forward, 6.0, inches, true);
}

// User defined function
void myblockfunction_Red_left() {
  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1, 1);
  Controller1.Screen.print("myblockfunction_Red_left");

  LeftDriveSmart.setStopping(brake);
  RightDriveSmart.setStopping(brake);

  clamp_status = 0.0;
  armThingy_status = 0.0;

  drive(reverse, 29.5, 2);

  clamp.set(true);
  clamp_status = 1.0;
  wait(0.5, seconds);


  intake.spinFor(reverse, 2500.0, degrees, true);
  turn(145.0, 2);

  intake.spinFor(reverse, 55.0, turns, false);

  drive(forward, 26.5, 6);

  turn(30.0, 2);

  drive(forward, 32.0, 2);
  turn(-130, 3);
  drive(forward, 37.0, 4);
  return;
}

// User defined function
void myblockfunction_Blue_left() {

  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1, 1);
  Controller1.Screen.print("myblockfunction_Blue_left");

  Drivetrain.setStopping(coast);
  intake.setVelocity(100.0, percent);
  Drivetrain.setDriveVelocity(30.0, percent);
  Drivetrain.setTurnVelocity(30.0, percent);
  clamp_status = 0.0;
  armThingy_status = 0.0;
  Drivetrain.driveFor(reverse, 24.0, inches, true);
  clamp.set(true);
  clamp_status = 1.0;
  wait(1.0, seconds);
  intake.spinFor(forward, 700.0, degrees, true);
  Drivetrain.setDriveVelocity(50.0, percent);
  Drivetrain.setTurnVelocity(50.0, percent);
  Drivetrain.turnFor(right, 78.0, degrees, true);
  intake.spinFor(forward, 1900.0, degrees, false);
  Drivetrain.driveFor(forward, 24.0, inches, true);
  Drivetrain.driveFor(reverse, 24.0, inches, true);
  Drivetrain.turnFor(right, 95.0, degrees, true);
  Drivetrain.driveFor(forward, 6.0, inches, true);
}

// "when started" hat block
int whenStarted1() {
  clamp.set(false);
  armThingy.set(false);
  intake.setVelocity(100.0, percent);
  Drivetrain.setDriveVelocity(80.0, percent);
  Drivetrain.setStopping(coast);
  Drivetrain.setTurnVelocity(80.0, percent);
  armThingy_status = 0.0;
  clamp_status = 0.0;
  auton_pick = 3.0;
  game_start = 0.0;
  Brain.Timer.clear();
  return 0;
}

// "when autonomous" hat block
int onauton_autonomous_0() {
  intake.setVelocity(50, percent);
  intake.spinFor(reverse, 90, degrees);
  intake.setVelocity(100, percent);
  if (auton_pick == 0.0) {
    Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(1, 1);
    Controller1.Screen.print("Blue left");
    // Blue left
    myblockfunction_Blue_left();
  }
  if (auton_pick == 1.0) {
    Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(1, 1);
    Controller1.Screen.print("Blue right");
    // Blue right
    myblockfunction_Blue_right();
  }
  if (auton_pick == 2.0) {
    Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(1, 1);
    Controller1.Screen.print("Red right");
    // Red right
    myblockfunction_Red_right();
  }
  if (auton_pick == 3.0) {
    Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(1, 1);
    Controller1.Screen.print("Red left");
    // Red left
    myblockfunction_Red_left();
  }
  if (auton_pick == 4.0) {
    Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(1, 1);
    Controller1.Screen.print("Programing skills");
    // Programing skills
    myblockfunction_Programing_skills();
  }
  if (auton_pick == 5.0) {
    Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(1, 1);
    Controller1.Screen.print("red Positive Autonomous");
    // Red positive side (not mine)
    myblockfunction_redPositiveAutonomous();
  }
  if (auton_pick == 6.0) {
    Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(1, 1);
    Controller1.Screen.print("blue Positive Autonomous");
    // Blue positive side (not mine)
    myblockfunction_bluePositiveAutonomous();
  }
  if (auton_pick == 7.0) {
    Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(1,1);
    Controller1.Screen.print("auton skills");
    myblockfunction_autonSkills();
  }
  return 0;
}

void myblockfunction_redPositiveAutonomous() {
  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1, 1);
  Controller1.Screen.print("Red Positive Autonomous");

  Drivetrain.setStopping(coast);
  intake.setVelocity(100.0, percent);
  
  // Go back with smooth acceleration and deceleration
  LeftDriveSmart.setStopping(coast);
  RightDriveSmart.setStopping(coast);
  
  // Start with gentle acceleration
  for (int i = 5; i <= 25; i += 4) {
    LeftDriveSmart.spin(reverse, i, percent);
    RightDriveSmart.spin(reverse, i, percent);
    wait(0.04, seconds);
  }
  
  // Maintain steady speed
  wait(1.3, seconds);
  
  // Gentle deceleration
  for (int i = 25; i >= 0; i -= 4) {
    LeftDriveSmart.spin(reverse, i, percent);
    RightDriveSmart.spin(reverse, i, percent);
    wait(0.05, seconds);
  }
  
  // Ensure motors are stopped
  LeftDriveSmart.stop(coast);
  RightDriveSmart.stop(coast);
  
  // Clamp
  clamp.set(true);
  clamp_status = 1.0;
  wait(0.5, seconds);
  
  // Run intake for 1 second and stop
  intake.spin(forward);
  wait(1.0, seconds);
  // Turn left 90 degrees at the end using a more direct approach
  LeftDriveSmart.spin(reverse, 28, percent);
  RightDriveSmart.spin(forward, 28, percent);
  wait(0.7, seconds);
  LeftDriveSmart.stop(coast);
  RightDriveSmart.stop(coast);

  // Move forward after turning
  for (int i = 10; i <= 40; i += 5) {
    LeftDriveSmart.spin(forward, i, percent);
    RightDriveSmart.spin(forward, i, percent);
    wait(0.05, seconds);
  }
  intake.spin(forward, 100, percent);
  wait(0.5, seconds); // Extended drive time with intake running for longer distance
  for (int i = 30; i >= 0; i -= 5) {
    LeftDriveSmart.spin(forward, i, percent);
    RightDriveSmart.spin(forward, i, percent);
    wait(0.05, seconds);
  }
  LeftDriveSmart.stop(coast);
  RightDriveSmart.stop(coast);

  //Turn slight right 
  LeftDriveSmart.spin(forward, 5, percent);
  RightDriveSmart.spin(reverse, 5, percent);
  wait(0.8, seconds);
  LeftDriveSmart.stop(brake); // Use brake instead of coast for more precision
  RightDriveSmart.stop(brake);

  // Start with gentle acceleration
  for (int i = 5; i <= 25; i += 4) {
    LeftDriveSmart.spin(reverse, i, percent);
    RightDriveSmart.spin(reverse, i, percent);
    wait(0.04, seconds);
  }
  
  // Maintain steady speed
  wait(3.0, seconds);
  
  // Gentle deceleration
  for (int i = 25; i >= 0; i -= 4) {
    LeftDriveSmart.spin(reverse, i, percent);
    RightDriveSmart.spin(reverse, i, percent);
    wait(0.05, seconds);
  }
  LeftDriveSmart.stop(coast);
  RightDriveSmart.stop(coast);

  intake.stop();
}

void myblockfunction_bluePositiveAutonomous() {
    Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(1, 1);
    Controller1.Screen.print("Red Positive Autonomous");
  
    Drivetrain.setStopping(coast);
    intake.setVelocity(100.0, percent);
    
    // Go back with smooth acceleration and deceleration
    LeftDriveSmart.setStopping(coast);
    RightDriveSmart.setStopping(coast);
    
    // Start with gentle acceleration
    for (int i = 5; i <= 25; i += 4) {
      LeftDriveSmart.spin(reverse, i, percent);
      RightDriveSmart.spin(reverse, i, percent);
      wait(0.04, seconds);
    }
    
    // Maintain steady speed
    wait(1.5, seconds);
    
    // Gentle deceleration
    for (int i = 25; i >= 0; i -= 4) {
      LeftDriveSmart.spin(reverse, i, percent);
      RightDriveSmart.spin(reverse, i, percent);
      wait(0.05, seconds);
    }
    
    // Ensure motors are stopped
    LeftDriveSmart.stop(coast);
    RightDriveSmart.stop(coast);
    
    // Clamp
    clamp.set(true);
    clamp_status = 1.0;
    wait(0.5, seconds);
    
    // Run intake for 1 second and stop
    intake.spin(forward);
    wait(1.0, seconds);

    // Turn left 90 degrees at the end using a more direct approach
    LeftDriveSmart.spin(forward, 25, percent);
    RightDriveSmart.spin(reverse, 25, percent);
    wait(0.8, seconds);
    LeftDriveSmart.stop(coast);
    RightDriveSmart.stop(coast);
  
    // Move forward after turning
    for (int i = 10; i <= 40; i += 5) {
      LeftDriveSmart.spin(forward, i, percent);
      RightDriveSmart.spin(forward, i, percent);
      wait(0.05, seconds);
    }
    intake.spin(forward, 100, percent);
    wait(0.5, seconds); // Extended drive time with intake running for longer distance
    for (int i = 30; i >= 0; i -= 5) {
      LeftDriveSmart.spin(forward, i, percent);
      RightDriveSmart.spin(forward, i, percent);
      wait(0.05, seconds);
    }
    LeftDriveSmart.stop(coast);
    RightDriveSmart.stop(coast);
    wait(1.5, seconds);
    intake.stop();
  
    //Turn slight right 
    LeftDriveSmart.spin(reverse, 45, percent);
    RightDriveSmart.spin(forward, 45, percent);
    wait(0.8, seconds);
    LeftDriveSmart.stop(brake); // Use brake instead of coast for more precision
    RightDriveSmart.stop(brake);
  
    // Start with gentle acceleration
    for (int i = 10; i <= 40; i += 5) {
        LeftDriveSmart.spin(forward, i, percent);
        RightDriveSmart.spin(forward, i, percent);
        wait(0.05, seconds);
      }
      wait(1.1, seconds); // Extended drive time with intake running for longer distance
      for (int i = 30; i >= 0; i -= 5) {
        LeftDriveSmart.spin(forward, i, percent);
        RightDriveSmart.spin(forward, i, percent);
        wait(0.05, seconds);
      }
      LeftDriveSmart.stop(coast);
      RightDriveSmart.stop(coast);
  
}


void myblockfunction_autonSkills() {
  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1, 1);
  Controller1.Screen.print("auton_skills");

  // Initialize robot configuration for optimized skills run
  Drivetrain.setStopping(coast);
  LeftDriveSmart.setStopping(coast);
  RightDriveSmart.setStopping(coast);
  intake.setVelocity(100.0, percent);
  wall_StakeMotor.setVelocity(100.0, percent);
  
  // ============== OPTIMIZED RING-COLLECTING SKILLS ROUTINE ==============
  
  // PHASE 1: Score preloaded ring on high stake (starting at position C facing East)
  // Score preloaded ring - robot starts with back facing stake
  intake.spin(forward, 100, percent); 
  wait(1.0, seconds);
  intake.stop();
  wait(0.2, seconds);
  
  // Move forward slightly to better align with mobile goal
  for (int i = 10; i <= 25; i += 5) {
    LeftDriveSmart.spin(forward, i, percent);
    RightDriveSmart.spin(forward, i, percent);
    wait(0.05, seconds);
  }
  wait(0.6, seconds); // Drive forward for alignment
  for (int i = 25; i >= 0; i -= 5) {
    LeftDriveSmart.spin(forward, i, percent);
    RightDriveSmart.spin(forward, i, percent);
    wait(0.05, seconds);
  }
  
  // PHASE 2: Turn and move to clamp a mobile goal (critical first step)
  // Turn toward nearest mobile goal
  turn(90.0, 2); // Changed to opposite direction so clamp side faces the mobile goal
  
  // Drive to mobile goal with smooth acceleration for precise approach
  for (int i = 10; i <= 35; i += 5) {
    LeftDriveSmart.spin(reverse, i, percent); // Changed to reverse to go left instead of right
    RightDriveSmart.spin(reverse, i, percent); // Changed to reverse to go left instead of right
    wait(0.05, seconds);
  }
  wait(0.8, seconds);
  for (int i = 35; i >= 0; i -= 5) {
    LeftDriveSmart.spin(reverse, i, percent); // Changed to reverse to go left instead of right
    RightDriveSmart.spin(reverse, i, percent); // Changed to reverse to go left instead of right
    wait(0.05, seconds);
  }
  
  // Secure mobile goal with clamp - essential for collecting rings
  clamp.set(true);
  clamp_status = 1.0;
  wait(0.5, seconds);
  
  // Turn left 90 degrees at the end using a more direct approach
  LeftDriveSmart.spin(reverse, 25, percent);
  RightDriveSmart.spin(forward, 25, percent);
  wait(0.7, seconds);
  LeftDriveSmart.stop(coast);
  RightDriveSmart.stop(coast);
  
  // Move forward after turning
  for (int i = 10; i <= 40; i += 5) {
    LeftDriveSmart.spin(forward, i, percent);
    RightDriveSmart.spin(forward, i, percent);
    wait(0.05, seconds);
  }
  intake.spin(forward, 100, percent);
  wait(1.0, seconds); // Extended drive time with intake running for longer distance
  for (int i = 30; i >= 0; i -= 5) {
    LeftDriveSmart.spin(forward, i, percent);
    RightDriveSmart.spin(forward, i, percent);
    wait(0.05, seconds);
  }
  LeftDriveSmart.stop(coast);
  RightDriveSmart.stop(coast);

  // Turn left 90 degrees at the end using a more direct approach
  LeftDriveSmart.spin(reverse, 30, percent);
  RightDriveSmart.spin(forward, 30, percent);
  wait(0.7, seconds);
  LeftDriveSmart.stop(coast);
  RightDriveSmart.stop(coast);
  wait(0.05, seconds);
  
  // Drive forward using direct motor control for a straight path
  for (int i = 10; i <= 25; i += 5) {
    LeftDriveSmart.spin(forward, i, percent);
    RightDriveSmart.spin(forward, i, percent);
    wait(0.05, seconds);
  }
  wait(0.9, seconds); // Calibrated for approximately 17 inches
  for (int i = 25; i >= 0; i -= 5) {
    LeftDriveSmart.spin(forward, i, percent);
    RightDriveSmart.spin(forward, i, percent);
    wait(0.05, seconds);
  }
  LeftDriveSmart.stop(coast);
  RightDriveSmart.stop(coast);

  // Turn left 90 degrees at the end using a more direct approach
  LeftDriveSmart.spin(reverse, 25, percent);
  RightDriveSmart.spin(forward, 25, percent);
  wait(0.7, seconds);
  LeftDriveSmart.stop(coast);
  RightDriveSmart.stop(coast);
  wait(0.05, seconds);

  // Drive forward using direct motor control for a straight path
  for (int i = 10; i <= 35; i += 5) {
    LeftDriveSmart.spin(forward, i, percent);
    RightDriveSmart.spin(forward, i, percent);
    wait(0.05, seconds);
  }
  wait(1.2, seconds); // Calibrated for approximately 17 inches
  for (int i = 35; i >= 0; i -= 5) {
    LeftDriveSmart.spin(forward, i, percent);
    RightDriveSmart.spin(forward, i, percent);
    wait(0.05, seconds);
  }
  LeftDriveSmart.stop(coast);
  RightDriveSmart.stop(coast);

  // As a backup in case the turn() function doesn't work:
  // Brute force approach with higher power
  wait(0.2, seconds);
  LeftDriveSmart.spin(forward, 25, percent);
  RightDriveSmart.spin(reverse, 25, percent);
  wait(0.8, seconds);
  LeftDriveSmart.stop(brake); // Use brake instead of coast for more precision
  RightDriveSmart.stop(brake);

  // Drive forward
  for (int i = 10; i <= 35; i += 5) {
    LeftDriveSmart.spin(forward, i, percent);
    RightDriveSmart.spin(forward, i, percent);
    wait(0.05, seconds);
  }
  wait(0.5, seconds); // Calibrated for approximately 17 inches
  for (int i = 35; i >= 0; i -= 5) {
    LeftDriveSmart.spin(forward, i, percent);
    RightDriveSmart.spin(forward, i, percent);
    wait(0.05, seconds);
  }
  LeftDriveSmart.stop(coast);
  RightDriveSmart.stop(coast);

  // As a backup in case the turn() function doesn't work:
  // Brute force approach with higher power
  wait(0.2, seconds);
  LeftDriveSmart.spin(forward, 45, percent);
  RightDriveSmart.spin(reverse, 45, percent);
  wait(0.8, seconds);
  LeftDriveSmart.stop(brake); // Use brake instead of coast for more precision
  RightDriveSmart.stop(brake);
  
  // Final step: Go backwards 10 inches
  wait(0.2, seconds);
  for (int i = 10; i <= 40; i += 5) {
    LeftDriveSmart.spin(reverse, i, percent);
    RightDriveSmart.spin(reverse, i, percent);
    wait(0.05, seconds);
  }
  wait(1.5, seconds); // Calibrated for approximately 10 inches backward
  for (int i = 40; i >= 0; i -= 5) {
    LeftDriveSmart.spin(reverse, i, percent);
    RightDriveSmart.spin(reverse, i, percent);
    wait(0.05, seconds);
  }
  LeftDriveSmart.stop(brake);
  RightDriveSmart.stop(brake);
  
  // Unclamp if needed
  clamp.set(false);
  clamp_status = 0.0;

  // Ensure intake is stopped
  intake.stop();
}

// "when Controller1 ButtonR1 pressed" hat block
void onevent_Controller1ButtonR1_pressed_0() {
  if (armThingy_status == 0.0) {
    armThingy.set(false);
    armThingy_status = 1.0;
  }
  else {
    armThingy.set(true);
    armThingy_status = 0.0;
  }
}

// "when Brain timer" hat block
void onevent_Brain_timer_0() {
  game_start = 1.0;
}

// "when Controller1 ButtonX pressed" hat block
void onevent_Controller1ButtonX_pressed_0() {
  if (clamp_status == 0.0) {
    clamp.set(true);
    clamp_status = 1.0;
  }
  else {
    clamp.set(false);
    clamp_status = 0.0;
  }
}

// "when Controller1 ButtonUp pressed" hat block
void onevent_Controller1ButtonUp_pressed_0() {
  if (game_start == 0.0) {
    auton_pick = auton_pick + 1.0;
    if (auton_pick > 7.0) {
      auton_pick = 0.0;
    }
    if (auton_pick == 0.0) {
      Controller1.Screen.clearScreen();
      Controller1.Screen.setCursor(1, 1);
      Controller1.Screen.print("Blue left");
      // Blue left
    }
    if (auton_pick == 1.0) {
      Controller1.Screen.clearScreen();
      Controller1.Screen.setCursor(1, 1);
      Controller1.Screen.print("Blue right");
      // Blue right
    }
    if (auton_pick == 2.0) {
      Controller1.Screen.clearScreen();
      Controller1.Screen.setCursor(1, 1);
      Controller1.Screen.print("Red right");
      // Red right
    }
    if (auton_pick == 3.0) {
      Controller1.Screen.clearScreen();
      Controller1.Screen.setCursor(1, 1);
      Controller1.Screen.print("Red left");
      // Red left
    }
    if (auton_pick == 4.0) {
      Controller1.Screen.clearScreen();
      Controller1.Screen.setCursor(1, 1);
      Controller1.Screen.print("Programing skills");
      // Programing skills
    }
    if (auton_pick == 5.0) {
      Controller1.Screen.clearScreen();
      Controller1.Screen.setCursor(1,1);
      Controller1.Screen.print("auton side red positive ");
      // Srivani Bethi
    }
    if (auton_pick == 6.0) {
      Controller1.Screen.clearScreen();
      Controller1.Screen.setCursor(1,1);
      Controller1.Screen.print("auton side blue positive ");
      // Srivani Bethe's other code
    }
    if (auton_pick == 7.0) {
      Controller1.Screen.clearScreen();
      Controller1.Screen.setCursor(1,1);
      Controller1.Screen.print("auton skills");
      // Srivani Bethe's "other" code
    }
  }
}

void pre_auton(void) {
  whenStarted1();
  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
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

void VEXcode_driver_task() {
  // Start the driver control tasks....

  while(Competition.isDriverControl() && Competition.isEnabled()) {this_thread::sleep_for(10);}

  return;
}

void VEXcode_auton_task() {
  // Start the auton control tasks....
  // Use red positive autonomous
  myblockfunction_bluePositiveAutonomous();
  
  // The following is bypassed to ensure red positive routine always runs
  // Uncomment this if you want to return to using the selection system
  // onauton_autonomous_0();
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


//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  vex::competition::bStopTasksBetweenModes = false;
  vexcodeInit();
  pre_auton();

  Competition.autonomous(VEXcode_auton_task);
  Competition.drivercontrol(VEXcode_driver_task);
  Controller1.ButtonR1.pressed(onevent_Controller1ButtonR1_pressed_0);
  Controller1.ButtonX.pressed(onevent_Controller1ButtonX_pressed_0);
  Controller1.ButtonUp.pressed(onevent_Controller1ButtonUp_pressed_0);
  Brain.Timer.event(onevent_Brain_timer_0,10000);

  // Run the pre-autonomous function.

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
