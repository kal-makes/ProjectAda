/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       controls.cpp                                              */
/*    Author:       Kalvin Q                                                  */
/*    Created:      Wed Oct 17 2021                                           */
/*    Description: config driving controls as well as lift controls           */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
using namespace vex;
//################################################### VARIABLES:
bool RemoteControlCodeEnabled = true;
// check if remote control is enabled
bool DrivetrainLNeedsToBeStopped_Controller1 = true;
bool DrivetrainRNeedsToBeStopped_Controller1 = true;
bool DrivetrainMNeedsToBeStopped_Controller1 = true;
bool Controller1UpDownButtonsControlMotorsStopped = true;
bool Controller1R1R2ButtonsControlMotorsStopped = true;
bool state = false;
bool toggle = false;
bool latch = false;
// toggle and latch for intake button
bool hi_limit = false;
// limit switch for armlift
bool drive_state = false;
bool DBG = false;
float position = 0;
bool goalGrabbed = false;

//################################################### SENSOR FUNCTIONS:
int distance_sensor() {
  if (checkTimer(500)) {
    int x = back_sonar.distance(inches);
    return x;
  } else {
    return 0;
  }
}
///////////////////////////////////////DRIVE
int _arcadeDrive_() {
  if (RemoteControlCodeEnabled && !drive_state) {
    int drivetrainLeftSideSpeed =
        Controller1.Axis3.position() + Controller1.Axis1.position();
    int drivetrainRightSideSpeed =
        Controller1.Axis3.position() - Controller1.Axis1.position();
    int drivetrainMiddlePartSpeed = -Controller1.Axis4.position();
    // get joystick values
    if ((drivetrainLeftSideSpeed > 5 || drivetrainLeftSideSpeed < -5) ||
        (drivetrainRightSideSpeed > 5 || drivetrainRightSideSpeed < -5) ||
        (drivetrainMiddlePartSpeed > 5 || drivetrainMiddlePartSpeed < -5)) {
      LeftDriveSmart.spin(fwd, drivetrainLeftSideSpeed / 2, pct);
      RightDriveSmart.spin(fwd, drivetrainRightSideSpeed / 2, pct);
      middleMotor.spin(fwd, drivetrainMiddlePartSpeed / 2, pct);
    } else {
      Drivetrain.stop(brake);
      middleMotor.stop(brake);
    }
  }
  task::sleep(20);
  return 0;
}
void _reverseSLOW_() {
  drive_state = true;
  while (Controller1.ButtonDown.pressing()) {
    Drivetrain.drive(reverse, 40, rpm);
    if (distance_sensor() <= 3) {
      Controller1.rumble(".");
    }
  }
  Drivetrain.stop();
  intakeMotor.stop();
  drive_state = false;
}

int _drive_() {
  if (!drive_state) {
    _arcadeDrive_();
  }
  task::sleep(20);
  return 0;
}

//################################################### INTAKE FUNCTIONS:
void _intake_() {
  intakeMotor.setVelocity(70, pct);
  if (!latch) {
    // latch is init to false so when the callback is triggered it latches to
    // true starts the intake motor
    latch = true;
    intakeMotor.spin(fwd);
  } else {
    // when callback is triggered a second time, motor stops and latches to
    // false
    intakeMotor.stop(coast);
    latch = false;
  }
  task::sleep(20);
}
void _intakeSLOW_() {
  drive_state = true;
  intakeMotor.setVelocity(70, pct);
  while (Controller1.ButtonUp.pressing()) {
    // when button is pressing, drive fwd while intaking
    Drivetrain.drive(fwd, 40, rpm);
    intakeMotor.spin(fwd);
  }
  // stop drive train and intake when not pressing button
  Drivetrain.stop();
  intakeMotor.stop();
  drive_state = false;
}

void _intake_increment_() {
  int current_velocity = intakeMotor.velocity(pct);
  int new_velocity = current_velocity + 5;
  intakeMotor.setVelocity(new_velocity, pct);
}

void _intake_decrement_() {
  int current_velocity = intakeMotor.velocity(pct);
  int new_velocity = current_velocity - 5;
  if (new_velocity < 0) {
    new_velocity = 0;
  }
  intakeMotor.setVelocity(new_velocity, pct);
}
//----------------- INTAKE INIT PASSED TO CALLBACK
int intake_init() {
  if (DBG) {
    Controller1.ButtonA.pressed(_intake_increment_);
  }
  Controller1.ButtonA.pressed(_intake_);
  // manager for intake callbacks
  return 0;
}
//################################################### ARM LIFT FUNCTIONS:

void _armLift_() {
  LiftMotors.setVelocity(100, percent);

  if ((LiftMotors.position(degrees) > 0) && !goalGrabbed) {
    // checks to see if Button R1 on controller is pressed and that the limit
    // switch is not triggered
    // set the velocity low
      while(LiftMotors.position(degrees) > 20)
      {
        LiftMotors.spin(reverse);
        if(LiftMotors.torque() >= 0.5){
          goalGrabbed = true;
          return;
        }
      }
      if(Switch_hi){
        LiftMotors.resetPosition();
      }
      goalGrabbed = true;
  }
  else{
    // checks to see if R2 is pressed so that we can lower the the arm down
    LiftMotors.spinTo(336, degrees);
    goalGrabbed = false;
  }
  LiftMotors.stop(hold);
  return;
}
///////////////////////////////////////SPECIAL FUNCTIONS
void _special_func_() {
  Controller1.ButtonDown.pressed(_reverseSLOW_);
  Controller1.ButtonUp.pressed(_intakeSLOW_);
  Controller1.ButtonL1.pressed(_armLift_);
}
///////////////////////////////////////DISPLAY
///////////////////////////////////////CALLBACKS

void init_callbacks() {
  intake_init();
  _special_func_();
}
///////////////////////////////////////TASK-MANAGER
void task_manager() {
  task ADAdrive = task(_arcadeDrive_);
}
