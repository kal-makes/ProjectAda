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
timer timer1;
// variables
bool RemoteControlCodeEnabled = true;
bool DrivetrainLNeedsToBeStopped_Controller1 = true;
bool DrivetrainRNeedsToBeStopped_Controller1 = true;
bool DrivetrainMNeedsToBeStopped_Controller1 = true;
bool Controller1UpDownButtonsControlMotorsStopped = true;
bool Controller1R1R2ButtonsControlMotorsStopped = true;
bool state = false;
bool toggle = false;
bool latch = false;
float position = 0;
bool hi_limit = false;
int t1 = 0;
void controller_display() {
  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(0, 0);
  // Controller1reen.print("ADA Battery");.Sc
}

int arcadeDrive() {
  if (RemoteControlCodeEnabled) {
    int drivetrainLeftSideSpeed =
        Controller1.Axis3.position() + Controller1.Axis1.position();
    int drivetrainRightSideSpeed =
        Controller1.Axis3.position() - Controller1.Axis1.position();
    int drivetrainMiddlePartSpeed = -Controller1.Axis4.position();
    // check if the value is inside of the deadband range
    if (drivetrainLeftSideSpeed < 5 && drivetrainLeftSideSpeed > -5) {
      // check if the left motor has already been stopped
      if (DrivetrainLNeedsToBeStopped_Controller1) {
        // stop the left drive motor
        LeftDriveSmart.setStopping(brake);
        LeftDriveSmart.stop();
        // tell the code that the left motor has been stopped
        DrivetrainLNeedsToBeStopped_Controller1 = false;
      }
    } else {
      // reset the toggle so that the deadband code knows to stop the left motor
      // nexttime the input is in the deadband range
      DrivetrainLNeedsToBeStopped_Controller1 = true;
    }
    // check if the value is inside of the deadband range
    if (drivetrainRightSideSpeed < 5 && drivetrainRightSideSpeed > -5) {
      // check if the right motor has already been stopped
      if (DrivetrainRNeedsToBeStopped_Controller1) {
        // stop the right drive motor
        RightDriveSmart.setStopping(brake);
        RightDriveSmart.stop();
        // tell the code that the right motor has been stopped
        DrivetrainRNeedsToBeStopped_Controller1 = false;
      }
    } else {
      // reset the toggle so that the deadband code knows to stop the right
      // motor next time the input is in the deadband range
      DrivetrainRNeedsToBeStopped_Controller1 = true;
    }

    // check if the value is inside of the deadband range
    if (drivetrainMiddlePartSpeed < 5 && drivetrainMiddlePartSpeed > -5) {
      // check if the right motor has already been stopped
      if (DrivetrainMNeedsToBeStopped_Controller1) {
        // stop the right drive motor
        middleMotor.setStopping(brake);
        middleMotor.stop();
        // tell the code that the right motor has been stopped
        DrivetrainMNeedsToBeStopped_Controller1 = false;
      }
    } else {
      // reset the toggle so that the deadband code knows to stop the right
      // motor next time the input is in the deadband range
      DrivetrainMNeedsToBeStopped_Controller1 = true;
    }

    // only tell the left drive motor to spin if the values are not in the
    // deadband range
    if (DrivetrainLNeedsToBeStopped_Controller1) {
      LeftDriveSmart.setVelocity(drivetrainLeftSideSpeed, percent);
      LeftDriveSmart.spin(forward);
    }
    // only tell the right drive motor to spin if the values are not in the
    // deadband range
    if (DrivetrainRNeedsToBeStopped_Controller1) {
      RightDriveSmart.setVelocity(drivetrainRightSideSpeed, percent);
      RightDriveSmart.spin(forward);
    }
    if (DrivetrainMNeedsToBeStopped_Controller1) {
      middleMotor.setVelocity(drivetrainMiddlePartSpeed, percent);
      middleMotor.spin(forward);
    }
  }

  return 0;
}

int armLift() {
  LiftMotors.setStopping(hold); // prevent slipping
  if (Controller1.ButtonR1.pressing() && !Switch_hi) {
    // checks to see if Button R1 on controller is pressed and that the limit
    // switch is not triggered
    if (!Switch_hi) {
      LiftMotors.setVelocity(35, percent);
      // set the velocity low
      LiftMotors.spin(forward);
      Controller1R1R2ButtonsControlMotorsStopped = false;
      // for future use
      position = LiftMotors.position(deg);
      // collects position data so that the lowest limit is a degree not another
      // limit switch
      state = false;
    }
  } else if (Controller1.ButtonR2.pressing()) {
    // checks to see if R2 is pressed so that we can lower the the arm down
    LiftMotors.setVelocity(35, percent);
    LiftMotors.spin(reverse);
    Controller1R1R2ButtonsControlMotorsStopped = false;
    position = LiftMotors.position(deg);
    state = true;
  }

  else if (!(Controller1.ButtonR1.pressing()) &&
           LiftMotors.position(deg)
               // simple control block that prevents slippage
               < position &&
           !(state)) {
    LiftMotors.setVelocity(25, percent);
    LiftMotors.setStopping(hold);
    LiftMotors.stop();
  } else if (!Controller1R1R2ButtonsControlMotorsStopped) {
    LiftMotors.stop();
    // set the toggle so that we don't constantly tell the motor to stop
    // when the buttons are released
    Controller1R1R2ButtonsControlMotorsStopped = true;
  }
  return 0;
}
int intake() {
  // two states for intake... either pressed with L1 or pressed with button up
  if (Controller1.ButtonL1.pressing()) {
    intakeMotor.setVelocity(70, percent);
    intakeMotor.spin(forward);
    state = true;
  } else if (Controller1.ButtonL2.pressing()) {
    intakeMotor.stop();
    state = false;
  }
  if (!state) {
    if (Controller1.ButtonUp.pressing()) {
      intakeMotor.setVelocity(70, percent);
      intakeMotor.spin(forward);
      Drivetrain.drive(forward, 50, rpm);
      Controller1UpDownButtonsControlMotorsStopped = false;
    } else if (!DrivetrainLNeedsToBeStopped_Controller1 ||
               !DrivetrainRNeedsToBeStopped_Controller1) {
      Controller1UpDownButtonsControlMotorsStopped = true;
      intakeMotor.stop();
      Drivetrain.stop();
      // return 0;
    }
  }
  return 1;
}

bool checkTimer( void ){
  int t2 = timer1.time(msec);
  if((t2-t1)>=  1000){
    return true;
  }
  return false;
}

int distance_sensor(){
  if(checkTimer()){
    int x = back_sonar.distance(inches);
    return x;
  }
  else{
    return 0;
  }
}

void dashboard( void ){
  Controller1.Screen.setCursor(0, 0);
  Controller1.Screen.print("Lift Distance: ");
  Controller1.Screen.print(distance_sensor());
  Controller1.Screen.clearLine();
}
  

void drive(void) {
  Controller1.Screen.clearScreen();
  while (1) {
    //idk what tasks are but I like them
    dashboard();
    task autoDrive(arcadeDrive);
    task autoLift(armLift);
    task autoIntake(intake);
    //wait(250, msec);
  }
}
