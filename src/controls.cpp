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

bool RemoteControlCodeEnabled = true;
bool DrivetrainLNeedsToBeStopped_Controller1 = true;
bool DrivetrainRNeedsToBeStopped_Controller1 = true;
bool DrivetrainMNeedsToBeStopped_Controller1 = true;
bool Controller1UpDownButtonsControlMotorsStopped = true;
bool state = false;
float position = 0;

void spin() {
  LeftDriveSmart.setVelocity(80, percent);
  LeftDriveSmart.spin(forward);
  RightDriveSmart.setVelocity(-80, percent);
  RightDriveSmart.spin(forward);
}

int arcadeDrive() {
  // while (true){
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
    if (Controller1.ButtonR1.pressing()) {
      LiftMotors.setVelocity(20, percent);
      LiftMotors.spin(forward);
    }
    if (Controller1.ButtonR2.pressing()) {
      LiftMotors.setVelocity(-20, percent);
      while (1) {
        if (Switch_low.pressing()) {
          LiftMotors.setStopping(brake);
          LiftMotors.stop();
          break;
        }
        LiftMotors.spin(forward);
      }
    }

    // if (Controller1.ButtonR1.pressing() && Controller1.ButtonR2.pressing()
    //     && Controller1.ButtonL1.pressing() &&
    //     Controller1.ButtonL2.pressing()) {
    //   while(1) {
    //     if (Controller1.ButtonR1.pressing() &&
    //     Controller1.ButtonL2.pressing()) {
    //       RightDriveSmart.setStopping(brake);
    //       LeftDriveSmart.setStopping(brake);
    //       RightDriveSmart.stop();
    //       LeftDriveSmart.stop();
    //       break;
    //     }
    //     spin();
    //   }
    // }
    // only tell the right drive motor to spin if the values are not in the
    // deadband range
  }
  wait(20, msec);
  return 0;
}

// return 0;
//}

bool hi_limit = false;

int armLift() {
  if (Controller1.ButtonR1.pressing()) {
        LiftMotors.setVelocity(35, percent);
        LiftMotors.spin(forward);
        Controller1UpDownButtonsControlMotorsStopped = false;
        position = LiftMotors.position(deg);
        state = false;

      } else if (Controller1.ButtonR2.pressing()) {
          if (Switch_low.pressing()){
            LiftMotors.stop();
          }
          else{
            LiftMotors.setVelocity(35, percent);
            LiftMotors.spin(reverse);
            Controller1UpDownButtonsControlMotorsStopped = false;
            position = LiftMotors.position(deg);
            state = true;

          }
      }
  else if(!(Controller1.ButtonR1.pressing()) && LiftMotors.position(deg)
  < position && !(state)){
    LiftMotors.setVelocity(25, percent);
    LiftMotors.setStopping(hold);
    LiftMotors.stop();
  }
  else if (!Controller1UpDownButtonsControlMotorsStopped) {
  LiftMotors.stop();
// set the toggle so that we don't constantly tell the motor to stop
// when the buttons are released
  Controller1UpDownButtonsControlMotorsStopped = true;
}
 /* hi_limit = Switch_hi.pressing();

  if (Controller1.ButtonR1.pressing()) {
    LiftMotors.setVelocity(20, percent);
    if (!hi_limit) {
      LiftMotors.spin(forward);
    }
  } else {
    LiftMotors.setStopping(brake);
    LiftMotors.stop();
  }
  if (Controller1.ButtonR2.pressing()) {
    LiftMotors.setVelocity(-20, percent);
    while (1) {
      if (Switch_low.pressing()) {
        LiftMotors.setStopping(brake);
        LiftMotors.stop();
        break;
      }
      LiftMotors.spin(forward);
    }
  }
  */
  return 0;
}

void drive(void) {
  while (1) {
    task autoDrive(arcadeDrive);
    task autoLift(armLift);
  }
}
