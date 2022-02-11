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
///////////////////////////////////////VARIABLES

bool RemoteControlCodeEnabled = true;
//check if remote control is enabled
bool DrivetrainLNeedsToBeStopped_Controller1 = true;
bool DrivetrainRNeedsToBeStopped_Controller1 = true;
bool DrivetrainMNeedsToBeStopped_Controller1 = true;
bool Controller1UpDownButtonsControlMotorsStopped = true;
bool Controller1R1R2ButtonsControlMotorsStopped = true;
bool state = false;

bool toggle = false;
bool latch = false;
//toggle and latch for intake button
float position = 0;

bool hi_limit = false;
//limit switch for armlift 
int t1 = 0;
bool drive_state = false;
bool DBG = false;

///////////////////////////////////////TIMERS 

timer timer1;
bool checkTimer( int duration ){
  int t2 = timer1.time(msec);
  if((t2-t1)>= duration){
    return true;
  }
  return false;
}

///////////////////////////////////////SENSORS

int distance_sensor(){
  if(checkTimer(500)){
    int x = back_sonar.distance(inches);
    return x;
  }
  else{
    return 0;
  }
}

///////////////////////////////////////DRIVE

int _arcadeDrive_(){
  if(RemoteControlCodeEnabled && !drive_state){
    int drivetrainLeftSideSpeed =
          Controller1.Axis3.position() + Controller1.Axis1.position();
    int drivetrainRightSideSpeed =
          Controller1.Axis3.position() - Controller1.Axis1.position();
    int drivetrainMiddlePartSpeed = -Controller1.Axis4.position();
    //get joystick values
    if((drivetrainLeftSideSpeed > 5 || drivetrainLeftSideSpeed < -5) ||
    (drivetrainRightSideSpeed > 5 || drivetrainRightSideSpeed < -5) ||
      (drivetrainMiddlePartSpeed > 5 || drivetrainMiddlePartSpeed < -5)){
        LeftDriveSmart.spin(fwd, drivetrainLeftSideSpeed/2, pct);
        RightDriveSmart.spin(fwd, drivetrainRightSideSpeed/2, pct);
        middleMotor.spin(fwd, drivetrainMiddlePartSpeed/2, pct);
      }else{
        Drivetrain.stop(brake); 
        middleMotor.stop(brake);
      }
  } 
     task::sleep(20);
     return 0;
}
void _reverseSLOW_(){
  drive_state = true;
  while(Controller1.ButtonDown.pressing()){
    Drivetrain.drive(reverse, 40, rpm);
    if(distance_sensor()<= 3){
      Controller1.rumble(".");
    }
  }
  Drivetrain.stop();
  intakeMotor.stop(); 
  drive_state = false;
}
int _drive_(){
  if(!drive_state){
    _arcadeDrive_();
  }
  task::sleep(20);
  return 0;
}

///////////////////////////////////////INTAKE

void _intake_(){
  intakeMotor.setVelocity(70, pct);
  if(!latch){
    //latch is init to false so when the callback is triggered it latches to true
    //starts the intake motor 
    latch = true;
    intakeMotor.spin(fwd);
  }else{
    //when callback is triggered a second time, motor stops and latches to false
    intakeMotor.stop();
    latch = false; 
  }
  task::sleep(20);
}
void _intakeSLOW_(){
  drive_state = true;
  intakeMotor.setVelocity(70, pct);
  while(Controller1.ButtonUp.pressing()){
    //when button is pressing, drive fwd while intaking
      Drivetrain.drive(fwd, 40, rpm);
      intakeMotor.spin(fwd);
  }
  //stop drive train and intake when not pressing button
  Drivetrain.stop();
  intakeMotor.stop(); 
  drive_state = false;
}
void _intake_increment_(){
  int current_velocity = intakeMotor.velocity(pct);
  int new_velocity = current_velocity + 5;
  intakeMotor.setVelocity(new_velocity, pct);
}
void _intake_decrement_(){
  int current_velocity = intakeMotor.velocity(pct);
  int new_velocity = current_velocity - 5;
  if(new_velocity < 0){
    new_velocity = 0;
  }
  intakeMotor.setVelocity(new_velocity, pct);
}
//----------------- INTAKE INIT PASSED TO CALLBACK
int intake_init(){
  if(DBG){
    Controller1.ButtonA.pressed(_intake_increment_);
  }
  Controller1.ButtonL1.pressed(_intake_);
  //manager for intake callbacks 
  return 0;
}

///////////////////////////////////////ARM-LIFT

int _armLift_() {
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
  }else if (Switch_hi){
    LiftMotors.resetRotation();
  }
  return 0;
}

///////////////////////////////////////SPECIAL FUNCTIONS

void _special_func_(){
  Controller1.ButtonDown.pressed(_reverseSLOW_);
  Controller1.ButtonUp.pressed(_intakeSLOW_);
}

///////////////////////////////////////DISPLAY

void controller_display() {
  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(0, 0);
  if (DBG){
    Controller1.Screen.print("Lift Deg: ");
    Controller1.Screen.print(LiftMotors.position(deg));
    Controller1.Screen.newLine();
    Controller1.Screen.print("Intake Vel (pct):");
    Controller1.Screen.print(intakeMotor.velocity(pct));
  }
}
void dashboard( void ){
  Controller1.Screen.setCursor(0, 0);
  Controller1.Screen.print("Lift Distance: ");
  Controller1.Screen.print(distance_sensor());
  Controller1.Screen.clearLine();
}

///////////////////////////////////////CALLBACKS

void init_callbacks(){
  intake_init();
  _special_func_();
}

///////////////////////////////////////TASK-MANAGER

void task_manager(){
  task ADAdrive = task(_arcadeDrive_);
  task ADAlift = task(_armLift_);
  controller_display(); 
}
