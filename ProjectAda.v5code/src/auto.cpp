#include "vex.h"
using namespace vex;

bool motor_flipped = false;
int degreeCounter = 5;
int counter = 0;
float threshold = 10.0;
bool complete = false;
int error;
int prevError=0;
int derivative;
int integral;
int totalError=0;
int turnError;
int prevTurnError = 0;
int turnDerivative;
float previous_average = NAN;

enum routine1{start, rotate, check, increment, finished};
int r1 = start;

void front_motor_settings(void) { frontLift.setVelocity(5, percent), frontLift.setStopping(vex::brake);}
void front_latch() {
  if (Switch_front) {
    motor_flipped = true;
  }
}
//################################################### FUNCTIONS:
/*--------------------------------------
  DRIVE PID FUNCTION:
  Used to send drive instructions 
  for ADA both laterally and rotationally
 ---------------------------------------
*/
/*--------------------------------------
  ULTRASONIC PID FUNCTION:
  Used to recieve sensor values and 
  move the robot laterally in the direction
  of the object
 ---------------------------------------
*/
int ultraSonicPID(){
  timer PIDTIMER;
  int t0 = 0;
  LeftDriveSmart.resetPosition();
  RightDriveSmart.resetPosition();
  float desired_value = back_sonar.distance(inches)+1;
  //desired_value = desired_value;
  //Controller1.Screen.print(desired_value);
  while(!complete){
  //float desired_value = back_sonar.distance(inches);
  double leftDrivepos = LeftDriveSmart.position(degrees);
  double rightDrivepos  = RightDriveSmart.position(degrees);

  leftDrivepos = degrees_to_inches(leftDrivepos);
  rightDrivepos = degrees_to_inches(rightDrivepos);
  double avgPosition = fabs((leftDrivepos+rightDrivepos)/2);
  Controller1.Screen.clearLine();
  Controller1.Screen.print(avgPosition);
  float error =  desired_value-avgPosition;
  derivative = error - prevError;

  double latMotorPower = (error*sonar_kP + derivative*sonar_kD + totalError*sonar_kI)/12;

  LeftDriveSmart.spin(reverse, latMotorPower, voltageUnits::volt);
  RightDriveSmart.spin(reverse, latMotorPower, voltageUnits::volt);

  prevError = error;
  if(previous_average != NAN && previous_average == error){
      int t2 = PIDTIMER.time(msec);
      if(t2 - t0 >= 300){
        complete = true;
        Controller1.Screen.clearLine();
        PIDTIMER.reset();
      }
    }else{
      previous_average = error;
    }
  }
  complete = false;
  return 1;
}

int lift_auto(float torque_){
  LiftMotors.setVelocity(40, percent);
  while(LiftMotors.torque() <= torque_){
    LiftMotors.spin(forward);
  }
  LiftMotors.stop(hold);
  return 1;
}

//################################################### ROUTINES:
//degrees +- 7 for accurate turns
/*--------------------------------------
  ROUTINE 0:
  First routine to run during autonomous.
  The routine unfolds ADA's intake and 
  back flap.
 ---------------------------------------
*/
int routine0(void){
  frontLift.resetRotation();
  frontLift.setVelocity(100, percent);
  LiftMotors.setVelocity(100, percent);
  intakeMotor.resetPosition();
  intakeMotor.setVelocity(100, percent);
  switch(r1){
    case start:
      r1= rotate;
      return 0;
    case rotate:
      //LiftMotors.spinTo(336, degrees);
      intakeMotor.spinTo(-40, degrees);
      frontLift.spinTo(-70, degrees);
      frontLift.spinTo(70, degrees);
      r1 = finished;
      return 0;
    case check:
      if(Switch_front){
        frontLift.spinTo(50, degrees);
        r1 = finished;
      }else{
        r1 = increment;
      }
      return 0;
    case finished:
      frontLift.stop();
      motor_flipped = true;
      //r1 = start;
      return 1;
    case increment:
      int x = frontLift.rotation(degrees)+degreeCounter;
      frontLift.spinTo(-x, degrees);
      r1 = check;
      return 0;
 }
 return 0;
}
/*--------------------------------------
  ROUTINE grabGoal:
  Routine drives 25" and rotates approx 
  90 deg to then backup to the alliance 
  goal and lift it until it has reached 
  the setpoint
 ---------------------------------------
*/
/*
int grabGoal(){
  //get alliance goal
  drivePID(25, 0,1);
  drivePID(0,-98, 1);//drive forward and turn left to center on the goal
  ultraSonicPID();//back into goal
  lift_auto(0.98);//lift until torque reaches 0.98lbs (when goal hits backend)
  return 1;
}
int intake_rings(){
  drivePID(5, 0, 1);
  drivePID(0, 98, 1);
  intakeMotor.setVelocity(70, percent);
  intakeMotor.spin(fwd);
  drivePID(26, 0, 2);
  drivePID(0, 98, 2);
  drivePID(28, 0, 2);
  drivePID(-3, 0, 2);
  return 1;
}
*/
int lift_down(){
  LiftMotors.setVelocity(100, percent);
  LiftMotors.spinTo(346, degrees);
  return 1;
}
void routine00(){
PID AdaPID;
task liftDown(lift_down);
AdaPID.startPID(-53, 0);
LiftMotors.spinTo(200, degrees);
AdaPID.startPID(30, 0);
LiftMotors.spinTo(346, degrees);
AdaPID.startPID(2, 0);
AdaPID.startPID(0, -90);

}
//intake
//################################################### SCRIPT:
//put routines in there
void auto_routine(void) {
routine00();
//DO NOT TOUCH//
while(!motor_flipped){
  routine0();
}

//grabGoal();
//intake_rings();


}
