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
float previous_average = 0;
enum routine1{start, rotate, check, increment, finished};
int r1 = start;
void front_motor_settings(void) { frontLift.setVelocity(5, percent), frontLift.setStopping(vex::brake);}
void front_latch() {
  if (Switch_front) {
    motor_flipped = true;
  }
}
timer PIDTIMER;
//################################################### FUNCTIONS:
/*--------------------------------------
  DRIVE PID FUNCTION:
  Used to send drive instructions 
  for ADA both laterally and rotationally
 ---------------------------------------
*/

int drivePID(float desiredPos, double desiredTurnPos){
//PIDinit(kP, kD, kI, turnKp, turnKd, turnKi);
//writing to PID file
//lateral movement//
desiredPos = inches_to_degrees(desiredPos);
//converst desired inches into degrees for easier use of motors
LeftDriveSmart.resetPosition();
RightDriveSmart.resetPosition();

//resets drive encoders
PIDinit();
while(!complete){
  double leftDrivepos = LeftDriveSmart.position(degrees);
  double rightDrivepos  = RightDriveSmart.position(degrees);
  double avgPosition = (leftDrivepos+rightDrivepos)/2;
  //retrieves drive train encoder values (deg)
  float error = desiredPos - avgPosition;
  derivative = error - prevError;
  //calculates error and also derivate
  double latMotorPower = (error*kP + derivative*kD + totalError*kI)/12;
  //PD Loop for lateral movement

  float turnDifference = leftDrivepos - rightDrivepos;
  turnError = desiredTurnPos - turnDifference;
  //error for turning
  //avg error to break loop when less than threshold
  turnDerivative = turnError - prevTurnError;
  double turnMotorPower= (turnError*kP + turnDerivative*kD+ totalError*kI)/12;
  //PD loop for rotational movement

  //spin motors
  LeftDriveSmart.spin(forward, latMotorPower + turnMotorPower, voltageUnits::volt);
  RightDriveSmart.spin(forward, latMotorPower - turnMotorPower, voltageUnits::volt);
  float avgPower = (LeftDriveSmart.voltage()+RightDriveSmart.voltage())/2;
  float avg_error = (turnError+error)/2;
  Controller1.Screen.clearLine();
  Controller1.Screen.print(avg_error);

  prevError = error;
  totalError += error;
  prevTurnError = turnError;
  counter = counter+20;
  PIDwrite(error, turnError, avgPower, counter, avgPosition, desiredPos, turnDifference, desiredPos);
  wait(20, msec);
  }
/*
}
*/
//if statement to break while loop
//Controller1.Screen.clearLine();
//Controller1.Screen.print(avg_error);
LeftDriveSmart.stop(brake);
RightDriveSmart.stop(brake);
return 1;
}

/*--------------------------------------
  ULTRASONIC PID FUNCTION:
  Used to recieve sensor values and 
  move the robot laterally in the direction
  of the object
 ---------------------------------------
*/
int ultraSonicPID(){
  LeftDriveSmart.resetPosition();
  RightDriveSmart.resetPosition();
  float desired_value = back_sonar.distance(inches);
  desired_value = desired_value-3;
  //Controller1.Screen.print(desired_value);
  while(error<=threshold){
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
  }
  return 1;
}

//################################################### ROUTINES:
/*--------------------------------------
  ROUTINE 0:
  First routine to run during autonomous.
  The routine unfolds ADA's intake and 
  back flap.
 ---------------------------------------
*/
int routine0(void){
  frontLift.resetRotation();
  LiftMotors.setVelocity(70, percent); 
  switch(r1){
    case start:
      r1= rotate;
      return 0;
    case rotate:
      LiftMotors.spinTo(-390, degrees);
      intakeMotor.spin(reverse, 80, pct);
      wait(500, msec);
      intakeMotor.stop();
      frontLift.spinTo(-45, degrees);
      r1 = check;
      return 0;
    case check:
      if(Switch_front){
        frontLift.spinTo(40, degrees);
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



//turn module 
//lift module
//intake module 

void auto_routine(void) {
drivePID(29 ,-180);//drive forward and turn left to center on the goal
wait(50, msec);
ultraSonicPID();
}
