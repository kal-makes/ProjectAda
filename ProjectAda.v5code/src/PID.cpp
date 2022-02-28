#include "pid.h"
#include "vex.h"

void PID::drivePID(){
  LeftDriveSmart.resetPosition();
  RightDriveSmart.resetPosition();
  float desiredPos_deg = inches_to_degrees(desiredPos);
  while(!complete){
    double leftDrivepos = LeftDriveSmart.position(degrees);
    double rightDrivepos  = RightDriveSmart.position(degrees);
    double avgPosition = (leftDrivepos+rightDrivepos)/2;
    //retrieves drive train encoder values (deg)
    error = desiredPos_deg - avgPosition;
    derivative = error - prevError;
    //calculates error and also derivate
    //PD Loop for lateral movement
    float latMotorPower = (error*kP + derivative*kD + totalError*kI);
    float turnDifference = rightDrivepos-leftDrivepos;
    turnDifference = degrees_to_inches(turnDifference);
    turnDifference = adjusted_angle(turnDifference);
    turnError = desiredTurn-turnDifference;
    //error for turning
    //avg error to break loop when less than threshold
    turnDerivative = turnError - prevTurnError;
    //PD loop for rotational movement
    //spin motors
    float turnMotorPower = (turnError*turnKp + turnDerivative*turnKd+ total_turnError*turnKi);
    LeftDriveSmart.spin(forward, latMotorPower - turnMotorPower, voltageUnits::volt);
    RightDriveSmart.spin(forward, latMotorPower + turnMotorPower, voltageUnits::volt);
    prevError = error;
    prevTurnError = turnError;
    counter = counter+20;
    if(desiredPos_deg != 0){
        pctError = fabs((avgPosition-desiredPos_deg)/(desiredPos_deg))*100;
    }else{
      pctError = fabs((turnDifference-desiredTurn)/(desiredTurn))*100;
    }
    if(pctError < 1.5){
      complete = true;
    }
  
  //PIDwrite(error, turnError, avgPower, counter, avgPosition, desiredPos, turnDifference, desiredPos);
  //wait(1, msec);
}
LeftDriveSmart.stop(brake);
RightDriveSmart.stop(brake);
LeftDriveSmart.resetPosition();
RightDriveSmart.resetPosition();
complete = false;
  }

void PID::startPID( float lat_setpoint, float rot_setpoint){
  desiredPos = lat_setpoint;
  desiredTurn = rot_setpoint;
  drivePID();
}