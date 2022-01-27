#include "vex.h"
using namespace vex;

bool motor_flipped = false;
int degreeCounter = 5;

enum routine1{start, rotate, check, increment, finished};
int r1 = start;



void front_motor_settings(void) { frontLift.setVelocity(5, percent), frontLift.setStopping(vex::brake);}
void front_latch() {
  if (Switch_front) {
    motor_flipped = true;
  }
}

int routine1(void){
  frontLift.resetRotation();
  switch(r1){
    case start:
      r1= rotate;
      return 0;
    case rotate:
      frontLift.spinTo(-40, degrees);
      wait(1000, msec);
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

//drive module 
//distance in inches!
int move_inches(float distance){
  //PID?
  Drivetrain.driveFor(forward, distance, inches, 100, rpm);
  return 0; 
}

float error = 0;
float previous_error= 0;
float integral = 0;
float derivative = 0;

float PID(float desired_value, float actual_value, float ki, float kd, float kp){
  error = desired_value - actual_value;
  derivative = error - previous_error;
  integral = integral + error;
  
  float drive_power = ((error * kp) + (integral * ki) + (derivative + kd));
  return drive_power;
}

//turn module 
//lift module
//intake module 

void auto_routine(void) {
  int x = 1;
  while (1) {
  if(!motor_flipped){
    routine1();
  }

  }
}
