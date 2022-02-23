#include <math.h>
#include "vex.h"
using namespace vex;
extern brain Brain;

double kP = 1.3;
double kD = 0.2;
double kI= 0.0;
float wheelDia = 4;
float gearRatio=0.33;
float width = 12.0;
double circumference = wheelDia*(M_PI);
double inches_deg = (circumference/360);
double degrees_inch = (360/circumference);

double turnKp = 2.0;
double turnKd = 0.0;
double turnKi = 0.0;

double sonar_kP = 7.0;
double sonar_kD = 0.0;
double sonar_kI = 0.0;

double acceptable_error = 0.0;

double adjusted_angle(float angle_){
  angle_ = angle_*((180/M_PI)/width);
  return angle_;
}
double inches_to_degrees(double pos){
 float conversion = ((pos)/(inches_deg))*gearRatio;
 return conversion;
}
double degrees_to_inches(double pos){
  double conversion = ((pos*inches_deg)/(gearRatio));
  return conversion;
}

#if VER == 2
  
#endif
//################################################### TIMER FUNCTIONS:
int t1 = 0;
timer timer1;
/*--------------------------------------
  CHECK TIMER FUNCTION:
  Determines if a duration has passed 
  and sends a true if so. 
 ---------------------------------------
*/
bool checkTimer(int duration) {
  int t2 = timer1.time(msec);
  if ((t2 - t1) >= duration) {
    timer1.reset();
    return true;
  }
  return false;
  t1 = t2;
}
