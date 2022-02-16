#include <math.h>
#include "vex.h"
using namespace vex;
extern brain Brain;

double kP = 1.3;
double kD = 0.2;
double kI= 0.0;
float wheelDia = 4;
float gearRatio=0.33;
double circumference = wheelDia*(M_PI);
double inches_deg = (circumference/360);

double turnKp = 0.5;
double turnKd = 0.1;
double turnKi = 0.0;

double acceptable_error = 0.0;


double inches_to_degrees(double pos){
 float conversion = ((pos)/(inches_deg))*gearRatio;
 return conversion;
}