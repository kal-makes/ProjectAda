#include <math.h>
#include "vex.h"
using namespace vex;

double kP = 0.0;
double kD = 0.0;
int wheelDia = 4;
int gearRatio=0;

double inch_to_degrees(double pos){
 double conversion = (pos*360)/(gearRatio*(wheelDia*M_PI));
 return conversion;
}