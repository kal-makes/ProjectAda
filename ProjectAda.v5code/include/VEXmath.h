using namespace vex;
extern brain Brain;

extern double kP;
extern double kD;
extern double kI;

extern double turnKd;
extern double turnKp;
extern double turnKi;

extern double sonar_kP;
extern double sonar_kD;
extern double sonar_kI;

double inches_to_degrees( double );
double degrees_to_inches( double );
double adjusted_angle( float angle_ );
bool checkTimer ( int );