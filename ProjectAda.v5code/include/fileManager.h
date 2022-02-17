
using namespace vex;
extern brain Brain;

int file( void ); 

int PIDinit( void );

void PIDwrite(double lat_error, double rot_error, double avg_power, int counter, float distance_val, float desired_distance, float turn_val, float desired_turn);

