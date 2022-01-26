using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern drivetrain Drivetrain;
extern motor_group LiftMotors;
extern limit Switch_low;
extern limit Switch_hi;
extern motor_group RightDriveSmart;
extern motor_group LeftDriveSmart;
extern motor middleMotor;
extern motor intakeMotor;
extern motor_group frontLift;
extern limit Switch_front; 

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );