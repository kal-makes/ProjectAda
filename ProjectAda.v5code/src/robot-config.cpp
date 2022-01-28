#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;
// VEXcode device constructors
controller Controller1 = controller(primary);
motor leftMotorA = motor(PORT1, ratio18_1, false);
motor leftMotorB = motor(PORT11, ratio18_1, false);
motor_group LeftDriveSmart = motor_group(leftMotorA, leftMotorB);
motor rightMotorA = motor(PORT10, ratio18_1, true);
motor rightMotorB = motor(PORT20, ratio18_1, true);
motor_group RightDriveSmart = motor_group(rightMotorA, rightMotorB);
drivetrain Drivetrain = drivetrain(LeftDriveSmart, RightDriveSmart, 319.19, 292.09999999999997, 9.906, mm, 1.6666666666666667);
motor middleMotor = motor(PORT12, ratio18_1, false);
motor LiftMotorsMotorA = motor(PORT2, ratio18_1, false);
motor LiftMotorsMotorB = motor(PORT3, ratio18_1, true);
motor_group LiftMotors = motor_group(LiftMotorsMotorA, LiftMotorsMotorB);
//limit Switch_low = limit(Brain.ThreeWirePort.A);
limit Switch_hi = limit(Brain.ThreeWirePort.H);
motor intakeMotor = motor(PORT15, ratio18_1,false);
motor rightTiltmotor = motor(PORT4, ratio18_1, false);
motor leftTiltmotor= motor(PORT19, ratio18_1, true);
motor_group frontLift = motor_group(rightTiltmotor, leftTiltmotor);
limit Switch_front = limit(Brain.ThreeWirePort.G);
sonar back_sonar = sonar(Brain.ThreeWirePort.F);
// VEXcode generated functions
// define variable for remote controller enable/disable
//bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}