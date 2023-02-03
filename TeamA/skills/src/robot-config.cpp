#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller;

motor L1 = motor(PORT13, ratio18_1, false); 
motor L2 = motor(PORT9, ratio18_1, false);
motor L3 = motor(PORT10, ratio18_1, true);
motor_group LeftDriveSmart = motor_group(L1, L2, L3);

motor R1 = motor(PORT12, ratio18_1, true); 
motor R2 = motor(PORT2, ratio18_1, true);
motor R3 = motor(PORT1, ratio18_1, false); 
motor_group RightDriveSmart = motor_group(R1, R2, R3);

inertial Inertia = inertial(PORT4);

int wheelTravel = 4 * M_PI;

drivetrain Drivetrain = drivetrain(LeftDriveSmart, RightDriveSmart, wheelTravel, 13.5, 18, inches, 1);

motor Shooter = motor(PORT3, ratio6_1, false);
motor Intake = motor(PORT11, ratio18_1, false);
digital_out Shooter_pneum = digital_out(Brain.ThreeWirePort.A);
digital_out String = digital_out(Brain.ThreeWirePort.B);

timer Debounce = timer();
timer PidDriveTimer = timer();

distance dist_sensor = distance(PORT5);
optical colour_sensor = optical(PORT7);
// VEXcode generated functions




/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Text.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}