#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller;

//signature Vision4__GOAL_RED = signature (1, 8889, 9689, 9289, 477, 815, 646, 3.7, 0);
signature Vision4__GOAL_RED = signature (1, 9039, 10037, 9538, -287, 267, -10, 6, 0);
signature Vision4__GOAL_BLUE = signature (2, -2253, -1599, -1926, 7081, 10527, 8804, 3.5, 0);
vision Vision4 = vision (PORT5, 50, Vision4__GOAL_RED, Vision4__GOAL_BLUE);
//vision Vision4 = vision (PORT5, 50, Vision4__GOAL_RED);
//vision Vision5 = vision (PORT5, 50, Vision4__GOAL_BLUE);


motor leftMotorA = motor(PORT1, ratio18_1, true); 
motor leftMotorB = motor(PORT2, ratio18_1, false);
motor leftMotorC = motor(PORT3, ratio18_1, true);
motor_group LeftDriveSmart = motor_group(leftMotorA, leftMotorB, leftMotorC);

motor rightMotorA = motor(PORT14, ratio18_1, false); 
motor rightMotorB = motor(PORT12, ratio18_1, true);
motor rightMotorC = motor(PORT15, ratio18_1, false); 
motor_group RightDriveSmart = motor_group(rightMotorA, rightMotorB, rightMotorC);

int wheelTravel = 4 * M_PI;

drivetrain Drivetrain = drivetrain(LeftDriveSmart, RightDriveSmart, wheelTravel, 13.5, 18, inches, 1);

motor Shooter = motor(PORT19, ratio6_1, true);
motor Intake = motor(PORT20, ratio18_1, false);
digital_out Shooter_pneum = digital_out(Brain.ThreeWirePort.A);
digital_out String = digital_out(Brain.ThreeWirePort.B);
// VEXcode generated functions

inertial imu = inertial(PORT16);

timer testtimer = timer();
timer DebounceTimer = timer();
timer PidDriveTimer = timer();
timer RollerTimer = timer();

distance dist_sensor = distance(PORT10);
optical opt_sensor = optical(PORT9, false);





/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Text.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialibjhfkyu
}