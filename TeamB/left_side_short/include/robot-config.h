using namespace vex;

extern brain Brain;

// VEXcode devices
extern drivetrain Drivetrain;
extern controller Controller;
extern motor_group LeftDriveSmart;
extern motor_group RightDriveSmart;
extern digital_out Shooter_pneum;
extern motor Shooter;
extern motor Intake;
extern digital_out String;
extern inertial imu;
extern timer testtimer;
extern timer DebounceTimer;
/** 
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );