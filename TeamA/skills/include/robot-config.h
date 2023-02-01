using namespace vex;

extern brain Brain;

// VEXcode devices
extern drivetrain Drivetrain;
extern controller Controller;
extern motor_group LeftDriveSmart;
extern motor_group RightDriveSmart;
extern digital_out Shooter_pneum;
extern digital_out String;
extern motor Shooter;
extern motor Intake;
extern inertial Inertia;
extern timer Debounce;
extern timer PidDriveTimer;
extern distance dist_sensor;
extern optical colour_sensor;

/** 
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );