/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Sun Nov 6 2022                                          */
/*    Description:  Left AWP                             */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// Drivetrain           drivetrain    11, 13, 14, 15, 16, 20        
// Shooter              motor         3               
// Intake               motor         12  
// Shooter_pneum        digital_out   A
// String               digital_out   B             
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h" 

using namespace vex;

// A global instance of competition
competition Competition;
long pid_turn_by(double angle);
long pid_drive(double distance, int32_t time=60000, double space=0);
void extShoot(void);
void driveBackwardTime(double time, double power);
long distance_pid_drive(double space);
#define PRINT_LEVEL_MUST 0
#define PRINT_LEVEL_NORMAL 1
#define PRINT_LEVEL_DEBUG 2

#define DEBUG_LEVEL PRINT_LEVEL_DEBUG

#define DEBUG_PRINT(dl, fmt, args...) {if (dl <= DEBUG_LEVEL) printf(fmt, ## args);}
// define your global instances of motors and other devices here

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void red_roller(void) {
  colour_sensor.setLight(ledState :: on);
  double hue = colour_sensor.hue();
  Intake.setVelocity(70, percent);
  while (hue > 45  && hue < 345) {
      Intake.spin(forward);
      hue = colour_sensor.hue();
      //printf("hue %.2f\n", hue);
  }
  colour_sensor.setLight(ledState :: off);
  Intake.stop();
}

void blue_roller(void) {
  colour_sensor.setLight(ledState :: on);
  double hue = colour_sensor.hue();
  printf("%.2f\n", hue);
  Intake.setVelocity(70, percent);
  Intake.spin(forward);
  while (hue > 260 || hue < 130) {
      hue = colour_sensor.hue();
  }
  colour_sensor.setLight(ledState :: off);
  Intake.stop();
}
void LaunchShoot(void) {
  Shooter_pneum.set(true);
  wait(100, msec);
  Shooter_pneum.set(false);
}

void TriShoot(void) {
  Shooter_pneum.set(true);
  wait(100, msec);
  Shooter.spin(forward, 8.1, volt);
  Shooter_pneum.set(false);
  wait(350, msec);
  Shooter_pneum.set(true);
  wait(200, msec);
  Shooter.spin(forward, 8.75, volt);
  Shooter_pneum.set(false);
  wait(350, msec);
  Shooter_pneum.set(true);
  wait(200, msec);
  Shooter_pneum.set(false);
  Shooter.spin(forward, 7, volt);
}
void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  Drivetrain.setDriveVelocity(80, percent);
  Intake.setVelocity(80, percent);
  Inertia.calibrate();
  while (Inertia.isCalibrating()) {
    wait(25, msec);
  }
  Drivetrain.setStopping(hold);
  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/
void tdrivefor(double rotation, double power, int32_t time) {
  Drivetrain.setTimeout(time, msec);
  Drivetrain.setDriveVelocity(power, percent);
  Drivetrain.driveFor(forward, rotation, inches);
}
void tdriverev(double rotation, double power, int32_t time) {
  Drivetrain.setTimeout(time, msec);
  Drivetrain.setDriveVelocity(power, percent);
  Drivetrain.driveFor(reverse, rotation, inches);
}
void drivefor(int deg, int vel) {
  Drivetrain.setDriveVelocity(vel, percent);
  Drivetrain.driveFor(forward, deg, mm);
}
void driverev(int deg, int vel) {
  Drivetrain.setDriveVelocity(vel, percent);
  Drivetrain.driveFor(reverse, deg, mm);
}
void driveturnright(int deg) {
  RightDriveSmart.spinFor(forward, deg, degrees, true);
  LeftDriveSmart.spinFor(forward, deg, degrees, true);
}
void driveturnleft(int deg) {
  LeftDriveSmart.spinFor(reverse, deg, degrees, true);
  RightDriveSmart.spinFor(reverse, deg, degrees, true);
}
void turn_135(void) {
  pid_turn_by(135);
}
void autonomous(void) {
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................

  /* roller 1
  pick up second disc from corner
  roller 2
  drive & shoot from centre of tile in front of roller 2
  turn to pick up row of 3 discs
  turn to shoot
  go straight to knock over stack of 3 & pick up discs
  turn and shoot
  move to pick up stack of 3 on way to roller 4
  roller 4
  turn to do roller 3
  drive 2 tiles & turn & shoot
  drive to centre of 4 tiles in roller 3/4 corner
  turn & expand */
  
   // setup that should prob be moved to pre-auton
  Drivetrain.setDriveVelocity(80, percent);
  // roller 1
  pid_drive(-6);
  pid_turn_by(90);
  pid_drive(27);
  pid_turn_by(90);
  tdriverev(2, 50, 400);
  red_roller();
  drivefor(100, 50);
  Intake.setVelocity(70, percent);
  Intake.spin(reverse);
  pid_turn_by(143); // pick up disc
  pid_drive(-16.5);

  pid_turn_by(-55); // roller 2
  tdriverev(9.5, 70, 1000);
  red_roller();

  pid_drive(4); // 18
  Shooter.spin(forward, 8, voltageUnits::volt);
  pid_turn_by(-88); // -90
  wait(1200, msec);
  pid_drive(10);
  Shooter_pneum.set(true); // shoot 3 discs
  wait(100, msec);
  Shooter.spin(forward, 8.7, volt);
  Shooter_pneum.set(false);
  wait(1200, msec);
  Shooter_pneum.set(true);
  wait(100, msec);
  Shooter.spin(forward, 8.7, volt);
  Shooter_pneum.set(false);
  wait(1200, msec);
  Shooter_pneum.set(true);
  wait(200, msec);
  Shooter_pneum.set(false); 
  Shooter.stop();
  Intake.stop();
  
  wait(1.5, sec);

  pid_turn_by(-135); // pick up row of 3 discs
  Intake.spin(reverse);
  driverev(150, 50);
  pid_drive(-8);
  pid_drive(5);
  pid_drive(-40);

  pid_turn_by(90);
  Shooter.spin(forward, 7, voltageUnits::volt);
  Intake.spin(forward);
  wait(3, sec);
  Shooter_pneum.set(true); // shoot 3 discs
  wait(100, msec);
  Shooter.spin(forward, 7.5, volt);
  Shooter_pneum.set(false);
  wait(1200, msec);
  Shooter_pneum.set(true);
  wait(100, msec);
  Shooter.spin(forward, 7.5, volt);
  Shooter_pneum.set(false);
  wait(1200, msec);
  Shooter_pneum.set(true);
  wait(200, msec);
  Shooter_pneum.set(false); 
  Shooter.stop();
  Intake.stop();

  pid_turn_by(-90); // pick up stack of 3 discs
  pid_drive(-15);
  pid_drive(5);
  Intake.spin(reverse);
  pid_drive(-18);
  wait(0.2, sec);
  pid_drive(-15);
  pid_turn_by(165);
  Shooter.spin(forward, 8.1, voltageUnits::volt);
  Shooter_pneum.set(true); // shoot 3 discs
  Intake.spin(forward);
  wait(100, msec);
  Shooter.spin(forward, 8.5, volt);
  Shooter_pneum.set(false);
  wait(1200, msec);
  Shooter_pneum.set(true);
  wait(100, msec);
  Shooter.spin(forward, 8.5, volt);
  Shooter_pneum.set(false);
  wait(1200, msec);
  Shooter_pneum.set(true);
  wait(200, msec);
  Shooter_pneum.set(false); 
  Shooter.stop();
  Intake.spin(reverse);
  pid_turn_by(135); // pick 3 discs
  pid_drive(-40);

  pid_turn_by(-60);
  Shooter.spin(forward, 8.2, voltageUnits::volt);
  wait(2, sec);
  Shooter_pneum.set(true); // shoot 3 discs
  wait(100, msec);
  Shooter.spin(forward, 8.7, volt);
  Shooter_pneum.set(false);
  wait(1200, msec);
  Shooter_pneum.set(true);
  wait(100, msec);
  Shooter.spin(forward, 8.7, volt);
  Shooter_pneum.set(false);
  wait(1200, msec);
  Shooter_pneum.set(true);
  wait(200, msec);
  Shooter_pneum.set(false); 
  Shooter.stop();

  // pick up stack of 3
  // shoot 3

  tdriverev(8, 60, 1500); // roller 4
  wait(300, msec);

  pid_drive(13);
  pid_turn_by(-90);
  tdriverev(13.5, 70, 2000); // roller 3
  wait(300, msec);

  pid_drive(13); // expansions
  pid_turn_by(45);
  extShoot();


  // Intake.setVelocity(70, percent);
  // Drivetrain.setDriveVelocity(80, percent);
  // Intake.spin(reverse);
  // driverev(90, 50);
  // wait(300, msec);
  // drivefor(100, 50);
  // pid_turn_by(135);
  // pid_drive(-18);
  // pid_turn_by(-40);
  // Shooter.spin(forward, 8, voltageUnits::volt);
  // tdriverev(12, 60, 1500);
  // Drivetrain.stop();
  // wait(800, msec);
  // pid_drive(5);
  // pid_turn_by(90);
  // pid_drive(-20);
  // pid_turn_by(90);
  // tdrivefor(20, 60, 1500);
  // Intake.stop();
  // Inertia.calibrate();
  // while (Inertia.isCalibrating()) {
  //   wait(25, msec);
  // }
  // pid_drive(-15);
  // pid_turn_by(90);
  // pid_drive(35);


  // Intake.spin(reverse);  // pick up disc
  // driverev(225, 50);
  // drivefor(100, 50);
  // pid_turn_by(-135);
  // driverev(100, 60); // do first roller
  // Shooter.spin(forward, 9, voltageUnits::volt);
  // wait(350, msec);
  // Intake.stop();
  // pid_drive(20);
  // pid_turn_by(90);
  // Intake.spin(forward); // do second roller
  // pid_drive(-21);
  // driverev(90, 50); 
  // wait(500, msec);
  // Intake.stop();
  // drivefor(100, 80);
  // pid_turn_by(-73);
  // pid_drive(55);
  // pid_turn_by(-15);
  // Shooter_pneum.set(true); // shoot 3 discs
  // wait(100, msec);
  // Shooter.spin(forward, 9, volt);
  // Shooter_pneum.set(false);
  // wait(400, msec);
  // Shooter_pneum.set(true);
  // wait(100, msec);
  // Shooter.spin(forward, 9, volt);
  // Shooter_pneum.set(false);
  // wait(400, msec);
  // Shooter_pneum.set(true);
  // wait(200, msec);
  // Shooter_pneum.set(false); 
  // Shooter.stop();

  // pid_turn_by(-5);
  // pid_drive(-40);
  // pid_turn_by(-135);
  // Intake.spin(reverse); // pick up 3 more discs
  // pid_drive(-30);
  // pid_turn_by(135);
  // tdriverev(65, 50, 500);
  // Shooter.spin(forward, 9, voltageUnits::volt);
  // Inertia.calibrate();
  // while(Inertia.isCalibrating()) {
  //   wait(25, msec);
  // }
  // Intake.stop();
  // pid_drive(10);
  // pid_turn_by(108);
  // Shooter_pneum.set(true); // shoot 3 discs
  // wait(100, msec);
  // Shooter.spin(forward, 9, volt);
  // Shooter_pneum.set(false);
  // wait(400, msec);
  // Shooter_pneum.set(true);
  // wait(100, msec);
  // Shooter.spin(forward, 8.5, volt);
  // Shooter_pneum.set(false);
  // wait(400, msec);
  // Shooter_pneum.set(true);
  // wait(200, msec);
  // Shooter_pneum.set(false); 
  // Shooter.stop();

  // Intake.spin(forward);
  // pid_turn_by(-18);
  // pid_drive(-80);
  // pid_turn_by(-90);
  // driverev(150, 60); // third roller
  // wait(350, msec);
  // pid_drive(31.5);
  // pid_turn_by(90);
  // pid_drive(-21); // fourth roller
  // wait(350, msec);
  // pid_drive(12);
  // Intake.stop();
  // pid_turn_by(-45);
  // pid_drive(-5);
  // extShoot(); // string shooter
  // pick up disc
  // go to goal
  // shoot 3 discs
  // pick up 2 more discs
  // go to goal
  // shoot 3 discs
  // go to other rollers
  // turn both
  // expansions

  }

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Text                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/
bool spin1 = false;
void SpinIntakeForwards(void){
  printf("test: %d \n", spin1);
  if (Debounce.value() < 0.1) {
    return;
  }
  Debounce.reset();
  if (spin1 == false){
    Intake.spin(forward, 80, percent);
    spin1 = true;
  }
  else {
    Intake.stop();
    spin1 = false;
  }

}

bool spin2 = false;
void SpinIntakeBackwards(void){
  printf("test: %d \n", spin2);
  if (Debounce.value() < 0.1) {
    return;
  }
  Debounce.reset();
  if (spin2 == false){
    Intake.spin(reverse, 80, percent);
    spin2 = true;
  }
  else {
    Intake.stop();
    spin2 = false;
  }
}


void extShoot(void) {
  String.set(true);
  wait(1, sec);
  String.set(false);
  wait(1, sec);
  String.set(true);
  wait(1, sec);
  String.set(false);
  wait(1, sec);
  String.set(true);
  wait(1, sec);
  String.set(false);

}

bool shootspin = false;
void SpinShooter(void) {
  printf("test: %d \n", shootspin);
  if (Debounce.value() < 0.1) {
    return;
  }
  Debounce.reset();
  if (shootspin == false) {
    Shooter.spin(forward, 7, voltageUnits::volt);
    shootspin = true;
    
  }
  else {
    Shooter.stop();
    shootspin = false;
  }

}

double turn_kp = 0.1; //1.5
double turn_ki = 0.0004; //0.0009
double turn_kd = 0;
double turn_tolerance = 0.2;    // we want to stop when we reach the desired angle +/- 1 degree

long pid_turn(double angle) {
  double delay = 20;   // Inertia can output reading at a rate of 50 hz (20 msec)
  long loop_count = 0;
  double error = 5000;
  double total_error = 0;
  double derivative = 0;
  double prev_error = 0;
  double voltage = 0;
  double min_volt = 2.5;   // we don't want to apply less than min_volt, or else drivetrain won't move
//  double max_volt = 11.5;  // we don't want to apply more than max volt, or else we may damage motor
  double max_volt = 6.5;  // we don't want to apply more than max volt, or else we may damage motor
  bool direction = true;

  //DEBUG_PRINT(PRINT_LEVEL_NORMAL, "Turn to angle %.2f, current angle %.2f\n", angle, Inertia.rotation());
  // keep turning until we reach desired angle +/- tolerance
  while (error > turn_tolerance) {
    error = angle - Inertia.rotation();
    if (error < 0) {
      error = error * -1;
      direction = false;
    } else {
      direction = true;
    }
    total_error += error;   // used for integration term
    derivative = error - prev_error;
    voltage = turn_kp * error + turn_ki * total_error - turn_kd * derivative;
    if (voltage < min_volt) {
        voltage = min_volt;
      } else if (voltage > max_volt) {
      voltage = max_volt;
    }
    //DEBUG_PRINT(PRINT_LEVEL_DEBUG, "error %.2f, voltage %.2f, direction %d, rotation %.2f\n", error, voltage, direction, Inertia.rotation());
    if (direction) {
      RightDriveSmart.spin(reverse, voltage, volt);
      LeftDriveSmart.spin(forward, voltage, volt);
    } else {
      RightDriveSmart.spin(forward, voltage, volt);
      LeftDriveSmart.spin(reverse, voltage, volt);
    }
    prev_error = error;
    wait(delay, msec);
    ++loop_count;
  }
  RightDriveSmart.stop();
  LeftDriveSmart.stop();
  //DEBUG_PRINT(PRINT_LEVEL_DEBUG, "Turn to angle %.2f, current angle %.2f, loop count %ld\n", angle, Inertia.rotation(), loop_count);
  return loop_count;
}

long pid_turn_by (double angle) 
{
  return pid_turn(Inertia.rotation() + angle);
}

////////////////////////////////////
// Find kp, ki and kd values that make pid most accurate
////////////////////////////////////
void tune_turn_pid(void)
{
  //turn_kp = 0.09; this was original
  turn_kp = 0.1;
  turn_ki = 0.0009;
  turn_tolerance = 0.2;
  turn_kd = 0.0;
  long loop_count;
  int i;
  Inertia.calibrate();
  wait(2, sec);
  for(i = 0; i < 10; ++i) {
    wait(2, sec);
    loop_count = pid_turn(90 * (i + 1));
    //DEBUG_PRINT(PRINT_LEVEL_NORMAL, "kd %.4f, loop count %ld, final angle %.2f\n", turn_kd, loop_count, Inertia.rotation());
  }

}

////////////////////////////////////DRIVE_PID////////////////////////////////////////

double drive_kp = 3;
double drive_ki = 0.0015;
double drive_kd = 0.09;
double drive_tolerance = 0.1;    // we want to stop when we reach the desired angle +/- 1 degree

long pid_drive(double distance, int32_t time, double space) {
  double delay = 20;   // Inertia can output reading at a rate of 50 hz (20 msec)
  long loop_count = 0;
  double error = 5000;
  double total_error = 0;
  double derivative = 0.1;
  double prev_error = 0;
  double voltage = 0;
  double min_volt = 2.5;   // we don't want to apply less than min_volt, or else drivetrain won't move
  double max_volt = 11.5;  // we don't want to apply more than max volt, or else we may damage motor
  bool direction = true;
  double rotation = distance / (4 * M_PI);
  double current_rotation = (RightDriveSmart.position(turns) + LeftDriveSmart.position(turns)) / 2;
  rotation += current_rotation;
  double start_time = PidDriveTimer.time(msec);
  double current_space = 144;
  if (dist_sensor.isObjectDetected()) {
    current_space = dist_sensor.objectDistance(inches);
  }

  //DEBUG_PRINT(PRINT_LEVEL_NORMAL, "Drive by distance %.2f, current_space %.2f, space %.2f\n", distance, current_space, space);
  // keep turning until we reach desired angle +/- tolerance
  while ((error > drive_tolerance) && (PidDriveTimer.time(msec) < (start_time + time)) && (current_space >= space )) {
    current_rotation = (RightDriveSmart.position(turns) + LeftDriveSmart.position(turns)) / 2;
    error = rotation - current_rotation;
    if (error < 0) {
      error = error * -1;
      direction = false;
    } else {
      direction = true;
    }
    total_error += error;   // used for integration term
    derivative = error - prev_error;
    voltage = drive_kp * error + drive_ki * total_error - drive_kd * derivative;
    if (voltage < min_volt) {
        voltage = min_volt;
      } else if (voltage > max_volt) {
      voltage = max_volt;
    }
    if ((loop_count < 20) && (voltage > min_volt)){
      voltage = min_volt + ((voltage - min_volt) / 20) * loop_count;
    }
    //DEBUG_PRINT(PRINT_LEVEL_DEBUG, "error %.2f, voltage %.2f, direction %d, angle %.2f\n", error, voltage, direction, Inertia.rotation());
    if (direction) {
      RightDriveSmart.spin(forward, voltage, volt);
      LeftDriveSmart.spin(forward, voltage, volt);
    } else {
      RightDriveSmart.spin(reverse, voltage, volt);
      LeftDriveSmart.spin(reverse, voltage, volt);
    }
    prev_error = error;
    if (dist_sensor.isObjectDetected()) {
      current_space = dist_sensor.objectDistance(inches);
    }
    wait(delay, msec);
    ++loop_count;
  }
  RightDriveSmart.stop();
  LeftDriveSmart.stop();
  //DEBUG_PRINT(PRINT_LEVEL_DEBUG, "drive by distance %.2f, loop count %ld\n", distance, loop_count);
  return loop_count;
}

////////////////////////////////////
// Find kp, ki and kd values that make pid most accurate
////////////////////////////////////
void tune_drive_pid(void)
{
  drive_kp = 3;
  drive_ki = 0.0015;
  drive_kd = 0.09;
  drive_tolerance = 0.2;
  long loop_count;
  int i;
  for(i = 0; i < 1; ++i) {
    wait(2, sec);
    loop_count = pid_drive(96);
    //DEBUG_PRINT(PRINT_LEVEL_NORMAL, "kd %.4f, loop count %ld, final angle %.2f\n", drive_kd, loop_count, Inertia.rotation());
  }
  for(i = 0; i < 1; ++i) {
    wait(2, sec);
    loop_count = pid_drive(-96);
    //DEBUG_PRINT(PRINT_LEVEL_NORMAL, "kd %.4f, loop count %ld, final angle %.2f\n", drive_kd, loop_count, Inertia.rotation());
  }
}
////////////////////////drive_pid/////////////////////////////////////
double distance_drive_kp = 3/12.3;
double distance_drive_ki = 0.0015/12.3;
double distance_drive_kd = 0.09/12.3;
double distance_drive_tolerance = 0.5;


////////////////////////DISTANCE_PID_DRIVE////////////
long distance_pid_drive(double space) {
  double delay = 20;   // imu can output reading at a rate of 50 hz (20 msec)
  long loop_count = 0;
  double error = 5000;
  double total_error = 0;
  double derivative = 0;
  double prev_error = 0;
  double voltage = 0;
  double min_volt = 2.5;   // we don't want to apply less than min_volt, or else drivetrain won't move
  double max_volt = 11.5;  // we don't want to apply more than max volt, or else we may damage motor
  bool direction = true;
  //double rotation = distance / (4 * M_PI);
  //double current_rotation = (RightDriveSmart.position(turns) + LeftDriveSmart.position(turns)) / 2;
  //rotation += current_rotation;
  //double start_time = PidDriveTimer.time(msec);
  double current_space = 144;
  if (dist_sensor.isObjectDetected()) {
    current_space = dist_sensor.objectDistance(inches);
  }
  DEBUG_PRINT(PRINT_LEVEL_NORMAL, "current_space %.2f, space %.2f\n", current_space, space);
  // keep turning until we reach desired angle +/- tolerance
  while (error > distance_drive_tolerance) {
    //current_rotation = (RightDriveSmart.position(turns) + LeftDriveSmart.position(turns)) / 2;
    error = current_space - space;
    if (error < 0) {
      error = error * -1;
      direction = false;
    } else {
      direction = true;
    }
    total_error += error;   // used for integration term
    derivative = error - prev_error;
    voltage = distance_drive_kp * error + distance_drive_ki * total_error - distance_drive_kd * derivative;
    if (voltage < min_volt) {
        voltage = min_volt;
      } else if (voltage > max_volt) {
      voltage = max_volt;
    }
    if ((loop_count < 20) && (voltage > min_volt)){
      voltage = min_volt + ((voltage - min_volt) / 20) * loop_count;
    }
    DEBUG_PRINT(PRINT_LEVEL_DEBUG, "error %.2f, voltage %.2f, direction %d, angle %.2f\n", error, voltage, direction, Inertia.rotation());
    if (direction) {
      RightDriveSmart.spin(forward, voltage, volt);
      LeftDriveSmart.spin(forward, voltage, volt);
    } else {
      RightDriveSmart.spin(reverse, voltage, volt);
      LeftDriveSmart.spin(reverse, voltage, volt);
    }
    prev_error = error;
    current_space = dist_sensor.objectDistance(inches);
    wait(delay, msec);
    ++loop_count;
  }
  RightDriveSmart.stop();
  LeftDriveSmart.stop();
  //DEBUG_PRINT(PRINT_LEVEL_DEBUG, "drive by distance %.2f, loop count %ld\n", distance loop_count);
  return loop_count;
}
void usercontrol(void) {
  // User control code here, inside the loop

  double turnImportance = 1;
  while (1) {

    double turnVal = Controller.Axis3.position(percent);
    double forwardVal = Controller.Axis1.position(percent);

    double turnVolts = turnVal * -0.12 * 1.4;
    double forwardVolts = forwardVal * 0.12 * 1.4 * (1 - (abs(turnVolts)/12.0) * turnImportance);
    if (turnVolts > 12) {
      turnVolts = 12;
    }
    if (forwardVolts > 12) {
      forwardVolts = 12;
    }
    // 12 - 12 = 0
    // 12 + 12 = 12(due to cap)

    LeftDriveSmart.spin(forward, forwardVolts - turnVolts, voltageUnits::volt);
    RightDriveSmart.spin(reverse, forwardVolts + turnVolts, voltageUnits::volt);

    Controller.ButtonL1.pressed(SpinIntakeForwards);
    Controller.ButtonL2.pressed(SpinIntakeBackwards);
    Controller.ButtonR1.pressed(SpinShooter);
    Controller.ButtonR2.pressed(TriShoot);
    Controller.ButtonY.pressed(extShoot);
    Controller.ButtonUp.pressed(LaunchShoot);
    Controller.ButtonRight.pressed(blue_roller);
    Controller.ButtonLeft.pressed(red_roller);
    





    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
