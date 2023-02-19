/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Clawbot Competition Template                              */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// Drivetrain           drivetrain    1, 10, D        
// ClawMotor            motor         3               
// ArmMotor             motor         8               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include <cmath>
#include "vex.h" 

using namespace vex;

// A global instance of competition
competition Competition;
void driveTrain(void);
void driveForward(double rotation, double power, int32_t time=60000);
void driveBackward(double rotation, double power, int32_t time=60000);
void turnRight(double angle, double power);
void turnLeft(double angle, double power);
long pid_turn_by(double angle);
long pid_drive(double distance, int32_t time=60000, double space=0);
void extShoot(void);
void driveBackwardTime(double time, double power);
void SpinIntakeBackwards(void);
long distance_pid_drive(double space);
void RollerAuto(int32_t time=2000);
void RollerAutoDrive();


bool spin1 = false;
bool shootspin = false;


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

void LaunchShoot(void) {
  // uint32_t prev_time = 0;
  // uint32_t cur_time = 0;
  // testtimer.reset();
  // while(testtimer.time() < 5000) {
  //   cur_time = testtimer.time();
  //   if (cur_time > prev_time) {
  //     // printf("%lu,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n", 
  //     // cur_time, Shooter.velocity(pct), Shooter.current(pct), 
  //     // Shooter.voltage(volt), Shooter.power(watt), Shooter.torque(Nm), 
  //     // Shooter.efficiency(pct), Shooter.temperature(fahrenheit));
  //     printf("%lu,%.2f,%.2f\n", 
  //     cur_time, Shooter.velocity(pct), Shooter.current(pct));
  //     prev_time = cur_time;
  //   }
  // }

  Shooter_pneum.set(true);
  wait(100, msec);
  Shooter.spin(forward, 10, volt);
  Shooter_pneum.set(false);
  wait(350, msec);
  Shooter_pneum.set(true);
  wait(100, msec);
  Shooter.spin(forward, 10, volt);
  Shooter_pneum.set(false);
  wait(350, msec);
  Shooter_pneum.set(true);
  wait(100, msec);
  Shooter_pneum.set(false);
  Shooter.stop();
  shootspin = false;
}

void LaunchShootFar(void) {
  Shooter_pneum.set(true);
  wait(100, msec);
  Shooter.spin(forward, 10.25, volt);
  Shooter_pneum.set(false);
  wait(700, msec);
  Shooter_pneum.set(true);
  wait(100, msec);
  Shooter_pneum.set(false);
  Shooter.stop();
}

void LongShoot(void) {
  // uint32_t prev_time = 0;
  // uint32_t cur_time = 0;
  // testtimer.reset();
  // while(testtimer.time() < 5000) {
  //   cur_time = testtimer.time();
  //   if (cur_time > prev_time) {
  //     // printf("%lu,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n", 
  //     // cur_time, Shooter.velocity(pct), Shooter.current(pct), 
  //     // Shooter.voltage(volt), Shooter.power(watt), Shooter.torque(Nm), 
  //     // Shooter.efficiency(pct), Shooter.temperature(fahrenheit));
  //     printf("%lu,%.2f,%.2f\n", 
  //     cur_time, Shooter.velocity(pct), Shooter.current(pct));
  //     prev_time = cur_time;
  //   }
  // }

  Shooter_pneum.set(true);
  wait(100, msec);
  Shooter.spin(forward, 11.25, volt);
  Shooter_pneum.set(false);
  wait(350, msec);
  Shooter_pneum.set(true);
  wait(100, msec);
  Shooter.spin(forward, 11.25, volt);
  Shooter_pneum.set(false);
  wait(350, msec);
  Shooter_pneum.set(true);
  wait(100, msec);
  Shooter_pneum.set(false);
  Shooter.stop();
  shootspin = false;
}


void LaunchShootMedium(void) {
  Shooter.spin(forward, 8.5, volt);
  Shooter_pneum.set(false);
  wait(600, msec);
  Shooter_pneum.set(true);
  wait(100, msec);
  Shooter.spin(forward, 9.5, volt);
  Shooter_pneum.set(false);
  wait(600, msec);
  Shooter_pneum.set(true);
  wait(100, msec);
  Shooter.spin(forward, 9.5, volt);
  Shooter_pneum.set(false);
  wait(650, msec);
  Shooter_pneum.set(true);
  wait(100, msec);
  Shooter_pneum.set(false);
  Shooter.stop();
}
    

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  Drivetrain.setDriveVelocity(80, percent);
  imu.calibrate();
  while (imu.isCalibrating()) {
    wait(25, msec);
  }
  RightDriveSmart.setStopping(hold);
  LeftDriveSmart.setStopping(hold);
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

void autonomous(void) {
  Shooter.spin(forward, 9, volt);
  pid_drive(-24);
  pid_turn_by(89);
  driveBackward(15, 30, 800);
  Intake.spin(forward, 70, percent);
  wait(300, msec);
  pid_drive(3);
  pid_turn_by(-2);
  LaunchShootFar();
  pid_turn_by(133);
  Intake.spin(reverse, 12, volt);
  pid_drive(-30);
  Shooter.spin(forward, 8.5, volt);
  pid_drive(-40);
  pid_turn_by(-98);
  LaunchShootMedium();
  Shooter.spin(forward, 2, volt);
  Shooter_pneum.set(true);
  wait(100, msec);
  Shooter_pneum.set(false);
  Shooter.stop();  

  return;
  pid_drive(-25);
  Shooter.spin(forward, 8.5, volt); //start shooter early to save time - 8
  pid_turn_by(90);
  // turn right = 90, 30
  driveBackward(5, 45, 1000);
  Intake.spin(forward, 100, percent); //doing rollers
  wait(375, msec);
  Intake.stop(); //rollers done
  pid_drive(3);
  pid_turn_by(-90); 
  // turn left = 80, 20
  //driveForward(25, 60);
  pid_drive(25);
  Intake.spin(reverse, 100, percent); //turn on intake for disc
  pid_turn_by(-96);
  //pid_turn_by(-20);
  //driveBackward(23, 60);
  pid_drive(-23);//intake disc
  //wait(1, sec);
  //Intake.stop();
  //Intake.stop();
  //driveBackward(6, 45);
  //pid_drive(-6);
  pid_turn_by(-158); //turn to shooting position
  //pid_turn_by(-70);
  //wait(2, sec);
  Shooter_pneum.set(true); //shoots first
  wait(100, msec);
  Shooter_pneum.set(false);
  Shooter.spin(forward, 11, volt);
  wait(600, msec);
  Shooter_pneum.set(true); //shoots second
  wait(100, msec);
  Shooter_pneum.set(false);
  Shooter.spin(forward, 11, volt);
  wait(600, msec);
  Shooter_pneum.set(true); //shoots third
  wait(100, msec);
  Shooter_pneum.set(false);
  Shooter.stop();
}

double turn_kp = 0.1; //1.5
double turn_ki = 0.0004; //0.0009
double turn_kd = 0;
double turn_tolerance = 0.2;    // we want to stop when we reach the desired angle +/- 1 degree

long pid_turn(double angle) {
  double delay = 20;   // imu can output reading at a rate of 50 hz (20 msec)
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

  DEBUG_PRINT(PRINT_LEVEL_NORMAL, "Turn to angle %.2f, current angle %.2f\n", angle, imu.rotation());
  // keep turning until we reach desired angle +/- tolerance
  while (error > turn_tolerance) {
    error = angle - imu.rotation();
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
    DEBUG_PRINT(PRINT_LEVEL_DEBUG, "error %.2f, voltage %.2f, direction %d, rotation %.2f\n", error, voltage, direction, imu.rotation());
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
  DEBUG_PRINT(PRINT_LEVEL_DEBUG, "Turn to angle %.2f, current angle %.2f, loop count %ld\n", angle, imu.rotation(), loop_count);
  return loop_count;
}

long pid_turn_by (double angle) 
{
  return pid_turn(imu.rotation() + angle);
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
  imu.calibrate();
  wait(2, sec);
  for(i = 0; i < 10; ++i) {
    wait(2, sec);
    loop_count = pid_turn(90 * (i + 1));
    DEBUG_PRINT(PRINT_LEVEL_NORMAL, "kd %.4f, loop count %ld, final angle %.2f\n", turn_kd, loop_count, imu.rotation());
  }

}

////////////////////////////////////DRIVE_PID////////////////////////////////////////

double drive_kp = 4.2; //3
double drive_ki = 0.0015;
double drive_kd = 0.09;
double drive_tolerance = 0.1;    // we want to stop when we reach the desired angle +/- 1 degree

long pid_drive(double distance, int32_t time, double space) {
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
  double rotation = distance / (4 * M_PI);
  double current_rotation = (RightDriveSmart.position(turns) + LeftDriveSmart.position(turns)) / 2;
  rotation += current_rotation;
  double start_time = PidDriveTimer.time(msec);
  double current_space = 144;
  if (dist_sensor.isObjectDetected()) {
    current_space = dist_sensor.objectDistance(inches);
  }

  DEBUG_PRINT(PRINT_LEVEL_NORMAL, "Drive by distance %.2f, current_space %.2f, space %.2f\n", distance, current_space, space);
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
    DEBUG_PRINT(PRINT_LEVEL_DEBUG, "error %.2f, voltage %.2f, direction %d, angle %.2f\n", error, voltage, direction, imu.rotation());
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
  DEBUG_PRINT(PRINT_LEVEL_DEBUG, "drive by distance %.2f, loop count %ld\n", distance, loop_count);
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
    DEBUG_PRINT(PRINT_LEVEL_NORMAL, "kd %.4f, loop count %ld, final angle %.2f\n", drive_kd, loop_count, imu.rotation());
  }
  for(i = 0; i < 1; ++i) {
    wait(2, sec);
    loop_count = pid_drive(-96);
    DEBUG_PRINT(PRINT_LEVEL_NORMAL, "kd %.4f, loop count %ld, final angle %.2f\n", drive_kd, loop_count, imu.rotation());
  }
}
////////////////////////drive_pid/////////////////////////////////////



////////////////////////DISTANCE_PID_DRIVE////////////

double distance_drive_kp = 3/12.3;
double distance_drive_ki = 0.0015/12.3;
double distance_drive_kd = 0.09/12.3;
double distance_drive_tolerance = 0.5;


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
    DEBUG_PRINT(PRINT_LEVEL_DEBUG, "error %.2f, voltage %.2f, direction %d, angle %.2f\n", error, voltage, direction, imu.rotation());
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
  //DEBUG_PRINT(PRINT_LEVEL_DEBUG, "drive by distance %.2f, loop count %ld\n", distance, loop_count);
  return loop_count;
}
//////////////////////////DISTANCE_PID_DRIVE///////////////////////


/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Text                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/
bool driveforward = false;


//drives robot backward
// rotation: how many turns of the wheel to travel backward
// power: velocity of wheels in percentage
void driveBackward(double rotation, double power, int32_t time) {
  Drivetrain.setTimeout(time, msec);
  Drivetrain.setDriveVelocity(power, percent);
  Drivetrain.driveFor(reverse, rotation, inches);
}

//drives robot forward
// rotation: how many turns of the wheel to travel backward
// power: velocity of wheels in percentage
void driveForward(double rotation, double power, int32_t time) {
  Drivetrain.setTimeout(time, msec);
  Drivetrain.setDriveVelocity(power, percent);
  Drivetrain.driveFor(forward, rotation, inches);
}

void turnRight(double angle, double power) {
  double start_angle = imu.yaw();
  angle = angle - (angle - start_angle) * 0.1;
  double current_angle = start_angle;
  LeftDriveSmart.setStopping(hold);
  RightDriveSmart.setStopping(hold);
  RightDriveSmart.spin(reverse, power, percent);
  LeftDriveSmart.spin(forward, power, percent);
  while(current_angle < (start_angle + angle)){
    wait(20, msec);
    current_angle = imu.yaw();
  }
  
  LeftDriveSmart.stop();
  RightDriveSmart.stop();
  printf("start angle %.2f. , end angle %.2f\n", start_angle, imu.yaw());
}

void turnLeft(double angle, double power) {
  double start_angle = imu.yaw();
  double current_angle = start_angle;
  LeftDriveSmart.spin(reverse, power, percent);
  RightDriveSmart.spin(forward, power, percent);
  while(current_angle > (start_angle - angle)){
    wait(20, msec);
    current_angle = imu.yaw();
  }
  LeftDriveSmart.stop();
  RightDriveSmart.stop();
}


/* bool turnLeft = false;
void leftdrivesmart(void){
  if (turnLeft == false){
    LeftDriveSmart.spin(forward, 100, percent);
    turnLeft = true;
  }
  else{
    LeftDriveSmart.stop();
    turnLeft = false;
  }
}

bool turnRight = false;
void rightdrivesmart(void){
  if (turnRight == false){
    RightDriveSmart.spin(forward, 100, percent);
    turnRight = true;
  }
  else{
    RightDriveSmart.stop();
    turnRight = false;
  }
}

*/

void SpinIntakeForwards(void){
  if (DebounceTimer.value() < 0.1) {
    return;
  }
  DebounceTimer.reset();
  if (spin1 == false){
    Intake.spin(forward, 100, percent);
    spin1 = true;
  }
  else {
    Intake.stop();
    spin1 = false;
  }

}

bool spin2 = false;
void SpinIntakeBackwards(void){
  if (DebounceTimer.value() < 0.1) {
    return;
  }
  DebounceTimer.reset();
  if (spin2 == false){
    Intake.spin(reverse, 100, percent);
    Shooter.stop();
    shootspin = false;
    spin2 = true;
  }
  else {
    Intake.stop();
    spin2 = false;
  }
}

void L1Release(void){
  printf("L1Release \n");
}

/* void RollerStop(void){
  Intake.stop();
}
*/

bool roller_spin = false;
void RollerSpinForwards(void){
  if (DebounceTimer.value() < 0.1) {
    return;
  }
  DebounceTimer.reset();
  if (spin2 == false){
    Intake.spin(forward, 50, percent);
    spin2 = true;
  }
  else {
    Intake.stop();
    spin2 = false;
  }
}

bool roller_spin2 = false;
void RollerSpinBackwards(void){
  if (DebounceTimer.value() < 0.1) {
    return;
  }
  DebounceTimer.reset();
  if (spin2 == false){
    Intake.spin(reverse, 50, percent);
    spin2 = true;
  }
  else {
    Intake.stop();
    spin2 = false;
  }
}

int check = 0;
void extCheck(void) {
  check = 1;
}

int check2 = 0;
void extCheck2(void) {
  check2 = 1;
}

int check3 = 0;
void extCheck3(void) {
  check3 = 1;
}

void extShoot(void) {
  String.set(true);
  wait(1, sec);
  String.set(false);
  String.set(true);

}

void SpinLong(void) {
  if (DebounceTimer.value() < 0.1) {
    return;
  }
  DebounceTimer.reset();
  if (shootspin == false) {
    Shooter.spin(forward, 8.25, volt); //7
//    Shooter.spin(forward, 50, percent);
    Intake.stop();
    spin2 = false;
    shootspin = true;
  }
  else {
    Shooter.stop();
    shootspin = false;
  }
}

void SpinShooter(void) {
  if (DebounceTimer.value() < 0.1) {
    return;
  }
  DebounceTimer.reset();
  if (shootspin == false) {
    Shooter.spin(forward, 6.75, volt); //7
//    Shooter.spin(forward, 50, percent);
    Intake.stop();
    spin2 = false;
    shootspin = true;
    
  }
  else {
    Shooter.stop();
    shootspin = false;
  }

}

void ShootOnce(void) {
  Shooter_pneum.set(true);
  wait(100, msec);
  Shooter_pneum.set(false);
}

void ShooterReverse(void) {
  if (DebounceTimer.value() < 0.1) {
    return;
  }
  DebounceTimer.reset();
  Shooter.spin(reverse, 11, volt);
}

void RollerAuto(int32_t time) {
  if (DebounceTimer.value() < 0.1) {
    return;
  }
  DebounceTimer.reset();
  double start_time = RollerTimer.time(msec);
  opt_sensor.objectDetectThreshold(253);
  opt_sensor.setLight(ledState :: on);
  double hue_val = opt_sensor.hue();
  Intake.spin(reverse, 70, percent);
  printf("Installed: %d, hue %.2f, detected %d\n", opt_sensor.installed(), hue_val,
    opt_sensor.isNearObject());
  while (((hue_val < 320) && (hue_val > 45)) && (RollerTimer.time(msec) < (start_time + time))) {
    //340 or higher means red - while less than that - will keep spinning
    //250 or lower means blue
    Intake.spin(reverse, 70, percent);
    hue_val = opt_sensor.hue();
//    printf("Installed: %d, hue %.2f, detected %d\n", opt_sensor.installed(), hue_val,
//      opt_sensor.isNearObject());
  }
  Intake.stop();
  opt_sensor.setLight(ledState :: off);
}

void RollerAutoDrive(void) {
  RollerAuto(2000);
}

double vision_kp = 0.01; //1.5
double vision_ki = 0.00004; //0.0009
double vision_kd = 0;
double vision_tolerance = 4;    // we want to stop when we reach the desired angle +/- 1 degree

uint32_t VisionPid(uint32_t GoalX) {
  double delay = 50;   // imu can output reading at a rate of 50 hz (20 msec)
  long loop_count = 0;
  double error = 5000;
  double total_error = 0;
  double derivative = 0;
  double prev_error = 0;
  double voltage = 0;
  uint32_t ObjNum = Vision4.takeSnapshot(Vision4__GOAL_RED);
  printf("ObjNum %lu\n", ObjNum);
  vision::object LObject = Vision4.largestObject;
  double min_volt = 2.5;   // we don't want to apply less than min_volt, or else drivetrain won't move
//  double max_volt = 11.5;  // we don't want to apply more than max volt, or else we may damage motor
  double max_volt = 6.5;  // we don't want to apply more than max volt, or else we may damage motor
  bool direction = true;
  DEBUG_PRINT(PRINT_LEVEL_NORMAL, "Turn to Goalx %lu, current x %d\n", GoalX, LObject.centerX);
  // keep turning until we reach desired angle +/- tolerance
  while (error > vision_tolerance) {
    error = GoalX - LObject.centerX;
    if (error > 0) {
      direction = false;
    } else {
      error = error * -1;
      direction = true;
    }
    total_error += error;   // used for integration term
    derivative = error - prev_error;
    voltage = vision_kp * error + vision_ki * total_error - vision_kd * derivative;
    if (voltage < min_volt) {
        voltage = min_volt;
      } else if (voltage > max_volt) {
      voltage = max_volt;
    }
    DEBUG_PRINT(PRINT_LEVEL_DEBUG, "error %.2f, voltage %.2f, direction %d, num %lu\n", error, voltage, direction, ObjNum);
    if (direction) {
      RightDriveSmart.spin(reverse, voltage, volt);
      LeftDriveSmart.spin(forward, voltage, volt);
    } else {
      RightDriveSmart.spin(forward, voltage, volt);
      LeftDriveSmart.spin(reverse, voltage, volt);
    }
    prev_error = error;
    wait(delay, msec);
    ObjNum = Vision4.takeSnapshot(Vision4__GOAL_RED);
    ++loop_count;
  }
  RightDriveSmart.stop();
  LeftDriveSmart.stop();
  DEBUG_PRINT(PRINT_LEVEL_DEBUG, "turn to goal %lu, current x %d, loop count %ld\n", GoalX, LObject.centerX, loop_count);
  return loop_count;
}


void VisionAlign(void) {
  if (DebounceTimer.value() < 0.1) {
    return;
  }
  DebounceTimer.reset();
  VisionPid(197);
}

void usercontrol(void) {
  // User control code here, inside the loop

  double turnImportance = 1;
  double speed_ratio = (11.0 / 5.0);
  //tune_turn_pid();
  while (1) {

    double turnVal = Controller.Axis3.position(percent);
    double forwardVal = Controller.Axis1.position(percent);

    
    double turnVolts = turnVal * -0.12;
    double forwardVolts = forwardVal * 0.12 * (1 - (abs(turnVolts)/12.0) * turnImportance);

    // 12 - 12 = 0
    // 12 + 12 = 12(due to cap)

    Controller.ButtonL1.pressed(SpinIntakeForwards);
    Controller.ButtonL2.pressed(SpinIntakeBackwards);
    Controller.ButtonR1.pressed(SpinShooter);
   // Controller.ButtonB.pressed(LeftDriveSmart);
    //Controller.ButtonX.pressed(RightDriveSmart);

    Controller.ButtonR2.pressed(LaunchShoot);
    Controller.ButtonY.pressed(extShoot);

    Controller.ButtonX.pressed(RollerSpinForwards);
    Controller.ButtonB.pressed(RollerSpinBackwards);

    Controller.ButtonUp.pressed(ShootOnce);
    Controller.ButtonDown.pressed(ShooterReverse);
    Controller.ButtonA.pressed(RollerAutoDrive);
    Controller.ButtonLeft.pressed(VisionAlign);
    continue;

    RightDriveSmart.spin(reverse, speed_ratio * (forwardVolts + turnVolts), voltageUnits::volt);
    LeftDriveSmart.spin(forward, speed_ratio * (forwardVolts - turnVolts), voltageUnits::volt);

//    printf("axis1: %.2f, axis3: %.2f, leftv: %.2f, rightv: %.2f\n",
//      forwardVal, turnVal, forwardVolts - turnVolts, forwardVolts + turnVolts);




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
