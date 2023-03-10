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
#include "robot-config.h"
#include "VisionConfig.h"

using namespace vex;

// A global instance of competition
competition Competition;
void driveTrain(void);
void driveForward(double rotation, double power, int32_t time=60000);
void driveBackward(double rotation, double power, int32_t time=60000);
void turnRight(double angle, double power);
void turnLeft(double angle, double power);
long pid_turn_by(double angle);
long pid_drive(double distance, double drive_kp=20, int32_t time=60000, double space=0);
void extShoot(void);
void driveBackwardTime(double time, double power);
void SpinIntakeBackwards(void);
long distance_pid_drive(double space);
void RollerAuto(int32_t time=2000);
void RollerAutoDrive();
void VisionAlignShoot(signature sig, int GoalX);
void CustomLaunch (double v1, double v2, double v3, double t1, double t2, double t3);
uint32_t VisionPid(int GoalX, signature ColorSig);

bool spin1 = false;
bool shootspin = false;
bool vision_in_prog = false;


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
  Shooter.spin(forward, 9.5, volt); //10
  Shooter_pneum.set(false);
  wait(350, msec);
  Shooter_pneum.set(true);
  wait(100, msec);
  Shooter.spin(forward, 9.5, volt);
  Shooter_pneum.set(false);
  wait(350, msec);
  Shooter_pneum.set(true);
  wait(100, msec);
  Shooter_pneum.set(false);
  Shooter.spin(forward, 6.25, volt); //6.75
  //Shooter.stop();
  //shootspin = false;
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
  Shooter.spin(forward, 8.15, volt);
  Shooter_pneum.set(false);
  wait(600, msec);
  Shooter_pneum.set(true);
  wait(100, msec);
  Shooter.spin(forward, 9.5, volt);
  Shooter_pneum.set(false);
  wait(500, msec);
  Shooter_pneum.set(true);
  wait(100, msec);
  Shooter.spin(forward, 9.5, volt);
  Shooter_pneum.set(false);
  wait(450, msec);
  Shooter_pneum.set(true);
  wait(100, msec);
  Shooter_pneum.set(false);
  Shooter.spin(forward, 6.75, volt);
  //Shooter.stop();
  //shootspin = false;
}

void RollerWhole(double distance, double time) {
  driveBackward(distance, 30, time); //goes back into rollers 4.5, 400
  RollerAuto(1500);
}

long LaunchShootCustom(double first_volt, double second_volt, double wait_time) {
  Shooter_pneum.set(true);
  wait(100, msec);
  Shooter.spin(forward, first_volt, volt);
  Shooter_pneum.set(false);
  wait(wait_time, msec);
  Shooter_pneum.set(true);
  wait(100, msec);
  Shooter.spin(forward, second_volt, volt);
  Shooter_pneum.set(false);
  wait(wait_time, msec);
  Shooter_pneum.set(true);
  wait(100, msec);
  Shooter_pneum.set(false);
  Shooter.stop();
  return 0;
}

void jerk(){
  driveForward(6, 40);
  driveBackward(1, 40);
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

  pid_drive(-7.5);
  pid_turn_by(-90); 
  Intake.spin(reverse, 12, volt);
  pid_drive(-18);
  pid_turn_by(8);
  pid_drive(-8);
  pid_drive(5);
  pid_turn_by(-8);
  pid_turn_by(-91);
  Intake.stop();
  RollerWhole(22, 1200); //does first roller
  pid_drive(4.5); //goes away from roller
  Intake.spin(reverse, 12, volt); //intake on
  pid_turn_by(133); //141
  pid_drive(-19); //picks up disc
  Shooter.spin(forward, 6.8, volt); //shooter on, 8
  pid_turn_by(-43); //-54
  Intake.stop();
  RollerWhole(12, 800); //does second roller
  
  
  pid_drive(6); //goes away from roller 
  pid_turn_by(-90);
  VisionPid(180, Vision4__GOAL_BLUE);
  pid_drive(33, 15);
  //pid_turn_by(3);
  VisionPid(180, Vision4__GOAL_BLUE); 
  LaunchShootCustom(8.5, 8.8, 500); //first shot
  
  //pid_turn_by(0.25);
  pid_drive(-11);

  pid_turn_by(-51);
  Intake.stop();
  //wait(1000, msec);;
  pid_drive(-28, 70);
  pid_drive(5);                                                                    
  //wait(500, msec);
  pid_turn_by(-3);
  //wait(500, msec);
  Intake.spin(reverse, 12, volt);
  /*pid_drive(-8, 3);
  wait(400, msec);
  pid_drive(2, -3);
  pid_drive(-8, 3);
  wait(400, msec);
  pid_drive(-14, -3); */
  pid_drive(-10, 5);
  pid_drive(-8, 3);
  Shooter.spin(forward, 7, volt);
  pid_drive(-9, 6);
  //pid_drive(-7, -3);
  pid_turn_by(130);
  pid_drive(3, 25); 
  VisionPid(175, Vision4__GOAL_RED); // second shot
  LaunchShootCustom(8.5, 9.45, 500);
  Shooter.stop();
  Intake.stop();
  pid_drive(-1.5);

  pid_turn_by(104);
  pid_drive(-15, 70);
  pid_drive(4);
  Intake.spin(reverse, 12, volt);
  Shooter.spin(forward, 7.2, volt);
  pid_drive(-9, 2);
  wait(300, msec);
  pid_drive(-7, 2);
  pid_turn_by(-50);
  pid_drive(5);
  VisionPid(185, Vision4__GOAL_RED);
  LaunchShootCustom(9.1, 9.3, 500); // third shot
  pid_turn_by(-25);
  pid_drive(-30, 35);
  pid_turn_by(134);
  Shooter.spin(forward, 6.7, volt);
  pid_drive(-39, 28);
  pid_turn_by(90);
  pid_drive(3);
  VisionPid(180, Vision4__GOAL_BLUE);
  LaunchShootCustom(8, 8.4, 500); // fourth shot
  Intake.stop();



  pid_drive(-12.5, 30);
  pid_turn_by(-136);
  pid_drive(-44, 70);
  pid_turn_by(90);
  pid_drive(-41, 70);
  pid_turn_by(-93);
  RollerWhole(21, 1100);
  pid_drive(13, 70);
  pid_turn_by(90);
  Intake.spin(reverse, 12, volt);
  pid_drive(-10, 30);
  Intake.stop();
  RollerWhole(15, 1000);
  pid_drive(15, 70);
  pid_turn_by(-50);
  extShoot();
  pid_drive(-8, 70);
  extShoot();
  extShoot();
  extShoot();
  extShoot();
  extShoot();
  return;


  pid_turn_by(23);
  pid_drive(-35);
  pid_turn_by(-63);
  extShoot();
  extShoot();
  extShoot();
  extShoot();
  extShoot();

  return;
/*

  


  pid_drive(10);
  pid_turn_by(-85);
  Intake.spin(reverse, 12, volt);
  for (int i = 0; i < 3; ++i) {
    driveBackward(18, 60, 900);
    pid_drive(10, 10);
  }

  Shooter.spin(forward, 7.5, volt);
  pid_drive(6);
  pid_turn_by(90);
  VisionPid(180, Vision4__GOAL_RED);
  LaunchShootCustom(8.5, 9, 400); // third shot
  pid_turn_by(-88);

  Intake.spin(reverse, 12, volt);
  for (int i = 0; i < 3; ++i) {
    driveBackward(18, 60, 900);
    pid_drive(10, 10);
  }

  Shooter.spin(forward, 7.5, volt);
  pid_drive(6);
  pid_turn_by(90);
  VisionPid(180, Vision4__GOAL_RED);
  LaunchShootCustom(8.5, 9, 400); // fourth shot
  pid_turn_by(40);



*/



  pid_turn_by(104.5);
  Intake.stop();
  pid_drive(-15, 50);
  pid_drive(5);
  //pid_turn_by(-2);
  Intake.spin(reverse, 12, volt);
  pid_drive(-3, 3);
  pid_drive(-4, 3);
  Shooter.spin(forward, 8.2, volt);
  pid_drive(-11, 6);
  //Intake.stop();
  pid_turn_by(-70);
  //jerk();
  VisionPid(185, Vision4__GOAL_RED);
  LaunchShootCustom(9.7, 10, 380); // third shot


  pid_turn_by(3);
  Intake.spin(reverse, 12, volt);
  pid_drive(-17); //-25
  pid_turn_by(117); // -55
  pid_drive(-35);
  //pid_turn_by(90);
  //pid_drive(18);
  Shooter.spin(forward, 7, volt);
  pid_turn_by(88);
  VisionPid(185, Vision4__GOAL_BLUE);
  LaunchShootCustom(8, 8.5, 380); // fourth shot


  Intake.spin(reverse, 12, volt);
  pid_turn_by(12);
  pid_drive(-31);
  pid_turn_by(-91.5);
  pid_drive(-40, 17);
  Shooter.spin(forward, 8.5, volt);
  pid_drive(-16, 17);
  pid_turn_by(-45);
  VisionPid(185, Vision4__GOAL_RED);
  LaunchShootCustom(9.75, 10.25, 500); //fifth shot


  pid_drive(-17, 30);
  pid_turn_by(86);
  driveBackward(15, 40, 900);
  pid_drive(13, 60);
  pid_turn_by(-90);
  driveBackward(25, 40, 1200);
  pid_drive(12, 60);
  pid_turn_by(40);
  extShoot();
  pid_drive(-15, 70);
  extShoot();
  extShoot();
  extShoot();
  extShoot();
  extShoot();

  return;



  pid_turn_by(50);
  Intake.stop();
  pid_drive(-27); //-38
  pid_turn_by(146.85);
  pid_drive(-22, 30);
  pid_drive(5); 
  pid_turn_by(10);
  Intake.spin(reverse, 12, volt);
  /*pid_drive(-10, 3);
  wait(400, msec);
  pid_drive(2, -3);
  pid_drive(-4, 3);
  wait(400, msec);
  pid_drive(-12, -3); */
  pid_drive(-24, -8);
  Shooter.spin(forward, 8, volt);
  jerk();
  pid_turn_by(190);
  wait(500, msec);
  VisionPid(180, Vision4__GOAL_RED); // fifth shot  
  LaunchShootCustom(9, 9.5, 500);

  return;

/*

  return;
  pid_turn_by(180);
  distance_pid_drive(30);
  pid_turn_by(92);



  Intake.stop();
  driveBackward(15, 100); //18
  pid_drive(3);
  Intake.spin(reverse, 12, volt); // intake on
  pid_drive(-5, 3);
  wait(400, msec);
  pid_drive(2, -3);
  pid_drive(-6, 3);
  wait(400, msec);
  pid_drive(2, -3);
  pid_drive(-9, 3);
  pid_drive(-2, 3);
  jerk();
  Shooter.spin(forward, 9, volt);
  pid_turn_by(190);
  wait(500, msec);
  VisionPid(185, Vision4__GOAL_RED);
  LaunchShootCustom(9.5, 10, 500); // second shot



  pid_turn_by(-180);
  Intake.stop();
  driveBackward(8, 80); //20
  pid_drive(3);
  Intake.spin(reverse, 12, volt); // intake on
  pid_drive(-6, 3);
  wait(400, msec);
  pid_drive(-5, 3);
  wait(400, msec);
  pid_drive(-10, 3);
  jerk();
  Shooter.spin(forward, 8, volt);
  pid_turn_by(190);
  wait(500, msec);
  VisionPid(185, Vision4__GOAL_RED);
  LaunchShootCustom(8.5, 9, 500); // third shot

*/


  pid_drive(10);
  pid_turn_by(35);
  pid_drive(-42, 12);
  pid_turn_by(90);
  pid_drive(-15);
  Shooter.spin(forward, 7.5, volt);
  pid_turn_by(90);
  VisionPid(185, Vision4__GOAL_BLUE); // sixth shot
  LaunchShootCustom(8.5, 9, 500);


  pid_turn_by(7);


  Intake.spin(reverse, 12, volt);
  pid_drive(-37);
  pid_turn_by(-90);
  pid_drive(-60);
  pid_turn_by(-45);
  distance_pid_drive(72);
  Shooter.spin(forward, 9, volt);
  pid_drive(-20);
  VisionPid(185, Vision4__GOAL_RED);
  LaunchShootCustom(10.25, 11, 500); // seventh shot
  return;



  pid_drive(-25);
  pid_turn_by(90);
  RollerWhole(15, 1000);
  Intake.stop();
  pid_drive(15);
  pid_turn_by(15);
  pid_drive(20, 30);
  pid_drive(-5);
  pid_turn_by(180);
  Intake.spin(reverse, 12, volt);
  pid_drive(-10, 3);
  wait(400, msec);
  pid_drive(2, -3);
  pid_drive(-6, 3);
  wait(400, msec);
  pid_drive(15, -3);
  Shooter.spin(forward, 8.5, volt);
  jerk();
  pid_turn_by(175);
  VisionPid(185, Vision4__GOAL_BLUE);
  LaunchShootCustom(9.5, 10, 500); // sixth shot



  pid_turn_by(25);
  Intake.stop();
  pid_drive(-30, 30);
  pid_drive(5);
  Intake.spin(reverse, 12, volt);
  pid_drive(-10, 3);
  wait(400, msec);
  pid_drive(2, -3);
  pid_drive(-6, 3);
  wait(400, msec);
  pid_drive(15, -3);
  Shooter.spin(forward, 9.75, volt);
  jerk();
  pid_turn_by(-15);
  VisionPid(185, Vision4__GOAL_BLUE); // sevent shot
  LaunchShootCustom(10.5, 11, 500);
  pid_turn_by(-15);
  pid_drive(-40);
  pid_turn_by(-90);
  RollerWhole(40, 3000);
  Intake.stop();
  pid_drive(28);
  pid_turn_by(45);
  extShoot();

  return;



  Shooter.spin(reverse, 7, volt);
  pid_drive(50);
  pid_turn_by(-45);
  LaunchShootCustom(7, 7, 300); // fourth shot
  return;
  pid_drive(-5);
  Intake.spin(reverse, 11.5, volt);
  pid_turn_by(-90);
  wait(500, msec);
  pid_drive(-23); //-20 
  pid_turn_by(-48);
  pid_drive(-33); //THIS IS PICKING UP TOO FAST - GETS STUCK
  Shooter.spin(reverse, 7, volt);
  pid_turn_by(84); //83
  Intake.stop();
  driveForward(9, 50, 1000);
  pid_turn_by(-1);
  //wait(200, msec);
  LaunchShootFar(); //second shot
  pid_turn_by(1);
  Shooter.stop();
  pid_drive(-10);
  pid_turn_by(-86);//-90
  driveBackward(35, 80);
  Intake.spin(reverse, 11.5, volt);
  pid_drive(-3);
  wait(100, msec);
  pid_drive(-4);
  wait(100, msec);
  pid_turn_by(-2); //we added this
  pid_drive(-8);


  //pid_drive(-10); //orignally -5
  //pid_turn_by(-5);
  //pid_drive(-5);
  Shooter.spin(reverse, 7, volt);
  pid_turn_by(48); //was 49
  pid_drive(10);
  distance_pid_drive(72);
  Intake.stop();
  wait(500, msec);
  LaunchShootFar(); // third shot
  /*pid_drive(-20);
  pid_turn_by(30);
  Intake.spin(reverse, 100, percent);
  pid_turn_by(30);
  pid_drive(30);
  Intake.stop();
  pid_turn_by(-30);
  pid_drive(-30);
  pid_turn_by(-20);
  Shooter.spin(reverse, 7, volt);
  pid_drive(30);
  LaunchShootFar(); */
  Intake.spin(reverse, 11.5, volt);
  pid_drive(-36);
  pid_turn_by(-90);
  Intake.stop();
  RollerWhole(20, 2000);
  Intake.spin(reverse, 11.5, volt);
  //wait(300, msec); //rollers done
  pid_drive(4.5); //goes away from rollers
  pid_turn_by(141); //135
  pid_drive(-20); //picks up disc //-20.5
  //wait(500, sec);
  pid_turn_by(-51); //-41
  Intake.stop();
  RollerWhole(12, 1500);
  pid_drive(14);
  pid_turn_by(-45);
  extShoot();
  pid_drive(-10);
  return;

  pid_drive(37); //drives toward goal
  pid_turn_by(-90); //turns to wall
  driveForward(14, 80, 800); //drives shooter side into wall
  imu.calibrate(); //calibrates
  while (imu.isCalibrating()) {
    wait(25, msec);
  }
  pid_drive(-5); //goes back
  pid_turn_by(87); //turns to shoot
  //pid_turn_by(-6); //turns 
  distance_pid_drive(52); //drives closer to goal to shoot 
  //pid_turn_by(-1);
  LaunchShoot(); //shoots first 3 discs
  Shooter.stop();
  //pid_turn_by(1);
  //below this is test code for calibrating after second shoot
  /*pid_drive(-35);
  pid_turn_by(-90);
  driveForward(14, 60, 1000);
  imu.calibrate(); //calibrates
  while (imu.isCalibrating()) {
    wait(25, msec);
  }
  pid_drive(-17); //-15
  Intake.spin(reverse, 100, percent);
  pid_turn_by(-42);
  pid_drive(-6);
  pid_turn_by(-6); //corrects for angled shooting */
  pid_drive(-43);
  pid_turn_by(-135); //turns to pick up 3 in a row discs
  Intake.spin(reverse,100, percent);
  pid_drive(-46); //picks up discs
  Shooter.spin(reverse, 7 , volt);
  pid_drive(-25); //still picking up discs/driving to position
  Intake.stop();
  pid_turn_by(140); //turn straight
  pid_drive(20);
  driveForward(30, 80, 1000); //drives shooter side into wall
  imu.calibrate(); //calibrates
  while (imu.isCalibrating()) {
    wait(25, msec);
  }
  pid_drive(-6.5); //goes back
  pid_turn_by(-90);
  //pid_drive(15, 3000);
  distance_pid_drive(52);
  pid_turn_by(-16);
  LaunchShoot();
  pid_turn_by(16);
  pid_drive(-51);
  pid_turn_by(-90);
  pid_drive(-10, 1000);
  Intake.spin(reverse, 11.5, volt);
  wait(300, msec);
  pid_drive(10);
  Intake.stop();
  pid_turn_by(135);
  pid_drive(-15);
  pid_turn_by(-47);
  pid_drive(-24, 2000);
  Intake.spin(reverse, 11.5, volt);
  wait(300, msec);
  Intake.stop();
  pid_drive(16);
  pid_turn_by(-45);
  pid_drive(-6);
  extShoot();
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

double abs_angle = 0;

long pid_turn_by (double angle) 
{
  abs_angle += angle;
  return pid_turn(abs_angle);
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

//double drive_kp = 15; //4.5 //3.2, then recently 3.5
double drive_ki = 0.0015;
double drive_kd = 0.09;
double drive_tolerance = 0.1;    // we want to stop when we reach the desired angle +/- 1 degree

long pid_drive(double distance, double drive_kp, int32_t time, double space) {
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
  //drive_kp = 3;
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
    //Shooter.stop();
    //shootspin = false;
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
//    Intake.stop();
//    spin2 = false;
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
    Shooter.spin(forward, 6.25, volt); //6.75
//    Shooter.spin(forward, 50, percent);
//    Intake.stop();
//    spin2 = false;
    shootspin = true;    
  }
  else {
    Shooter.stop();
    shootspin = false;
  }

}

void ShootOnce(void) {
  Shooter_pneum.set(true);
  wait(80, msec);
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

double vision_kp = 0.05; //1.5
double vision_ki = 0.00004; //0.0009
double vision_kd = 0;
double vision_tolerance = 1;    // we want to stop when we reach the desired angle +/- 1 degree

uint32_t VisionPid(int GoalX, signature ColorSig) {
  vision_in_prog = true;
  double delay = 50;   // imu can output reading at a rate of 50 hz (20 msec)
  long loop_count = 0;
  double error = 5000;
  double total_error = 0;
  double derivative = 0;
  double prev_error = 0;
  double voltage = 0;
  uint32_t ObjNum = Vision4.takeSnapshot(ColorSig);
  bool direction = true;
  printf("ObjNum %lu\n", ObjNum);
  vision::object LObject = Vision4.largestObject;
  error = GoalX - LObject.centerX;
  if (error > 0) {
    direction = false;
  } else {
    error = error * -1;
    direction = true;
  }
  double min_volt = 2.5;   // we don't want to apply less than min_volt, or else drivetrain won't move
//  double max_volt = 11.5;  // we don't want to apply more than max volt, or else we may damage motor
  double max_volt = 6.5;  // we don't want to apply more than max volt, or else we may damage motor
  DEBUG_PRINT(PRINT_LEVEL_NORMAL, "Turn to Goalx %d, current x %d, error %.2f\n", GoalX, LObject.centerX, error);
  // keep turning until we reach desired angle +/- tolerance
  while (error > vision_tolerance) {
    total_error += error;   // used for integration term
    derivative = error - prev_error;
    voltage = vision_kp * error + vision_ki * total_error - vision_kd * derivative;
    if (voltage < min_volt) {
        voltage = min_volt;
      } else if (voltage > max_volt) {
      voltage = max_volt;
    }
    if (direction) {
      RightDriveSmart.spin(reverse, voltage, volt);
      LeftDriveSmart.spin(forward, voltage, volt);
    } else {
      RightDriveSmart.spin(forward, voltage, volt);
      LeftDriveSmart.spin(reverse, voltage, volt);
    }
    prev_error = error;
    wait(delay, msec);
    ObjNum = Vision4.takeSnapshot(ColorSig);
    error = GoalX - LObject.centerX;
    if (error > 0) {
      direction = false;
    } else {
      error = error * -1;
      direction = true;
    }
    DEBUG_PRINT(PRINT_LEVEL_DEBUG, "error %.2f, voltage %.2f, direction %d, num %lu\n", error, voltage, direction, ObjNum);
    ++loop_count;
  }
  RightDriveSmart.stop();
  LeftDriveSmart.stop();
  ObjNum = Vision4.takeSnapshot(Vision4__GOAL_RED);
  DEBUG_PRINT(PRINT_LEVEL_DEBUG, "turn to goal %lu, current x %d, loop count %ld\n", GoalX, LObject.centerX, loop_count);
  vision_in_prog = false;
  return loop_count;
}

double vd_kp = 0.05; //1.5
double vd_ki = 0.00004; //0.0009
double vd_kd = 0;
double vd_tolerance = 1;    // we want to stop when we reach the desired angle +/- 1 degree

uint32_t VisionDrive(int GoalW, signature GoalSig) {
  double delay = 50;   // imu can output reading at a rate of 50 hz (20 msec)
  long loop_count = 0;
  double error = 5000;
  double total_error = 0;
  double derivative = 0;
  double prev_error = 0;
  double voltage = 0;
  uint32_t ObjNum = Vision4.takeSnapshot(GoalSig);
  bool direction = true;
  printf("ObjNum %lu\n", ObjNum);
  vision::object LObject = Vision4.largestObject;
  error = GoalW - LObject.width;
  if (error > 0) {
    direction = false;
  } else {
    error = error * -1;
    direction = true;
  }
  double min_volt = 2.5;   // we don't want to apply less than min_volt, or else drivetrain won't move
//  double max_volt = 11.5;  // we don't want to apply more than max volt, or else we may damage motor
  double max_volt = 6.5;  // we don't want to apply more than max volt, or else we may damage motor
  DEBUG_PRINT(PRINT_LEVEL_NORMAL, "Turn to Goalw %d, current width %d, error %.2f\n", GoalW, LObject.width, error);
  // keep turning until we reach desired angle +/- tolerance
  while (error > vd_tolerance) {
    total_error += error;   // used for integration term
    derivative = error - prev_error;
    voltage = vd_kp * error + vd_ki * total_error - vd_kd * derivative;
    if (voltage < min_volt) {
        voltage = min_volt;
      } else if (voltage > max_volt) {
      voltage = max_volt;
    }
    if (direction) {
      RightDriveSmart.spin(reverse, voltage, volt);
      LeftDriveSmart.spin(reverse, voltage, volt);
    } else {
      RightDriveSmart.spin(forward, voltage, volt);
      LeftDriveSmart.spin(forward, voltage, volt);
    }
    prev_error = error;
    wait(delay, msec);
    ObjNum = Vision4.takeSnapshot(Vision4__GOAL_RED);
    error = GoalW - LObject.width;
    if (error > 0) {
      direction = false;
    } else {
      error = error * -1;
      direction = true;
    }
    DEBUG_PRINT(PRINT_LEVEL_DEBUG, "error %.2f, voltage %.2f, direction %d, num %lu\n", error, voltage, direction, ObjNum);
    ++loop_count;
  }
  RightDriveSmart.stop();
  LeftDriveSmart.stop();
  ObjNum = Vision4.takeSnapshot(Vision4__GOAL_RED);
  DEBUG_PRINT(PRINT_LEVEL_DEBUG, "turn to goal %d, current w %d, loop count %ld\n", GoalW, LObject.width, loop_count);
  return loop_count;
}

void VisionAlignShoot(signature sig, int GoalX) {
  VisionPid(GoalX, sig);
  vision::object LObject = Vision4.largestObject;
  int width = LObject.width;
  double volt = (width * -.032258) + 9.23871;
  CustomLaunch(volt, 10, 10, 2000, 350, 350);
}

double DefaultV = 6.75;
void CustomLaunch (double v1, double v2, double v3, double t1, double t2, double t3) {
  Shooter.spin(forward, v1, volt);
  wait(t1, msec);
  Shooter_pneum.set(true);
  wait(100, msec);
  Shooter.spin(forward, v2, volt);
  Shooter_pneum.set(false);
  wait(t2, msec);
  Shooter_pneum.set(true);
  wait(100, msec);
  Shooter.spin(forward, v3, volt);
  Shooter_pneum.set(false);
  wait(t3, msec);
  Shooter_pneum.set(true);
  wait(100, msec);
  Shooter_pneum.set(false);
  Shooter.spin(forward, DefaultV, volt);
}

void VisionAlign(void) {
  if (DebounceTimer.value() < 0.1) {
    return;
  }
  DebounceTimer.reset();
  vision_in_prog = true;
  VisionPid(190, Vision4__GOAL_BLUE);
  vision_in_prog = false;
  return;
  VisionPid(190, Vision4__GOAL_RED);
  VisionDrive(55, Vision4__GOAL_RED); //7volts = 88width
  VisionPid(190, Vision4__GOAL_RED);
  vision_in_prog = false;
}

void VisionAlignRed(void) {
  VisionPid(185, Vision4__GOAL_RED);
}

void VisionAlignBlue(void) {
  VisionPid(185, Vision4__GOAL_BLUE);
}

void usercontrol(void) {
  // User control code here, inside the loop
  Shooter.spin(forward, DefaultV, volt);

  double turnImportance = 1;
  double speed_ratio = (11.0 / 5.0);

    Controller.ButtonL1.pressed(SpinIntakeForwards);
    Controller.ButtonL2.pressed(SpinIntakeBackwards);
    Controller.ButtonR1.pressed(SpinShooter);
   // Controller.ButtonB.pressed(LeftDriveSmart);
    //Controller.ButtonX.pressed(RightDriveSmart);

    Controller.ButtonR2.pressed(LaunchShoot);
    Controller.ButtonY.pressed(extShoot);

    //Controller.ButtonX.pressed(RollerSpinForwards);
    //Controller.ButtonB.pressed(RollerSpinBackwards);

    Controller.ButtonX.pressed(LaunchShootMedium);

    Controller.ButtonUp.pressed(ShootOnce);
    //Controller.ButtonDown.pressed(ShooterReverse);
    Controller.ButtonA.pressed(RollerAutoDrive);
    Controller.ButtonLeft.pressed(VisionAlignRed);
    Controller.ButtonRight.pressed(VisionAlignBlue);

  //tune_turn_pid();
  while (1) {

    double turnVal = Controller.Axis3.position(percent);
    double forwardVal = Controller.Axis1.position(percent);

    
    double turnVolts = turnVal * -0.12;
    double forwardVolts = forwardVal * 0.12 * (1 - (abs(turnVolts)/12.0) * turnImportance);

    // 12 - 12 = 0
    // 12 + 12 = 12(due to cap)


    if (vision_in_prog == false) {
      RightDriveSmart.spin(reverse, speed_ratio * (forwardVolts + turnVolts), voltageUnits::volt);
      LeftDriveSmart.spin(forward, speed_ratio * (forwardVolts - turnVolts), voltageUnits::volt);
    }

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
