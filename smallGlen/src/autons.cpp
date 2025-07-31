#include "main.h"

//#include "lem.hpp"

/////
// For instalattion, upgrading, documentations and tutorials, check out website!
// https://ez-robotics.github.io/EZ-Template/
/////


const int DRIVE_SPEED = 127; // This is 110/127 (around 87% of max speed).  We don't suggest making this 127.
                             // If this is 127 and the robot tries to heading correct, it's only correcting by
                             // making one side slower.  When this is 87%, it's correcting by making one side
                             // faster and one side slower, giving better heading correction.
const int TURN_SPEED  = 90;
const int SWING_SPEED = 90;



/*

std::shared_ptr<okapi::OdomChassisController> odomChassis =
  okapi::ChassisControllerBuilder()
    .withMotors( 
      {-1, 2, -13},             // Left-Side Motors
      {8, -9, 19}               // Right-Side Motors
    )
    .withGains(
        {0.001, 0, 0.0001}, // Distance controller gains
        {0.001, 0, 0.0001}, // Turn controller gains
        {0.001, 0, 0.0001}  // Angle controller gains (helps drive straight)
    )
    
    //.withSensors(okapi::IMU{'A'})
    .withDimensions(okapi::AbstractMotor::gearset::green, {{4, 11.5}, okapi::imev5GreenTPR})
    .withOdometry() // use the same scales as the chassis (above)
    .buildOdometry(); // build an odometry chassis

    */
    
   /*
   auto chassis = okapi::ChassisControllerBuilder()
	.withMotors({1, 2, 3}, {4, 5, 6}) // left motor, right motor, negative if reversed
	.withDimensions(okapi::AbstractMotor::gearset::green, {{3.25_in, 12_in}, imev5GreenTPR}), //gearset, wheel size, track diameter, motor tpr
	//.withSensors(ADIEncoder{'A', 'B'}, ADIEncoder{'C', 'D'}, ADIEncoder{'E', 'F'}) // left encoder, right encoder, middle encoder
	//.withOdometry({{2.75_in, 8_in, 4_in, 2.75_in}, quadEncoderTPR}, StateMode::CARTESIAN) // tracking wheel size, odom track diam, dist to middle wheel, middle tracking wheel diam, state mode
    .withLogger(std::make_shared<Logger>(TimeUtilFactory::createDefault().getTimer(), "/ser/sout", Logger::LogLevel::debug)
    .buildOdometry();
    */


// std::shared_ptr<ChassisController> chassis =
//   ChassisControllerBuilder()
//     .withMotors(1, -2)
//     // Green gearset, 4 in wheel diam, 11.5 in wheel track
//     .withDimensions(AbstractMotor::gearset::green, {{4_in, 11.5_in}, imev5GreenTPR})
//     .build();





///
// Constants
///

// It's best practice to tune constants when the robot is empty and with heavier game objects, or with lifts up vs down.
// If the objects are light or the cog doesn't change much, then there isn't a concern here.

void default_constants() {
  chassis.set_slew_min_power(80, 80);
  chassis.set_slew_distance(7, 7);
  chassis.set_pid_constants(&chassis.headingPID, 11, 0, 30, 0);
  chassis.set_pid_constants(&chassis.forward_drivePID, 0.45, 0, 5, 0);
  chassis.set_pid_constants(&chassis.backward_drivePID, 0.45, 0, 5, 0);
  chassis.set_pid_constants(&chassis.turnPID, 5, 0.003, 35, 15);
  chassis.set_pid_constants(&chassis.swingPID, 7, 0, 45, 0);
}

void one_mogo_constants() {
  chassis.set_slew_min_power(80, 80);
  chassis.set_slew_distance(7, 7);
  chassis.set_pid_constants(&chassis.headingPID, 11, 0, 20, 0);
  chassis.set_pid_constants(&chassis.forward_drivePID, 0.45, 0, 5, 0);
  chassis.set_pid_constants(&chassis.backward_drivePID, 0.45, 0, 5, 0);
  chassis.set_pid_constants(&chassis.turnPID, 5, 0.003, 35, 15);
  chassis.set_pid_constants(&chassis.swingPID, 7, 0, 45, 0);
}

void two_mogo_constants() {
  chassis.set_slew_min_power(80, 80);
  chassis.set_slew_distance(7, 7);
  chassis.set_pid_constants(&chassis.headingPID, 11, 0, 20, 0);
  chassis.set_pid_constants(&chassis.forward_drivePID, 0.45, 0, 5, 0);
  chassis.set_pid_constants(&chassis.backward_drivePID, 0.45, 0, 5, 0);
  chassis.set_pid_constants(&chassis.turnPID, 5, 0.003, 35, 15);
  chassis.set_pid_constants(&chassis.swingPID, 7, 0, 45, 0);
}

void exit_condition_defaults() {
  chassis.set_exit_condition(chassis.turn_exit, 100, 2, 500, 10, 500, 500);
  chassis.set_exit_condition(chassis.swing_exit, 100, 3, 500, 7, 500, 500);
  chassis.set_exit_condition(chassis.drive_exit, 80, 30, 300, 220, 500, 500);
}

void modified_exit_condition() {
  chassis.set_exit_condition(chassis.turn_exit, 100, 3, 500, 7, 500, 500);
  chassis.set_exit_condition(chassis.swing_exit, 100, 3, 500, 7, 500, 500);
  chassis.set_exit_condition(chassis.drive_exit, 80, 50, 300, 150, 500, 500);
}



///
// Drive Example
///
void drive_example() {
  // The first parameter is target inches
  // The second parameter is max speed the robot will drive at
  // The third parameter is a boolean (true or false) for enabling/disabling a slew at the start of drive motions
  // for slew, only enable it when the drive distance is greater then the slew distance + a few inches


  chassis.set_drive_pid(24, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_drive_pid(-12, DRIVE_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(-12, DRIVE_SPEED);
  chassis.wait_drive();
}



///
// Turn Example
///
void turn_example() {
  // The first parameter is target degrees
  // The second parameter is max speed the robot will drive at


  chassis.set_turn_pid(90, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_turn_pid(45, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_turn_pid(0, TURN_SPEED);
  chassis.wait_drive();
}



///
// Combining Turn + Drive
///
void drive_and_turn() {
  chassis.set_drive_pid(24, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_turn_pid(45, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_turn_pid(-45, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_turn_pid(0, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(-24, DRIVE_SPEED, true);
  chassis.wait_drive();
}



///
// Wait Until and Changing Max Speed
///
void wait_until_change_speed() {
  // wait_until will wait until the robot gets to a desired position


  // When the robot gets to 6 inches, the robot will travel the remaining distance at a max speed of 40
  chassis.set_drive_pid(24, DRIVE_SPEED, true);
  chassis.wait_until(6);
  chassis.set_max_speed(40); // After driving 6 inches at DRIVE_SPEED, the robot will go the remaining distance at 40 speed
  chassis.wait_drive();

  chassis.set_turn_pid(45, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_turn_pid(-45, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_turn_pid(0, TURN_SPEED);
  chassis.wait_drive();

  // When the robot gets to -6 inches, the robot will travel the remaining distance at a max speed of 40
  chassis.set_drive_pid(-24, DRIVE_SPEED, true);
  chassis.wait_until(-6);
  chassis.set_max_speed(40); // After driving 6 inches at DRIVE_SPEED, the robot will go the remaining distance at 40 speed
  chassis.wait_drive();
}



///
// Swing Example
///
void swing_example() {
  // The first parameter is ez::LEFT_SWING or ez::RIGHT_SWING
  // The second parameter is target degrees
  // The third parameter is speed of the moving side of the drive


  chassis.set_swing_pid(ez::LEFT_SWING, 45, SWING_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(24, DRIVE_SPEED, true);
  chassis.wait_until(12);

  chassis.set_swing_pid(ez::RIGHT_SWING, 0, SWING_SPEED);
  chassis.wait_drive();
}



///
// Auto that tests everything
///
void combining_movements() {
  chassis.set_drive_pid(24, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_turn_pid(45, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_swing_pid(ez::RIGHT_SWING, -45, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_turn_pid(0, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(-24, DRIVE_SPEED, true);
  chassis.wait_drive();
}



///
// Interference example
///
void tug (int attempts) {
  for (int i=0; i<attempts-1; i++) {
    // Attempt to drive backwards
    printf("i - %i", i);
    chassis.set_drive_pid(-12, 127);
    chassis.wait_drive();

    // If failsafed...
    if (chassis.interfered) {
      chassis.reset_drive_sensor();
      chassis.set_drive_pid(-2, 20);
      pros::delay(1000);
    }
    // If robot successfully drove back, return
    else {
      return;
    }
  }
}

// If there is no interference, robot will drive forward and turn 90 degrees. 
// If interfered, robot will drive forward and then attempt to drive backwards. 
void interfered_example() {
 chassis.set_drive_pid(24, DRIVE_SPEED, true);
 chassis.wait_drive();

 if (chassis.interfered) {
   tug(3);
   return;
 }

 chassis.set_turn_pid(90, TURN_SPEED);
 chassis.wait_drive();
}
/*
void odomTest(int motor1, int motor2){
  std::shared_ptr<OdomChassisController> chassis1 =
  ChassisControllerBuilder()
  .withMotors(motor1, motor2)
  .withDimensions(AbstractMotor::gearset::green, {{4_in, 11.5_in}, imev5GreenTPR})
  .withSensors(ADIEncoder{'A', 'B'}, ADIEncoder{'C', 'D', true})
  // specify the tracking wheels diameter (2.75 in), track (7 in), and TPR (360)
  .withOdometry({{2.75_in, 7_in}, quadEncoderTPR}, StateMode::FRAME_TRANSFORMATION)
  .buildOdometry();
}

*/

const int positionMultiplier = 2.5;


void extendIntake(){
  // extendRight.move_absolute(130 * positionMultiplier, 127);
  // extendLeft.move_absolute(130 * positionMultiplier, 127);
  // pros::delay(900);
}

void retractIntake(){
  // extendRight.move_absolute(25 * positionMultiplier, 127);
  // extendLeft.move_absolute(25 * positionMultiplier, 127);
  // pros::delay(400);
  // intake.brake();
  // intake2.brake();
  // pros::delay(200);
  // //pros::delay(500);
}

bool autoCat = true;
bool shootCata = false;

void keepCatapultLoaded(void* param){
  // while(true){
  //   while(autoCat){
  //     if(cataLimit.get_value()){
  //       if(shootCata){
  //         catapult.move(127);
  //         catapult2.move(127);
  //         pros::delay(100);
  //       }else{
  //         catapult.move(8);
  //         catapult2.move(8);
          
  //       }
  //     }else{
  //       catapult.move(127);
  //       catapult2.move(127);
  //     }
  //     pros::delay(10);
  //   }
  //   pros::delay(10);
  // }
}


void testAuto(){
  // pros::Task cataLoaded(keepCatapultLoaded, (void*)"shoot", "Shoot");
  // autoCat = true;


  // intake.move(127);
  // pros::delay(700); 

  // extendIntake();
  // retractIntake();
  // pros::delay(500);


  // shootCata = true;
  // pros::delay(200);
  // shootCata = false;
}

void programmingSkillz(){
  // pros::Task cataLoaded(keepCatapultLoaded, (void*)"shoot", "Shoot");
  // autoCat = true;

  // intake.move(127);
  // intake2.move(127);

  // for(unsigned i = 0; i < 22; ++i){
    
  //   extendIntake();
  //   extendRight.move_absolute(25 * positionMultiplier, 127);
  //   extendLeft.move_absolute(25 * positionMultiplier, 127);
  //   pros::delay(1350);

  //   shootCata = true;
  //   pros::delay(200);
  //   shootCata = false;
  //   pros::delay(500);
  // }
}

void autoTest(){
/*
  pros::Task cataLoaded(keepCatapultLoaded, (void*)"shoot", "Shoot");
  autoCat = true;
  
  // GRAB THE BALL IN THE CORNER
  pros::delay(500);
  shootCata = true;
  pros::delay(200);
  shootCata = false;
  pros::delay(400);
  
  intake.move(127);
  intake2.move(127);

  extendIntake();

  
  retractIntake(); 
  

  
  // TURN TOWARDS THE GOAL

  chassis.set_drive_pid(-8, DRIVE_SPEED);
  chassis.wait_drive();
  //chassis.wait_until(10);

  chassis.set_turn_pid(115, TURN_SPEED);
  chassis.wait_drive();
  


  // DROP THE BALL AND DRIVE FORWARD TO SCORE

  

  chassis.set_drive_pid(28, DRIVE_SPEED);
  //chassis.wait_drive();
  chassis.wait_until(2);
  intake.move(-127);
  intake2.move(-127);
  chassis.wait_drive();
  extendRight.move_absolute(0 * positionMultiplier, 127);
  extendLeft.move_absolute(0 * positionMultiplier, 127);

  extendRight.brake();
  extendLeft.brake();
  

  //chassis.set_drive_pid(-10, DRIVE_SPEED, true);
  //chassis.wait_drive();
  

  //chassis.set_turn_pid(-45, TURN_SPEED);
  //chassis.wait_drive();

  //chassis.set_drive_pid(15, DRIVE_SPEED, true);
  //chassis.wait_drive();

  

  
  // BACK UP A BIT AND TURN TOWARDS THE MIDDLE

  intake.brake();
  intake2.brake();
  
  chassis.set_drive_pid(-10, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_turn_pid(20, TURN_SPEED);
  chassis.wait_drive();

  extendRight.move_absolute(25 * positionMultiplier, 127);
  extendLeft.move_absolute(25 * positionMultiplier, 127);
  


  // DRIVE TOWARDS MIDDLE AND TURN TOWARDS CENTER

  chassis.set_drive_pid(-24, DRIVE_SPEED);
  chassis.wait_drive();

  chassis.set_turn_pid(120, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(20, DRIVE_SPEED);
  chassis.wait_drive();
  
  chassis.set_turn_pid(45, TURN_SPEED);
  chassis.wait_drive();

  // PUSH TRIBALLS OVER THE MIDDLE BORDER

  chassis.set_drive_pid(-30, DRIVE_SPEED);
  chassis.wait_drive();

  // MOVE BACK A BIT AND SWING TURN TO GET CONTESTED TRIBALL

  chassis.set_drive_pid(9, DRIVE_SPEED);
  chassis.wait_drive();

  intake.move(127);
  intake2.move(127);

  chassis.set_swing_pid(ez::LEFT_SWING, 120, SWING_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(3, DRIVE_SPEED);
  chassis.wait_drive();

  pros::delay(300); // GIVE INTAKE A BIT OF TIME TO COOK

  // BACK UP FROM THE LINE AND TURN  TOWARDS THE LOADING ZONE

  chassis.set_drive_pid(-13, DRIVE_SPEED);
  chassis.wait_drive();

  chassis.set_turn_pid(0, TURN_SPEED);
  chassis.wait_drive();

  intake.brake();
  intake2.brake();

  // DRIVE TOWARDS THE LOADING ZONE AND SHOOT THE CATAPULT MID DRIVE

  chassis.set_drive_pid(53, DRIVE_SPEED);
  chassis.wait_until(20);
  shootCata = true;
  pros::delay(200);
  shootCata = false;
  chassis.wait_drive();


  intake.move(127);
  intake2.move(127);

  for(unsigned i = 0; i < 10; ++i){
    
    extendIntake();
    extendRight.move_absolute(25 * positionMultiplier, 127);
    extendLeft.move_absolute(25 * positionMultiplier, 127);
    pros::delay(1350);

    shootCata = true;
    pros::delay(200);
    shootCata = false;
    pros::delay(500);
  }
  
  
*/
  /*
  */
}


