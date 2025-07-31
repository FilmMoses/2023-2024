//#include "lem.cpp"
#include "main.h"

const int MOTOR_MAX_VOLTAGE = 127;
const double positionMultiplier = 2.5;
//int MOTOR_MAX_VOLTAGE = 127;


// lemlib::Drivetrain_t drivetrain {
//     &leftSideMotors, // left drivetrain motors
//     &rightSideMotors, // right drivetrain motors
//     10, // track width
//     4, // wheel diameter
//     360 // wheel rpm
// };

// lemlib::OdomSensors_t sensors {
//   nullptr,
// 	nullptr,
// 	nullptr,
//   nullptr, // we don't have a second tracking wheel, so we set it to nullptr
//   &inertialSensor // inertial sensor
// };

// lemlib::ChassisController_t lateralController {
//     8, // kP
//     30, // kD
//     1, // smallErrorRange
//     100, // smallErrorTimeout
//     3, // largeErrorRange
//     500, // largeErrorTimeout
//     5 // slew rate
// };
 
// // turning PID
// lemlib::ChassisController_t angularController {
//     4, // kP
//     40, // kD
//     1, // smallErrorRange
//     100, // smallErrorTimeout
//     3, // largeErrorRange
//     500, // largeErrorTimeout
//     0 // slew rate
// };

//lemlib::Chassis lemChassis(drivetrain, lateralController, angularController, sensors);
// Chassis constructor
Drive chassis (
  // Left Chassis Ports (negative port will reverse it!)
  //   the first port is the sensored port (when trackers are not used!)
  //{7, 8, 9, 10}
  {7, -8, -9, -10}

  // Right Chassis Ports (negative port will reverse it!)
  //   the first port is the sensored port (when trackers are not used!)
  //,{13, 14, 15, 17}
  ,{-13, 14, 15, 17}

  // IMU Port
  ,20

  // Wheel Diameter (Remember, 4" wheels are actually 4.125!)
  //    (or tracking wheel diameter)
  ,2.5

  // Cartridge RPM
  //   (or tick per rotation if using tracking wheels)
  ,600

  // External Gear Ratio (MUST BE DECIMAL)
  //    (or gear ratio of tracking wheel)
  // eg. if your drive is 84:36 where the 36t is powered, your RATIO would be 2.333.
  // eg. if your drive is 36:60 where the 60t is powered, your RATIO would be 0.6.
  ,2


  // Uncomment if using tracking wheels
  /*
  // Left Tracking Wheel Ports (negative port will reverse it!)
  // ,{1, 2} // 3 wire encoder
  // ,8 // Rotation sensor

  // Right Tracking Wheel Ports (negative port will reverse it!)
  // ,{-3, -4} // 3 wire encoder
  // ,-9 // Rotation sensor
  */

  // Uncomment if tracking wheels are plugged into a 3 wire expander
  // 3 Wire Port Expander Smart Port
  // ,1
);



/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  // Print our branding over your terminal :D
  ez::print_ez_template();
  
  pros::delay(500); // Stop the user from doing anything while legacy ports configure.


  // Configure your chassis controls
  chassis.toggle_modify_curve_with_controller(true); // Enables modifying the controller curve with buttons on the joysticks
  chassis.set_active_brake(0); // Sets the active brake kP. We recommend 0.1.
  chassis.set_curve_default(0, 0); // Defaults for curve. If using tank, only the first parameter is used. (Comment this line out if you have an SD card!)  
  default_constants(); // Set the drive to your own constants from autons.cpp!

  // These are already defaulted to these buttons, but you can change the left/right curve buttons here!
  // chassis.set_left_curve_buttons (pros::E_CONTROLLER_DIGITAL_LEFT, pros::E_CONTROLLER_DIGITAL_RIGHT); // If using tank, only the left side is used. 
  // chassis.set_right_curve_buttons(pros::E_CONTROLLER_DIGITAL_Y,    pros::E_CONTROLLER_DIGITAL_A);

  // Autonomous Selector using LLEMU
  ez::as::auton_selector.add_autons({
    Auton("Example Drive\n\nDrive forward and come back.", drive_example),
    Auton("Example Turn\n\nTurn 3 times.", turn_example),
    Auton("Drive and Turn\n\nDrive forward, turn, come back. ", drive_and_turn),
    Auton("Drive and Turn\n\nSlow down during drive.", wait_until_change_speed),
    Auton("Swing Example\n\nSwing, drive, swing.", swing_example),
    Auton("Combine all 3 movements", combining_movements),
    Auton("Interference\n\nAfter driving forward, robot performs differently if interfered or not.", interfered_example),
  });

  // Initialize chassis and auton selector
  chassis.initialize();
  //lemChassis.calibrate();
  ez::as::initialize();
  // blocker.tare_position();
  // autonSweeper.tare_position();


  // WING CODE
  // leftWing.digitalWrite(LOW);
  // rightWing.digitalWrite(HIGH);
  leftWing.set_value(false);
  rightWing.set_value(false);


  // leftWing.pinMode('C', OUTPUT);
  // pros::digitalWrite(1, LOW);

}



/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
  // . . .
}



/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
  // . . .
}



/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
  chassis.reset_pid_targets(); // Resets PID targets to 0
  chassis.reset_gyro(); // Reset gyro position to 0
  chassis.reset_drive_sensor(); // Reset drive sensors to 0
  //chassis.set_drive_brake(MOTOR_BRAKE_HOLD); // Set motors to hold.  This helps autonomous consistency.       //aaaaaaaaaaaaaaa

  //ez::as::auton_selector.call_selected_auton(); // Calls selected auton from autonomous selector.
  autonTest();
}



/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void blockerTask(void* param){
  
}

void opcontrol() {
  
  int leftWingCounter = 10;
  int rightWingCounter = 10;
  bool leftWingExtended = false;
  bool rightWingExtended = false;


  // This is preference to what you like to drive on.
  //chassis.set_drive_brake(MOTOR_BRAKE_COAST);               //aaaaaaaaaaaaa

  while (true) {

    chassis.tank(); // Tank control
    // chassis.arcade_standard(ez::SPLIT); // Standard split arcade
    // chassis.arcade_standard(ez::SINGLE); // Standard single arcade
    // chassis.arcade_flipped(ez::SPLIT); // Flipped split arcade
    // chassis.arcade_flipped(ez::SINGLE); // Flipped single arcade

    // . . .
    // Put more user control code here!
    // . . .


    // if(master.get_digital(DIGITAL_L2)){
    //   intake.move(MOTOR_MAX_VOLTAGE);
    // }else if(master.get_digital(DIGITAL_L1)){
    //   intake.move(-MOTOR_MAX_VOLTAGE);
    // }else{
    //   intake.brake();
    // }


    // if(master.get_digital(DIGITAL_R2)){
    //   blocker.move_absolute(0 * positionMultiplier, 127);
    // }else if(master.get_digital(DIGITAL_R1)){
    //   blocker.move_absolute(100 * 23, 127);
    // }
    

    // LIFT

    // if(master.get_digital(DIGITAL_UP)){
    //   lift.move(127);
    // }else if(master.get_digital(DIGITAL_DOWN)){
    //   lift.move(-127);
    // }else{
    //   lift.move(-20);
    // }
    if(master.get_digital(DIGITAL_A)){
      // flywheel.move(127);
      // flywheel2.move(127);
      flywheel.move_velocity(550);
      flywheel2.move_velocity(550);
    }
    if(master.get_digital(DIGITAL_B)){
      flywheel.move_velocity(0);
      flywheel2.move_velocity(0);
    }
    

    if(master.get_digital(DIGITAL_L2)){
      intake.move(127);
      intake2.move(127);
    }else if(master.get_digital(DIGITAL_R2)){
      intake.move(-127);
      intake2.move(-127);
    }else{
      intake.brake();
      intake2.brake();
    }
    // if(master.get_digital(DIGITAL_UP)){
    //   intake.move(127);
    //   intake2.move(127);
    // }
    // if(master.get_digital(DIGITAL_DOWN)){
    //   intake.move(-127);
    //   intake2.move(-127);
    // }
    // if(master.get_digital(DIGITAL_LEFT)){
    //   intake.brake();
    //   intake2.brake();
    // }

    // WING CODE

    // if(master.get_digital(DIGITAL_L1)){
    //   leftWing.set_value(true);
    // }

    if(master.get_digital(DIGITAL_R1) && (rightWingCounter >=  30)){
      rightWingExtended = !rightWingExtended;
      rightWingCounter = 0;
    }
    

    if(master.get_digital(DIGITAL_L1) && (leftWingCounter >= 30)){
      leftWingExtended = !leftWingExtended;
      leftWingCounter = 0;
    }

    if(rightWingExtended){
      rightWing.set_value(true);
    }
    if(!rightWingExtended){
      rightWing.set_value(false);
    }

    if(leftWingExtended){
      leftWing.set_value(true);
    }
    if(!leftWingExtended){
      leftWing.set_value(false);
    }
    rightWingCounter++;
    leftWingCounter++;

    if(master.get_digital(DIGITAL_UP)){
      booper.move(127);
    }else if(master.get_digital(DIGITAL_DOWN)){
      booper.move(-127);
    }else{
      booper.brake();
    }
    
    pros::delay(ez::util::DELAY_TIME); // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}
