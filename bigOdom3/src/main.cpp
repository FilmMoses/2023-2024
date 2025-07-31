#include "main.h"

#include "EZ-Template/drive/drive.hpp"
#include "autons.hpp"
#include "pros/rtos.hpp"


pros::controller_digital_e_t rightWingButton = pros::E_CONTROLLER_DIGITAL_R1;
pros::controller_digital_e_t leftWingButton = pros::E_CONTROLLER_DIGITAL_L1;

pros::controller_digital_e_t intakeButton = pros::E_CONTROLLER_DIGITAL_L2;
pros::controller_digital_e_t outtakeButton = pros::E_CONTROLLER_DIGITAL_R2;

pros::controller_digital_e_t liftUpButton = pros::E_CONTROLLER_DIGITAL_UP;
pros::controller_digital_e_t liftDownButton = pros::E_CONTROLLER_DIGITAL_DOWN;

pros::controller_digital_e_t leftMatchLoadButton = pros::E_CONTROLLER_DIGITAL_RIGHT;
pros::controller_digital_e_t rightMatchLoadButton = pros::E_CONTROLLER_DIGITAL_Y;




// Drive motors.  These are used in GUI and Template

// PLANETARY REAR MID FRONT
std::vector<int> left_motors = {-2, -3, -10, 9};
std::vector<int> right_motors = {11, -13, 12, 20};
// std::vector<int> left_motors = {-20, -19, 8, -10};
// std::vector<int> right_motors = {11, 12, -16, 15};

// Chassis constructor
ez::Drive chassis(
    left_motors,   // Left motors
    right_motors,  // Right motors
    19,            // IMU Port
    4.0,           // Wheel size
    400.0,         // Cart RPM
    1.0,           // Gear ratio
    11.75          // Drive width
);

// GUI Constructor
ez::GUI display(
    {{pros::Motor(left_motors[0]), "left 1"},
     {pros::Motor(left_motors[1]), "left 2"},
     {pros::Motor(right_motors[1]), "right 2"},
     {pros::Motor(right_motors[0]), "right 1"},
     {pros::Motor(left_motors[2]), "left 3"},
     {pros::Motor(left_motors[3]), "left 4"},
     {pros::Motor(right_motors[3]), "right 4"},
     {pros::Motor(right_motors[2]), "right 3"},
     {intake[0], "l inta"},
     {intake[1], "r inta"}},

    { {"double180", double180},
      {"processing", processingAuto},
      {"test", testAuto},
     {" offense 3 ball, touches bar ", offense3ball},
     {" offense 2 ball, touches bar ", offense2ball},
     {"offense 3 ball, doesnt touch bar", offense3ball_nobar},
     {"offense 2 ball, doesnt touch bar", offense2ball_nobar},
     {"defense 2 ball, grab closer ball, touches bar", defense2ball},
     {"defense 2 ball interrupt, touches bar", defense2ballinterupt_barrier},
     {"defense 2 ball, grab closer ball, doesnt touch bar", defense2ball_nobar},
     {"defense 2 ball interrupt, doesnt touch bar", defense2ballinterupt_barrier_nobar},
     {" drive forward and come back ", drive_example},  // ez-gui bug with "drive forward and come back" doesn't wiggle
     {"turn 3 times", turn_example},
     {"drive forward, turn, come back", drive_and_turn},
     {"slow down during drive", wait_until_change_speed},
     {"swing in 'S' curve", swing_example},
     {"combining all 3 movements", combining_movements},
     {"after driving forward, robot performs differently if interfered or not", interfered_example}});

// Initialize
void initialize() {
  // Print our branding over your terminal :D
  ez::ez_template_print();

  pros::delay(500);  // Stop the user from doing anything while legacy ports configure

  // Configure your chassis controls
  chassis.opcontrol_curve_buttons_toggle(true);  // Enables modifying the controller curve with buttons on the joysticks
  chassis.opcontrol_drive_activebrake_set(0);    // Sets the active brake kP. We recommend 0.1.
  chassis.opcontrol_curve_default_set(0, 0);     // Defaults for curve. If using tank, only the first parameter is used. (Comment this line out if you have an SD card!)
  default_constants();                           // Set the drive to your own constants from autons.cpp!

  // These are already defaulted to these buttons, but you can change the left/right curve buttons here!
  // chassis.opcontrol_curve_buttons_left_set (pros::E_CONTROLLER_DIGITAL_LEFT, pros::E_CONTROLLER_DIGITAL_RIGHT); // If using tank, only the left side is used.
  // chassis.opcontrol_curve_buttons_right_set(pros::E_CONTROLLER_DIGITAL_Y,    pros::E_CONTROLLER_DIGITAL_A);

  // Initialize chassis and auton selector
  chassis.initialize();
  ez::as::initialize();
  display.enable();
  master.rumble(".");

  // Initialize subsystems
  intake_init();
  hang.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  hang2.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
}

// Auton
void autonomous() {
  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();               // Reset drive sensors to 0
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Set motors to hold.  This helps autonomous consistency
  chassis.odom_pose_set({0, 0, 0});
  chassis.drive_odom_enable(true);
  chassis.SPACING = 0.5;
  pros::delay(10);

  ez::as::auton_selector.selected_auton_call();  // Calls selected auton from autonomous selector
  display.auton_call();

  // chassis.pid_odom_ptp_set({{12, 24}, fwd, 110});
  // chassis.pid_wait();

  // chassis.pid_odom_pp_set({{{0, 24}, fwd, 110},
  //                         {{24, 24}, fwd, 110}});
  // chassis.pid_wait();

  // chassis.pid_odom_injected_pp_set({{{0, 24}, fwd, 110},
  //                         {{24, 24}, fwd, 110}});
  // chassis.pid_wait();

  // chassis.pid_odom_smooth_pp_set({{{0, 24}, fwd, 110},
  //                         {{24, 24}, rev, 110}});
  chassis.pid_wait();
  /*double x = 12;
  double y = 24;
  int speed = 60;

  chassis.pid_odom_smooth_pp_set({{{0, y}, fwd, speed},
                                  {{x, y}, fwd, speed},
                                  {{x, 0}, fwd, speed},
                                  {{0, 0}, fwd, speed}},
                                 true);
  chassis.pid_wait();*/

  // chassis.pid_odom_injected_pp_set({{{0, 7}, fwd, 110}}, false);
  // chassis.pid_wait_until(4_in);
  // chassis.pid_wait();
}

void leftMatchLoad(){
  chassis.left_motors.at(0).move_relative(200, 127);
  chassis.left_motors.at(1).move_relative(200, 127);
  chassis.left_motors.at(2).move_relative(200, 127);
  chassis.left_motors.at(3).move_relative(200, 127);

  chassis.right_motors.at(0).move_relative(-200, 127);
  chassis.right_motors.at(1).move_relative(-200, 127);
  chassis.right_motors.at(2).move_relative(-200, 127);
  chassis.right_motors.at(3).move_relative(-200, 127);

  pros::delay(400);

  chassis.left_motors.at(0).move_relative(-200, 127);
  chassis.left_motors.at(1).move_relative(-200, 127);
  chassis.left_motors.at(2).move_relative(-200, 127);
  chassis.left_motors.at(3).move_relative(-200, 127);

  chassis.right_motors.at(0).move_relative(200, 127);
  chassis.right_motors.at(1).move_relative(200, 127);
  chassis.right_motors.at(2).move_relative(200, 127);
  chassis.right_motors.at(3).move_relative(200, 127);

  pros::delay(400);
}

void rightMatchLoad(){
  chassis.right_motors.at(0).move_relative(200, 127);
  chassis.right_motors.at(1).move_relative(200, 127);
  chassis.right_motors.at(2).move_relative(200, 127);
  chassis.right_motors.at(3).move_relative(200, 127);

  chassis.left_motors.at(0).move_relative(-200, 127);
  chassis.left_motors.at(1).move_relative(-200, 127);
  chassis.left_motors.at(2).move_relative(-200, 127);
  chassis.left_motors.at(3).move_relative(-200, 127);

  pros::delay(400);

  chassis.right_motors.at(0).move_relative(-200, 127);
  chassis.right_motors.at(1).move_relative(-200, 127);
  chassis.right_motors.at(2).move_relative(-200, 127);
  chassis.right_motors.at(3).move_relative(-200, 127);

  chassis.left_motors.at(0).move_relative(200, 127);
  chassis.left_motors.at(1).move_relative(200, 127);
  chassis.left_motors.at(2).move_relative(200, 127);
  chassis.left_motors.at(3).move_relative(200, 127);
  
  pros::delay(400);
}


// Opcontrol
void opcontrol() {
  // This is preference to what you like to drive on

  int leftWingCounter = 10;
  int rightWingCounter = 10;
  bool leftWingExtended = false;
  bool rightWingExtended = false;

  bool leftPivot = false;
  bool rightPivot = false;
  int leftPivotCounter = 10;
  int rightPivotCounter = 10;

  bool liftUp = false;
  bool climb = false; 

  int intakeAntiStopCount = 0;
  bool out = false;

  bool climbing = false;
  hang.tare_position();
  hang2.tare_position();

  chassis.drive_brake_set(MOTOR_BRAKE_COAST);
  chassis.drive_odom_enable(false);

  while (true) {
    // PID Tuner
    // After you find values that you're happy with, you'll have to set them in auton.cpp

    // if (!pros::competition::is_connected()) {
    //   // Enable / Disable PID Tuner
    //   //  When enabled:
    //   //  * use A and Y to increment / decrement the constants
    //   //  * use the arrow keys to navigate the constants
    //   if (master.get_digital_new_press(DIGITAL_X)) {
    //     chassis.pid_tuner_toggle();
    //     if (chassis.pid_tuner_enabled())
    //       pros::lcd::set_background_color(LV_COLOR_HEX(0xFFC0CB));
    //   }

    //   // Trigger the selected autonomous routine
    //   if (master.get_digital_new_press(DIGITAL_B)) {
    //     autonomous();
    //     chassis.drive_brake_set(MOTOR_BRAKE_COAST);
    //     chassis.drive_odom_enable(false);
    //   }

      // chassis.pid_tuner_iterate();  // Allow PID Tuner to iterate

    //   // Modifying track width
    //   //  only allow this to run when PID Tuner is disabled
    //   if (!chassis.pid_tuner_enabled()) {
    //     // When A is pressed, increase track width
    //     if (master.get_digital_new_press(DIGITAL_A)) {
    //       chassis.drive_width_set(chassis.drive_width_get() + 0.1);
    //       printf("%.2f\n", chassis.drive_width_get());
    //     }
    //     // When Y is pressed, decrease track width
    //     else if (master.get_digital_new_press(DIGITAL_Y)) {
    //       chassis.drive_width_set(chassis.drive_width_get() - 0.1);
    //       printf("%.2f\n", chassis.drive_width_get());
    //     }
    //   }
    //}

    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
      autonomous();
      chassis.drive_brake_set(MOTOR_BRAKE_COAST);
      chassis.drive_odom_enable(false);
    }
    //printf("l1 : %f, l2 : %f, l3 : %f, l4 : %f, r1 : %f, r2 : %f, r3 : %f, r4 : %f e : %f\n", chassis.left_motors.at(0).get_position(), chassis.left_motors.at(1).get_position(), chassis.left_motors.at(2).get_position(), chassis.left_motors.at(3).get_position(), chassis.right_motors.at(0).get_position(), chassis.right_motors.at(1).get_position(), chassis.right_motors.at(2).get_position(), chassis.right_motors.at(3).get_position(), chassis.imu.get_rotation());

    intake_opcontrol();

    chassis.opcontrol_tank();  // Tank control

    


    if(master.get_digital(intakeButton)){
      intakeMotor.move(127);
      intakeMotor2.move(127);
    }else if(master.get_digital(outtakeButton)){
      intakeMotor.move(-127);
      intakeMotor2.move(-127);
    }else{
      intakeMotor.brake();
      intakeMotor2.brake();
    }

    // TEST INTAKE ANTI-STUCK CODE 

    // intakeMotor.move(127);
    // intakeMotor2.move(127);

    // if(intakeMotor.get_actual_velocity() <= 10){
    //   out = true;
    // }
    // if(out || intakeAntiStopCount <= 2){
    //   intakeMotor.move(-40);
    //   intakeMotor2.move(-40);
    //   intakeAntiStopCount++;
    // }
    // if(intakeAntiStopCount >= 3){
    //   intakeAntiStopCount = 0;
    // }

    if(master.get_digital(rightWingButton) && (rightWingCounter >=  30)){
      rightWingExtended = !rightWingExtended;
      rightWingCounter = 0;
    }
    

    if(master.get_digital(leftWingButton) && (leftWingCounter >= 30)){
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

   

    // LIFT


    if(master.get_digital(liftUpButton)){
      // hang.move(127);
      // hang2.move(127);
      liftUp = true;
      climbing = false;
    }else if(master.get_digital(liftDownButton)){
      // hang.move(-127);
      // hang2.move(-127);
      climb = true;
    }
    // else{
    //   hang.brake();
    //   hang2.brake();
    // }

    if(climb){
      hang.move_absolute(-30 * 16, 127);
      hang2.move_absolute(-30 * 16, 127);
    }
    else if(liftUp){
      hang.move_absolute(90 * 16, 127);
      hang2.move_absolute(90 * 16, 127);
    }

    


    /*if (master.get_digital(liftDownButton)) {
      liftMotor1.move(-127);
      liftMotor2.move(-127);
      climbing = false;
    }else if (master.get_digital(liftUpButton) || climbing) {
      // liftMotor1.move(127);
      // liftMotor2.move(127);
      liftMotor1.move_absolute(7300, 127);
      liftMotor2.move_absolute(7300, 127);
      climbing = true;
      armDown = true;
    }  else {
      liftMotor1.brake();
      liftMotor2.brake();
    }*/

    if(master.get_digital(leftMatchLoadButton) && (leftPivotCounter >= 10)){
      leftPivot = true;
    }
    if(master.get_digital(rightMatchLoadButton) && (rightPivotCounter >= 10)){
      rightPivot = true;
    }

    leftPivotCounter++;
    rightPivotCounter++;



    if(leftPivot){
      leftMatchLoad();
      leftPivotCounter = 0;
      leftPivot = false;
    }

    if(rightPivot){
      rightMatchLoad();
      rightPivotCounter = 0;
      rightPivot = false;
    }  

    
    
  

    pros::delay(ez::util::DELAY_TIME);  // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}

void disabled() {
}
void competition_initialize() {
}