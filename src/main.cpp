#include "lemlib/api.hpp" // IWYU pragma: keep
#include "main.h"
#include "lemlib/chassis/chassis.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "robot.h"
#include <cmath>
#include <cerrno>


/*
 * Runs initialization code. This occurs as soon as the program is started.
  
   All other competition modes are blocked by initialize; it is recommended
   to keep execution time for this mode under a few seconds.
 */
void initialize() {

    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    lift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    // reset lift position to 0
    lift.move_velocity(-50);
    pros::delay(500);
    lift.move_velocity(0);
    lift.tare_position();

    Clamp.set_value(clampState);

    // thread to for brain screen and position logging
    pros::Task screenTask([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            uint32_t timestamp = pros::millis();
            pros::lcd::print(3, "Lift Position: %d, Raw: %i", lift.get_position(), lift.get_raw_position(&timestamp)); // lift position and velocity
            pros::lcd::print(4, "Lift Velocity: %d", lift.get_actual_velocity()); // lift velocity
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources
            pros::delay(50);
        }
    });
}

/*
 * Runs while the robot is disabled
 */
void disabled() {}

/*
 * Runs after initialize if the robot is connected to field control
 */
void competition_initialize() {}

/*
 * Runs during auto
 */
void autonomous() {
    chassis.setPose(0, 0, 0);
    lift.tare_position();

    // * score first pin
    lift.move_absolute(160, 100); // raise lift DURING motion
    pros::delay(200);
    chassis.moveToPoint(0, 8, 500, {.minSpeed = 100, .earlyExitRange = 6});
    // // chassis.swingToPoint(18, 27, lemlib::DriveSide::RIGHT, 500, {.maxSpeed = 80});
    chassis.swingToHeading(60, lemlib::DriveSide::RIGHT, 1000, {.maxSpeed = 80}); // face goal

    pros::delay(400);

    lift.move_absolute(20, 40); // lower lift
    pros::delay(400);
    chassis.moveToPoint(4, 13, 800, {.maxSpeed = 80}); // forward wiggle to score pin
    pros::delay(300);
    Clamp.set_value(true); // release preload
    lift.move_absolute(50, 50); // out of the way

    pros::delay(500);

    // * to second pin and cup
    chassis.swingToHeading(0, lemlib::DriveSide::RIGHT, 600, {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 80}); // dodge goal
    pros::delay(300);
    chassis.moveToPoint(-2, 26, 1000, {.minSpeed = 40, .earlyExitRange = 4}); // to second pin and cup
    lift.move_absolute(85, 50); // raise lift DURING motion
    chassis.swingToPoint(21, 52, lemlib::DriveSide::RIGHT, 500, {.direction = lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 80}); // face cup
    
    pros::delay(200);
    
    // * pick up second pin and cup

    lift.move_absolute(90, 50); // just to make sure yk?
    chassis.moveToPoint(21, 51, 700, {.maxSpeed = 25, .earlyExitRange = 10}); // move within range
    pros::delay(400);
    lift.move_absolute(60, 30); // lower lift to pick up cup
    
    pros::delay(300); // wait for lift
    Clamp.set_value(false);

    pros::delay(800);

    // * score second pin and cup
    lift.move_absolute(180, 80); // raise pin+cup
    pros::delay(200);
    // // chassis.turnToPoint(20, 20, 700, {.maxSpeed = 80, .minSpeed = 40, .earlyExitRange = 10});
    chassis.turnToHeading(150, 1000, {.direction = lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 40}); // face goal
    pros::delay(400);
    // // chassis.moveToPoint(16, 34, 1000, {.maxSpeed = 30, .earlyExitRange = 12}); // to goal
    lift.move_absolute(220, 50); // raise lift DURING motion
    
    pros::delay(600);

    chassis.moveToPoint(14, 36, 1000, {.forwards = false, .maxSpeed = 30}); // backwards wiggle
    pros::delay(10);
    lift.move_relative(-100, 30); // lower onto goal
    pros::delay(1500);
    Clamp.set_value(true); // release pin+cup
}

/*
 * Runs in driver control
 */
void opcontrol() {
    // controller
    // loop to continuously update motors
    while (true) {
        // get joystick positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // move the chassis with curvature drive
        chassis.arcade(leftY, rightX);

        // Lift
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            lift.move_velocity(100);
        }

        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            lift.move_velocity(-100);
        }

        else {
            lift.move_velocity(0);
        }

        // Lift Macro
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
            lift.move_absolute(120, 50);
            pros::delay(1000);
        }

        // Clamp
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
            Clamp.set_value(!clampState);
            clampState = !clampState;
        }
        
        // delay to save resources
        pros::delay(10);
    }
}