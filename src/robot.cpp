#include "robot.h"
#include "pros/abstract_motor.hpp"
#include "pros/motor_group.hpp"


// * CONTROLLER
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// * INERTIAL SENSOR
pros::Imu imu(14);

// * LIFT
pros::Motor lift(-6, pros::MotorGearset::red, pros::v5::MotorUnits::degrees);

// * CLAMP
bool clampState = LOW;
pros::adi::DigitalOut Clamp(1, clampState);

// * ODOMETRY
pros::Rotation horizontalEnc(-21);
pros::Rotation verticalEnc(15);
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_275, 0);
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_275, 0);
lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            nullptr, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// * DRIVETRAIN
pros::MotorGroup lMotors({-10, -11, -13}, pros::MotorGearset::blue);
pros::MotorGroup rMotors({2, 12, 20}, pros::MotorGearset::blue);
lemlib::Drivetrain drivetrain(&lMotors,
                             &rMotors,
                              10,
                           lemlib::Omniwheel::NEW_4,
                                     360,
                         2
);

// * PID SETTINGS
lemlib::ControllerSettings linearController(8.567, // proportional gain (kP)
                                            0.01467, // integral gain (kI)
                                            3.067, // derivative gain (kD)
                                   0, // anti windup
                                    0, // small error range, in inches
                             0, // small error range timeout, in milliseconds
                                    0, // large error range, in inches
                             0, // large error range timeout, in milliseconds
                                  0 // maximum acceleration (slew)
);
lemlib::ControllerSettings angularController(3, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             20.67, // derivative gain (kD)
                                    0, // anti windup
                                     0, // small error range, in degrees
                              0, // small error range timeout, in milliseconds
                                     0, // large error range, in degrees
                              0, // large error range timeout, in milliseconds
                                           0 // maximum acceleration (slew)
);

// * DRIVING CURVE CONTROL
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// * STEERING CURVE CONTROL
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

// * CHASSIS
lemlib::Chassis chassis(drivetrain,
                        linearController,
                        angularController,
                        sensors,
                        &throttleCurve,
                        &steerCurve
);
