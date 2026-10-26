#pragma once

#include "lemlib/api.hpp" // IWYU pragma: keep
#include "main.h" // IWYU pragma: keep
#include "pros/adi.hpp"

extern pros::Controller controller;

extern pros::MotorGroup leftMotors;
extern pros::MotorGroup rightMotors;

extern pros::Motor lift;

extern bool clampState;
extern pros::adi::DigitalOut Clamp;

extern pros::Imu imu;

extern pros::Rotation horizontalEnc;
extern pros::Rotation verticalEnc;
extern lemlib::TrackingWheel horizontal;
extern lemlib::TrackingWheel vertical;
extern lemlib::OdomSensors sensors;

extern lemlib::Drivetrain drivetrain;

extern lemlib::ControllerSettings linearController;

extern lemlib::ControllerSettings angularController;

extern lemlib::ExpoDriveCurve throttleCurve;

extern lemlib::ExpoDriveCurve steerCurve;

extern lemlib::Chassis chassis;

extern lemlib::PID liftPID;