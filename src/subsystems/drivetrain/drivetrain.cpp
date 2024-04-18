#include "drivetrain.h"

Motor* Drivetrain::frontLeftMotor = nullptr;
Motor* Drivetrain::frontRightMotor = nullptr;
Motor* Drivetrain::backRightMotor = nullptr;
Motor* Drivetrain::backLeftMotor = nullptr;

void Drivetrain::setup() {
    Drivetrain::frontLeftMotor = new Motor(DrivetrainConstants::bluePin3, DrivetrainConstants::greenPin3, true);
    Drivetrain::frontRightMotor = new Motor(DrivetrainConstants::bluePin4, DrivetrainConstants::greenPin4);
    Drivetrain::backRightMotor = new Motor(DrivetrainConstants::bluePin, DrivetrainConstants::greenPin, true);
    Drivetrain::backLeftMotor = new Motor(DrivetrainConstants::bluePin2, DrivetrainConstants::greenPin2, true);
    Drivetrain::frontLeftMotor->setup();
    Drivetrain::frontRightMotor->setup();
    Drivetrain::backRightMotor->setup();
    Drivetrain::backLeftMotor->setup();
}

/**
 * Rotates the robot in place
 * 
 * @param rotation The robot rotates counterclockwise or clockwise [-1, 1].
 * @return void
*/
void Drivetrain::rotate(double rotation) {
    Drivetrain::frontLeftMotor->writeAndSetPower(rotation);
    Drivetrain::frontRightMotor->writeAndSetPower(-rotation);
    Drivetrain::backRightMotor->writeAndSetPower(-rotation);
    Drivetrain::backLeftMotor->writeAndSetPower(rotation);
}

float PRIDE_PI = 3.14159;

/**
 * Drives the drivetrain
 * 
 * @param angle_rad The direction the robot will travel. 0 is east [0, 2pi].
 * @param power How fast the robot travels in the direction from [0, 1].
 * @param rotation The robot rotates counterclockwise or clockwise [-1, 1].
 * @return void
*/
void Drivetrain::drive(double angle_rad, double power, double rotation) {
    // Calculate x and y components from the angle

    // to fix front and back being reversed
    angle_rad -= MathConstants::PRIDE_PI / 2;  // Subtract π/2 radians (90 degrees)

    double x = cos(angle_rad);
    double y = sin(angle_rad);

    // Calculate individual motor speeds based on desired x, y, rotation, and power
    double cos42 = cos(42.0 * M_PI / 180.0);
    double sin42 = sin(42.0 * M_PI / 180.0);
    double velFL = power * ((cos42 * x - sin42 * y) + rotation);
    double velFR = power * ((cos42 * x + sin42 * y) - rotation);
    double velBR = power * ((cos42 * x - sin42 * y) - rotation);
    double velBL = power * ((cos42 * x + sin42 * y) + rotation);

    Drivetrain::frontLeftMotor->writeAndSetPower(velFL);
    Drivetrain::frontRightMotor->writeAndSetPower(velFR);
    Drivetrain::backLeftMotor->writeAndSetPower(velBL);
    Drivetrain::backRightMotor->writeAndSetPower(velBR);
}

void Drivetrain::stop() {
    Drivetrain::frontLeftMotor->stop();
    Drivetrain::frontRightMotor->stop();
    Drivetrain::backRightMotor->stop();
    Drivetrain::backLeftMotor->stop();
}

void Drivetrain::print() {
    Serial.println("Front Left Motor: " + String(Drivetrain::frontLeftMotor->currentVelocity));
    Serial.println("Front Right Motor: " + String(Drivetrain::frontRightMotor->currentVelocity));
    Serial.println("Back Right Motor: " + String(Drivetrain::backRightMotor->currentVelocity));
    Serial.println("Back Left Motor: " + String(Drivetrain::backLeftMotor->currentVelocity));
}