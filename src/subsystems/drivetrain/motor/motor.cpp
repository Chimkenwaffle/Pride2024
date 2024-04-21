#include "motor.h"

/**
 * Motor constructor, takes in the pins for the motor
 * 
 * @param getEnPin The enable pin for the motor
 * @param getPhPin The phase pin for the motor
 * @param getReversed Whether the motor is reversed or not
 * @return Motor
*/
Motor::Motor(int getEnPin, int getPhPin, bool getReversed) {
    enPin = getEnPin;
    phPin = getPhPin;
    reversed = getReversed;
    currentVelocity = 0;
};

/**
 * Setup motor pins
 * 
 * @return void
*/
void Motor::setup() {
    pinMode(enPin, OUTPUT);
    pinMode(phPin, OUTPUT);
    stop();
}

/**
 * Write the power to the motor and set the power
 * 
 * @param power The power to set the motor to [-1, 1].
 * if power is negative, the motor will spin in reverse
 * @return void
*/
void Motor::writeAndSetPower(float power) {
    setPower(power);
    write();
}

/** 
 * Set local power of the motor. Need to call write() to actually set the motor power
 * 
 * @param power The power to set the motor to [-1, 1].
 * if power is negative, the motor will spin in reverse
 * @return void
*/
void Motor::setPower(float power) {
    currentVelocity = power * 255.0;
}

/**
 * Write the power to the motor
 * 
 * @return void
*/
void Motor::write() {
    setDirection();
    analogWrite(enPin, abs(currentVelocity));
}


/** 
 * Set the direction of the motor
 * 
 * @return void
*/
void Motor::setDirection() {
    if (currentVelocity > 0) {
        // Forward
        digitalWriteFast(phPin, reversed ? LOW : HIGH);
        // if (reversed) {
        //     analogWrite(phPin, 0);
        // } else {
        //     analogWrite(phPin, 255);
        // }
    } else {
        // Backward
        digitalWriteFast(phPin, reversed ? HIGH : LOW);
        // if (reversed) {
        //     analogWrite(phPin, 255);
        // } else {
        //     analogWrite(phPin, 0);
        // }
    }
}

/**
 * Stop the motor
 * 
 * @return void
*/
void Motor::stop() {
    analogWrite(enPin, 0);
    digitalWriteFast(phPin, 0);
}