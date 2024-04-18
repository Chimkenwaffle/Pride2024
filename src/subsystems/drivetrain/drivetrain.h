#pragma once
#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

#include <Arduino.h>
#include "motor/motor.h"
#include "constants.hpp"

class Drivetrain {
    public:
        static Motor* frontLeftMotor;
        static Motor* frontRightMotor;
        static Motor* backRightMotor;
        static Motor* backLeftMotor;

        static void setup();
        static void drive(double angle_rad, double power, double rotation);
        static void rotate(double rotation);
        static void print();
        static void stop();
};

#endif