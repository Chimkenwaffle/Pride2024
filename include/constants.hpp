#pragma once
#ifndef LIBCONSTANTS_HPP
#define LIBCONSTANTS_HPP

//  File Name : LibConstants.hpp    Purpose : Global Constants for Lib Utils

// #define DEBUG true

namespace MathConstants
{
    const float PRIDE_PI = 3.14159;
}

namespace DrivetrainConstants
{
    const int bluePin = 1;
    const int greenPin = 0;

    const int bluePin2 = 3;
    const int greenPin2 = 2;

    const int bluePin3 = 6;
    const int greenPin3 = 5;

    const int bluePin4 = 8;
    const int greenPin4 = 7;
}

namespace GyroConstants
{
    const int gyroResetPin = 22;
}

namespace SuperStateConstants
{
    const int redPin = 29;
    const int greenPin = 28;
    const int bluePin = 25;
}

namespace SwitchesConstants {
    const int SwitchOnePin = 17;
    const int SwitchTwoPin = 16;
    const int SwitchThreePin = 15;
    const int SwitchFourPin = 14;
    const int SwitchFivePin = 13;
    const int PullDownThreshold = 400;
    const int PullUpThreshold = 1000;
}

namespace BallSensorConstants {
    const int CS1 = 32;
    const int CS2 = 30;
    const int CS3 = 4;
    const int BALL_SENSORS = 24;
}

// #define DEBUG_LINE_SENSOR true
namespace LineSensorConstants {
    const int CS4 = 24;
    const int CS5 = 10;
    const int CS6 = 9;
    const int LINE_SENSORS = 24;
    const int READ_TIMES = 3;
    const int LINE_READ_MIN = 20;
    const int LINE_READ_MAX = 900;
    const int NUM_SENSORS_PICKUP_THRESHOLD = LINE_SENSORS / 2;
    // const int SENSOR_OFFSET[24] = {50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50};
}

namespace SPIBus {
    const int MOSI = 11;
    const int MISO = 12;
    const int SCK = 13;
}

// #define DEBUG_LOCATION_SENSOR true
namespace LocationSensorConstants {
    const int ECHO_PIN_RIGHT = 40;
    const int ECHO_PIN_FRONT = 39;
    const int ECHO_PIN_LEFT = 38;
    const int ECHO_PIN_BACK = 37;
    const int TRIG_PIN_BACK = 36;
    const int TRIG_PIN_LEFT = 35;
    const int TRIG_PIN_FRONT = 34;
    const int TRIG_PIN_RIGHT = 33;
}

#endif