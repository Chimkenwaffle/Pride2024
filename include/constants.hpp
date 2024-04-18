#pragma once
#ifndef LIBCONSTANTS_HPP
#define LIBCONSTANTS_HPP

//  File Name : LibConstants.hpp    Purpose : Global Constants for Lib Utils

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
    const int Ball_SENSORS = 24;
}

namespace LineSensorConstants {
    const int CS1 = 24;
    const int CS2 = 10;
    const int CS3 = 9;
    const int Line_SENSORS = 24;
}

namespace SPIBus {
    const int MOSI = 11;
    const int MISO = 12;
    const int SCK = 13;
}

#endif