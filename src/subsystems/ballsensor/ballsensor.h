#pragma once
#ifndef LIBBALLSENSOR_HPP
#define LIBBALLSENSOR_HPP

#include <Arduino.h>
#include <Adafruit_MCP3008.h>
#include <tuple>
#include <PrideUtils.h>
#include "constants.hpp"

using namespace std;

class BallSensor {
    public:
        static Adafruit_MCP3008 adc1;
        static Adafruit_MCP3008 adc2;
        static Adafruit_MCP3008 adc3;
        static float cosVals[BallSensorConstants::Ball_SENSORS];
        static float sinVals[BallSensorConstants::Ball_SENSORS];
        static float ball_angle_rad;
        static PrideUtils::AngleDeg ball_angle_deg;
        static float ball_mag;

        static int ballValues[BallSensorConstants::Ball_SENSORS];

        static void setup();

        static void read();

        static int readBallSensor(int x);

        static tuple<float, float> getBallAngleVector(bool refreshValues);
};

#endif