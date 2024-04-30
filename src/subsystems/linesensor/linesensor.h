#pragma once
#ifndef LINESENSOR_H
#define LINESENSOR_H

#include <Arduino.h>
#include <Adafruit_MCP3008.h>
#include <SD.h>
#include <SPI.h>
#include "constants.hpp"
#include "../superstate/superstate.h"
#include "../drivetrain/drivetrain.h"
#include "../gyro/gyro.h"
#include "PrideUtils.h"

using namespace std;
using namespace PrideUtils;

class LineSensor {
    public:
        static Adafruit_MCP3008 adc4;
        static Adafruit_MCP3008 adc5;
        static Adafruit_MCP3008 adc6;
        static bool isPickedUp;

        static int thresholds[LineSensorConstants::LINE_SENSORS];
        static int pickup_thresholds[LineSensorConstants::LINE_SENSORS];
        static int readings[LineSensorConstants::LINE_SENSORS];
        static float cosVals[LineSensorConstants::LINE_SENSORS];
        static float sinVals[LineSensorConstants::LINE_SENSORS];
        static Vector lineSensorVectors[LineSensorConstants::LINE_SENSORS];
        static Vector previousVector;
        static Vector currentVector;
        static AngleRad previousHeading;
        static bool isOver;
        static bool firstTime;
        static void setup();
        static void readThresholds();
        static void saveThresholds();

        static void read();

        static int readLineSensor(int x);

        static void processLineSensors(bool refreshValues = false);
};

#endif