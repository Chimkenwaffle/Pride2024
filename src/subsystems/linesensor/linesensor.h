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

struct sensor_group_data {
    int bestI;
    int bestJ;
    int closestAngle;
};

enum ApproxVerticalResults {
    LEFT,
    RIGHT,
    IN_FRONT,
    BEHIND
};

class LineSensor {
    public:
        static Adafruit_MCP3008 adc4;
        static Adafruit_MCP3008 adc5;
        static Adafruit_MCP3008 adc6;
        static bool isPickedUp;

        static int thresholds[LineSensorConstants::LINE_SENSORS];
        static int pickup_thresholds[LineSensorConstants::LINE_SENSORS];
        static int readings[LineSensorConstants::LINE_SENSORS];
        static int previous_readings[LineSensorConstants::LINE_SENSORS];
        static float cosVals[LineSensorConstants::LINE_SENSORS];
        static float sinVals[LineSensorConstants::LINE_SENSORS];
        static Vector lineSensorVectors[LineSensorConstants::LINE_SENSORS];
        static Vector previousVector;
        static Vector currentVector;
        static AngleRad previousHeading;
        static bool isOver;
        static bool firstTime;
        static bool triggeredSensors[LineSensorConstants::LINE_SENSORS];
        static int detections;
        static int lineSensorGroups;
        static int pickedUpSensors;
        static sensor_group_data lineSensorGroupData;
        static void setup();
        static void readThresholds();
        static void saveThresholds();

        static void read();

        static int readLineSensor(int x);

        static void processLineSensors(bool refreshValues = false);
        static void processLineSensorsDefense(bool refreshValues = false);
        
        static void preProcessLineSensors();
        static bool checkIfPickedUp();
        static Vector simpleVectorSum();
        static Vector findDesiredVector();
        static sensor_group_data calculateLineSensorGroups(); 
        static pair<Vector, Vector> getAveragedLineSensorVectors();
        static bool checkIfOnLine();
        static ApproxVerticalResults checkApproxVertical();
};

#endif