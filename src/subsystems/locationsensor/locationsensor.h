#pragma once
#ifndef LOCATIONS_SENSOR_H
#define LOCATIONS_SENSOR_H

#include <PrideUtils.h>
#include <Arduino.h>
#include <constants.hpp>
#include <TeensyThreads.h>

class LocationSensor {
    public:
        static int threadID;
        static void setup();
        static void startThread();
        static int toBack;
        static float frBack;
        static bool backGood;
        static int toFront;
        static bool frontGood;
        static int toLeft;
        static int frLeft;
        static bool leftGood;
        static int toRight;
        static int frRight;
        static bool rightGood;
    private:
        static void loop();
        static bool readingVertical;
};

#endif