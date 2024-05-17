#pragma once
#ifndef DEFENSE_H
#define DEFENSE_H

#include <PrideUtils.h>
#include <Arduino.h>
#include <subsystems/linesensor/linesensor.h>
#include <subsystems/superstate/superstate.h>
#include <subsystems/drivetrain/drivetrain.h>
#include <subsystems/gyro/gyro.h>
#include <subsystems/ballsensor/ballsensor.h>
#include <TeensyThreads.h>
#include <subsystems/locationsensor/locationsensor.h>

using namespace PrideUtils;

class DefenseAlgorithm {
    public:
        static fPIDController ballAngleController;
        static fPIDController rotationController;
        static void init();
        static void loop(int threadID);
        static float distanceToBack;
        static int increment;
        static AngleRad angleOffset;
        static AngleRad ballAngleError;
};

#endif