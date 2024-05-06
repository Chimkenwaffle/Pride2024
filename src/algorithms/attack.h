#pragma once
#ifndef ATTACK_H
#define ATTACK_H

#include <Arduino.h>
#include <PrideUtils.h>
#include <subsystems/ballsensor/ballsensor.h>
#include <subsystems/superstate/superstate.h>
#include <subsystems/drivetrain/drivetrain.h>
#include <subsystems/gyro/gyro.h>
#include <subsystems/linesensor/linesensor.h>

using namespace PrideUtils;
using namespace std;

class Attack {
    public:
        static void init();
        static void loop(int threadID);
};

#endif