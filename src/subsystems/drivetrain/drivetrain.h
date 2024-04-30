#pragma once
#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

#include <Arduino.h>
#include <unordered_map>
#include "motor/motor.h"
#include "PrideUtils.h"
#include "constants.hpp"

using namespace PrideUtils;
using namespace std;

enum VectorPriority {
    LOW_PRIORITY,
    MEDIUM_PRIORITY,
    HIGH_PRIORITY,
    OVERRIDE_PRIORITY
};

enum AlgorithmName {
    ATTACK,
    LINE_AVOID
};

#define SET_VECTOR_PRIORITY(ID, PRIO) \
  Drivetrain::vectorMap[ID] = make_pair(PRIO, Vector(0, 0))

class Drivetrain {
    public:
        static Motor* frontLeftMotor;
        static Motor* frontRightMotor;
        static Motor* backRightMotor;
        static Motor* backLeftMotor;

        static Vector currentVector;
        static unordered_map<int, std::pair<VectorPriority, Vector>> vectorMap;
        static float rotation;

        static void setup();
        static void setVector(AlgorithmName name, Vector vec);
        static void setPriority(AlgorithmName name, VectorPriority priority);
        static void driveByVectors(float power);
        static void drive(double angle_rad, double power, double rotation);
        static void rotate(double rotation);
        static void print();
        static void stop();
};

#endif