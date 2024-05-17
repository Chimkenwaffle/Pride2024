#pragma once
#ifndef SUPERSTATE_H
#define SUPERSTATE_H
#include <Arduino.h>
#include <unordered_map>
#include "constants.hpp"

enum State {
    INIT_FAILED,
    WAITING_TO_CALIBRATE,
    GREEN_CALIBRATING,
    WAITING_TO_SPIN_CALIBRATE,
    SPIN_CALIBRATING,
    READY,
    ATTACKING,
    ATTACKING_WITH_BALL,
    LINE_AVOIDING,
    PICKED_UP,
    NO_BALL_FOUND,
    FULL_POWER
};

class Color {
    public:
        int r;
        int g;
        int b;
        bool flashing;

        Color(int r, int g, int b, bool flashing = false);
};

extern std::unordered_map<State, Color*> stateColors;

class SuperState {
    public:
        static State currentState;
        static void setup();
        static void changeState(State newState);
        static void update(bool force = false);
};

#endif