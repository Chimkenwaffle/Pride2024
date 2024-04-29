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
    PICKED_UP
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