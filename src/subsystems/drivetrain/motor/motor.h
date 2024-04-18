#pragma once
#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

class Motor {
    public: 
        int enPin;
        int phPin;
        int currentVelocity;
        bool reversed;

        Motor(int enPin, int phPin, bool reversed = false);
        void setup();
        void writeAndSetPower(float power);
        void setPower(float power);
        void write();
        void stop();
    
    private:
        void setDirection();
};

#endif