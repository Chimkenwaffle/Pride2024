#pragma once
#ifndef LIBSWITCHES_HPP
#define LIBSWITCHES_HPP

#include <Arduino.h>
#include "constants.hpp"

class Switches {
  public:
    static void setup();
    static bool getSwitchOne();
    static bool getSwitchTwo();
    static bool getSwitchThree();
    static bool getSwitchFour();
    static bool getSwitchFive();
};


#endif 