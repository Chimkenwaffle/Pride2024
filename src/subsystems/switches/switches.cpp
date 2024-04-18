#include "switches.h"

void Switches::setup() {
    pinMode(SwitchesConstants::SwitchOnePin, INPUT_PULLDOWN);
    pinMode(SwitchesConstants::SwitchTwoPin, INPUT_PULLUP);
    // pinMode(SwitchesConstants::SwitchThreePin, INPUT_PULLUP);
    // pinMode(SwitchesConstants::SwitchFourPin, INPUT_PULLUP);
    pinMode(SwitchesConstants::SwitchFivePin, INPUT_PULLUP);
}

bool Switches::getSwitchOne() {
    // Serial.println(analogRead(SwitchesConstants::SwitchOnePin));
    return analogRead(SwitchesConstants::SwitchOnePin) > SwitchesConstants::PullDownThreshold;
}

bool Switches::getSwitchTwo() {
    // Serial.println(analogRead(SwitchesConstants::SwitchTwoPin));
    return analogRead(SwitchesConstants::SwitchTwoPin) > SwitchesConstants::PullUpThreshold;
}

bool Switches::getSwitchThree() {
    // Serial.println(analogRead(SwitchesConstants::SwitchThreePin));
    // return analogRead(SwitchesConstants::SwitchThreePin) > SwitchesConstants::PullUpThreshold;
    return false;
}

bool Switches::getSwitchFour() {
    // Serial.println(analogRead(SwitchesConstants::SwitchFourPin));
    // return analogRead(SwitchesConstants::SwitchFourPin) > SwitchesConstants::PullUpThreshold;
    return false;
}

bool Switches::getSwitchFive() {
    // Serial.println("[SWITCHES] Switch 5 Doesn't work, returning false");
    Serial.println(analogRead(SwitchesConstants::SwitchFivePin));
    return false;   
}