#include "superstate.h"

// Define static member variable currentState of SuperState class
State SuperState::currentState;

std::unordered_map<State, Color*> stateColors = {
    {INIT_FAILED, new Color(255, 0, 0, false)}, // red
    {WAITING_TO_CALIBRATE, new Color(165, 36, 0)},  // yellow
    {GREEN_CALIBRATING, new Color(165, 36, 0, true)}, // yellow flashing
    {WAITING_TO_SPIN_CALIBRATE, new Color(145, 8, 29)}, // magenta 
    {SPIN_CALIBRATING, new Color(145, 8, 29, true)}, // magenta flashing
    {READY, new Color(40, 85, 0, true)},   // green flashing
    {PICKED_UP, new Color(0, 0, 255, false)},   // blue solid
    {ATTACKING, new Color(255, 0, 123, true)}, // purple flashing
    {ATTACKING_WITH_BALL, new Color(255, 0, 255, true)}, // purple flashing
    {LINE_AVOIDING, new Color(0, 255, 255, true)}, // cyan flashing
    {NO_BALL_FOUND, new Color(32, 2, 70, false)}, // purple
    {FULL_POWER, new Color(255, 255, 255, false)} // white 
};

// Define the constructor of the Color class
Color::Color(int r, int g, int b, bool flashing) : r(r), g(g), b(b), flashing(flashing) {}

const Color* off = new Color (0, 0, 0);

void SuperState::setup() {
    SuperState::currentState = State::WAITING_TO_CALIBRATE;
    pinMode(SuperStateConstants::redPin, OUTPUT);
    pinMode(SuperStateConstants::greenPin, OUTPUT);
    pinMode(SuperStateConstants::bluePin, OUTPUT);
    SuperState::update(true);
}

void SuperState::changeState(State newState) {
    SuperState::currentState = newState;
}

bool on = true;

void writeColor(const Color& color) {
    analogWrite(SuperStateConstants::redPin, color.r);
    analogWrite(SuperStateConstants::greenPin, color.g);
    analogWrite(SuperStateConstants::bluePin, color.b);
}


void SuperState::update(bool forced) {

    if (forced || !stateColors[SuperState::currentState]->flashing) {
        writeColor(*stateColors[SuperState::currentState]);
        return;
    }

    if (on) {
        writeColor(*stateColors[SuperState::currentState]);
    } else {
        writeColor(*off);
    }

    on = !on;
}
