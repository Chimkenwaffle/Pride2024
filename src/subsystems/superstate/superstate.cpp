#include "superstate.h"

// Define static member variable currentState of SuperState class
State SuperState::currentState;

std::unordered_map<State, Color*> stateColors = {
    {INIT_FAILED, new Color(0, 0, 255, false)}, // red
    {CALIBRATING, new Color(165, 36, 0)},  // yellow
    {READY, new Color(40, 85, 0, true)},   // green flashing
    {PICKED_UP, new Color(0, 0, 255, false)}   // blue solid
};

// Define the constructor of the Color class
Color::Color(int r, int g, int b, bool flashing) : r(r), g(g), b(b), flashing(flashing) {}

const Color* off = new Color (0, 0, 0);

void SuperState::setup() {
    SuperState::currentState = State::CALIBRATING;
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
