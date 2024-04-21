#include "linesensor.h"

Adafruit_MCP3008 LineSensor::adc4;
Adafruit_MCP3008 LineSensor::adc5;
Adafruit_MCP3008 LineSensor::adc6;
bool LineSensor::isPickedUp = false;

int LineSensor::readings[LineSensorConstants::LINE_SENSORS] = {0};
int LineSensor::thresholds[LineSensorConstants::LINE_SENSORS] = {500};
int LineSensor::pickup_thresholds[LineSensorConstants::LINE_SENSORS] = {150};
int lineSensorIndexArray[LineSensorConstants::LINE_SENSORS];

int lineMapInputToOutput(int input) {
	int output = -1;
	if (input >= 1 && input <= 5) {
		output = input + 11;
	} else if (input <= 13) {
		output = input - 5;
	} else if (input <= 21) {
		output = input + 3;
	} else if (input <= 24) {
		output = input - 13;
	}
	return output;
}

void LineSensor::setup() {
    adc4.begin(27, 11, 12, LineSensorConstants::CS4);
    adc5.begin(27, 11, 12, LineSensorConstants::CS5);
    adc6.begin(27, 11, 12, LineSensorConstants::CS6);

    Serial.println("Fuck you Kai. -Anand 2024");
    for (int i = 0; i < LineSensorConstants::LINE_SENSORS; i++) {
        // fixes lineMapInputToOutput to proper index of [0, 23]
        lineSensorIndexArray[i] = lineMapInputToOutput(i + 1);
        Serial.println("Line Sensor Index: " + String(i) + " " + String(lineSensorIndexArray[i]));
    }
}

int LineSensor::readLineSensor(int x) {
    int y = lineSensorIndexArray[x]-1;
    int reading;
    if (y < 8) {
        reading = adc4.readADC(y);
    } else if (y < 16) {
        reading = adc5.readADC(y - 8);
    } else {
        reading = adc6.readADC(y - 16);
    }
    return reading;
}

void LineSensor::read() {
    for (int i = 0; i < LineSensorConstants::LINE_SENSORS; i++) {
        int average = 0;
        for (int j = 0; j < LineSensorConstants::READ_TIMES; j++) {
            auto read = readLineSensor(i);

            int freezePrevention = 0;
            while (read < LineSensorConstants::LINE_READ_MIN || read > LineSensorConstants::LINE_READ_MAX) {
                read = readLineSensor(i);
                if (freezePrevention > 2) {
                    read = 0;
                    break;
                }
                freezePrevention += 1;
            }

            average += read;
        }
        average /= LineSensorConstants::READ_TIMES;

        readings[i] = average;
        // Serial.print(String(average) + " ");
    }
    // Serial.println();
}

void LineSensor::processLineSensors(bool refreshValues) {
    if (refreshValues) {
        LineSensor::read();
    }

    int numSensorsPickedup = 0;
    for (int i = 0; i < LineSensorConstants::LINE_SENSORS; i++) {
        if (readings[i] < pickup_thresholds[i]) {
            numSensorsPickedup++;
        }
    }
    if (numSensorsPickedup > LineSensorConstants::NUM_SENSORS_PICKUP_THRESHOLD) {
        isPickedUp = true;
        SuperState::changeState(State::PICKED_UP);
    } else {
        isPickedUp = false;
        SuperState::changeState(State::READY);
    }
}