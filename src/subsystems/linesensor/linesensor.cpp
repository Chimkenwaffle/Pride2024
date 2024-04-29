#include "linesensor.h"

Adafruit_MCP3008 LineSensor::adc4;
Adafruit_MCP3008 LineSensor::adc5;
Adafruit_MCP3008 LineSensor::adc6;
bool LineSensor::isPickedUp = false;

int LineSensor::readings[LineSensorConstants::LINE_SENSORS] = {0};
int LineSensor::thresholds[LineSensorConstants::LINE_SENSORS] = {500};
int LineSensor::pickup_thresholds[LineSensorConstants::LINE_SENSORS] = {150};
int lineSensorIndexArray[LineSensorConstants::LINE_SENSORS];
Vector LineSensor::lineSensorVectors[LineSensorConstants::LINE_SENSORS] = {Vector(0,0)};
Vector LineSensor::previousVector = Vector(0,0);
Vector LineSensor::currentVector = Vector(0,0);
Vector LineSensor::storedVector = Vector(0,0);
Vector LineSensor::previousDriveDirection = Vector(0,0);
bool LineSensor::held180Case = false;
bool LineSensor::previousCase = false;
bool LineSensor::isOver = false;
bool LineSensor::firstTime = true;


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

    // Serial.println("Fuck you Kai. -Anand 2024");
    for (int i = 0; i < LineSensorConstants::LINE_SENSORS; i++) {
        // fixes lineMapInputToOutput to proper index of [0, 23]
        lineSensorIndexArray[i] = lineMapInputToOutput(i + 1);
        // Serial.println("Line Sensor Index: " + String(i) + " " + String(lineSensorIndexArray[i]));
    }

    if (!SD.begin(BUILTIN_SDCARD)) {
        Serial.println("[LS] Failed to initialize builtin SD Card");
    }

    // creates array containing the angles of each sensor
    // Serial.print("Line Sensor Vectors: ");
    for(int i = 0; i < LineSensorConstants::LINE_SENSORS; i++) {
        LineSensor::lineSensorVectors[i] = AngleDeg(360/24 * i).toVector();
        // Serial.print(String(i) + ": " + lineSensorVectors[i].toString() + ", ");
    }

    Serial.println("[LS] IsOver: " + String(isOver));
}

File thresholdFile;
const char *THRESHOLD_FILE_NAME = "threshold.csv";

void LineSensor::readThresholds() {
    thresholdFile = SD.open(THRESHOLD_FILE_NAME);
    if (!thresholdFile) {
        Serial.println("[LS] Failed to access threshold.csv");
        return;
    }
    // reading thresholds
    int i = 0;
    while (thresholdFile.available()) {
        int currentThreshold = thresholdFile.parseInt();
        Serial.print(currentThreshold);
        Serial.print(" ");
        if (i > LineSensorConstants::LINE_SENSORS * 2 - 1) {
            break;
        }
        if (i < LineSensorConstants::LINE_SENSORS) {
            thresholds[i] = currentThreshold;
        } else {
            pickup_thresholds[i - LineSensorConstants::LINE_SENSORS] = currentThreshold;
        }
        i++;
    }
    Serial.println();

    Serial.print("Tresholds: ");
    for(int i = 0; i < 24; i++) {
        Serial.print(thresholds[i]);
        Serial.print(" ");
    }
    Serial.println();
    Serial.print("Pickup Tresholds: ");
    for(int i = 0; i < 24; i++) {
        Serial.print(pickup_thresholds[i]);
        Serial.print(" ");
    }
    Serial.println();
    Serial.println("[LS] Successfully read thresholds");
}

void LineSensor::saveThresholds() {
    thresholdFile = SD.open(THRESHOLD_FILE_NAME, FILE_WRITE);
    if (!thresholdFile) {
        Serial.println("[LS] Failed to save thresholds");
        return;
    }
    thresholdFile.truncate();
    // writing thresholds
    for (int i = 0; i < LineSensorConstants::LINE_SENSORS; i++) {
        thresholdFile.print(LineSensor::thresholds[i]);
        if (i != LineSensorConstants::LINE_SENSORS-1) {
            thresholdFile.print(",");
        }
    }
    thresholdFile.println();
    // writing pickup_thresholds
    for (int i = 0; i < LineSensorConstants::LINE_SENSORS; i++) {
        thresholdFile.print(LineSensor::pickup_thresholds[i]);
        if (i != LineSensorConstants::LINE_SENSORS-1) {
            thresholdFile.print(",");
        }

    }
    // thresholdFile.println();
    thresholdFile.close();
    Serial.println("[LS] Thresholds successfully saved.");
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
    
    int pickupSensors = 0;
    int lineSensorGroups = 0;
    int detections = 0;
    // creates boolean array of triggered sensors
    bool triggered[LineSensorConstants::LINE_SENSORS] = {0};
    for (int i = 0; i < LineSensorConstants::LINE_SENSORS; i++) {
        if (readings[i] > thresholds[i]) {
            triggered[i] = true;
            detections++;
            // Serial.print("w");

            if (i == LineSensorConstants::LINE_SENSORS - 1) {
                if(!triggered[0] && !triggered[LineSensorConstants::LINE_SENSORS - 2]) {
                    lineSensorGroups++;
                } else if (triggered[0] && triggered[LineSensorConstants::LINE_SENSORS - 2]) {
                    lineSensorGroups--;
                }
            } else if (i == 0) {
                lineSensorGroups++;
            } else if (!triggered[i - 1]) {
                lineSensorGroups++;
            }
        } else if (readings[i] < pickup_thresholds[i]){
            pickupSensors++;
            // Serial.print("a");
        } else {
            // Serial.print("g");
        }
        // Serial.print(readings[i]);
        // Serial.print(" ");
    }
    // Serial.println();

    if (pickupSensors > LineSensorConstants::NUM_SENSORS_PICKUP_THRESHOLD) {
        isPickedUp = true;
        SuperState::changeState(State::PICKED_UP);
    } else {
        isPickedUp = false;
        SuperState::changeState(State::READY);
    }

    int bestI = 0;
    int bestJ = 0;

    int closestAngle = 0;
    // finds furthest triggered sensors
    for (int i = 0; i < LineSensorConstants::LINE_SENSORS; i++) {
        if (triggered[i]) {
            for (int j = 0; j < LineSensorConstants::LINE_SENSORS; j++) {
                if (triggered[j]) {
                    const float diff = lineSensorVectors[i]
                                                .toAngleDeg()
                                                .angleDifference(lineSensorVectors[j]
                                                .toAngleDeg()).value;
                    if (abs(180 - diff) < abs(180 - closestAngle)) {
                        bestI = i;
                        bestJ = j;
                        closestAngle = diff;
                    }
                }
            }
        }
    }

    if (bestI + bestJ != 0) {
        currentVector = lineSensorVectors[bestI] + lineSensorVectors[bestJ];
        currentVector = currentVector.flip();

        Vector driveDirection = currentVector;
        if (currentVector.isZero()) {
            driveDirection = previousDriveDirection;
            Serial.print("180 Case: ");
            held180Case = true;
            if (!previousCase) {
                storedVector = previousDriveDirection;
            }
            if (previousCase && held180Case) {
                driveDirection = storedVector;
            }
        } 

        previousCase = held180Case;
        held180Case = false;
        
        Serial.print(String(!currentVector.isZero()) + " ");
        Serial.print("Current Vector: " + String(currentVector.toAngleDeg().value) + " Previous Vector: " + String(previousVector.toAngleDeg().value));
        
        if (!firstTime) {
            const float difference = currentVector.toAngleDeg().angleDifference(previousVector.toAngleDeg()).value;
            if (abs(difference) > 90) {
                isOver = !isOver;
            }
        }

        if (isOver) {
            driveDirection = driveDirection.flip();
            if (currentVector.isZero()) {
                driveDirection = driveDirection.flip();
            }
        }

        Serial.print(" Over: " + String(isOver));
        Serial.print(" Drive Direction: ");
        Serial.println(driveDirection.toAngleDeg().value);

        Drivetrain::drive(driveDirection.toAngleRad().value, .5, 0);

        previousVector = currentVector;
        previousDriveDirection = driveDirection;

        firstTime = false;
    } else {
        firstTime = true;
        Drivetrain::stop();
    }
}