#include "linesensor.h"

Adafruit_MCP3008 LineSensor::adc4;
Adafruit_MCP3008 LineSensor::adc5;
Adafruit_MCP3008 LineSensor::adc6;
bool LineSensor::isPickedUp = false;

int LineSensor::readings[LineSensorConstants::LINE_SENSORS] = {0};
int LineSensor::previous_readings[LineSensorConstants::LINE_SENSORS] = {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 ,-1, -1, -1, -1, -1};
int LineSensor::thresholds[LineSensorConstants::LINE_SENSORS] = {500};
int LineSensor::pickup_thresholds[LineSensorConstants::LINE_SENSORS] = {150};
int lineSensorIndexArray[LineSensorConstants::LINE_SENSORS];
Vector LineSensor::lineSensorVectors[LineSensorConstants::LINE_SENSORS] = {Vector(0,0)};
Vector LineSensor::previousVector = Vector(0,0);
Vector LineSensor::currentVector = Vector(0,0);
AngleRad LineSensor::previousHeading = AngleRad(0);
bool LineSensor::isOver = false;
bool LineSensor::firstTime = true;

bool LineSensor::triggeredSensors[LineSensorConstants::LINE_SENSORS] = {0};
int LineSensor::detections = 0;
int LineSensor::lineSensorGroups = 0;
int LineSensor::pickedUpSensors = 0;
sensor_group_data LineSensor::lineSensorGroupData = {0, 0, 0};

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
    SET_VECTOR_PRIORITY(AlgorithmName::LINE_AVOID, VectorPriority::OVERRIDE_PRIORITY);
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
        if (previous_readings[i] == -1) {
            previous_readings[i] = average;
        }
        readings[i] = (.1 * previous_readings[i] + .9 * average);
        previous_readings[i] = readings[i];
        // Serial.print(String(average) + " ");
    }
    // Serial.println();
}

void LineSensor::preProcessLineSensors() {
    pickedUpSensors = 0;
    detections = 0;
    lineSensorGroups = 0;
    for (int i = 0; i < LineSensorConstants::LINE_SENSORS; i++) {
        if (readings[i] > thresholds[i]) {
            triggeredSensors[i] = true;
            detections++;
            #ifdef DEBUG_LINE_SENSOR
                Serial.print("w");
            #endif
            if (i == LineSensorConstants::LINE_SENSORS - 1) {
                if(!triggeredSensors[0] && !triggeredSensors[LineSensorConstants::LINE_SENSORS - 2]) {
                    lineSensorGroups++;
                } else if (triggeredSensors[0] && triggeredSensors[LineSensorConstants::LINE_SENSORS - 2]) {
                    lineSensorGroups--;
                }
            } else if (i == 0) {
                lineSensorGroups++;
            } else if (!triggeredSensors[i - 1]) {
                lineSensorGroups++;
            }
        } else if (readings[i] < pickup_thresholds[i]){
            pickedUpSensors++;
            triggeredSensors[i] = false;
            #ifdef DEBUG_LINE_SENSOR
                Serial.print("a");
            #endif
        } else {
            triggeredSensors[i] = false;
            #ifdef DEBUG_LINE_SENSOR
                Serial.print("g");
            #endif
        }
        #ifdef DEBUG_LINE_SENSOR
            Serial.print(readings[i]);
            Serial.print(" ");
        #endif
    }
    #ifdef DEBUG_LINE_SENSOR
        Serial.println();
    #endif
}

bool LineSensor::checkIfPickedUp() {
    if (pickedUpSensors > LineSensorConstants::NUM_SENSORS_PICKUP_THRESHOLD) {
        isPickedUp = true;
        SuperState::changeState(State::PICKED_UP);
        return true;
    } else {
        isPickedUp = false;
        return false;
    }
}

pair<Vector, Vector> LineSensor::getAveragedLineSensorVectors() {
    Vector IVec = Vector(0, 0);
    Vector JVec = Vector(0, 0);
    int numIVec = 0;
    int numJVec = 0;
    bool solvingForI = true;
    bool iHasBeenSolvedFor = false;
    for (int i = 0; i < LineSensorConstants::LINE_SENSORS; i++) {
        if (triggeredSensors[i]) {
            if (solvingForI) {
                iHasBeenSolvedFor = true;
                IVec = IVec + lineSensorVectors[i];
                numIVec++;
            } else {
                JVec = JVec + lineSensorVectors[i];
                numJVec++;
            }
        } else if (iHasBeenSolvedFor) {
            solvingForI = false;
        }
    }
    if (numIVec != 0) {
        IVec = IVec / numIVec;
    }
    if (numJVec != 0) {
        JVec = JVec / numJVec;
    }
    return pair<Vector, Vector>(IVec, JVec);
}

sensor_group_data LineSensor::calculateLineSensorGroups() {
    int bestI = 0;
    int bestJ = 0;

    int largestAngle = 0;
    // finds furthest triggered sensors
    for (int i = LineSensorConstants::LINE_SENSORS - 1; i >= 0; i--) {
        if (triggeredSensors[i]) {
            for (int j = LineSensorConstants::LINE_SENSORS - 1; j >= 0; j--) {
                if (triggeredSensors[j]) {
                    const float diff = lineSensorVectors[i]
                                                .toAngleDeg()
                                                .angleDifference(lineSensorVectors[j]
                                                .toAngleDeg()).value;
                    // Serial.print("diff: ");
                    // Serial.print(diff);
                    // Serial.println(" I: " + String(i) + " J: " + String(j));

                    if (abs(180 - abs(diff)) <= abs(180 - abs(largestAngle))) {
                        bestI = i;
                        bestJ = j;
                        largestAngle = diff;
                        // Serial.print("Reassigning Largest Angle: ");
                        // Serial.print(largestAngle);
                        // Serial.println(" I: " + String(i) + " J: " + String(j));
                    }
                }
            }
        }
    }
    // Serial.print("Best I: ");
    // Serial.println(bestI);
    // Serial.print("Best J: ");
    // Serial.println(bestJ);
    lineSensorGroupData = {bestI, bestJ, largestAngle};
    // Serial.println("Best I: " + String(bestI) + " Best J: " + String(bestJ) + " Largest Angle: " + String(largestAngle));
    return sensor_group_data{bestI, bestJ, largestAngle};
}

bool LineSensor::checkIfOnLine() {
    return (lineSensorGroupData.closestAngle > 160);
}

const int offset = 60;
ApproxVerticalResults LineSensor::checkApproxVertical() {
    Vector currentVector = lineSensorVectors[lineSensorGroupData.bestI] + lineSensorVectors[lineSensorGroupData.bestJ];
    AngleDeg currentAngle = currentVector.toAngleDeg();
    if (currentAngle.value > 90 - offset && currentAngle.value < 90 + offset) {
        return ApproxVerticalResults::IN_FRONT;
    } else if (currentAngle.value < -90 + offset && currentAngle.value > -90 - offset) {
        return ApproxVerticalResults::BEHIND;
    } else if ((currentVector.x > 0 && currentVector.y > 0) || (currentVector.x < 0 && currentVector.y < 0)) {
        return ApproxVerticalResults::RIGHT;
    } else {
        return ApproxVerticalResults::LEFT;
    }
}

Vector LineSensor::simpleVectorSum() {
    Vector retVal = Vector(0,0);
    for (int i = 0; i < LineSensorConstants::LINE_SENSORS; i++) {
        if (triggeredSensors[i]) {
            retVal = retVal + lineSensorVectors[i];
        }
    }
    return retVal;
}

Vector LineSensor::findDesiredVector() {
    if (lineSensorGroupData.bestI == 0 && lineSensorGroupData.bestJ == 0) {
        return Vector(0, 0);
        firstTime = true;
    }
    Vector currentVector = lineSensorVectors[LineSensor::lineSensorGroupData.bestI] + lineSensorVectors[LineSensor::lineSensorGroupData.bestJ];
    if (previousVector.isZero()) {
        previousVector = currentVector;
    }

    if (currentVector.isZero() || lineSensorGroups == 3) {
        // 180 Case. Use previous vector
        AngleRad changeInHeading = Gyro::heading - previousHeading;
        currentVector = (changeInHeading + previousVector.toAngleRad()).toVector();
        previousVector = currentVector;
        previousHeading = Gyro::heading;
    } else {
        previousHeading = Gyro::heading;
    }

    if (!isOver) {
        currentVector = currentVector.flip();
    }

    const float difference = currentVector.toAngleDeg().angleDifference(previousVector.toAngleDeg()).value;
    if (!firstTime || lineSensorGroups == 3) {
        if (abs(difference) > 90) {
            isOver = !isOver;
            currentVector = currentVector.flip();
        }
    }

    firstTime = false;
    previousVector = currentVector;
    return currentVector;
}

// void LineSensor::processLineSensors(bool refreshValues) {
//     if (refreshValues) {
//         LineSensor::read();
//     }
    
//     int pickupSensors = 0;
//     int lineSensorGroups = 0;
//     int detections = 0;
//     // creates boolean array of triggered sensors
//     bool triggered[LineSensorConstants::LINE_SENSORS] = {0};
//     for (int i = 0; i < LineSensorConstants::LINE_SENSORS; i++) {
//         if (readings[i] > thresholds[i]) {
//             triggered[i] = true;
//             detections++;
//             // Serial.print("w");

//             if (i == LineSensorConstants::LINE_SENSORS - 1) {
//                 if(!triggered[0] && !triggered[LineSensorConstants::LINE_SENSORS - 2]) {
//                     lineSensorGroups++;
//                 } else if (triggered[0] && triggered[LineSensorConstants::LINE_SENSORS - 2]) {
//                     lineSensorGroups--;
//                 }
//             } else if (i == 0) {
//                 lineSensorGroups++;
//             } else if (!triggered[i - 1]) {
//                 lineSensorGroups++;
//             }
//         } else if (readings[i] < pickup_thresholds[i]){
//             pickupSensors++;
//             // Serial.print("a");
//         } else {
//             // Serial.print("g");
//         }
//         // Serial.print(readings[i]);
//         // Serial.print(" ");
//     }
//     // Serial.println();

//     if (pickupSensors > LineSensorConstants::NUM_SENSORS_PICKUP_THRESHOLD) {
//         isPickedUp = true;
//         SuperState::changeState(State::PICKED_UP);
//         return;
//     } else {
//         isPickedUp = false;
//         // SuperState::changeState(State::READY);
//     }

//     int bestI = 0;
//     int bestJ = 0;

//     int closestAngle = 0;
//     // finds furthest triggered sensors
//     for (int i = 0; i < LineSensorConstants::LINE_SENSORS; i++) {
//         if (triggered[i]) {
//             for (int j = 0; j < LineSensorConstants::LINE_SENSORS; j++) {
//                 if (triggered[j]) {
//                     const float diff = lineSensorVectors[i]
//                                                 .toAngleDeg()
//                                                 .angleDifference(lineSensorVectors[j]
//                                                 .toAngleDeg()).value;
//                     if (abs(180 - diff) < abs(180 - closestAngle)) {
//                         bestI = i;
//                         bestJ = j;
//                         closestAngle = diff;
//                     }
//                 }
//             }
//         }
//     }

//     if (bestI + bestJ != 0) {
//         // we are on a line!
//         currentVector = lineSensorVectors[bestI] + lineSensorVectors[bestJ];
        
//         if (previousVector.isZero()) {
//             previousVector = currentVector;
//         }

//         if (currentVector.isZero() || lineSensorGroups == 3) {
//             // 180 Case. Use previous vector
//             // TODO: Fix angle difference
//             AngleRad changeInHeading = Gyro::heading - previousHeading;
//             currentVector = (changeInHeading + previousVector.toAngleRad()).toVector();
//             previousVector = currentVector;
//             previousHeading = Gyro::heading;
//             // Serial.println("180 Case: " + String(changeInHeading.value));
//         } else {
//             previousHeading = Gyro::heading;
//         }
        
//         if (!isOver) {
//             // we are in the field so we need to flip our vector sum to point torward the field
//             currentVector = currentVector.flip();
//         }

//         const float difference = currentVector.toAngleDeg().angleDifference(previousVector.toAngleDeg()).value;
//         if (!firstTime || lineSensorGroups == 3) {
//             if (abs(difference) > 90) {
//                 isOver = !isOver;
//                 currentVector = currentVector.flip();
//             }
//         }

//         // Drivetrain::drive(currentVector.toAngleRad().value, .5, 0);
//         Drivetrain::setVector(LINE_AVOID, currentVector);

//         firstTime = false;
//         previousVector = currentVector;
//         // Serial.println("I: " + String(bestI) + " J:" + String(bestJ) + "Difference: " + String(difference) +  " Current Vector: " + String(currentVector.toAngleDeg().value) + " Previous Vector: " + String(previousVector.toAngleDeg().value) + " IsOver: " + String(isOver));
//     } else {
//         if (firstTime == false) {
//             Serial.println("NO Line Detected");
//         }
//         firstTime = true;
//         Drivetrain::setVector(LINE_AVOID, Vector(0, 0));
//         // Serial.println("No Line Detected");
//         // Drivetrain::stop();
//     }
// }

// void LineSensor::processLineSensorsDefense(bool refreshValues) {
//     if (refreshValues) {
//         LineSensor::read();
//     }
    
//     int pickupSensors = 0;
//     int lineSensorGroups = 0;
//     int detections = 0;
//     // creates boolean array of triggered sensors
//     bool triggered[LineSensorConstants::LINE_SENSORS] = {0};
//     for (int i = 0; i < LineSensorConstants::LINE_SENSORS; i++) {
//         if (readings[i] > thresholds[i]) {
//             triggered[i] = true;
//             detections++;
//             // Serial.print("w");

//             if (i == LineSensorConstants::LINE_SENSORS - 1) {
//                 if(!triggered[0] && !triggered[LineSensorConstants::LINE_SENSORS - 2]) {
//                     lineSensorGroups++;
//                 } else if (triggered[0] && triggered[LineSensorConstants::LINE_SENSORS - 2]) {
//                     lineSensorGroups--;
//                 }
//             } else if (i == 0) {
//                 lineSensorGroups++;
//             } else if (!triggered[i - 1]) {
//                 lineSensorGroups++;
//             }
//         } else if (readings[i] < pickup_thresholds[i]){
//             pickupSensors++;
//             // Serial.print("a");
//         } else {
//             // Serial.print("g");
//         }
//         // Serial.print(readings[i]);
//         // Serial.print(" ");
//     }
//     // Serial.println();

//     if (pickupSensors > LineSensorConstants::NUM_SENSORS_PICKUP_THRESHOLD) {
//         isPickedUp = true;
//         SuperState::changeState(State::PICKED_UP);
//         return;
//     } else {
//         isPickedUp = false;
//         // SuperState::changeState(State::READY);
//     }

//     int bestI = 0;
//     int bestJ = 0;

//     int closestAngle = 0;
//     // finds furthest triggered sensors
//     for (int i = 0; i < LineSensorConstants::LINE_SENSORS; i++) {
//         if (triggered[i]) {
//             for (int j = 0; j < LineSensorConstants::LINE_SENSORS; j++) {
//                 if (triggered[j]) {
//                     const float diff = lineSensorVectors[i]
//                                                 .toAngleDeg()
//                                                 .angleDifference(lineSensorVectors[j]
//                                                 .toAngleDeg()).value;
//                     if (abs(180 - diff) < abs(180 - closestAngle)) {
//                         bestI = i;
//                         bestJ = j;
//                         closestAngle = diff;
//                     }
//                 }
//             }
//         }
//     }

//     if (bestI + bestJ != 0) {
//         // on line = bueno!
//         currentVector = lineSensorVectors[bestI] + lineSensorVectors[bestJ];

//         if (currentVector.isZero()) {
//             Drivetrain::setVector(LINE_AVOID, Vector(0, 0));
//         } else {
//             Drivetrain::setVector(LINE_AVOID, currentVector);
//             Drivetrain::power = .5;
//         }
//     } else {
//         Drivetrain::setVector(LINE_AVOID, Vector(0, -1));
//         Drivetrain::power = .5;
//     }
// }