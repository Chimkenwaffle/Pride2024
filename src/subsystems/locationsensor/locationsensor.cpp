#include "locationsensor.h"

const int fieldWidth = 182;
const int fieldHeight = 243;
bool LocationSensor::readingVertical = true;
int LocationSensor::toBack = 0;
float LocationSensor::frBack = 0;
int LocationSensor::toFront = 0;
int LocationSensor::toLeft = 75;
int LocationSensor::frLeft = 0;
int LocationSensor::toRight = 75;
int LocationSensor::frRight = 0;
bool LocationSensor::backGood = false;
bool LocationSensor::frontGood = false;
bool LocationSensor::leftGood = false;
bool LocationSensor::rightGood = false;

int LocationSensor::threadID = 0;

void LocationSensor::setup() {
    pinMode(LocationSensorConstants::TRIG_PIN_BACK, OUTPUT);
    pinMode(LocationSensorConstants::ECHO_PIN_BACK, INPUT);
    pinMode(LocationSensorConstants::TRIG_PIN_LEFT, OUTPUT);
    pinMode(LocationSensorConstants::ECHO_PIN_LEFT, INPUT);
    pinMode(LocationSensorConstants::TRIG_PIN_FRONT, OUTPUT);
    pinMode(LocationSensorConstants::ECHO_PIN_FRONT, INPUT);
    pinMode(LocationSensorConstants::TRIG_PIN_RIGHT, OUTPUT);
    pinMode(LocationSensorConstants::ECHO_PIN_RIGHT, INPUT);

}

void LocationSensor::startThread() {
    // threads.addThread(LocationSensor::loop);
    LocationSensor::threadID = threads.addThread(LocationSensor::loop);
}

void LocationSensor::loop() {
    while (1) {
        if (readingVertical) {
            float durationBack, distanceBack, durationFront, distanceFront;
            digitalWrite(LocationSensorConstants::TRIG_PIN_BACK, LOW);
            digitalWrite(LocationSensorConstants::TRIG_PIN_FRONT, LOW);
            threads.delay_us(2);
            digitalWrite(LocationSensorConstants::TRIG_PIN_FRONT, HIGH);
            threads.delay_us(10);
            digitalWrite(LocationSensorConstants::TRIG_PIN_FRONT, LOW);

            durationFront = pulseIn(LocationSensorConstants::ECHO_PIN_FRONT, HIGH);
            distanceFront = (durationFront * 0.034 / 2);
            digitalWrite(LocationSensorConstants::TRIG_PIN_BACK, HIGH);
            threads.delay_us(10);
            digitalWrite(LocationSensorConstants::TRIG_PIN_BACK, LOW);
            durationBack = pulseIn(LocationSensorConstants::ECHO_PIN_BACK, HIGH);
            distanceBack = (durationBack * 0.034 / 2);
            if (distanceFront < 130 && distanceFront != 0) {
                toFront = static_cast<int>(round(distanceFront));
                frontGood = true;
            } else {
                frontGood = false;
            }
            if (distanceBack < 130 && distanceBack != 0) {
                toBack = static_cast<int>(round(distanceBack));
                frBack = distanceBack;
                backGood = true;
            } else {
                backGood = false;
            }
            // Serial.println(" Front: " + String(toFront) + " Back: " + String(toBack));
            #ifdef DEBUG_LOCATION_SENSOR
            Serial.println(" Front " + String(frontGood) + " " + String(toFront) + " Back " + String(backGood) + " " + String(toBack));
            #endif
        } else {
            float durationLeft, distanceLeft, durationRight, distanceRight;
            digitalWrite(LocationSensorConstants::TRIG_PIN_LEFT, LOW);
            // digitalWrite(LocationSensorConstants::TRIG_PIN_RIGHT, LOW);
            threads.delay_us(2);
            digitalWrite(LocationSensorConstants::TRIG_PIN_LEFT, HIGH);
            // digitalWrite(LocationSensorConstants::TRIG_PIN_RIGHT, HIGH);
            threads.delay_us(10);
            digitalWrite(LocationSensorConstants::TRIG_PIN_LEFT, LOW);
            durationLeft = pulseIn(LocationSensorConstants::ECHO_PIN_LEFT, HIGH);
            distanceLeft = (durationLeft * 0.034 / 2);
            


            digitalWrite(LocationSensorConstants::TRIG_PIN_RIGHT, LOW);
            threads.delay_us(2);
            digitalWrite(LocationSensorConstants::TRIG_PIN_RIGHT, HIGH);
            threads.delay_us(10);
            digitalWrite(LocationSensorConstants::TRIG_PIN_RIGHT, LOW);
            

            durationRight = pulseIn(LocationSensorConstants::ECHO_PIN_RIGHT, HIGH);
            distanceRight = (durationRight * 0.034 / 2);
            int temp = static_cast<int>(round(distanceLeft));
            if (distanceLeft < 50 && distanceLeft != 0) {
                toLeft = temp;
                leftGood = true;
            } else {
                leftGood = false;
            }
            frLeft = temp;
            temp = static_cast<int>(round(distanceRight));
            if (distanceRight < 50 && distanceRight != 0) {
                toRight = temp;
                rightGood = true;
            } else {
                rightGood = false;
            }
            frRight = temp;
            // Serial.println("Left: " + String(distanceLeft) + " Right: " + String(distanceRight));
            #ifdef DEBUG_LOCATION_SENSOR
            Serial.print("Left " + String(leftGood) + " " + String(toLeft) + " Right " + String(rightGood) + " " + String(toRight));
            #endif
        }

        threads.delay_us(10000);
        readingVertical = !readingVertical;
    }
}