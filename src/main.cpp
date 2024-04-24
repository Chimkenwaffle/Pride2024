#include <Arduino.h>
#include "subsystems/drivetrain/drivetrain.h"
#include "subsystems/gyro/gyro.h"
#include "subsystems/switches/switches.h"
#include "subsystems/ballsensor/ballsensor.h"
#include "subsystems/superstate/superstate.h"
#include "subsystems/linesensor/linesensor.h"
#include "constants.hpp"
#include <Adafruit_MCP3008.h>
#include <PrideUtils.h>

using namespace PrideUtils;

Adafruit_MCP3008 adc4;
Adafruit_MCP3008 adc5;
Adafruit_MCP3008 adc6;

const double rotation_P = .4;
const double rotation_D = 0.1;

void setup() {
  
  Serial.begin(9600);
  delay(250);

  if (CrashReport) {
    Serial.print(CrashReport);
    Serial.println();
    Serial.flush();
  }

  SuperState::setup();
  
  bool success = Gyro::setup();
  Gyro::setOrigin();
  Drivetrain::setup();
  Switches::setup();
  BallSensor::setup();
  LineSensor::setup();
  LineSensor::readThresholds();

  // setup failed! 
  if (!success) {
    SuperState::changeState(State::INIT_FAILED);
    while (1) {
      SuperState::update();
      Serial.println("[SETUP] Failed to initialize gyro");
      delay(1000);
    }
  }

  SuperState::changeState(State::CALIBRATING);
  SuperState::update(true);

  while (Switches::getSwitchOne() == false) {
    // Serial.println("[SETUP] Calibrating...");
    Drivetrain::stop();
    if (Switches::getSwitchTwo() == true) {
      Serial.println("[SETUP] Calibrating Line Sensors");

      int maxReadings[LineSensorConstants::LINE_SENSORS] = {0};
      int minReadings[LineSensorConstants::LINE_SENSORS];

      for (int i = 0; i < LineSensorConstants::LINE_SENSORS; i++) {
        minReadings[i] = 800;
      }

      while(Switches::getSwitchTwo() == true) {
        LineSensor::read();
        for (int i = 0; i < LineSensorConstants::LINE_SENSORS; i++) {
          if (LineSensor::readings[i] > maxReadings[i]) {
            maxReadings[i] = LineSensor::readings[i];
          }
          if (LineSensor::readings[i] < minReadings[i]) {
            minReadings[i] = LineSensor::readings[i];
          }
        }
        delay(1000/10);
      }

      for (int i = 0; i < LineSensorConstants::LINE_SENSORS; i++) {
        LineSensor::thresholds[i] = maxReadings[i] + 45;
        LineSensor::pickup_thresholds[i] = max(minReadings[i] - 60, 0);
      }
      Serial.print("Thresholds: ");
      for (int i = 0; i < LineSensorConstants::LINE_SENSORS; i++) {
        Serial.print(String(LineSensor::thresholds[i]) + " ");
      }

      Serial.println();
      Serial.print("Pickup Thresholds: ");
      for (int i = 0; i < LineSensorConstants::LINE_SENSORS; i++) {
        Serial.print(String(LineSensor::pickup_thresholds[i]) + " ");
      }
      Serial.println();
      LineSensor::saveThresholds();
    } 
    // Serial.println("Switch 1" + String(Switches::getSwitchOne()) + "\n Switch 2" + String(Switches::getSwitchTwo()) + "\n Switch 3" + String(Switches::getSwitchThree()) + "\n Switch 4" + String(Switches::getSwitchFour()) + "\n Switch 5" + String(Switches::getSwitchFive()));
    delay(100);
  }
  Serial.println("[SETUP] Calibrated");
  delay(200);

  SuperState::changeState(State::READY);
}

float dampen(float x) {
  return fmin(1.0, 0.048 * pow(M_E, 3.2 * x));
  // return fmax(0, fmin(1, 0.02 * pow(1.0001, 20 * (x - 10))));
}


float getBallOffset(float inAngle) {
  float formulaAngle = inAngle > 180.0 ? 360 - inAngle : inAngle;
  return (fmin(0.05 * pow(M_E, 0.15 * formulaAngle + 1.2), 90));
}

AngleRad prev_robo_angle = 0;

void loop() {
  SuperState::update();

  // BallSensor::getBallAngleVector(true);
  
  // AngleDeg cartesianBallAngle = BallSensor::ball_angle_deg;
  // AngleDeg forwardAngle = cartesianBallAngle.forwardAngle();
  // // Serial.print("Ball Angle: " + String(cartesianBallAngle.toString() + " Forwared Angle: " + forwardAngle.toString()));

  // float offset = getBallOffset(fabs(forwardAngle.value));

  // // float finalAngle = forwardAngle.value > 0 ? forwardAngle.value + offset : forwardAngle.value - offset;

  // AngleDeg finalAngle = forwardAngle > 0 ? forwardAngle + offset * dampen(BallSensor::ball_mag) : forwardAngle - offset * dampen(BallSensor::ball_mag);

  // float driveAngle = finalAngle.cartesianAngle().toRad().value;

  // // Serial.println("Final Angle: " + finalAngle.toString() + " C FInal Angle: " + finalAngle.cartesianAngle().toString() + " Offset: " + String(offset));

  // // change to angleDeg or angleRad to confirm
  // AngleRad roboAngle = Gyro::getHeading(false);

  // // can modify the .2 value 
  // // might need to swap roboAgle & prev_robo_angle 
  // float rotation = max(-.2, min(.2, roboAngle.value * rotation_P + (roboAngle.value - prev_robo_angle.value) * rotation_D));
  // Serial.println(roboAngle.toDeg().toString() +" : " + String(rotation));
  // // Serial.println(String(roboAngle) + " : " + String(rotation));

  // // Serial.println(" : " + String(rotation));

  // // Serial.print(BallSensor::ballValues[6]);

  // // Serial.print(" : ");

  // // Serial.println(BallSensor::ball_mag);

  // if (BallSensor::ballValues[6] > 600 && BallSensor::ball_mag > .85) {
  //   SuperState::changeState(State::READY);
  //   Drivetrain::drive(3.14/2, .5, 0);
  // } else {
  //   SuperState::changeState(State::INIT_FAILED);
  //   Drivetrain::drive(driveAngle, .5, rotation);
  // }


  // prev_robo_angle = roboAngle;

  LineSensor::processLineSensors(true);

  delay(1000/10);
}

