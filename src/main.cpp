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
  // if (!success) {
  //   SuperState::changeState(State::INIT_FAILED);
  //   while (1) {
  //     SuperState::update();
  //     Serial.println("[SETUP] Failed to initialize gyro");
  //     delay(1000);
  //   }
  // }


  while (Switches::getSwitchOne() == false) {
    SuperState::changeState(State::WAITING_TO_CALIBRATE);
    SuperState::update(true);
    // Serial.println("[SETUP] Calibrating...");
    Drivetrain::stop();
    if (Switches::getSwitchTwo() == true) {

      Serial.println("[SETUP] Calibrating Line Sensors");

      int maxGreenReadings[LineSensorConstants::LINE_SENSORS] = {0};
      int minGreenReadings[LineSensorConstants::LINE_SENSORS];

      for (int i = 0; i < LineSensorConstants::LINE_SENSORS; i++) {
        minGreenReadings[i] = 800;
      }
      // Changes superstate to calibrating
      SuperState::changeState(State::GREEN_CALIBRATING);

      while(Switches::getSwitchTwo() == true) {
        // Makes LED flash
        SuperState::update();
        Drivetrain::rotate(.2);
        LineSensor::read();
        for (int i = 0; i < LineSensorConstants::LINE_SENSORS; i++) {
          if (LineSensor::readings[i] > maxGreenReadings[i]) {
            maxGreenReadings[i] = LineSensor::readings[i];
          }
          if (LineSensor::readings[i] < minGreenReadings[i]) {
            minGreenReadings[i] = LineSensor::readings[i];
          }
        }
        delay(1000/10);
      }

      Drivetrain::stop();

      for (int i = 0; i < LineSensorConstants::LINE_SENSORS; i++) {
        LineSensor::pickup_thresholds[i] = max(minGreenReadings[i] - 150, 0);
      }

      SuperState::changeState(State::WAITING_TO_SPIN_CALIBRATE);
      SuperState::update();
      while(Switches::getSwitchThree() == false) {
        delay(100);
      }

      int maxWhiteReadings[24] = {0};
      if(Switches::getSwitchThree() == true) {
        SuperState::changeState(State::SPIN_CALIBRATING);
        Serial.println("[SETUP] Starting Spinning Calibration");
        while(Switches::getSwitchThree() == true) {
          SuperState::update();
          Drivetrain::rotate(.2);
          LineSensor::read();
          for (int i = 0; i < LineSensorConstants::LINE_SENSORS; i++) {
            if (LineSensor::readings[i] > maxWhiteReadings[i]) {
              maxWhiteReadings[i] = LineSensor::readings[i];
            }
          }
          delay(50);
        }
      }

      Drivetrain::stop();

      Serial.print("Green: ");
      for(int i = 0; i < LineSensorConstants::LINE_SENSORS; i++) {
        Serial.print(maxGreenReadings[i]);
        Serial.print(" ");
      }

      Serial.println();

      Serial.print("White: ");
      for(int i = 0; i < LineSensorConstants::LINE_SENSORS; i++) {
        Serial.print(maxWhiteReadings[i]);
        Serial.print(" ");
      }
      Serial.println();

      bool greenGreaterThanWhite = false;
      for(int i = 0; i < LineSensorConstants::LINE_SENSORS; i++) {
        if(maxGreenReadings[i] > maxWhiteReadings[i]) {
          greenGreaterThanWhite = true;
        }
        LineSensor::thresholds[i] = (maxGreenReadings[i] + maxWhiteReadings[i]) / 2;
      }

      Serial.print("Threshold Values: ");
      for(int i = 0; i < LineSensorConstants::LINE_SENSORS; i++) {
        Serial.print(LineSensor::thresholds[i]);
        Serial.print(" ");
      }
      Serial.println();

      if(greenGreaterThanWhite) { 
        Serial.print("[ERROR] GREEN LINE SENSOR READINGS GREATER THAN WHITE LINE SENSOR READINGS !!!");
        SuperState::changeState(State::INIT_FAILED);
        while(true) {
          SuperState::update();
          delay(100);
        }
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

  // Drivetrain::drive(2*3.14, .5, 0);
  // prev_robo_angle = roboAngle;

  LineSensor::processLineSensors(true);

  delay(1000/5);
}

