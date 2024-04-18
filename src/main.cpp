#include <Arduino.h>
#include "subsystems/drivetrain/drivetrain.h"
#include "subsystems/gyro/gyro.h"
#include "subsystems/switches/switches.h"
#include "subsystems/ballsensor/ballsensor.h"
#include "subsystems/superstate/superstate.h"
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

  // Line sensor ADC
  // TODO: Replace w/ line sensor subsystem l8r
  adc4.begin(27, 11, 12, 24);
  adc5.begin(27, 11, 12, 10);
  adc6.begin(27, 11, 12, 9);

  
  while (Switches::getSwitchOne() == false) {
    Serial.println("[SETUP] Calibrating...");
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

float prev_robo_angle = 0;

void loop() {
  SuperState::update();

  BallSensor::getBallAngleVector(true);
  
  AngleDeg cartesianBallAngle = BallSensor::ball_angle_deg;
  AngleDeg forwardAngle = cartesianBallAngle.forwardAngle();
  // Serial.print("Ball Angle: " + String(cartesianBallAngle.toString() + " Forwared Angle: " + forwardAngle.toString()));

  float offset = getBallOffset(fabs(forwardAngle.value));

  // float finalAngle = forwardAngle.value > 0 ? forwardAngle.value + offset : forwardAngle.value - offset;

  AngleDeg finalAngle = forwardAngle > 0 ? forwardAngle + offset * dampen(BallSensor::ball_mag) : forwardAngle - offset * dampen(BallSensor::ball_mag);

  float driveAngle = finalAngle.cartesianAngle().toRad().value;

  // Serial.println("Final Angle: " + finalAngle.toString() + " C FInal Angle: " + finalAngle.cartesianAngle().toString() + " Offset: " + String(offset));

  float roboAngle = Gyro::getData().yaw;


  float rotation = max(-.2, min(.2, roboAngle * rotation_P + (roboAngle - prev_robo_angle) * rotation_D));
  Serial.println(String(roboAngle) + " : " + String(rotation));

  Drivetrain::drive(driveAngle, 1, rotation);

  prev_robo_angle = roboAngle;

  delay(1000/10);
}

