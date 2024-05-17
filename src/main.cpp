#include <Arduino.h>
#include "subsystems/drivetrain/drivetrain.h"
#include "subsystems/gyro/gyro.h"
#include "subsystems/switches/switches.h"
#include "subsystems/ballsensor/ballsensor.h"
#include "subsystems/superstate/superstate.h"
#include "subsystems/linesensor/linesensor.h"
#include "subsystems/locationsensor/locationsensor.h"
#include "algorithms/defense.h"
#include "algorithms/attack.h"
#include "constants.hpp"
#include <Adafruit_MCP3008.h>
#include <PrideUtils.h>
#include <TeensyThreads.h>

using namespace PrideUtils;

// const double rotation_P = .4;
// const double rotation_D = 0.1;

void setup() {
  
  Serial.begin(115200);
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
  LocationSensor::setup();
  LineSensor::setup();
  LineSensor::readThresholds();

  LocationSensor::startThread();

  // setup failed! 
  if (!success) {
    SuperState::changeState(State::INIT_FAILED);
    while (1) {
      SuperState::update();
      Serial.println("[SETUP] Failed to initialize gyro");
      Gyro::setup();
      delay(1000);
    }
  }

  SET_VECTOR_PRIORITY(AlgorithmName::ATTACK, VectorPriority::MEDIUM_PRIORITY);

  DefenseAlgorithm::init();
  // Attack::init();

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
      while(Switches::getSwitchTwo() == false) {
        delay(100);
      }

      int maxWhiteReadings[24] = {0};
      if(Switches::getSwitchTwo() == true) {
        SuperState::changeState(State::SPIN_CALIBRATING);
        Serial.println("[SETUP] Starting Spinning Calibration");
        while(Switches::getSwitchTwo() == true) {
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
        int difference = abs(maxGreenReadings[i] - maxWhiteReadings[i]);
        LineSensor::thresholds[i] = maxGreenReadings[i] + difference / 2 ;
      }

      LineSensor::thresholds[17] = LineSensor::thresholds[17] - 10;
      LineSensor::thresholds[9] = LineSensor::thresholds[9] - 10;
      LineSensor::thresholds[7] = LineSensor::thresholds[7] - 10;

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

bool foutOfBounds = false;
Vector previousNormalLine = Vector(-5,-5);

void loop() {
  SuperState::update();
  // LineSensor::read();
  // LineSensor::preProcessLineSensors();
  // sensor_group_data owo = LineSensor::getLargestGap();
  // Serial.print(owo.bestI);
  // Serial.print(" ");
  // Serial.print(owo.bestJ);
  // Serial.print(" ");
  // Serial.println((LineSensor::lineSensorVectors[owo.bestI] + LineSensor::lineSensorVectors[owo.bestJ]).toAngleDeg().value);
  // sensor_group_data data = LineSensor::calculateLineSensorGroups();
  // Vector normalLineSensor = Vector(0,0);
  // if (data.bestI != 0 || data.bestJ != 0) {
  //   normalLineSensor = LineSensor::lineSensorVectors[data.bestI] + LineSensor::lineSensorVectors[data.bestJ];

  //   if (LineSensor::lineSensorGroups == 3) {
  //     Serial.print("#3 ");
  //     normalLineSensor = previousNormalLine;
  //   }

  //   Serial.print(data.bestI);
  //   Serial.print(" ");
  //   Serial.print(data.bestJ);

  //   Serial.print(" Normal Line Sensor: ");
  //   Serial.print(normalLineSensor.toAngleDeg().value);

  //   Serial.print(" Previous Normal Line Sensor: ");
  //   Serial.print(previousNormalLine.toAngleDeg().value);

  //   if (previousNormalLine.x != -5) {
  //     Serial.print(" HERE ");
  //     float angleDiff = normalLineSensor.toAngleDeg().angleDifference(previousNormalLine.toAngleDeg()).value;
  //     if (abs(angleDiff) > 90) {
  //       foutOfBounds = !foutOfBounds;
  //     } 
  //   }

  //   Serial.print(" out of bounds ");
  //   Serial.print(foutOfBounds);

  //   Serial.print(" outa here: ");
  //   Vector outaHere = normalLineSensor;
  //   outaHere = foutOfBounds ? outaHere : outaHere.flip();
  //   Serial.print(outaHere.toAngleDeg().value);

  //   Vector projectedOne = (outaHere.toAngleRad() + AngleRad(3.14/2)).toVector();
  //   Vector projectedTwo = (outaHere.toAngleRad() + AngleRad(-3.14/2)).toVector();
  //   Serial.print(" Possible proj vectors: ");
  //   Serial.print(projectedOne.toAngleDeg().value);
  //   Serial.print(" ");
  //   Serial.println(projectedTwo.toAngleDeg().value);

  //   previousNormalLine = normalLineSensor;
  // } else {
  //   previousNormalLine = Vector(-5, -5);
  // }

  // BallSensor::getBallAngleVector(true);
  // Serial.println(BallSensor::ball_angle_deg.value);
  // LineSensor::read();
  // LineSensor::preProcessLineSensors();
  // DefenseAlgorithm::loop(LocationSensor::threadID);
  // Drivetrain::frontLeftMotor->writeAndSetPower(1);
  // Drivetrain::frontRightMotor->writeAndSetPower(.2);
  // Drivetrain::backRightMotor->writeAndSetPower(-.2);
  // Drivetrain::backLeftMotor->writeAndSetPower(-1);
  // Drivetrain::drive(3.14/2, .2, 0);
  // Drivetrain::print();
  // Drivetrain::drive(0, 1, 0);


  Attack::loop(LocationSensor::threadID);
  // delay(250);


  // Drivetrain::drive(180, 1, 0);
  // if()

}
