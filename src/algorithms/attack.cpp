#include "attack.h"

void Attack::init() {
    // Initialize the attack algorithm
    Drivetrain::power = 0.75;
}

float dampen(float x) {
  return fmin(1.0, 0.1 * pow(M_E, 3.0 * x));
  // return fmax(0, fmin(1, 0.02 * pow(1.0001, 20 * (x - 10))));
}


float getBallOffset(float inAngle) {
  float formulaAngle = inAngle > 180.0 ? 360 - inAngle : inAngle;
  return (fmin(0.05 * pow(M_E, 0.15 * formulaAngle + 1.8), 90));
}


const double rotation_P = .4;
const double rotation_D = 0.1;
AngleRad prev_robo_angle = 0;
void Attack::loop(int threadID) {
    BallSensor::getBallAngleVector(true);

    AngleDeg cartesianBallAngle = BallSensor::ball_angle_deg;
    AngleDeg forwardAngle = cartesianBallAngle.forwardAngle();

    float offset = getBallOffset(fabs(forwardAngle.value));
    float dampenPercent = dampen(BallSensor::ball_mag);
    offset *= dampenPercent;

    AngleDeg finalAngle = forwardAngle > 0 ? forwardAngle + offset : forwardAngle - offset;

    Vector driveAngle = finalAngle.cartesianAngle().toVector();

    // Serial.println("Ball Angle: " + String(cartesianBallAngle.value) + ", Forward Angle: " + String(forwardAngle.value) + ", offset: " + String(offset) + ", Dampen Percent: " + String(dampenPercent) + ", Final Angle: " + String(finalAngle.cartesianAngle().value));

    AngleRad roboAngle = Gyro::getHeading();

    float rotation = max(-.2, min(.2, roboAngle.value * rotation_P + (roboAngle.value - prev_robo_angle.value) * rotation_D));
    prev_robo_angle = roboAngle;

  // Serial.println("Ball Mag: " + String(BallSensor::ball_mag) + " ball 6: " + String(BallSensor::ballValues[6]));

    if (BallSensor::ballValues[6] > 600 && BallSensor::ball_mag > .85) {
    SuperState::changeState(State::ATTACKING_WITH_BALL);
    Drivetrain::setVector(ATTACK, Vector(0, 1));
  } else {
    SuperState::changeState(State::ATTACKING);
    Drivetrain::setVector(ATTACK, driveAngle);
  }

  Drivetrain::rotation = rotation;
  // // Serial.println("Rotation: " + String(rotation) + ", Robo Angle: " + String(roboAngle.value));
  // LineSensor::read();
  // LineSensor::preProcessLineSensors();
  // if (LineSensor::checkIfPickedUp()) {
  //   SuperState::changeState(State::PICKED_UP);
  //   Drivetrain::stop();
  // }
  // LineSensor::calculateLineSensorGroups();
  // Vector lineSensorDesiredVector = LineSensor::findDesiredVector();
  // // Serial.print("Line Sensor Desired Vector: ");
  // // Serial.print(lineSensorDesiredVector.toAngleDeg().value);
  // // Serial.print(" Ball Angle: ");
  // // Serial.println(cartesianBallAngle.value);
  // if (lineSensorDesiredVector.isZero()) {
  //   Drivetrain::driveByVectors();
  //   return;
  // }

  // // we on a line :skull:
  // Vector torwardLineVector = LineSensor::isOver == false ? lineSensorDesiredVector.flip() : lineSensorDesiredVector;
  // // check if angle between torwardLineVector and driveAngle is not less than 90
  // if (torwardLineVector.toAngleDeg().angleDifference(driveAngle.toAngleDeg()).value > 90) {
  //   // Serial.println("HERE?");
  //   // u not even going torwards the line buddy, just keep doing watchu doing
  //   Drivetrain::driveByVectors();
  //   return;
  // } else if (lineSensorDesiredVector.toAngleDeg().angleDifference(driveAngle.toAngleDeg()).value > 90) {
  //   // we are on the line and we are driving over it
  //   Drivetrain::setVector(LINE_AVOID, lineSensorDesiredVector*2 + driveAngle);
  //   return;
  // }
  // Serial.println("fr line vector: " + String(lineSensorDesiredVector.toAngleDeg().value) + " driving: " + String(driveAngle.toAngleDeg().value));

  // // shit we are on and going torward it
  
  // AngleRad flipped90 = lineSensorDesiredVector + AngleRad(3.14/2);
  // AngleRad flippedNeg90 = lineSensorDesiredVector + AngleRad(-3.14/2);

  // Serial.print("line vector: ");
  // Serial.print(lineSensorDesiredVector.toAngleDeg().value);
  // Serial.print(" driving: ");
  // AngleRad desiredRadians = driveAngle.toAngleRad();
  // if (desiredRadians.angleDifference(flipped90).value < desiredRadians.angleDifference(flippedNeg90).value) {
  //   Drivetrain::drive(flipped90.value, Drivetrain::power, rotation);
  //   Serial.println(flipped90.toDeg().value);
  // } else {
  //   Serial.println(flippedNeg90.toDeg().value);
  //   Drivetrain::drive(flippedNeg90.value, Drivetrain::power, rotation);
  // }
  LineSensor::processLineSensors(true);

  Drivetrain::driveByVectors();


  // delay(1000/100);
}