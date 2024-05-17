#include "attack.h"

Vector previousNormalLineSensor = Vector(-5,-5);
// Vector projectedOne = (outaHere + AngleRad(3.14/2)).toVector();
// Vector projectedTwo = (outaHere + AngleRad(-3.14/2)).toVector();

float power = 0.7;
bool lineAvoidExtreme = false;

bool outOfBounds = false;

AngleRad outaHere = 0;
bool corner = false;
int increment = 0;
bool wasOverCorner = false;

void Attack::init() {
    // Initialize the attack algorithm
    Drivetrain::power = 1.0;
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
fPIDController rotationController = fPIDController(rotation_P,0, rotation_D);
AngleRad prev_robo_angle = 0;
void Attack::loop(int threadID) {
    BallSensor::getBallAngleVector(true);

    AngleDeg cartesianBallAngle = BallSensor::ball_angle_deg;
    AngleDeg forwardAngle = cartesianBallAngle.forwardAngle();

    float offset = getBallOffset(fabs(forwardAngle.value));
    float dampenPercent = dampen(BallSensor::ball_mag);
    offset *= dampenPercent;

    AngleDeg finalAngle = forwardAngle > 0 ? forwardAngle + offset : forwardAngle - offset;

    // Serial.println("Ball Angle: " + String(cartesianBallAngle.value) + ", Forward Angle: " + String(forwardAngle.value) + ", offset: " + String(offset) + ", Dampen Percent: " + String(dampenPercent) + ", Final Angle: " + String(finalAngle.cartesianAngle().value));

    AngleRad roboAngle = Gyro::getHeading();

    float rotation = max(-.2, min(.2, roboAngle.value * rotation_P + (roboAngle.value - prev_robo_angle.value) * rotation_D));
    prev_robo_angle = roboAngle;

  
  LineSensor::read();
  LineSensor::preProcessLineSensors();
  sensor_group_data data = LineSensor::calculateLineSensorGroups();
  Vector normalLineSensor = Vector(0,0);

  // Serial.print("Picked Up Sensors: ");
  // Serial.println(LineSensor::detections);

  if (LineSensor::checkIfPickedUp()) {
    SuperState::changeState(State::PICKED_UP);
    outOfBounds = false;
    corner = false;
    wasOverCorner = false;
    Drivetrain::stop();
    return;
  } else if (LineSensor::detections > 12) {
    SuperState::changeState(State::INIT_FAILED);
  } else {
    SuperState::changeState(State::READY);
  }
  if ((data.bestI != 0 || data.bestJ != 0) && data.bestI != data.bestJ) {
    // Serial.print("HI");
    normalLineSensor = LineSensor::lineSensorVectors[data.bestI] + LineSensor::lineSensorVectors[data.bestJ];

    if (corner) {
      normalLineSensor = outaHere.toVector();
    }

    // if (LineSensor::lineSensorGroups == 3) {
    //   Serial.print("#3 ");
    //   normalLineSensor = previousNormalLineSensor;
    // }
    
    // Serial.print(data.bestI);
    // Serial.print(" ");
    // Serial.print(data.bestJ);

    // Serial.print(" NLS: ");
    // Serial.print(normalLineSensor.toAngleDeg().value);

    // Serial.print(" PNLS: ");
    // Serial.print(previousNormalLineSensor.toAngleDeg().value);

    if (previousNormalLineSensor.x != -5) {
      // Serial.print(" HERE ");
      float angleDiff = normalLineSensor.toAngleDeg().angleDifference(previousNormalLineSensor.toAngleDeg()).value;
      Serial.print("angle diff: ");
      Serial.print(normalLineSensor.toAngleDeg().value);
      Serial.print(" ; ");
      Serial.print(previousNormalLineSensor.toAngleDeg().value);
      Serial.print(" ");
      if (abs(angleDiff) > 90) {
        outOfBounds = !outOfBounds;
      }
    } else if (corner == true) {
      normalLineSensor = outaHere.toVector();
    }

    Serial.print("Out: ");
    Serial.println(outOfBounds ? "T" : "F");
    Serial.print("Corner: ");
    Serial.println(corner ? "T" : "F");
    // Serial.println("---");


    if(LineSensor::detections > 12) {
      // outaHere = previousNormalLineSensor.toAngleRad();
      // normalLineSensor = previousNormalLineSensor;
      corner = true;
      wasOverCorner = true;
      sensor_group_data owo = LineSensor::getLargestGap();
      Serial.print(owo.bestI);
      Serial.print(" ");
      Serial.print(LineSensor::lineSensorVectors[owo.bestI].toString());
      Serial.print(",");
      Serial.print(owo.bestJ);
      Serial.print(" ");
      Serial.print(LineSensor::lineSensorVectors[owo.bestJ].toString());
      Serial.print(";");
      Serial.print(owo.closestAngle);
      Serial.print(" : ");
      if (owo.closestAngle == 12) {
        outaHere = outaHere;
      } else {
        outaHere = (LineSensor::lineSensorVectors[owo.bestI] + LineSensor::lineSensorVectors[owo.bestJ]).toAngleRad();
      }
      normalLineSensor = outaHere.toVector();
      // previousNormalLineSensor = outaHere.toVector();
    } else {
      corner = false;
      outaHere = normalLineSensor.toAngleRad();
    }
    previousNormalLineSensor = normalLineSensor;


    Vector temp = outaHere.toVector();
    // Serial.print(" Away: ");
    if (outOfBounds == false && !corner) {
      Serial.println("FLIPPING");
      outaHere = temp.flip().toAngleRad();
    } else {
      increment++;
      if (wasOverCorner && increment > 0) {
        increment = 0;
        wasOverCorner = false;
      }
    }
    // outaHere = outOfBounds ? outaHere : outaHere + AngleRad(3.14);
    Serial.println(outaHere.toDeg().value);

    // Serial.print(" PPV: ");
    // Serial.print(projectedOne.toDeg().value);
    // Serial.print(" ");
    // Serial.print(projectedTwo.toDeg().value);
    // Serial.print(" ");

  } else {
    previousNormalLineSensor = Vector(-5, -5);
    corner = false;
  }

  if ((data.bestI == 0 && data.bestJ == 0) || (data.bestI == data.bestJ)) {
    if (outOfBounds) {
        // Serial.print("OUT OF BOUNDS: ");
        // Serial.println(outaHere.toDeg().value);
        Drivetrain::drive(outaHere.value, power, 0);
        return;
    } else {
      // Serial.print("WE GOOD: ")
      Drivetrain::drive(finalAngle.cartesianAngle().toRad().value, power, rotation);
      return;
    }
  } else if (lineAvoidExtreme) {

    Drivetrain::drive(outaHere.value, power, rotation);

    return;
  }


  if (corner) {
    float angleDiff = outaHere.toDeg().angleDifference(finalAngle.cartesianAngle()).value;
    if (fabs(angleDiff) < 45) {
      Drivetrain::drive(finalAngle.cartesianAngle().toRad().value, power, 0);
    } else if (fabs(angleDiff) < 160) {
      Drivetrain::stop();
    } else {
      Drivetrain::drive(outaHere.value, power, rotation);
      // AngleRad projectedAngleOne = (outaHere + AngleRad(3.14/5));
      // AngleRad projectedAngleTwo = (outaHere + AngleRad(-3.14/5));
      // float projectedAngleDiffOne = projectedAngleOne.toDeg().angleDifference(cartesianBallAngle).value;
      // float projectedAngleDiffTwo = projectedAngleTwo.toDeg().angleDifference(cartesianBallAngle).value;
      // if (fabs(projectedAngleDiffOne) > fabs(projectedAngleDiffTwo)) {
      //   Drivetrain::drive(projectedAngleTwo.value, power, rotation);
      // } else {
      //   Drivetrain::drive(projectedAngleOne.value, power, rotation);
      // }

    }
  } else if ((data.bestI != 0 || data.bestJ != 0) && (data.bestI != data.bestJ)) {
    float angleDiff = outaHere.toDeg().angleDifference(finalAngle.cartesianAngle()).value;
    float ballAngleDiff = outaHere.toDeg().angleDifference(cartesianBallAngle).value;
    if (fabs(angleDiff) < 90) {
      // Serial.print("Following ball (on line) ");
      // Serial.println(finalAngle.cartesianAngle().value);
      Drivetrain::drive(finalAngle.cartesianAngle().toRad().value, power, 0);
    } else if (abs(ballAngleDiff) > 160 && BallSensor::ball_mag > .8) {
      // Serial.println("STOP BITCH");
      Drivetrain::stop();
    } else {
      // Serial.print("Proj Line Follow: ");
      AngleRad projectedAngleOne = (outaHere + AngleRad(3.0/2));
      AngleRad projectedAngleTwo = (outaHere + AngleRad(-3.0/2));
      float projectedAngleDiffOne = projectedAngleOne.toDeg().angleDifference(cartesianBallAngle).value;
      float projectedAngleDiffTwo = projectedAngleTwo.toDeg().angleDifference(cartesianBallAngle).value;
      if (fabs(projectedAngleDiffOne) > fabs(projectedAngleDiffTwo)) {
        Drivetrain::drive(projectedAngleTwo.value, power, rotation);
        // Serial.println(projectedAngleTwo.toDeg().value);
      } else {
        Drivetrain::drive(projectedAngleOne.value, power, rotation);
        // Serial.println(projectedAngleOne.toDeg().value);
      }
        
    }
  }
  


  // delay(1000/100);
}