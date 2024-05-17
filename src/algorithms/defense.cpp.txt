#include "defense.h"

AngleRad DefenseAlgorithm::angleOffset = 0;
AngleRad DefenseAlgorithm::ballAngleError = 0;
float DefenseAlgorithm::distanceToBack = 33.0;

fPDController ballMagControler = fPDController(0.1, 55);

// x-axis
fPDController DefenseAlgorithm::ballAngleController = 
    fPDController(14, 9.5);

// rotation
fPDController DefenseAlgorithm::rotationController =
    fPDController(.46, 0.2);

// y-axis
fPDController wallController = fPDController(4.5, 3.5);

void DefenseAlgorithm::init() {
    Serial.println("Defense Algorithm Initialized");
}

void DefenseAlgorithm::loop(int threadID) {
    DefenseAlgorithm::angleOffset = Gyro::getHeading();
    
    float rotation = rotationController.update(DefenseAlgorithm::angleOffset.value);
    rotation = fclamp(rotation, -0.3, 0.3);

    float fYError = 0;
    if (fabs(rotation) < .2) {
        fYError = -(LocationSensor::frBack - distanceToBack) / 30.0;

        fYError = wallController.update(fYError);

        fYError = fclamp(fYError, -1, 1);
    }
    Serial.print("FYERROR: ");
    Serial.println(fYError);

    BallSensor::read();
    BallSensor::getBallAngleVector(false);
    AngleRad ballAngle = AngleRad(BallSensor::ball_angle_rad);
    DefenseAlgorithm::ballAngleError = ballAngle.forwardAngle();

    Serial.print("Ball Angle ERROR: ");
    Serial.println(DefenseAlgorithm::ballAngleError.value);

    if (fabs(DefenseAlgorithm::ballAngleError.value) < 0.05) {
        Serial.println("HERE");
        DefenseAlgorithm::ballAngleError = 0;
    }


    Serial.print("Ball Angle: ");
    Serial.println(ballAngle.toDeg().value);

    float ballPower = DefenseAlgorithm::ballAngleError.value;
    ballPower = ballAngleController.update(ballPower);
    ballPower = fclamp(ballPower, -1, 1);

    if (BallSensor::ball_mag < 0.3) { 
        ballPower = 0;
    }

    LineSensor::read();
    LineSensor::preProcessLineSensors();
    sensor_group_data data = LineSensor::calculateLineSensorGroups();
    Vector direction = Vector(0, 0);
    Vector normalLineSensor = Vector(0 , 0);
    // Serial.println((String)(data.bestI != 0 || data.bestJ != 0));
    if (data.bestI != 0 || data.bestJ != 0) {
        normalLineSensor = LineSensor::lineSensorVectors[data.bestI] + LineSensor::lineSensorVectors[data.bestJ];

        // Serial.print("Normal Angle: ");
        // Serial.println(normalLineSensor.toAngleDeg().value);

        AngleRad rightAngle = normalLineSensor + AngleRad(3.14 / 2.0);
        AngleRad leftAngle = normalLineSensor + AngleRad(-3.14 / 2.0);
        
        // Serial.print("Right Angle: ");
        // Serial.println(rightAngle.toDeg().value);
        // Serial.print("Left Angle: ");
        // Serial.println(leftAngle.toDeg().value);

        AngleRad rightBetween = rightAngle.angleDifference(ballAngle);
        AngleRad leftBetween = leftAngle.angleDifference(ballAngle);

        // Serial.print("Right Between Angle: ");
        // Serial.println(rightBetween.toDeg().value);
        // Serial.print("Left Between Angle: ");
        // Serial.println(leftBetween.toDeg().value);

        if (fabs(rightBetween.value) < fabs(leftBetween.value)) {
            direction = rightAngle.toVector();
        } else {
            direction = leftAngle.toVector();
        }

        Serial.print("Direction: ");
        Serial.println(direction.toAngleDeg().value);
        distanceToBack = 33.0;
    } else {
        // somehow we are not on the line :skull:
        // ballPower = 0;
        // distanceToBack = 25.0;
        if (LocationSensor::leftGood) {
            // left side D:
            direction = Vector(1, 0);
            ballPower = 0.5;
        } else if (LocationSensor::rightGood) {
            // right side D:
            direction = Vector(-1, 0);
            ballPower = 0.5;
        } else {
            // we in the middle :SKULL:
            Vector right = Vector(2, 0);
            Vector left = Vector(-2, 0);
            if (LocationSensor::toBack < distanceToBack) {
                right.y = .1;
                left.y = .1;
            } else {
                right.y = -.1;
                left.y = -.1;
            }

            AngleRad leftDifference = left.toAngleRad().angleDifference(ballAngle);
            AngleRad rightDifference = right.toAngleRad().angleDifference(ballAngle);

            if (fabs(leftDifference.value) < fabs(rightDifference.value)) {
                direction = left;
            } else {
                direction = right;
            }
        }
    }

    float ballAugment = fabs(ballMagControler.update(BallSensor::ball_mag));
    ballAugment = fclamp(ballAugment, 0, 1);
    Serial.print("Ball Augment: ");
    Serial.println(ballAugment);
    

    Serial.print("Ball Power: ");
    Serial.println(ballPower);
    if (ballPower < 0) {
        ballPower *= 2.0;
    } else {
        ballPower *= 1.4;
    }
    ballPower = fclamp(ballPower, -1, 1);
    // if (LocationSensor::toBack < 25) {
    //     Serial.println("TOO CLOSE");
    //     ballPower = 0;
    //     ballAugment = 0;
    //     direction = Vector(0,0);
    // }

    if (fabs(direction.y) > fabs(direction.x) && direction.y < 0) {
        Serial.println("MORE Y THAN X");
        direction = Vector(0, 0);
        ballPower = 0;
    }


    float power = sqrt(rotation * rotation + fYError * fYError + ballPower * ballPower * (1+ ballAugment * ballAugment));
    power = fmin(1, power);

    if (LineSensor::checkIfPickedUp()) {
        power = 0;
    } else {
        SuperState::changeState(State::READY);
    }
    Drivetrain::vectorDrive(Vector(0, fYError) + direction, power, rotation);
    Serial.println("---");
}