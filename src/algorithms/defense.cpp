#include "defense.h"

AngleRad DefenseAlgorithm::angleOffset = 0;
AngleRad DefenseAlgorithm::ballAngleError = 0;

// x-axis
fPDController DefenseAlgorithm::ballAngleController = 
    fPDController(2, 4);

// rotation
fPDController DefenseAlgorithm::rotationController =
    fPDController(.36, 0.2);

// y-axis
fPDController wallController = fPDController(4, 5);

void DefenseAlgorithm::init() {
    Serial.println("Defense Algorithm Initialized");
}

void DefenseAlgorithm::loop(int threadID) {
    DefenseAlgorithm::angleOffset = Gyro::getHeading();
    
    float rotation = rotationController.update(DefenseAlgorithm::angleOffset.value);
    rotation = fclamp(rotation, -0.3, 0.3);

    float fYError = 0;
    if (fabs(rotation) < .2) {
        fYError = -(LocationSensor::frBack - 33.0) / 30.0;
        // Serial.println(fYError);

        fYError = wallController.update(fYError);

        fYError = fclamp(fYError, -0.35, 0.35);
    }
    Serial.print("FYERROR: ");
    Serial.println(fYError);

    BallSensor::read();
    BallSensor::getBallAngleVector(false);
    AngleRad ballAngle = AngleRad(BallSensor::ball_angle_rad);
    DefenseAlgorithm::ballAngleError = ballAngle.forwardAngle();


    // Serial.print("Angle: ");
    // Serial.println(ballAngle.toDeg().value);

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

    } else {
        ballPower = 0;
    }

    Serial.print("Ball Power: ");
    Serial.println(ballPower);

    float power = sqrt(rotation * rotation + fYError * fYError + ballPower * ballPower);
    power = fmin(1, power);

    if (LineSensor::checkIfPickedUp()) {
        power = 0;
    } else {
        SuperState::changeState(State::READY);
    }
    Drivetrain::vectorDrive(Vector(0, fYError) + direction, power, rotation);
    Serial.println("---");
}

// void DefenseAlgorithm::loop(int threadID) {
//     DefenseAlgorithm::angleOffset = Gyro::getHeading();
//     BallSensor::read();
//     BallSensor::getBallAngleVector(false);
//     DefenseAlgorithm::ballAngleError = AngleRad(BallSensor::ball_angle_rad).forwardAngle();

//     float rotation = rotationController.update(DefenseAlgorithm::angleOffset.value);
//     rotation = fclamp(rotation, -0.3, 0.3);

//     float xError = DefenseAlgorithm::ballAngleError.value;
//     xError = ballAngleController.update(xError);
//     xError = fclamp(xError, -1, 1);

//     float fYError = 0;
//     if (fabs(rotation) < .2) {

//         // threads.suspend(threadID);
//         int yError = -(LocationSensor::toBack - 31);
//         // Serial.println(LocationSensor::toBack);
//         // threads.restart(threadID);
//         fYError = float(yError) / 30.0;
//         fYError = wallController.update(fYError);
//         if (fYError > .35) {
//             fYError = .35;
//         } else if (fYError < -.35) {
//             fYError = -.35;
//         }
//     }
//     LineSensor::read();
//     LineSensor::preProcessLineSensors();
//     // LineSensor::calculateLineSensorGroups();
//     // bool onLine = LineSensor::checkIfOnLine();
//     // Serial.println("On Line: " + String(onLine));
//     if (LocationSensor::leftGood) {
//         if (LocationSensor::toLeft < 55) {
//             if (xError > 0) {
//                 xError += .2;
//             } else {
//                 xError = .2;
//             }
//         } else if (xError < 0) {
//             xError = 0;
//         }
//     } else if (LocationSensor::rightGood) {
//         if (LocationSensor::toRight < 55) {
//             if (xError < 0) {
//                 xError += -.2;
//             } else {
//                 xError = -.2;
//             }
//             // xError += -.2;
//         } else if (xError > 0) {
//             xError = 0;
//         }
//     } 
//     // Serial.println("Y Error: " + String(fYError) + " Rotation: " + String(rotation) + " xError: " + String(xError)); 
//     // Serial.print("Ball Mag: " + String(BallSensor::ball_mag) + " Ball Angle: " + String(BallSensor::ball_angle_deg.value));

//     if (xError == 0 && BallSensor::ball_mag < .3) {
//         // we not moving at all rn ?!?!
//         // center ourselves in the field on the x axis
//         int toLeft = LocationSensor::frLeft;
//         int toRight = LocationSensor::frRight;

//         // Serial.println("To Left: " + String(toLeft) + " To Right: " + String(toRight));

//         if (abs(toLeft - toRight) > 5) {
//             // Serial.println("NOT CENTERED");
//             // ok we are not centered
//             if (toLeft > toRight) {
//                 xError = -.2;
//             } else {
//                 xError = .2;
//             }
//         }
//     }

//     if (LineSensor::checkIfPickedUp()) {
//         SuperState::changeState(State::PICKED_UP);
//         Drivetrain::stop();
//         return;
//     } else {
//         SuperState::changeState(State::READY);
//     }
//     // Serial.println("Error: " + String(DefenseAlgorithm::ballAngleError.value));

//     // Vector lineSensorVector = LineSensor::simpleVectorSum();
//     // LineSensor::calculateLineSensorGroups();
//     // float magnitude = fclamp((lineSensorVector * (179.0 - float(LineSensor::lineSensorGroupData.closestAngle))/(5*179.0)).magnitude(), -.25, .25);
//     // Serial.println("Line Sensor Vector: " + String(lineSensorVector.toAngleDeg().value) + " : " + String(magnitude));
//     float power = fmin(1, fYError*fYError + rotation*rotation + xError*xError);
//     // Serial.println("Power: " + String(power));
//     if (power < .1) {
//         power = 0;
//     }
//     // Serial.println(" Power: " + String(power) + " XError: " + String(xError));
//     Drivetrain::vectorDrive(Vector(xError, fYError), power, rotation);
// }

