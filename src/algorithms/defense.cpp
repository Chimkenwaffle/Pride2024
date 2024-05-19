#include "defense.h"

AngleRad DefenseAlgorithm::angleOffset = 0;
AngleRad DefenseAlgorithm::ballAngleError = 0;
float DefenseAlgorithm::distanceToBack = 33.0;
float distanceFront = 1.5;
float distanceBack = 0.8;
int timeUpdating = 0;
int timer = 0;
float initialDistanceToBack = DefenseAlgorithm::distanceToBack;
int maxIncrement = 1000;
int DefenseAlgorithm::increment = maxIncrement;
fPIDController ballMagControler = fPIDController(0.1, 0, 55);

// x-axis
fPIDController DefenseAlgorithm::ballAngleController = 
    fPIDController(1.75,0.00,.02);

// rotation
fPIDController DefenseAlgorithm::rotationController =
    fPIDController(.7,0, 0.2);

// y-axis
fPIDController wallController = fPIDController(0.7, 0, 5.2);

bool noDirectionYPrevious = false;

void DefenseAlgorithm::init() {
    Serial.println("Defense Algorithm Initialized");
}

void DefenseAlgorithm::loop(int threadID) {
    DefenseAlgorithm::angleOffset = Gyro::getHeading();
    // Serial.println(Gyro::getHeading().toDeg().value);
    
    float rotation = rotationController.update(DefenseAlgorithm::angleOffset.value);
    rotation = fclamp(rotation, -0.3, 0.3);

    float fYError = 0;
    if (fabs(rotation) < .2 && (LocationSensor::frBack < distanceToBack - distanceBack || LocationSensor::frBack > distanceToBack + distanceFront)) {
        fYError = -(LocationSensor::frBack - distanceToBack) / distanceToBack;

        fYError = wallController.update(fYError);

        fYError = fclamp(fYError, -1, 1);
    }

    // Serial.print("FYERROR: ");
    // Serial.println(fYError);
    // Serial.print("FRBACK: ");
    // Serial.println(LocationSensor::frBack);

    BallSensor::read();
    BallSensor::getBallAngleVector(false);
    AngleRad ballAngle = AngleRad(BallSensor::ball_angle_rad);
    // if we need to calculate changed rotation based on where robot is
    // AngleRad ballAngle = AngleRad(BallSensor::ball_angle_rad) + angleOffset;
    while (ballAngle.value < -180.0) {
        ballAngle = ballAngle + AngleRad(2 * MathConstants::PRIDE_PI);
    }
    DefenseAlgorithm::ballAngleError = ballAngle.forwardAngle();

    // Serial.print("Ball Angle ERROR: ");
    // Serial.println(DefenseAlgorithm::ballAngleError.value);

    // Serial.print("Ball Angle: ");
    // Serial.println(ballAngle.toDeg().value);

    float ballPower = DefenseAlgorithm::ballAngleError.value;
    ballPower = ballAngleController.update(ballPower);
    ballPower = fclamp(ballPower, -1, 1);

    LineSensor::read();
    LineSensor::preProcessLineSensors();
    sensor_group_data data = LineSensor::calculateLineSensorGroups();
    Vector direction = Vector(0, 0);
    Vector normalLineSensor = Vector(0 , 0);
    // Serial.println((String)(data.bestI != 0 || data.bestJ != 0));
    // Serial.print("Tolerance: ");
    // Serial.print(90.0 - ballAngle.toDeg().value);

    float angleSpread = 10 / BallSensor::ball_mag;
    angleSpread = min(angleSpread, 15);

    // Serial.print("Ball Magnitude: ");
    // Serial.println(BallSensor::ball_mag);
    // Serial.print("Angle Spread: ");
    // Serial.println(angleSpread);

    bool corner = false;
    bool conditionThree = false;
    bool conditionOne = false;
    bool conditionTwo = false;
    // float degreeBehind = 45.0;
    // on line && ball in front
    if (fabs(90 - ballAngle.toDeg().value) < angleSpread && (data.bestI != 0 || data.bestJ != 0)) {
        // Serial.println("Condition 1");
        conditionOne = true;
        direction = Vector(0, 0);
        ballPower = 0;
        fYError = 0;
        // when you are on line && ball is in field && the ball isnt directly in front of you
    } else if ((data.bestI != 0 || data.bestJ != 0) && fabs(90.0 - ballAngle.toDeg().value) > angleSpread ) {
        // Serial.println("Condition 2");
        conditionTwo = true;
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

        // Serial.print("Ball Angle: ");
        // Serial.println(ballAngle.toDeg().value);

        // Serial.print("Right Between Angle: ");
        // Serial.println(fabs(rightBetween.toDeg().value));
        // Serial.print("Left Between Angle: ");
        // Serial.println(fabs(leftBetween.toDeg().value));

        if (fabs(rightBetween.value) < fabs(leftBetween.value)) {
            direction = rightAngle.toVector();
            // direction = Vector(angleOffset, 0);
        } else {
            direction = leftAngle.toVector();
            // direction = Vector(-1, 0);
        }

        if (fabs(direction.y) > fabs(direction.x) + .3 && direction.y < 0) {
            direction.y = 0;
        } else if (fabs(direction.y) > fabs(direction.x) + .3 && direction.y > 0 ) {
            // direction.y = -.2;
        }

        float edgecaseTolerance = 15.0;
        float triggerTolerance = 25.0;
        distanceToBack = initialDistanceToBack;
        // edge case when on corners
        if (normalLineSensor.x * normalLineSensor.y > 0.1 && LocationSensor::rightGood && (ballAngle.toDeg().value < 90 && ballAngle.toDeg().value > -90 - triggerTolerance)) {
            // Serial.println("Right Side Case");
            // quadrant 1
            if (normalLineSensor.x > 0) {
                if(normalLineSensor.toAngleDeg().value < 90 - edgecaseTolerance) {
                    direction = Vector(0, 0);
                    // distanceToBack = 33.5;
                    ballPower = 0;
                    corner = true;
                }
            // quadrant 3
            } else {
                if (normalLineSensor.toAngleDeg().value < -90 - edgecaseTolerance) {
                    direction = Vector(0, 0);
                    // distanceToBack = 33.5;
                    ballPower = 0;
                    corner = true;
                }
            }
            increment = 0;

        } else if (normalLineSensor.x * normalLineSensor.y < -0.1 && LocationSensor::leftGood && (ballAngle.toDeg().value > 90 || ballAngle.toDeg().value < -90 + triggerTolerance)) {
            // Serial.println("Left Side Case");
            // quadrant 4
            if (normalLineSensor.x > 0) {
                if(normalLineSensor.toAngleDeg().value > -90 + edgecaseTolerance) {
                    direction = Vector(0, 0);
                    // distanceToBack = 33.5;
                    ballPower = 0;
                    corner = true;
                }
            // quadrant 2
            } else {
                if (normalLineSensor.toAngleDeg().value > 90 + edgecaseTolerance) {
                    direction = Vector(0, 0);
                    // distanceToBack = 33.5;
                    ballPower = 0;
                    corner = true;
                }
            }
            increment = 0;

            if (fabs(direction.y) > fabs(direction.x) && direction.y < 0) {
                // Serial.println("MORE Y THAN X");
                direction = Vector(0, 0);
                ballPower = 0;
            }
        } 
        // else if (fabs(data.closestAngle) < 166) {
            // if (normalLineSensor.y < 0) {
            //     direction += normalLineSensor*.4;
            //     direction.toUnitVector();
            // } 
            // if (normalLineSensor)
            
        // }

        if (direction.y > direction.x + .2 && direction.y > 0) {
            direction.y *= .5;
        }

        fYError = 0;

        // off the line
    } else {
      
        conditionThree = true;
        // not on line and not on edge
        if (!LocationSensor::leftGood && !LocationSensor::rightGood) {
            // When ball is on 
            // Serial.println("Neither Good");
            if (DefenseAlgorithm::ballAngleError.value > 0) {
                if (LocationSensor::frBack > distanceToBack + distanceFront) {
                    direction = Vector(1, -0.5);
                } else if (LocationSensor::frBack < distanceToBack - distanceBack) {
                    direction = Vector (1, 0.5);
                }
            } else if (DefenseAlgorithm::ballAngleError.value < 0) {
                if (LocationSensor::frBack > distanceToBack + distanceFront) {
                    direction = Vector(-1, -0.5);
                } else if (LocationSensor::frBack < distanceToBack - distanceBack) {
                    direction = Vector (-1, 0.5);
                }
            }
        } else if (LocationSensor::leftGood) {
            // Serial.println("Left Good");
            if (LocationSensor::frBack > distanceToBack + distanceFront) {
                direction = Vector(1, -0.5);
            } else if (LocationSensor::frBack < distanceToBack - distanceBack) {
                direction = Vector(1, 0.5);
            } else {
                direction = Vector(1, 0);
            }
        } else if (LocationSensor::rightGood) {
            // Serial.println("Right Good");
            if(LocationSensor::frBack > distanceToBack + distanceFront) {
                direction = Vector(-1, -0.5);
            } else if (LocationSensor::frBack < distanceToBack - distanceBack) {
                direction = Vector(-1, 0.5);
            } else {
                direction = Vector(-1, 0);
            }
        }
    }

    // Serial.print("Ball Power: ");
    // Serial.println(ballPower);


    
    // if (LocationSensor::frBack > distanceToBack + 1.5 && direction.y != 0 && direction.x != 0) {
    //     direction.y -= .15;
    //     fYError *= 1.2;
    // } 
    
    if (fYError < 0) {
        fYError *= .5;
    }


    bool ready = false;
    float power = 0;
    if (LineSensor::checkIfPickedUp()) {
        SuperState::changeState(State::PICKED_UP);
    } else {
        if (BallSensor::ball_mag > 0.2) {
            SuperState::changeState(State::READY);
            ready = true;
        }

        power = sqrt(rotation * rotation + fYError * fYError + ballPower * ballPower * (1 + BallSensor::ball_mag));
        power = fmin(1, power);

        if (ready) {
            if(power > 0.95) {
                SuperState::changeState(State::FULL_POWER);
            }
        }
    }
    AngleDeg normalLineDeg = AngleDeg(normalLineSensor.toAngleDeg().value);
    if (normalLineDeg.value < 0) {
        normalLineDeg += 360;
    }

    float distanceFromEdge = 20;
    float edgeAngleOffset = 35;
    if((LocationSensor::frRight > distanceFromEdge && LocationSensor::frLeft > distanceFromEdge) || ((normalLineDeg.value > edgeAngleOffset && normalLineDeg.value < 360 - edgeAngleOffset) && (normalLineDeg.value < 180 - edgeAngleOffset || normalLineDeg.value > 180 + edgeAngleOffset)) || corner) {
        timeUpdating = millis();
    }

    // Serial.print("Normal Vector ");
    // Serial.println(normalLineDeg.value);
    // Serial.print("frLeft:");
    // Serial.println(LocationSensor::frLeft);
    
    int currentTime = millis();
    if (timeUpdating + 500 < currentTime) {
        if(LocationSensor::backGood) {
            if (LocationSensor::rightGood) {
                direction = Vector(-1, 0);
            } else if (LocationSensor::leftGood) {
                direction = Vector(1, 0);
            }
        } else if (!LocationSensor::backGood) {
            if (LocationSensor::rightGood) {
                direction = Vector(-1, -1);
            } else if (LocationSensor::leftGood) {
                direction = Vector(1, -1);
            }
        }
    } 

    // ball on left or right
    if(ballAngle.toDeg().value > 90.0) {
        SuperState::changeState(State::INIT_FAILED);
    } else if(ballAngle.toDeg().value < 90.0) {
        SuperState::changeState(State::PICKED_UP);
    }
    // Serial.print("Ball Magnitude: ");
    // Serial.println(BallSensor::ball_mag);
    
    if(conditionOne) {
        SuperState::changeState(State::INIT_FAILED);
    } 
    if(conditionTwo) {
        SuperState::changeState(State::PICKED_UP);
    }
    if(conditionThree) {
        SuperState::changeState(State::READY);
    }

    if (LineSensor::checkIfPickedUp()) {
        power = 0;
    }

    if (corner) {
        SuperState::changeState(State::WAITING_TO_CALIBRATE);
    }
    
    // Serial.print(" Direction: ");
    // Serial.println(direction.toAngleDeg().value);

    Drivetrain::vectorDrive(Vector(0, fYError) + direction, power, rotation);
    // Serial.println();
    // Serial.println("---");
}
