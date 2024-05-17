#include "drivetrain.h"

Motor* Drivetrain::frontLeftMotor = nullptr;
Motor* Drivetrain::frontRightMotor = nullptr;
Motor* Drivetrain::backRightMotor = nullptr;
Motor* Drivetrain::backLeftMotor = nullptr;

Vector Drivetrain::currentVector = Vector(0, 0);
unordered_map<int, std::pair<VectorPriority, Vector>>
    Drivetrain::vectorMap =
        unordered_map<int, std::pair<VectorPriority, Vector>>();
float Drivetrain::rotation = 0;
float Drivetrain::power = 0;

void Drivetrain::setup() {
    Drivetrain::frontLeftMotor = new Motor(DrivetrainConstants::bluePin3, DrivetrainConstants::greenPin3, true);
    Drivetrain::frontRightMotor = new Motor(DrivetrainConstants::bluePin4, DrivetrainConstants::greenPin4);
    Drivetrain::backRightMotor = new Motor(DrivetrainConstants::bluePin, DrivetrainConstants::greenPin);
    Drivetrain::backLeftMotor = new Motor(DrivetrainConstants::bluePin2, DrivetrainConstants::greenPin2, true);
    Drivetrain::frontLeftMotor->setup();
    Drivetrain::frontRightMotor->setup();
    Drivetrain::backRightMotor->setup();
    Drivetrain::backLeftMotor->setup();
}

void Drivetrain::setVector(AlgorithmName name, Vector vec) {
    Drivetrain::vectorMap[name].second = vec;
}

void Drivetrain::setPriority(AlgorithmName name, VectorPriority priority) {
    Drivetrain::vectorMap[name].first = priority;
}

void Drivetrain::driveByVectors() {
    currentVector = Vector(0, 0);
    bool overidden = false;
    for (auto& process : Drivetrain::vectorMap) {
        switch (process.second.first) {
            case VectorPriority::LOW_PRIORITY:
                currentVector += process.second.second * .5;
                break;
            case VectorPriority::MEDIUM_PRIORITY:
                currentVector += process.second.second * 1;
                break;
            case VectorPriority::HIGH_PRIORITY:
                currentVector += process.second.second * 2;
                break;
            case VectorPriority::OVERRIDE_PRIORITY: 
                if (!process.second.second.isZero()) {
                    currentVector = process.second.second;
                    overidden = true;
                }
                break;
            
        }
        if (overidden) {
            break;
        }
    }

    // Serial.println(overidden);

    if (currentVector.isZero()) {
        Drivetrain::rotate(Drivetrain::rotation);
    }

    AngleRad angle = currentVector.toAngleRad();

    Serial.print("Drive angle: ");
    Serial.println(angle.toDeg().value);
    Serial.println("Power: " + String(Drivetrain::power));

    Drivetrain::drive(angle.value, SuperState::currentState == State::PICKED_UP ? 0 : 1.0, rotation);
}

/**
 * Rotates the robot in place
 * 
 * @param rotation The robot rotates counterclockwise or clockwise [-1, 1].
 * @return void
*/
void Drivetrain::rotate(double rotation) {
    Drivetrain::frontLeftMotor->writeAndSetPower(rotation);
    Drivetrain::frontRightMotor->writeAndSetPower(-rotation);
    Drivetrain::backRightMotor->writeAndSetPower(-rotation);
    Drivetrain::backLeftMotor->writeAndSetPower(rotation);
}

float PRIDE_PI = 3.14159;

/**
 * Drives the drivetrain
 * 
 * @param angle_rad The direction the robot will travel. 0 is east [0, 2pi].
 * @param power How fast the robot travels in the direction from [0, 1].
 * @param rotation The robot rotates counterclockwise or clockwise [-1, 1].
 * @return void
*/
void Drivetrain::drive(double angle_rad, double power, double rotation) {
    // Calculate x and y components from the angle

    // to fix front and back being reversed
    // angle_rad -= MathConstants::PRIDE_PI / 2;  // Subtract π/2 radians (90 degrees)

    double x = cos(angle_rad);
    double y = sin(angle_rad);

    // Calculate individual motor speeds based on desired x, y, rotation, and power
    double cos42 = cos(42.0 * MathConstants::PRIDE_PI / 180.0);
    double sin42 = sin(42.0 * MathConstants::PRIDE_PI / 180.0);
    float velFL = ((cos42 * y + sin42 * x) + rotation);
    float velFR = ((cos42 * y - sin42 * x) - rotation);
    float velBR = ((cos42 * y + sin42 * x) - rotation);
    float velBL = ((cos42 * y - sin42 * x) + rotation);

    float velMax = max(max(abs(velFL), abs(velFR)), max(abs(velBR), abs(velBL)));

    velFL = velFL / velMax;
    velFR = velFR / velMax;
    velBR = velBR / velMax;
    velBL = velBL / velMax;

    velFL *= power;
    velFR *= power;
    velBR *= power;
    velBL *= power;

    velFL = fclamp(velFL, -1, 1);
    velFR = fclamp(velFR, -1, 1);
    velBR = fclamp(velBR, -1, 1);
    velBL = fclamp(velBL, -1, 1);


    velFL = (float)(lround(velFL * 1000) / 1000.0);
    velFR = (float)(lround(velFR * 1000) / 1000.0);
    velBR = (float)(lround(velBR * 1000) / 1000.0);
    velBL = (float)(lround(velBL * 1000) / 1000.0);


    Drivetrain::frontLeftMotor->writeAndSetPower(velFL);
    Drivetrain::frontRightMotor->writeAndSetPower(velFR);
    Drivetrain::backLeftMotor->writeAndSetPower(velBL);
    Drivetrain::backRightMotor->writeAndSetPower(velBR);
}

const float wheelAngleRad = 42.0 * M_PI / 180.0;
double cos42 = cos(42.0 * M_PI / 180.0);
double sin42 = sin(42.0 * M_PI / 180.0);

void Drivetrain::vectorDrive(Vector vec, float power, float rotation) {
    // Serial.print(" Going: ");
    // Serial.print(vec.toAngleDeg().value);
    // Serial.print(" Power: ");
    // Serial.println(power);
    // return;
    if (vec.isZero()) {
        Drivetrain::rotate(rotation);
        return;
    }
    Drivetrain::drive(vec.toAngleRad().value, power, rotation);
}

void Drivetrain::stop() {
    Drivetrain::frontLeftMotor->stop();
    Drivetrain::frontRightMotor->stop();
    Drivetrain::backRightMotor->stop();
    Drivetrain::backLeftMotor->stop();
}

void Drivetrain::print() {
    Serial.println("Front Left Motor: " + String(Drivetrain::frontLeftMotor->currentVelocity));
    Serial.println("Front Right Motor: " + String(Drivetrain::frontRightMotor->currentVelocity));
    Serial.println("Back Right Motor: " + String(Drivetrain::backRightMotor->currentVelocity));
    Serial.println("Back Left Motor: " + String(Drivetrain::backLeftMotor->currentVelocity));
}