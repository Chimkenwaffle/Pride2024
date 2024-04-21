#include <PrideMath.h>
#include <math.h>

const float PRIDE_PI = 3.14159;

namespace PrideUtils {

/**
 * @brief Construct a new Angle Deg:: Angle Deg object
*/
AngleDeg::AngleDeg(float value) {
    this->value = value;
}

/**
 * @brief Translates angle from cartesian plane to forward plane
 * 
 * @return AngleDeg 
*/
AngleDeg AngleDeg::forwardAngle() {
    float angle = fmod((450.0 - this->value), 360.0);
    if (angle > 180.0) {
        angle -= 360.0;
    }
    return AngleDeg(angle);
}
/**
 * @brief Translates angle from forward plane to cartesian plane
 * 
 * @return AngleDeg 
*/
AngleDeg AngleDeg::cartesianAngle() {
    float angle = fmod((360.0 - this->value + 90.0), 360.0);
    if (angle > 180.0) {
        angle -= 360.0;
    }
    return AngleDeg(angle);
}

AngleDeg AngleDeg::angleDifference(AngleDeg other) {
    auto angle1 = this->value;
    auto angle2 = other.value;
    if (angle1 < 0)
        angle1 += 360.0;
    if (angle2 < 0)
        angle2 += 360.0;
    float diff = fmod((angle2 - angle1 + 180), 360) - 180;
    return AngleDeg{diff < -180 ? diff + 360 : diff};
}

AngleRad AngleDeg::toRad() {
    return AngleRad(this->value * PRIDE_PI / 180);
}

Vector AngleDeg::toVector() {
    float x = cos(this->toRad().value);
    float y = sin(this->toRad().value);
    return Vector(x, y);
}

String AngleDeg::toString() {
    return "AngleDeg: " + String(this->value);
}

AngleDeg AngleDeg::operator+(float a) {
  return AngleDeg(value + a);
}

AngleDeg AngleDeg::operator-(float a) {
  return AngleDeg(value - a);
}

AngleDeg AngleDeg::operator*(double a) {
  return AngleDeg(value * a);
}

AngleDeg AngleDeg::operator+=(const double other) {
  value += other;
  return *this;
}

AngleDeg AngleDeg::operator-=(const double other) {
  value -= other;
  return *this;
}

void AngleDeg::operator=(double other) {
  value = other;
}

bool AngleDeg::operator>(const double other) {
  return value > other;
}

AngleRad::AngleRad(float value) {
    this->value = value;
}

AngleRad AngleRad::forwardAngle() {
    return this->toDeg().forwardAngle().toRad();
}

AngleRad AngleRad::cartesianAngle() {
    return this->toDeg().cartesianAngle().toRad();
}

AngleRad AngleRad::angleDifference(AngleRad other) {
    return this->toDeg().angleDifference(other.toDeg()).toRad();
}

AngleDeg AngleRad::toDeg() {
    return this->value * 180 / PRIDE_PI;
}

Vector AngleRad::toVector() {
    float x = cos(this->value);
    float y = sin(this->value);
    return Vector(x, y);
}

String AngleRad::toString() {
    return "AngleRad: " + String(this->value);
}

Vector::Vector(float x, float y) {
    this->x = x;
    this->y = y;
}

Vector::Vector() {
    this->x = 0;
    this->y = 0;
}

float Vector::magnitude() {
    return sqrt(this->x*this->x + this->y*this->y);
}

AngleRad Vector::toAngleRad() {
    return AngleRad(atan(this->y / this->x));
}

AngleDeg Vector::toAngleDeg() {
    return AngleDeg(atan(this->y / this->x) * 180 / PRIDE_PI);
}

Vector Vector::toUnitVector() {
    float angle = this->toAngleRad().value;
    float x = cos(angle);
    float y = sin(angle);
    return Vector(x, y);
}

Vector Vector::operator+(Vector const& a) {
  return Vector(x + a.x, y + a.y);
}

Vector Vector::operator-(Vector const& a) {
  return Vector(x - a.x, y - a.y);
}

Vector Vector::operator*(float const& f) {
  return Vector(x * f, y * f);
}

Vector Vector::operator/(float const& f) {
  return Vector(x / f, y / f);
}

Vector Vector::operator+=(const Vector other) {
  x = x + other.x;
  y = y + other.y;
  return *this;
}

Vector Vector::operator-=(const Vector other) {
  x = x - other.x;
  y = y - other.y;
  return *this;
}

void Vector::operator=(Vector const& other) {
  this->x = other.x;
  this->y = other.y;
}

}