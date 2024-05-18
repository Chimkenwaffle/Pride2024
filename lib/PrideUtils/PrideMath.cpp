#include <PrideMath.h>
#include <math.h>

const float PRIDE_PI = 3.14159;
using namespace std;
namespace PrideUtils {

  float fclamp(float value, float min, float max) {
    return value < min ? min : (value > max ? max : value);
  }

  fPIDController::fPIDController(float p, float i, float d) {
    this->p = p;
    this->i = i;
    this->d = d;
    this->lastError = 0;
  }

  float fPIDController::update(float error) {
    float derivative = error - this->lastError;
    this->lastError = error;
    this->iSum += error;
    if (this->iSum > 100) {
      this->iSum = 100;
    } else if (this->iSum < -100) {
      this->iSum = -100;
    }
    return this->p * error + this->d * derivative + this->i * this->iSum;
  }

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
    if (angle1 < 0) {
      angle1 += 360.0;
    }
    if (angle2 < 0) {
      angle2 += 360.0;
    }
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

  AngleDeg AngleDeg::operator+(const float a) {
    return AngleDeg(value + a);
  }

  AngleDeg AngleDeg::operator-(const float a) {
    return AngleDeg(value - a);
  }

  AngleDeg AngleDeg::operator*(const float a) {
    return AngleDeg(value * a);
  }

  AngleDeg AngleDeg::operator%(const float other) {
    return AngleDeg(fmod(value, other));
  }

  AngleDeg AngleDeg::operator+=(const float other) {
    value += other;
    return *this;
  }

  AngleDeg AngleDeg::operator-=(const float other) {
    value -= other;
    return *this;
  }

  void AngleDeg::operator=(float other) {
    value = other;
  }

  bool AngleDeg::operator>(const float other) {
    return value > other;
  }

  AngleRad::AngleRad(float value) {
    this->value = value;
  }

  AngleRad AngleRad::forwardAngle() {
    float angle = fmod((2.5*PRIDE_PI - this->value), 2*PRIDE_PI);
    if (angle > PRIDE_PI) {
        angle -= 2*PRIDE_PI;
    }
    return AngleRad(angle);
  }

  AngleRad AngleRad::cartesianAngle() {
    float angle = fmod((2*PRIDE_PI - this->value + PRIDE_PI/2), 2*PRIDE_PI);
    if (angle > PRIDE_PI) {
      angle -= 2*PRIDE_PI;
    }
    return AngleRad(angle);
  }

  AngleRad AngleRad::angleDifference(AngleRad other) {
    auto angle1 = this->value;
    auto angle2 = other.value;
    if (angle1 < 0) {
      angle1 += 2*PRIDE_PI;
    }
    if (angle1 > 2*PRIDE_PI) {
      angle1 -= 2*PRIDE_PI;
    }
    if (angle2 < 0) {
      angle2 += 2*PRIDE_PI;
    }
    if (angle2 > 2*PRIDE_PI) {
      angle2 -= 2*PRIDE_PI;
    }
    
    float diff = fmod((angle2 - angle1 + PRIDE_PI), 2 * PRIDE_PI) - PRIDE_PI;
    return AngleRad{diff < -PRIDE_PI ? diff + 2*PRIDE_PI : diff};
  }

  AngleDeg AngleRad::toDeg() {
    return this->value * 180 / PRIDE_PI;
  }

  AngleRad AngleRad::operator+(AngleRad a) {
    return AngleRad(value + a.value);
  }

  AngleRad AngleRad::operator-(AngleRad a) {
    return AngleRad(value - a.value);
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
    return AngleRad(atan2(this->y, this->x));
  }

  AngleDeg Vector::toAngleDeg() {
    return AngleDeg(atan2(this->y, this->x) * 180 / PRIDE_PI);
  }

  String Vector::toString() {
    return "(" + String(x) + ", " + String(y) + ")";
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

  AngleRad Vector::operator+(const AngleRad other) {
    AngleRad vecAngle = this->toAngleRad();
    return vecAngle + other;
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

  Vector Vector::operator+=(const AngleRad other) {
    Vector otherVector = AngleRad(other.value).toVector();
    x = x + otherVector.x;
    y = y + otherVector.y;
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

  AngleRad Vector::angleBetween(Vector other) {
    // Calculate the angle using atan2
    float angleRad = atan2(other.y, other.x) - atan2(y, x);

    // Normalize the angle to be between -180 and 180 degrees
    angleRad = fmod(angleRad + PRIDE_PI, 2 * PRIDE_PI) - PRIDE_PI;
    return AngleRad(angleRad);
  }

  Vector Vector::flip() {
    return Vector(-1 * this->x, -1 * this->y); 
  }

  bool Vector::isZero() {
    return (fabs(x) < 0.01 && fabs(y) < 0.01);
  }

}