#pragma once
#ifndef PRIDE_MATH_H
#define PRIDE_MATH_H

#include <math.h>
#include <Arduino.h>

namespace PrideUtils {

struct AngleRad;
struct Vector;

struct AngleDeg {
    float value;

    AngleDeg(float value);

    AngleDeg forwardAngle();
    AngleDeg cartesianAngle();
    AngleDeg angleDifference(AngleDeg other);
    
    AngleDeg operator+(float a);
    AngleDeg operator-(float a);
    AngleDeg operator*(double a);
    AngleDeg operator+=(const double other);
    AngleDeg operator-=(const double other);
    bool operator>(const double other);
    void operator=(double other);

    AngleRad toRad();
    Vector toVector();
    String toString();
};

struct AngleRad {
    float value;

    AngleRad(float value);

    AngleRad forwardAngle();
    AngleRad cartesianAngle();
    AngleRad angleDifference(AngleRad other);
    
    AngleDeg toDeg();
    Vector toVector();
    String toString();
};

struct Vector {
    float x;
    float y;

    Vector(float x, float y);
    Vector();

    Vector operator+(Vector const& a);
    Vector operator-(Vector const& a);
    Vector operator*(float const& f);
    Vector operator/(float const& f);
    Vector operator+=(const Vector other);
    Vector operator-=(const Vector other);
    void operator=(Vector const& other);

    float magnitude();
    AngleRad toAngleRad();
    AngleDeg toAngleDeg();
    Vector toUnitVector();
    
};

}

#endif