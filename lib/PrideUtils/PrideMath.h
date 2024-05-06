#pragma once
#ifndef PRIDE_MATH_H
#define PRIDE_MATH_H

#include <math.h>
#include <Arduino.h>
#include <cmath>

namespace PrideUtils {

    float fclamp(float value, float min, float max);

    struct fPDController;
    struct AngleRad;
    struct Vector;

    struct AngleDeg {
        float value;

        AngleDeg(float value);

        AngleDeg forwardAngle();
        AngleDeg cartesianAngle();
        AngleDeg angleDifference(AngleDeg other);
        
        AngleDeg operator+(const float a);
        AngleDeg operator-(const float a);
        AngleDeg operator*(const float a);
        AngleDeg operator%(const float other);
        AngleDeg operator+=(const float other);
        AngleDeg operator-=(const float other);
        bool operator>(const float other);
        void operator=(float other);

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

        AngleRad operator+(AngleRad a);
        AngleRad operator-(AngleRad a);
    };

    struct Vector {
        float x;
        float y;

        Vector(float x, float y);
        Vector();

        Vector operator+(Vector const& a);
        AngleRad operator+(const AngleRad other);
        Vector operator-(Vector const& a);
        Vector operator*(float const& f);
        Vector operator/(float const& f);
        Vector operator+=(const Vector other);
        Vector operator+=(const AngleRad other);
        Vector operator-=(const Vector other);
        void operator=(Vector const& other);

        AngleRad angleBetween(Vector other);

        float magnitude();
        AngleRad toAngleRad();
        AngleDeg toAngleDeg();
        String toString();
        Vector toUnitVector();
        Vector flip();
        bool isZero();
        
    };

    struct fPDController {
        float p;
        float d;
        float lastError;

        fPDController(float p, float d);

        float update(float error);
    };

}

#endif