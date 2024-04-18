#pragma once
#ifndef GYRO_H
#define GYRO_H

#include <Adafruit_BNO08x.h>
#include "constants.hpp"

struct gyro_data {
  float yaw;
  float pitch;
  float roll;
};

class Gyro {
  public:
    static gyro_data offset;
    static bool setup();
    static void setOrigin();
    static void reset();
    static gyro_data getData(bool printOuts = true);
    static gyro_data latest_data;
  private:
    static void setReports(sh2_SensorId_t reportType, long report_interval);       
    static void quaternionToEuler(float qr, float qi, float qj, float qk, gyro_data* ypr, bool degrees = false);
    static void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, gyro_data* ypr, bool degrees = false);
    static void quaternionToEulerGI(sh2_GyroIntegratedRV_t* rotational_vector, gyro_data* ypr, bool degrees = false);
};

#endif