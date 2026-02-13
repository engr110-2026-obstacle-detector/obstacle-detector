#ifndef ORIENTATION_SENSOR_MPU6050_H
#define ORIENTATION_SENSOR_MPU6050_H
#include "orientationSensor.h"
#include <Arduino.h>
class OrientationSensorMpu6050 : public OrientationSensor {
public:
    OrientationSensorMpu6050()
    {
    }
    bool begin()
    {
        return true;
    }
    bool run()
    {
        return true;
    }
    uint32_t getMillisOfLastData()
    {
        return 0;
    }
    bool isMeasurementReady()
    {
        return true;
    }
    void getOrientationData(OrientationData& data)
    {
        data.Ax = NAN;
        data.Ay = NAN;
        data.Az = NAN;
        data.Gx = NAN;
        data.Gy = NAN;
        data.Gz = NAN;
        data.roll = NAN;
        data.pitch = NAN;
        data.yaw = NAN;
    }
};

#endif // ORIENTATION_SENSOR_MPU6050_H
