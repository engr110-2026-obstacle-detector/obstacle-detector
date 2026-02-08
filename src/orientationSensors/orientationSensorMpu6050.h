#ifndef ORIENTATION_SENSOR_MPU6050_H
#define ORIENTATION_SENSOR_MPU6050_H
#include "orientationSensor.h"
class OrientationSensorMpu6050 : public OrientationSensor {
public:
    bool begin(){
        return true;
    }
    bool run(){
        return true;
    }
    uint32_t getMillisOfLastData() {
        return 0;
    }
    bool isMeasurementReady() {
        return true;
    }
};

#endif // ORIENTATION_SENSOR_MPU6050_H
