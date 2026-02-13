#ifndef ORIENTATION_SENSOR_H
#define ORIENTATION_SENSOR_H
#include <stdint.h>
#include <stdbool.h>
#include "orientationData.h"
class OrientationSensor {
public:
    virtual bool begin() = 0;
    virtual bool run() = 0;
    virtual uint32_t getMillisOfLastData() = 0;
    virtual bool isMeasurementReady() = 0;
    virtual void getOrientationData(OrientationData& data) = 0;
};

#endif // ORIENTATION_SENSOR_H
