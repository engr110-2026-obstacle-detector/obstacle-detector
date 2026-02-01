#ifndef DISTANCE_SENSOR_H
#define DISTANCE_SENSOR_H
#include "distanceData.h"
#include <stdint.h>
#include <stdbool.h>
class DistanceSensor {
public:
    virtual bool begin() = 0;
    virtual bool run() = 0;
    virtual uint32_t getMillisOfLastData() = 0;
    virtual bool isMeasurementReady() = 0;
    virtual bool getDistanceData(DistanceData data[], int32_t startCol, int32_t startRow, int32_t arrayCols, int32_t arrayRows) = 0;
};
#endif // DISTANCE_SENSOR_H
