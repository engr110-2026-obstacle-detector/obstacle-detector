#ifndef LINE_SENSOR_H
#define LINE_SENSOR_H
#include <stdint.h>
#include <stdbool.h>
class LineSensor {
public:
    virtual bool begin() = 0;
    virtual bool run() = 0;
    virtual uint32_t getMillisOfLastData() = 0;
    virtual bool isMeasurementReady() = 0;
    virtual int8_t getLinePosition() = 0;
    virtual bool isLineDetected() = 0;
};

#endif // LINE_SENSOR_H
