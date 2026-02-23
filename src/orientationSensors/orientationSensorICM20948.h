#ifndef ORIENTATION_SENSOR_ICM20948_H
#define ORIENTATION_SENSOR_ICM20948_H
#include "orientationSensor.h"
#include <Arduino.h>
#include <ICM_20948.h>
class OrientationSensorICM20948 : public OrientationSensor {
protected:
    ICM_20948_SPI _icm;
    pin_size_t _csPin;
    SPIClass& _spi;
    ICM_20948_AGMT_t agmt;
    uint32_t _lastDataMillis;
    bool _initialized;
    bool _hasNewData;

public:
    OrientationSensorICM20948(SPIClass& spi, uint8_t csPin)
        : _spi(spi)
        , _csPin(csPin)
    {
        _lastDataMillis = 0;
        _initialized = false;
        _hasNewData = false;
    }
    bool begin()
    {
        _icm.begin(_csPin, _spi);
        _initialized = _icm.status == ICM_20948_Stat_Ok;
        return _initialized;
    }
    bool run()
    {
        if (!_initialized) {
            return false;
        }
        if (_icm.dataReady()) {
            _icm.getAGMT();
            agmt = _icm.agmt;
            _lastDataMillis = millis();
            _hasNewData = true;
            return true;
        } else {
            return false;
        }
    }
    uint32_t getMillisOfLastData()
    {
        return _lastDataMillis;
    }
    bool isMeasurementReady()
    {
        return _hasNewData;
    }
    void getOrientationData(OrientationData& data)
    {
        data.Ax = _icm.getAccMG(agmt.acc.axes.x) / 1000.0;
        data.Ay = _icm.getAccMG(agmt.acc.axes.y) / 1000.0;
        data.Az = _icm.getAccMG(agmt.acc.axes.z) / 1000.0;
        data.Gx = _icm.getGyrDPS(agmt.gyr.axes.x);
        data.Gy = _icm.getGyrDPS(agmt.gyr.axes.y);
        data.Gz = _icm.getGyrDPS(agmt.gyr.axes.z);
    }
};

#endif // ORIENTATION_SENSOR_ICM20948_H
