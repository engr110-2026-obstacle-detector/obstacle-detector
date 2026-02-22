#ifndef ORIENTATION_SENSOR_MPU6050_H
#define ORIENTATION_SENSOR_MPU6050_H
#include "orientationSensor.h"
#include <Arduino.h>
#include <MPU6050.h>
#include <Wire.h>
class OrientationSensorMpu6050 : public OrientationSensor {
protected:
    MPU6050 _mpu;
    uint32_t _lastDataMillis;
    bool _initialized;
    bool _hasNewData;
    float _ax, _ay, _az, _gx, _gy, _gz;

public:
    OrientationSensorMpu6050(uint8_t address, TwoWire& wire)
        : _mpu(address, &wire)
    {
        _lastDataMillis = 0;
        _initialized = false;
        _hasNewData = false;
    }
    bool begin()
    {
        _mpu.initialize();
        _initialized = _mpu.testConnection();
        Serial.print("MPU6050 connection ");
        Serial.println(_initialized ? "successful" : "failed");
        return _initialized;
    }
    bool run()
    {
        if (!_initialized) {
            return false;
        }
        if (_mpu.testConnection()) {
            int16_t axRaw, ayRaw, azRaw, gxRaw, gyRaw, gzRaw;
            _mpu.getMotion6(&axRaw, &ayRaw, &azRaw, &gxRaw, &gyRaw, &gzRaw);
            _ax = axRaw * _mpu.get_acce_resolution();
            _ay = ayRaw * _mpu.get_acce_resolution();
            _az = azRaw * _mpu.get_acce_resolution();
            _gx = gxRaw * _mpu.get_gyro_resolution();
            _gy = gyRaw * _mpu.get_gyro_resolution();
            _gz = gzRaw * _mpu.get_gyro_resolution();

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
        data.Ax = _ax;
        data.Ay = _ay;
        data.Az = _az;
        data.Gx = _gx;
        data.Gy = _gy;
        data.Gz = _gz;
        _hasNewData = false;
    }
};

#endif // ORIENTATION_SENSOR_MPU6050_H
