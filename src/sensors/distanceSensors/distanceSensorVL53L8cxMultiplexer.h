#ifndef DISTANCE_SENSOR_VL53L8CX_MULTIPLEXER_H
#define DISTANCE_SENSOR_VL53L8CX_MULTIPLEXER_H
#include "distanceSensor.h"
#include <Arduino.h>
#include <Wire.h>
#include <vl53l8cx.h>
class DistanceSensorVL53L8cxMultiplexer : public DistanceSensor {
protected:
    const uint8_t _height = 8;
    const uint8_t _width = 8;
    TwoWire& _i2cBus;
    uint8_t _muxAddress;
    uint8_t _muxChannel;
    VL53L8CX _sensor;
    bool _initialized = false;
    VL53L8CX_ResultsData Results;
    bool _newDataAvailable = false;
    uint32_t _millisOfLastData = 0;

public:
    DistanceSensorVL53L8cxMultiplexer(TwoWire& i2cBus, uint8_t muxAddress, uint8_t muxChannel)
        : _i2cBus(i2cBus)
        , _sensor(VL53L8CX(&_i2cBus, -1))
    {
        _muxAddress = muxAddress;
        _muxChannel = muxChannel;
    }
    ~DistanceSensorVL53L8cxMultiplexer()
    {
    }

    bool begin()
    {
        // Select the multiplexer channel
        _i2cBus.beginTransmission(_muxAddress);
        _i2cBus.write(1 << _muxChannel);
        _i2cBus.endTransmission();
        uint8_t status;
        _sensor.begin();
        status = _sensor.init();
        if (status) {
            // Serial.print("VL53L8CX initialization failed\r\n");
            return false;
        }
        status = _sensor.set_resolution(VL53L8CX_RESOLUTION_8X8);
        if (status) {
            // Serial.print("VL53L8CX set resolution failed\r\n");
            return false;
        }
        status = _sensor.set_ranging_frequency_hz(15); // 15hz max for 8x8, 60hz max for 4x4
        if (status) {
            // Serial.print("VL53L8CX set ranging frequency failed\r\n");
            return false;
        }
        status = _sensor.set_ranging_mode(VL53L8CX_RANGING_MODE_CONTINUOUS);
        if (status) {
            // Serial.print("VL53L8CX set ranging mode failed\r\n");
            return false;
        }
        _initialized = true;

        status = _sensor.start_ranging();

        return true;
    }
    bool run()
    {
        if (!_initialized) {
            return false;
        }
        _i2cBus.beginTransmission(_muxAddress);
        _i2cBus.write(1 << _muxChannel);
        _i2cBus.endTransmission();

        uint8_t NewDataReady = 0;
        uint8_t status;
        status = _sensor.check_data_ready(&NewDataReady);
        if (status) {
            // Serial.print("VL53L8CX check data ready failed\r\n");
            return false;
        }
        if (NewDataReady) {
            status = _sensor.get_ranging_data(&Results);
            if (status) {
                // Serial.print("VL53L8CX get ranging data failed\r\n");
                return false;
            } else {
                _millisOfLastData = millis();
                _newDataAvailable = true;
            }
        } else {
            // no data, no errors
        }
        return true; // no errors
    }
    uint32_t getMillisOfLastData()
    {
        return _millisOfLastData;
    }
    bool isMeasurementReady()
    {
        return _newDataAvailable;
    }
    /**
     * @brief copies distance data into provided array
     * @param  data[]: array of DistanceData structs that will be filled with distance data. It can be a 2D array
     * @param  startCol: starting column index in the data array where sensor data will be placed
     * @param  startRow: starting row index in the data array where sensor data will be placed
     * @param  arrayCols: total number of columns in the data array
     * @param  arrayRows: total number of rows in the data array
     * @retval false if not initialized, no new data, or out of bounds in the data array; true if data is successfully copied
     */
    bool getDistanceData(DistanceData* data, int32_t startCol, int32_t startRow, int32_t arrayCols, int32_t arrayRows)
    {
        if (!_initialized) {
            return false;
        }
        if (!_newDataAvailable) {
            return false;
        }
        _newDataAvailable = false;
        for (int row = 0; row < _height; row++) {
            for (int col = 0; col < _width; col++) {
                int32_t arrayIndex = (startRow + row) * arrayCols + (startCol + col);
                int32_t sensorIndex = row * _width + col;
                if (arrayIndex >= arrayCols * arrayRows) {
                    return false; // prevent out-of-bounds write
                }
                data[arrayIndex].distanceMm = Results.distance_mm[sensorIndex];
                data[arrayIndex].isValid = (Results.target_status[sensorIndex] == 5); // 5 means fully valid measurement
            }
        }
        return true;
    }
};
#endif // DISTANCE_SENSOR_VL53L8CX_MULTIPLEXER_H
