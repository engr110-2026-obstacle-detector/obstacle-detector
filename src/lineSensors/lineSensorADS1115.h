#ifndef LINE_SENSOR_ADS1115_H
#define LINE_SENSOR_ADS1115_H

#include "lineSensor.h"
#include <ADS1115_WE.h>
#include <Arduino.h>
#include <Wire.h>
#include <stdbool.h>
#include <stdint.h>

class LineSensorADS1115 : public LineSensor {
protected:
    const static uint8_t _numAdcChannels = 4;
    ADS1115_WE _adc;
    bool _initialized = false;
    bool _newData = false;
    int16_t _readings[_numAdcChannels] = { 0, 0, 0, 0 };
    uint32_t _millisOfLastData = 0;
    uint8_t channel = 0;
    bool _direction = true;
    const ADS1115_MUX channelMap[_numAdcChannels] = {
        ADS1115_COMP_0_GND,
        ADS1115_COMP_1_GND,
        ADS1115_COMP_2_GND,
        ADS1115_COMP_3_GND
    };

public:
    LineSensorADS1115(TwoWire& i2cBus, uint8_t i2cAddress)
        : _adc(&i2cBus, i2cAddress)
    {
    }
    ~LineSensorADS1115()
    {
    }

    bool begin()
    {
        if (!_adc.init()) {
            Serial.println("ADS1115 not connected!");
            return false;
        }

        _adc.setVoltageRange_mV(ADS1115_RANGE_4096);
        // _adc.setConvRate(ADS1115_128_SPS);

        _adc.setCompareChannels_nonblock(channelMap[0]);

        _adc.startSingleMeasurement();

        _initialized = true;
        return true;
    }

    bool run()
    {
        if (!_initialized) {
            return false;
        }

        if (_adc.isBusy()) {
            return true;
        }

        _readings[channel] = _adc.getRawResult();

        if (_direction) {
            channel++;
            if (channel >= _numAdcChannels) {
                channel = _numAdcChannels - 1;
                _direction = false;
                _newData = true;
            }
        } else {
            if (channel == 0) {
                _direction = true;
                _newData = true;
            } else {
                channel--;
            }
        }

        _adc.setCompareChannels_nonblock(channelMap[channel]);
        _adc.startSingleMeasurement();

        _millisOfLastData = millis();

        return true;
    }

    uint32_t getMillisOfLastData()
    {
        // Return the timestamp of the last data read
        return _millisOfLastData;
    }

    bool isMeasurementReady()
    {
        // Return true if new measurement data is available
        return _newData;
    }

    int8_t getLinePosition()
    {
        _newData = false;
        // Calculate and return the line position

        return 0;
    }

    bool isLineDetected()
    {
        // Return true if a line is detected
        return false;
    }

    void getRawReadings(int16_t* readingsBuffer)
    {
        for (uint8_t i = 0; i < _numAdcChannels; i++) {
            readingsBuffer[i] = _readings[i];
        }
    }
};

#endif // LINE_SENSOR_ADS1115_H
