#ifndef COMPLEMENTARY_ORIENTATION_FILTER_H
#define COMPLEMENTARY_ORIENTATION_FILTER_H
#include "orientationSensors/orientationData.h"
#include <Arduino.h>
class ComplementaryOrientationFilter {
protected:
    float _alpha;
    float _angle;
    float _offset;
    bool firstUpdate = true;
    uint32_t lastUpdateMillis = 0;
    float gyroZero = 0;

    float _filteredAngle;

public:
    /**
     * @brief
     * @note
     * @param  alpha: larger means listen to the accelerometer more
     * @param  offset:
     * @retval
     */
    ComplementaryOrientationFilter(float alpha, float offset)
        : _alpha(alpha)
        , _angle(0)
        , _filteredAngle(0)
        , _offset(offset)
    {
        _alpha = constrain(_alpha, 0, 1);
        _alpha = 1;
    }
    float update(OrientationData& data)
    {
        float angleFromAcc = atan2(-data.Ax, data.Az) * 180 / PI + _offset; // TODO: make axes configurable

        if (firstUpdate) {
            lastUpdateMillis = millis();
            firstUpdate = false;
            _angle = angleFromAcc;
            _filteredAngle = _angle;
            gyroZero = data.Gy;
        } else {
            uint32_t now = millis();
            float gyroRate = data.Gy; // TODO: make this configurable

            // TODO: handle -180 to 180 case
            _angle = _angle + (gyroRate - gyroZero) * (now - lastUpdateMillis) / 1000.0;
            _angle = _angle * (1 - _alpha) + angleFromAcc * _alpha;

            gyroZero = gyroZero * (1 - 0.01 * _alpha) + gyroRate * 0.01 * _alpha;

            lastUpdateMillis = now;
        }
        _filteredAngle = _filteredAngle * (1 - .1) + _angle * .1;
        return _filteredAngle;
    }
};

#endif // COMPLEMENTARY_ORIENTATION_FILTER_H