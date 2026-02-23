#ifndef COMPLEMENTARY_ORIENTATION_FILTER_H
#define COMPLEMENTARY_ORIENTATION_FILTER_H
#include "orientationSensors/orientationData.h"
#include <Arduino.h>
class ComplementaryOrientationFilter {
protected:
    float _alpha;
    float _angle;
    float _offset;

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
        , _offset(offset)
    {
    }
    float update(OrientationData& data)
    {
        static bool firstUpdate = true;
        static uint32_t lastUpdateMillis = 0;
        float angleFromAcc = atan2(-data.Ax, data.Az) * 180 / PI + _offset; // TODO: make axes configurable
        while (angleFromAcc < -180)
            angleFromAcc += 360;
        while (angleFromAcc > 180)
            angleFromAcc -= 360;
        if (firstUpdate) {
            lastUpdateMillis = millis();
            firstUpdate = false;
            _angle = angleFromAcc;
        } else {
            uint32_t now = millis();
            float gyroRate = data.Gy; // TODO: make this configurable
            // TODO: handle -180 to 180 case
            _angle = _angle + gyroRate * (now - lastUpdateMillis) / 1000.0;
            _angle = _angle * (1 - _alpha) + angleFromAcc * _alpha;
            lastUpdateMillis = now;
        }
        return _angle;
    }
};

#endif // COMPLEMENTARY_ORIENTATION_FILTER_H