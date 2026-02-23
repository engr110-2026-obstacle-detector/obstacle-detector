#ifndef MISC_H
#define MISC_H
void hornRun()
{
    horn.beep(digitalRead(hornPin) == LOW);
    horn.run();
}

void tiltDetect()
{
    static float accelAvg = 1;
    static bool tiltAlarmOn = false;
    static uint32_t tiltAlarmStartTime = 0;
    centralOrientationSensor.run();
    if (centralOrientationSensor.isMeasurementReady()) {
        centralOrientationSensor.getOrientationData(centralOrientationData);
        accelAvg = accelAvg * 0.999 + centralOrientationData.Az * 0.001;
    }
    if (tiltAlarmOn == false && accelAvg < 0.85) {
        tiltAlarmOn = true;
        audioBoard.playTrack(TRACK_TILT);
        tiltAlarmStartTime = millis();
    }
    if (tiltAlarmOn && millis() - tiltAlarmStartTime > TRACK_TILT_TIME) {
        horn.alarm(true);
    } else {
        horn.alarm(false);
    }
}
#endif // MISC_H
