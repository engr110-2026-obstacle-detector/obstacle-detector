#ifndef MISC_H
#define MISC_H
void hornRun()
{
    horn.beep(digitalRead(hornPin) == LOW);
}

void tiltDetect()
{
    centralOrientationSensor.run();
    if (centralOrientationSensor.isMeasurementReady()) {
        centralOrientationSensor.getOrientationData(centralOrientationData);
    }
    horn.alarm(centralOrientationData.Ay > 0);
}
#endif // MISC_H
