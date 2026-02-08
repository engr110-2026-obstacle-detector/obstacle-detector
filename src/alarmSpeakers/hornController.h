#ifndef HORN_CONTROLLER_H
#define HORN_CONTROLLER_H
#include "alarmSpeaker.h"
#include <Arduino.h>
class HornController {
protected:
    AlarmSpeaker& alarmSpeaker;
    bool alarming = false;
    uint32_t alarmStartMillis = 0;

public:
    HornController(AlarmSpeaker& speaker)
        : alarmSpeaker(speaker)
    {
    }
    void begin()
    {
        alarmSpeaker.begin();
    }
    void run()
    {
        if (alarming) {
            uint32_t mils = millis() - alarmStartMillis;
            // play SOS in Morse code: ... --- ...
            uint32_t index = (mils / 250) % (3 * 2 + 3 + 3 * 4 + 3 + 3 * 2 + 7);
            if (index < 3 * 2) { // 3 dots
                if (index % 2 == 0) {
                    alarmSpeaker.playLoudestFrequency();
                } else {
                    alarmSpeaker.stop();
                }
            } else if (index < 3 * 2 + 3) { // wait
            } else if (index < 3 * 2 + 3 + 3 * 4) { // 3 dashes
                if ((index - (3 * 2 + 3)) % 4 < 3) {
                    alarmSpeaker.playLoudestFrequency();
                } else {
                    alarmSpeaker.stop();
                }
            } else if (index < 3 * 2 + 3 + 3 * 4 + 3) { // wait
            } else if (index < 3 * 2 + 3 + 3 * 4 + 3 + 3 * 2) { // 3 dots
                if ((index - (3 * 2 + 3 + 3 * 4 + 3)) % 2 == 0) {
                    alarmSpeaker.playLoudestFrequency();
                } else {
                    alarmSpeaker.stop();
                }
            } else { // wait
            }
        }
    }
    void beep(bool hornButton)
    {
        if (alarming) {
            return;
        }
    }
    void alarm(bool alarmOn)
    {
        if (alarmOn) {
            if (!alarming) {
                alarmStartMillis = millis();
                alarming = true;
                alarmSpeaker.playLoudestFrequency();
            }

        } else {
            if (alarming) {
                alarming = false;
                alarmSpeaker.stop();
            }
        }
    }
};

#endif // HORN_CONTROLLER_H
