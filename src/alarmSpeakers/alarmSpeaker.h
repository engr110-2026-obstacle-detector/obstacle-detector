#ifndef ALARM_SPEAKER_H
#define ALARM_SPEAKER_H
#include <stdint.h>
class AlarmSpeaker {
public:
    virtual void begin() = 0;
    virtual void stop() = 0;
    virtual void playFrequency(uint32_t frequency) = 0;
    virtual void playLoudestFrequency() = 0;
};

#endif // ALARM_SPEAKER_H
