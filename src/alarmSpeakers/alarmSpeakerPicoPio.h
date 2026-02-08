#ifndef ALARM_SPEAKER_PICO_PIO_H
#define ALARM_SPEAKER_PICO_PIO_H
#include "alarmSpeaker.h"
#include "hardware/pio.h"
#include "stdint.h"

class AlarmSpeakerPicoPio : public AlarmSpeaker {
protected:
    uint32_t _loudestFrequency;
    PIO pio;
    uint sm;
    uint32_t _pin1;

public:
    /**
     * @brief controls two pins for bidirectionally driving a speaker, using the RP2040's PIO
     * @param  pin: pin and pin+1 will be used for output to the speaker, they must be consecutive.
     * @param  loudestFrequency: frequency in Hz that makes the speaker play its loudest tone
     */
    AlarmSpeakerPicoPio(const uint32_t pin, uint32_t loudestFrequency = 3200)
    {
        _loudestFrequency = loudestFrequency;
        _pin1 = pin;
    }
    void playLoudestFrequency()
    {
        playFrequency(_loudestFrequency);
    }
    void begin();
    void stop()
    {
        playFrequency(0);
    }
    void playFrequency(uint32_t frequency);
};

#endif // ALARM_SPEAKER_PICO_PIO_H
