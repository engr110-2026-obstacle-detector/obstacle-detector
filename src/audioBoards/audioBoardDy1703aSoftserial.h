#ifndef AUDIO_BOARD_DY1703A_SOFTSERIAL_H
#define AUDIO_BOARD_DY1703A_SOFTSERIAL_H

#include "audioBoard.h"
#include <Arduino.h>
#include <SoftwareSerial.h>

class AudioBoardDY1703aSoftSerial : public AudioBoard {
public:
    void begin();
    void playTrack(uint32_t track);
    void stop();
    bool isPlaying();
    AudioBoardDY1703aSoftSerial(pin_size_t rxPin, pin_size_t txPin)
        : serial(rxPin, txPin)
    {
    }

protected:
    SoftwareSerial serial;
};

#endif // AUDIO_BOARD_DY1703A_SOFTSERIAL_H
