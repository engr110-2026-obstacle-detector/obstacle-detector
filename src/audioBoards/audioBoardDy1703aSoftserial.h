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
    void run();
    uint32_t getMillisOfLastCommunication();
    AudioBoardDY1703aSoftSerial(pin_size_t rxPin, pin_size_t txPin)
        : _serial(rxPin, txPin)
    {
    }

protected:
    SoftwareSerial _serial;
    uint32_t _lastCommunicationMillis = 0;
};

#endif // AUDIO_BOARD_DY1703A_SOFTSERIAL_H
