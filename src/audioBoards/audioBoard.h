#ifndef AUDIO_BOARD_H
#define AUDIO_BOARD_H
#include <stdint.h>
class AudioBoard {
public:
    virtual void begin() = 0;
    virtual void playTrack(uint32_t track) = 0;
    virtual void stop() = 0;
    virtual bool isPlaying() = 0;
    virtual void run() = 0;
    virtual uint32_t getMillisOfLastCommunication() = 0;

protected:
    bool _isPlaying = false;
    uint32_t _lastStatusRequest = 0;
    uint8_t _statusResponse[5] = { 0, 0, 0, 0, 0 };
};

#endif // AUDIO_BOARD_H
