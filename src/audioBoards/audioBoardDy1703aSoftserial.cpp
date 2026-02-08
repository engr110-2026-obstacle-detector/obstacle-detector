#include "audioBoardDy1703aSoftserial.h"

void AudioBoardDY1703aSoftSerial::begin()
{
    serial.begin(9600);

}

void AudioBoardDY1703aSoftSerial::playTrack(uint32_t track)
{
}

void AudioBoardDY1703aSoftSerial::stop()
{
}

bool AudioBoardDY1703aSoftSerial::isPlaying()
{
    return false;
}