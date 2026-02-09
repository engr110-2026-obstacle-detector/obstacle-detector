#include "audioBoardDy1703aSoftserial.h"

void AudioBoardDY1703aSoftSerial::begin()
{
    _serial.begin(9600);
}

/**
 * @param track number between 0 and 999
 */
void AudioBoardDY1703aSoftSerial::playTrack(uint32_t track)
{
    if (track > 999) {
        return;
    }
    uint8_t command[13];
    command[0] = 0xAA; // start
    command[1] = 0x08; // specify device and path
    command[2] = 0x09; // length
    command[3] = 0x01; // device=SD
    command[4] = 0x2F; // path=/
    command[5] = 0x30 + (track / 100) % 10; // hundreds digit
    command[6] = 0x30 + (track / 10) % 10; // tens digit
    command[7] = 0x30 + track % 10; // ones digit
    command[8] = 0x2A; // *
    command[9] = 0x4D; // M
    command[10] = 0x50; // P
    command[11] = 0x33; // 3
    command[12] = 0;
    // checksum: sum of all data from the start code to the end data before the crc
    for (int i = 0; i < 12; i++) {
        command[12] += command[i];
    }
    _serial.write(command, 13);
}

void AudioBoardDY1703aSoftSerial::stop()
{
    uint8_t command[4];
    command[0] = 0xAA; // start
    command[1] = 0x04;
    command[2] = 0x00;
    command[3] = 0xAE;
    _serial.write(command, 4);
}

bool AudioBoardDY1703aSoftSerial::isPlaying()
{
    return _isPlaying;
}

void AudioBoardDY1703aSoftSerial::run()
{
    // 10 times per second send AA 01 00 ab
    if (millis() - _lastStatusRequest > 100) {
        uint8_t command[4];
        command[0] = 0xAA; // start
        command[1] = 0x01; // query status
        command[2] = 0x00;
        command[3] = 0xAB;
        _serial.write(command, 4);
        _lastStatusRequest = millis();
    }
    // play state response is AA 01 01 <state> <checksum>
    while (_serial.available() >= 1) {
        // shift status response left by one byte and read new byte into end
        for (int i = 0; i < 4; i++) {
            _statusResponse[i] = _statusResponse[i + 1];
        }
        _statusResponse[4] = _serial.read();
        if (_statusResponse[0] == 0xAA && _statusResponse[1] == 0x01 && _statusResponse[2] == 0x01 && _statusResponse[4] == (0xAA + 0x01 + 0x01 + _statusResponse[3]) % 256) {
            _isPlaying = _statusResponse[3] == 0x01;
            _lastCommunicationMillis = millis();
        }
    }
}

uint32_t AudioBoardDY1703aSoftSerial::getMillisOfLastCommunication()
{
    return _lastCommunicationMillis;
}
