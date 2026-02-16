#ifndef POWER_H
#define POWER_H
#include "audioBoards/audioBoard.h"
#include "audioBoards/audioTracks.h"
#include <Arduino.h>
class PowerControl {
protected:
    AudioBoard& _audioBoard;
    uint8_t _onLatchPin;
    uint8_t _chargeDetectPin;
    uint8_t _batMonPin;
    uint8_t _powerButtonPin;
    bool _powerButtonActiveState;
    float _voltsPerADCUnit;
    float _lowBatteryThreshold;
    bool _powerButtonPreviouslyPressed = true;
    uint32_t _powerButtonPressStartTime = 0;
    void (*_powerOffCallback)();

public:
    PowerControl(AudioBoard& audioBoard, void (*powerOffCallback)(), uint8_t onLatchPin, uint8_t chargeDetectPin, uint8_t batMonPin, uint8_t powerButtonPin, bool powerButtonActiveState, float voltsPerADCUnit, float lowBatteryThreshold)
        : _audioBoard(audioBoard)
        , _powerOffCallback(powerOffCallback)
    {
        _onLatchPin = onLatchPin;
        _chargeDetectPin = chargeDetectPin;
        _batMonPin = batMonPin;
        _powerButtonPin = powerButtonPin;
        _powerButtonActiveState = powerButtonActiveState;
        _voltsPerADCUnit = voltsPerADCUnit;
        _lowBatteryThreshold = lowBatteryThreshold;
    }
    void start()
    {
        pinMode(_onLatchPin, OUTPUT);
        digitalWrite(_onLatchPin, HIGH);
        pinMode(_chargeDetectPin, INPUT);
        pinMode(_batMonPin, INPUT);
        pinMode(_powerButtonPin, INPUT);

        delay(500); // give power time to stabilize before measuring it
        if (digitalRead(_chargeDetectPin) == HIGH) {
            Serial.println("Charging detected on startup, powering off to charge...");
            // _audioBoard.playTrack(TRACK_CHARGING); // TODO: add a charging track
            // delay(TRACK_CHARGING_TIME); // TODO: better way to wait for track to finish
            powerOff();
        }
        checkLowBattery();
    }

    void run()
    {
        checkCharging();
        // TODO: check warning battery
        checkLowBattery();

        checkPowerButton();
    }
    void powerOff()
    {
        Serial.println("Powering off...");
        // _audioBoard.playTrack(TRACK_POWER_OFF); //TODO: add a power off track
        delay(2000); // delay(TRACK_POWER_OFF_TIME);
        digitalWrite(_onLatchPin, LOW);
        delay(100); // should lose power before this line completes
        _powerOffCallback();
        Serial.println("If you see this message, power off failed");
        _audioBoard.playTrack(TRACK_ERROR_GENERIC); // TODO: add a track that says let go of power button
        // delay(TRACK_ERROR_GENERIC_TIME);
        delay(5000); // and some additional time
        // TODO: play track that says to contact support or unplug battery
        while (true) {
            Serial.println("trying to turn off...");
            digitalWrite(_onLatchPin, LOW);
            delay(100);
        }
    }

protected:
    void checkCharging()
    {
        if (digitalRead(_chargeDetectPin) == HIGH) {
            Serial.println("Charging detected while running, powering off to charge...");
            // _audioBoard.playTrack(TRACK_CHARGING); // TODO: add a charging track
            // delay(TRACK_CHARGING_TIME); // TODO: better way to wait for track to finish
            powerOff();
        }
    }
    void checkLowBattery()
    {
        if (analogRead(_batMonPin) * _voltsPerADCUnit < _lowBatteryThreshold) {
            Serial.println("Low battery, powering off...");
            // _audioBoard.playTrack(TRACK_LOW_BATTERY); // TODO: add a low battery track ("CurbScout battery low, please charge, powering off now.")
            // delay(TRACK_LOW_BATTERY_TIME);
            powerOff();
        }
    }
    void checkPowerButton()
    {
        if (digitalRead(_powerButtonPin) == _powerButtonActiveState) {
            if (!_powerButtonPreviouslyPressed) {
                _powerButtonPreviouslyPressed = true;
                _powerButtonPressStartTime = millis();
            } else { // _powerButtonPreviouslyPressed=true, button is being held
                if (millis() - _powerButtonPressStartTime > 100 && _powerButtonPressStartTime != 0) { // TODO: make this a constant
                    Serial.println("Power button held, powering off...");
                    powerOff();
                }
            }
        } else {
            _powerButtonPreviouslyPressed = false;
            _powerButtonPressStartTime = millis();
        }
    }
};

#endif // POWER_H
