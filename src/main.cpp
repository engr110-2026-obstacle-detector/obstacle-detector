
#include "distanceSensors/distanceData.h"
#include "distanceSensors/distanceSensorVL53L8cxMultiplexer.h"

#include "lineSensors/lineSensorADS1115.h"

#include "alarmSpeakers/alarmSpeakerPicoPio.h"
#include "alarmSpeakers/hornController.h"

#include "audioBoards/audioBoardDy1703aSoftserial.h"

#include "audioBoards/audioTracks.h"

#include "orientationSensors/orientationData.h"
#include "orientationSensors/orientationSensor.h"
#include "orientationSensors/orientationSensorICM20948.h"
#include "orientationSensors/orientationSensorMpu6050.h"

#include "power.h"

#include <Arduino.h>

#include <Wifi.h>

// CONSTANTS
const uint32_t startup1Timeout = 45000; // milliseconds
float lowBatteryThreshold = 3.5;

// PINS
// power
const uint8_t onLatchPin = 20;
const uint8_t chargeDetectPin = 21;
const uint8_t batMonPin = 28;
const uint8_t powerButtonPin = 11;

// control panel
const uint8_t hornPin = 7;
const uint8_t linePin = 6;

// frontSensors
const uint8_t frontSensorSDA = 2;
const uint8_t frontSensorSCL = 3;
TwoWire& frontSensorI2CBus = Wire1;
const uint8_t frontSensorMuxAddress = 0x70;
const uint8_t frontSensorIMUAddress = 0x68;

// central orientation sensor
SPIClass& centralOrientationSensorSPI = SPI;
const uint8_t centralOrientationSensorCSPin = 17;

// alarm speaker
const uint8_t alarmSpeakerPin = 14; // also uses alarmSpeakerPin+1
const uint32_t alarmSpeakerLoudestFrequency = 500; // Hz, 3200 is the true loudest but it hurts my ears

// audio board
const uint8_t audioBoardTxPin = 12;
const uint8_t audioBoardRxPin = 13;

// END OF PINS SECTION

OrientationSensorICM20948 centralOrientationSensor(centralOrientationSensorSPI, centralOrientationSensorCSPin);
OrientationData centralOrientationData;

OrientationSensorMpu6050 frontOrientationSensor(frontSensorIMUAddress, frontSensorI2CBus);
OrientationData frontOrientationData;

const uint8_t frontSensorDataWidth = 3 * 8;
const uint8_t frontSensorDataHeight = 8; // 8 rows
DistanceSensorVL53L8cxMultiplexer sensorLeft(frontSensorI2CBus, frontSensorMuxAddress, 1); // left is mux channel 1
DistanceSensorVL53L8cxMultiplexer sensorCenter(frontSensorI2CBus, frontSensorMuxAddress, 2); // center is mux channel 2
DistanceSensorVL53L8cxMultiplexer sensorRight(frontSensorI2CBus, frontSensorMuxAddress, 3); // right is mux channel 3
volatile DistanceData distanceData[frontSensorDataHeight][frontSensorDataWidth]; // 8 rows (height) x 24 cols (3 sensors of 8 cols each)

// const uint8_t lineSensor1Address = 0x49;
// LineSensorADS1115 lineSensorBack(Wire, lineSensor1Address);

AlarmSpeakerPicoPio alarmSpeaker(alarmSpeakerPin, alarmSpeakerLoudestFrequency);
HornController horn(alarmSpeaker);

AudioBoardDY1703aSoftSerial audioBoard(audioBoardRxPin, audioBoardTxPin); // rx pin, tx pin

float voltsPerADCUnit = 0.00512;
void powerOffCallback()
{
    // called after attempting to cut power, in case cutting power fails, to try to stop everything else as much as possible while waiting for power to actually cut
    alarmSpeaker.stop();
}
PowerControl powerControl(audioBoard, powerOffCallback, onLatchPin, chargeDetectPin, batMonPin, powerButtonPin, 1, voltsPerADCUnit, lowBatteryThreshold);

volatile bool setup1Done = false;
volatile bool setupDone = false;

#include "misc.h"

void setup()
{
    powerControl.on();
    audioBoard.begin();
    powerControl.start();
    delay(15);
    Serial.begin(500000);
    Serial.println("Serial starting...");

    SPI.begin();
    centralOrientationSensor.begin();

    pinMode(hornPin, INPUT_PULLUP);
    pinMode(linePin, INPUT_PULLUP);

    horn.begin(); // also calls begin() on the alarm speaker
    audioBoard.playTrack(TRACK_POWER_UP);
    // delay(TRACK_POWER_UP_TIME);
    setupDone = true;
    Serial.println("Serial initialized");

    bool error = false;
    while (!setup1Done) {
        if (millis() > startup1Timeout) {
            Serial.println("Setup1 taking a long time...");
            audioBoard.playTrack(TRACK_ERROR_GENERIC);
            error = true;
            break;
        }
        powerControl.run();
        audioBoard.run();
        hornRun();
        tiltDetect();
    }
    alarmSpeaker.stop();
    Serial.println("Setup done");
    if (!error) {
        audioBoard.playTrack(TRACK_POWERED_ON);
    }
}

// vl53l8cx sensors provide lots of data but take significant amounts of time to transmit it over I2C so I'll use the second processor of the pico to read the sensors
void setup1()
{
    while (!setupDone) {
        delay(10);
    }

    pinMode(frontSensorSDA, OUTPUT_12MA);
    pinMode(frontSensorSCL, OUTPUT_12MA);
    frontSensorI2CBus.setSDA(frontSensorSDA);
    frontSensorI2CBus.setSCL(frontSensorSCL);
    frontSensorI2CBus.begin();
    frontSensorI2CBus.setTimeout(25, false);
    Serial.println("I2C bus initialized");

    sensorLeft.begin();
    Serial.println("Left sensor initialized");

    frontOrientationSensor.begin();

    sensorCenter.begin();
    Serial.println("Center sensor initialized");

    sensorRight.begin();
    Serial.println("Right sensor initialized");

    // Wire.setSDA(lineSensorSDA);
    // Wire.setSCL(lineSensorSCL);
    // Wire.begin();
    // Wire.setTimeout(25, false);
    // lineSensorBack.begin();
    // lineSensorFront.begin();

    // longRangeTop.begin();
    // longRangeBottom.begin();

    setup1Done = true;
}

void loop1()
{ // slow loop for sensors that are slow to poll
    sensorLeft.run();
    delay(5);
    sensorCenter.run();
    delay(5);
    sensorRight.run();
    delay(5);
    // lineSensorFront.run();
    frontOrientationSensor.run();
    // lineSensorBack.run();
    delay(5);
}

void loop()
{ // fast main loop
    powerControl.run();
    horn.run();
    audioBoard.run();
    hornRun();

    tiltDetect(); // calls centralOrientationSensor.run();

    // Serial.println("Running main loop");

    if (frontOrientationSensor.isMeasurementReady()) {
        frontOrientationSensor.getOrientationData(frontOrientationData);
        Serial.println();
        Serial.print(frontOrientationData.Ax);
        Serial.print("\t");
        Serial.print(frontOrientationData.Ay);
        Serial.print("\t");
        Serial.print(frontOrientationData.Az);
        Serial.print("\t");
        Serial.print(frontOrientationData.Gx);
        Serial.print("\t");
        Serial.print(frontOrientationData.Gy);
        Serial.print("\t");
        Serial.print(frontOrientationData.Gz);
        Serial.println();
        Serial.println();
    }

    // if (lineSensorBack.isMeasurementReady()) {
    //     int8_t linePos = lineSensorBack.getLinePosition();
    // }
    // if (lineSensorFront.isMeasurementReady()) {
    //     int8_t linePos = lineSensorFront.getLinePosition();
    // }

    bool anythingNewFromFrontSensors = false;
    if (sensorLeft.isMeasurementReady()) {
        anythingNewFromFrontSensors = true;
        sensorLeft.getDistanceData((DistanceData*)distanceData, 0, 0, frontSensorDataWidth, frontSensorDataHeight);
    }
    if (sensorCenter.isMeasurementReady()) {
        anythingNewFromFrontSensors = true;
        sensorCenter.getDistanceData((DistanceData*)distanceData, 8, 0, frontSensorDataWidth, frontSensorDataHeight);
    }
    if (sensorRight.isMeasurementReady()) {
        anythingNewFromFrontSensors = true;
        sensorRight.getDistanceData((DistanceData*)distanceData, 16, 0, frontSensorDataWidth, frontSensorDataHeight);
    }

    static bool detectedObjectInFront = false;

    if (anythingNewFromFrontSensors) {
        if (distanceData[4][12].isValid && distanceData[4][12].distanceMm < 100) {
            if (!detectedObjectInFront) {
                detectedObjectInFront = true;
                audioBoard.playTrack(TRACK_OBJECT_FRONT);
            }
        } else {
            if (detectedObjectInFront) {
                detectedObjectInFront = false;
            }
        }

        for (int row = 0; row < frontSensorDataHeight; row++) {
            for (int col = 0; col < frontSensorDataWidth; col++) {
                if (distanceData[row][col].isValid) {
                    Serial.print(distanceData[row][col].distanceMm);
                }
                Serial.print("\t");
            }
            Serial.println();
        }
        Serial.println();
    }
}
