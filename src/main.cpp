
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

#include <Arduino.h>

#include <Wifi.h>

OrientationSensorICM20948 centralOrientationSensor(SPI, 17);
OrientationData centralOrientationData;

const uint8_t frontSensorMuxAddress = 0x70;
const uint8_t frontSensorDataWidth = 3 * 8;
const uint8_t frontSensorDataHeight = 8; // 8 rows
DistanceSensorVL53L8cxMultiplexer sensorLeft(Wire, frontSensorMuxAddress, 1);
DistanceSensorVL53L8cxMultiplexer sensorCenter(Wire, frontSensorMuxAddress, 2);
DistanceSensorVL53L8cxMultiplexer sensorRight(Wire, frontSensorMuxAddress, 3);
volatile DistanceData distanceData[frontSensorDataHeight][frontSensorDataWidth]; // 8 rows (height) x 24 cols (3 sensors of 8 cols each)

const uint8_t lineSensor1Address = 0x49;
LineSensorADS1115 lineSensorBack(Wire, lineSensor1Address);

AlarmSpeakerPicoPio alarmSpeaker(14, 300); // uses pins 14 and 15
HornController horn(alarmSpeaker);

AudioBoardDY1703aSoftSerial audioBoard(12, 13); // rx pin, tx pin

volatile bool setup1Done = false;
volatile bool setupDone = false;

uint8_t onPin = 20;

void setup()
{
    pinMode(onPin, OUTPUT);
    digitalWrite(onPin, HIGH); // stay on

    delay(15);
    Serial.begin(500000);
    Serial.println("Serial starting...");
    delay(500); // give power to audio board time to stabilize

    SPI.begin();
    centralOrientationSensor.begin();

    audioBoard.begin();
    horn.begin(); // also calls begin() on the alarm speaker
    audioBoard.playTrack(TRACK_POWER_UP);
    setupDone = true;
    Serial.println("Serial initialized");

    while (!setup1Done) {
        if (millis() > 1000) {
            Serial.println("Setup1 taking a long time...");
            audioBoard.playTrack(TRACK_ERROR_GENERIC);
            break;
        }
        delay(10);
    }
    Serial.println("Setup done");
    audioBoard.playTrack(TRACK_POWERED_ON);
}

// vl53l8cx sensors provide lots of data but take significant amounts of time to transmit it over I2C so I'll use the second processor of the pico to read the sensors
void setup1()
{
    while (!setupDone) {
        delay(10);
    }

    pinMode(4, OUTPUT_12MA);
    pinMode(5, OUTPUT_12MA);
    // Wire.begin();
    // Wire.setTimeout(25, false);
    // Serial.println("I2C bus initialized");
    // sensorLeft.begin();
    // Serial.println("Left sensor initialized");
    // sensorCenter.begin();
    // Serial.println("Center sensor initialized");
    // sensorRight.begin();
    // Serial.println("Right sensor initialized");

    // Wire1.begin();
    // lineSensorBack.begin();
    // lineSensorFront.begin();

    // longRangeTop.begin();
    // longRangeBottom.begin();

    setup1Done = true;
}

void loop1()
{ // slow loop for sensors that are slow to poll
    sensorLeft.run();
    delay(10);
    sensorCenter.run();
    delay(10);
    sensorRight.run();
    delay(10);
}

void loop()
{ // fast main loop
    horn.run();
    audioBoard.run();
    centralOrientationSensor.run();

    if (centralOrientationSensor.isMeasurementReady()) {
        centralOrientationSensor.getOrientationData(centralOrientationData);
        Serial.print("Orientation sensor data: Ax ");
        Serial.print(centralOrientationData.Ax);
        Serial.print(" Ay ");
        Serial.print(centralOrientationData.Ay);
        Serial.print(" Az ");
        Serial.print(centralOrientationData.Az);
        Serial.print(" Gx ");
        Serial.print(centralOrientationData.Gx);
        Serial.print(" Gy ");
        Serial.print(centralOrientationData.Gy);
        Serial.print(" Gz ");
        Serial.print(centralOrientationData.Gz);
        Serial.println();
    }
    horn.alarm(centralOrientationData.Ay > 0);

    if (millis() > 50000) {
        digitalWrite(onPin, LOW); // turn off
        delay(1000); // should lose power before this line completes
        digitalWrite(onPin, LOW);
        audioBoard.playTrack(TRACK_ERROR_GENERIC); // error, unable to power off, contact support
    }

    // Serial.println("Running main loop");

    // lineSensorBack.run();
    // if (lineSensorBack.isMeasurementReady()) {
    //     int8_t linePos = lineSensorBack.getLinePosition();
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
