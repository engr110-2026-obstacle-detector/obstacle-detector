
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
const uint32_t alarmSpeakerLoudestFrequency = 2800; // Hz, 3200 is the true loudest but it hurts my ears
const uint32_t alarmSpeakerHornFrequency = 220;

// audio board
const uint8_t audioBoardTxPin = 12;
const uint8_t audioBoardRxPin = 13;

// END OF PINS SECTION

OrientationSensorICM20948 centralOrientationSensor(centralOrientationSensorSPI, centralOrientationSensorCSPin);
OrientationData centralOrientationData;

OrientationSensorMpu6050 frontOrientationSensor(frontSensorIMUAddress, frontSensorI2CBus);
OrientationData frontOrientationData;
#include "orientationSensors/complementaryOrientationFilter.h"
ComplementaryOrientationFilter frontSensorPitchFilter(0.01, -135); // alpha, offset
float frontSensorPitchAngle = 0;

const uint8_t frontSensorDataWidth = 3 * 8;
const uint8_t frontSensorDataHeight = 8; // 8 rows
DistanceSensorVL53L8cxMultiplexer sensorLeft(frontSensorI2CBus, frontSensorMuxAddress, 1); // left is mux channel 1
DistanceSensorVL53L8cxMultiplexer sensorCenter(frontSensorI2CBus, frontSensorMuxAddress, 2); // center is mux channel 2
DistanceSensorVL53L8cxMultiplexer sensorRight(frontSensorI2CBus, frontSensorMuxAddress, 3); // right is mux channel 3
volatile DistanceData distanceData[frontSensorDataHeight][frontSensorDataWidth]; // 8 rows (height) x 24 cols (3 sensors of 8 cols each)

// const uint8_t lineSensor1Address = 0x49;
// LineSensorADS1115 lineSensorBack(Wire, lineSensor1Address);

AlarmSpeakerPicoPio alarmSpeaker(alarmSpeakerPin, alarmSpeakerLoudestFrequency);
HornController horn(alarmSpeaker, alarmSpeakerHornFrequency);

AudioBoardDY1703aSoftSerial audioBoard(audioBoardRxPin, audioBoardTxPin); // rx pin, tx pin

float voltsPerADCUnit = 0.00512;
void powerOffCallback()
{
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
    delay(TRACK_POWER_UP_TIME);
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
    // Serial.println("I2C bus initialized");
    frontOrientationSensor.begin();

    sensorLeft.begin();
    // Serial.println("Left sensor initialized");

    sensorCenter.begin();
    // Serial.println("Center sensor initialized");

    sensorRight.begin();
    // Serial.println("Right sensor initialized");

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
    frontOrientationSensor.run();

    // lineSensorFront.run();
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

    static bool complainedAboutFrontPitchAngle = false;
    if (frontOrientationSensor.isMeasurementReady()) {
        frontOrientationSensor.getOrientationData(frontOrientationData);

        frontSensorPitchAngle = frontSensorPitchFilter.update(frontOrientationData);

        // Serial.println(frontSensorPitchAngle);

        // TODO: compare to angle from central sensor
        // if (!complainedAboutFrontPitchAngle && abs(frontSensorPitchAngle) > 45) {
        //     complainedAboutFrontPitchAngle = true;
        //     audioBoard.playTrack(TRACK_FRONT_SENSOR_NOT_LEVEL);
        //     Serial.println("front sensor not level");
        // }
        // if (complainedAboutFrontPitchAngle && abs(frontSensorPitchAngle) < 20) {
        //     complainedAboutFrontPitchAngle = false;
        // }
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

    static int32_t frontSensorInitialization = 10;
    static int32_t frontSensorZeros[frontSensorDataHeight][frontSensorDataWidth] = { 0 }; // distances measured from flat floor at startup
    static int32_t frontSensorZerosCounts[frontSensorDataHeight][frontSensorDataWidth] = { 0 }; // number of measurements taken for each cell of frontSensorZeros, used to calculate average

    if (digitalRead(linePin) == LOW) { // for testing, re-zero when line button is pushed
        frontSensorInitialization = 10;
        for (int row = 0; row < frontSensorDataHeight; row++) {
            for (int col = 0; col < frontSensorDataWidth; col++) {
                frontSensorZeros[row][col] = 0;
                frontSensorZerosCounts[row][col] = 0;
            }
        }
    }

    if (anythingNewFromFrontSensors) {
        if (frontSensorInitialization > 0) {
            frontSensorInitialization--;

            for (int row = 0; row < frontSensorDataHeight; row++) {
                for (int col = 0; col < frontSensorDataWidth; col++) {
                    if (distanceData[row][col].isValid) {
                        frontSensorZerosCounts[row][col]++;
                        frontSensorZeros[row][col] += distanceData[row][col].distanceMm;
                    }
                }
            }

            if (frontSensorInitialization == 0) {
                for (int row = 0; row < frontSensorDataHeight; row++) {
                    for (int col = 0; col < frontSensorDataWidth; col++) {
                        frontSensorZeros[row][col] /= frontSensorZerosCounts[row][col];
                    }
                }

                Serial.println("Front sensors initialized with zero values of:");
                for (int row = 0; row < frontSensorDataHeight; row++) {
                    for (int col = 0; col < frontSensorDataWidth; col++) {
                        Serial.print(frontSensorZeros[row][col]);
                        Serial.print("\t");
                    }
                    Serial.println();
                }
                Serial.println();
                delay(1000);
            }
        } else {
            const int LEFT = 0;
            const int CENTER = 1;
            const int RIGHT = 2;
            const int DROP = 0;
            const int OBJECT = 1;
            int detectionCounts[2][3] = { 0 }; // [object/drop][left/center/right]
            int32_t objectThresholdPerThousand = -75;
            int32_t dropThresholdPerThousand = 75;
            for (int row = 0; row < frontSensorDataHeight; row++) {
                for (int col = 0; col < frontSensorDataWidth; col++) {
                    if (distanceData[row][col].isValid && frontSensorZeros[row][col] != 0) {
                        int32_t adjustedDistance = distanceData[row][col].distanceMm - frontSensorZeros[row][col];
                        // Serial.print(adjustedDistance * 1000 / frontSensorZeros[row][col]);
                        if (adjustedDistance < frontSensorZeros[row][col] * objectThresholdPerThousand / 1000) {
                            detectionCounts[OBJECT][col / 8]++;
                            // Serial.print("o ");
                        } else if (adjustedDistance > frontSensorZeros[row][col] * dropThresholdPerThousand / 1000) {
                            detectionCounts[DROP][col / 8]++;
                            // Serial.print("d ");
                        }
                    } else {
                        // Serial.print("x ");
                    }
                    // Serial.print("\t");
                }
                // Serial.println();
            }

            Serial.print("objectPixels:\t");
            for (int i = 0; i < 3; i++) {
                Serial.print(detectionCounts[OBJECT][i]);
                Serial.print("\t");
            }
            Serial.print("\t");
            Serial.print("dropPixels:\t");
            for (int i = 0; i < 3; i++) {
                Serial.print(detectionCounts[DROP][i]);
                Serial.print("\t");
            }
            Serial.println();

            static bool alertedYet[2][3] = { false }; // [object/drop][left/center/right]
            const int dropPixelDetectionThreshold = 8; // alert if above this
            const int objectPixelDetectionThreshold = 8;
            const int dropPixelDetectionThreshold_Low = 5; // reset if below this
            const int objectPixelDetectionThreshold_Low = 5;

            const int alertTracks[2][3] = {
                { TRACK_DROP_LEFT, TRACK_DROP_FRONT, TRACK_DROP_RIGHT },
                { TRACK_OBJECT_LEFT, TRACK_OBJECT_FRONT, TRACK_OBJECT_RIGHT },
            };

            for (int type = 0; type < 2; type++) {
                for (int pos = 0; pos < 3; pos++) {
                    if (!alertedYet[type][pos] && detectionCounts[type][pos] >= (type == OBJECT ? objectPixelDetectionThreshold : dropPixelDetectionThreshold)) {
                        alertedYet[type][pos] = true;
                        if (!audioBoard.isPlaying()) {
                            audioBoard.playTrack(alertTracks[type][pos]);
                        }
                    } else if (alertedYet[type][pos] && detectionCounts[type][pos] < (type == OBJECT ? objectPixelDetectionThreshold_Low : dropPixelDetectionThreshold_Low)) {
                        alertedYet[type][pos] = false;
                        // audioBoard.stop();
                    }
                }
            }

            // if (distanceData[4][12].isValid && distanceData[4][12].distanceMm < 100) {
            //     if (!detectedObjectInFront) {
            //         detectedObjectInFront = true;
            //         audioBoard.playTrack(TRACK_OBJECT_FRONT);
            //     }
            // } else {
            //     if (detectedObjectInFront) {
            //         detectedObjectInFront = false;
            //     }
            // }
        }
    }

    // if (anythingNewFromFrontSensors) {
    //     for (int row = 0; row < frontSensorDataHeight; row++) {
    //         for (int col = 0; col < frontSensorDataWidth; col++) {
    //             if (distanceData[row][col].isValid) {
    //                 Serial.print(distanceData[row][col].distanceMm);
    //             }
    //             Serial.print("\t");
    //         }
    //         Serial.println();
    //     }
    //     Serial.println();
    // }

    // if (frontOrientationSensor.isMeasurementReady()) {
    //     frontOrientationSensor.getOrientationData(frontOrientationData);
    //     Serial.println();
    //     Serial.print(frontOrientationData.Ax);
    //     Serial.print("\t");
    //     Serial.print(frontOrientationData.Ay);
    //     Serial.print("\t");
    //     Serial.print(frontOrientationData.Az);
    //     Serial.print("\t");
    //     Serial.print(frontOrientationData.Gx);
    //     Serial.print("\t");
    //     Serial.print(frontOrientationData.Gy);
    //     Serial.print("\t");
    //     Serial.print(frontOrientationData.Gz);
    //     Serial.println();
    //     Serial.println();
    // }
}
