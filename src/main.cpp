
#include "distanceSensors/distanceData.h"
#include "distanceSensors/distanceSensorVL53L8cxMultiplexer.h"

#include "lineSensors/lineSensorADS1115.h"

#include "alarmSpeakers/alarmSpeakerPicoPio.h"
#include "alarmSpeakers/hornController.h"

#include <Arduino.h>

#include <Wifi.h>

const uint8_t frontSensorMuxAddress = 0x70;
const uint8_t frontSensorDataWidth = 3 * 8;
const uint8_t frontSensorDataHeight = 8; // 8 rows
DistanceSensorVL53L8cxMultiplexer sensorLeft(Wire1, frontSensorMuxAddress, 1);
DistanceSensorVL53L8cxMultiplexer sensorCenter(Wire1, frontSensorMuxAddress, 2);
DistanceSensorVL53L8cxMultiplexer sensorRight(Wire1, frontSensorMuxAddress, 3);
volatile DistanceData distanceData[frontSensorDataHeight][frontSensorDataWidth]; // 8 rows (height) x 24 cols (3 sensors of 8 cols each)

const uint8_t lineSensor1Address = 0x49;
LineSensorADS1115 lineSensorBack(Wire, lineSensor1Address);

AlarmSpeakerPicoPio alarmSpeaker(14); // uses pin 14 and 15
HornController horn(alarmSpeaker);

volatile bool setupDone = false;

void setup()
{
    Serial.begin(500000);
    // leftAudio.begin();
    // rightAudio.begin();

    while (!setupDone) {
        delay(100);
    }
}

// vl53l8cx sensors provide lots of data but take significant amounts of time to transmit it over I2C so I'll use the second processor of the pico to read the sensors
void setup1()
{
    Wire.begin();

    horn.begin(); // also calls begin() on the alarm speaker

    // lineSensorBack.begin();
    // lineSensorFront.begin();

    // longRangeTop.begin();
    // longRangeBottom.begin();

    // Wire1.begin();
    // sensorLeft.begin();
    // sensorCenter.begin();
    // sensorRight.begin();

    setupDone = true;
}
void loop1()
{ // slow loop for sensors that are slow to poll
    sensorLeft.run();
    sensorCenter.run();
    sensorRight.run();
}

void loop()
{ // fast main loop
    horn.run();

    // horn.alarm(millis() < 30000);

    lineSensorBack.run();
    if (lineSensorBack.isMeasurementReady()) {
        int8_t linePos = lineSensorBack.getLinePosition();
    }

    bool anythingNew = false;
    if (sensorLeft.isMeasurementReady()) {
        anythingNew = true;
        sensorLeft.getDistanceData((DistanceData*)distanceData, 0, 0, frontSensorDataWidth, frontSensorDataHeight);
    }
    if (sensorCenter.isMeasurementReady()) {
        anythingNew = true;
        sensorCenter.getDistanceData((DistanceData*)distanceData, 8, 0, frontSensorDataWidth, frontSensorDataHeight);
    }
    if (sensorRight.isMeasurementReady()) {
        anythingNew = true;
        sensorRight.getDistanceData((DistanceData*)distanceData, 16, 0, frontSensorDataWidth, frontSensorDataHeight);
    }

    if (anythingNew) {
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
