
#include "sensors/distanceSensors/distanceData.h"
#include "sensors/distanceSensors/distanceSensorVL53L8cxMultiplexer.h"

#include <Arduino.h>

#include <Wifi.h>

const uint8_t muxAddress = 0x70;
DistanceSensorVL53L8cxMultiplexer sensorLeft(Wire, muxAddress, 1);
DistanceSensorVL53L8cxMultiplexer sensorCenter(Wire, muxAddress, 2);
DistanceSensorVL53L8cxMultiplexer sensorRight(Wire, muxAddress, 3);

const uint8_t dataWidth = 2 * 8;
const uint8_t dataHeight = 8; // 8 rows

DistanceData distanceData[dataHeight][dataWidth]; // 8 rows (height) x 24 cols (3 sensors of 8 cols each)

#define WIRE Wire

void setup()
{
    Serial.begin(500000);

    Wire.begin();

    sensorLeft.begin();
    sensorCenter.begin();
    // sensorRight.begin();
}

void loop()
{
    sensorLeft.run();
    sensorCenter.run();
    // sensorRight.run();

    bool anythingNew = false;
    if (sensorLeft.isMeasurementReady()) {
        anythingNew = true;
        sensorLeft.getDistanceData((DistanceData*)distanceData, 0, 0, dataWidth, dataHeight);
    }
    if (sensorCenter.isMeasurementReady()) {
        anythingNew = true;
        sensorCenter.getDistanceData((DistanceData*)distanceData, 8, 0, dataWidth, dataHeight);
    }
    if (sensorRight.isMeasurementReady()) {
        anythingNew = true;
        sensorRight.getDistanceData((DistanceData*)distanceData, 16, 0, dataWidth, dataHeight);
    }

    if (anythingNew) {
        for (int row = 0; row < dataHeight; row++) {
            for (int col = 0; col < dataWidth; col++) {
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
