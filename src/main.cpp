
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

// // Components.
// VL53L8CX sensor_vl53l8cx_top(&DEV_I2C, 0);

// uint8_t res = VL53L8CX_RESOLUTION_8X8;
// uint8_t status;

// void print_result(VL53L8CX_ResultsData* Result);

// /* Setup ---------------------------------------------------------------------*/
// void setup()
// {
//     SerialPort.begin(500000);

//     WiFi.mode(WIFI_OFF);

//     // Initialize I2C bus.
//     DEV_I2C.begin();

//     // Configure VL53L8CX component.
//     sensor_vl53l8cx_top.begin();
//     status = sensor_vl53l8cx_top.init();
//     if (status) {
//         SerialPort.print("VL53L8CX initialization failed\r\n");
//         while (1)
//             ;
//     }
//     status = sensor_vl53l8cx_top.set_resolution(res);
//     if (status) {
//         SerialPort.print("VL53L8CX set resolution failed\r\n");
//         while (1)
//             ;
//     }
//     status = sensor_vl53l8cx_top.set_ranging_frequency_hz(15); // 15hz max for 8x8, 60hz max for 4x4
//     if (status) {
//         SerialPort.print("VL53L8CX set ranging frequency failed\r\n");
//         while (1)
//             ;
//     }
//     status = sensor_vl53l8cx_top.set_ranging_mode(VL53L8CX_RANGING_MODE_CONTINUOUS);
//     if (status) {
//         SerialPort.print("VL53L8CX set ranging mode failed\r\n");
//         while (1)
//             ;
//     }

//     // Start Measurements
//     status = sensor_vl53l8cx_top.start_ranging();
// }

// void loop()
// {
//     VL53L8CX_ResultsData Results;
//     uint8_t NewDataReady = 0;

//     status = sensor_vl53l8cx_top.check_data_ready(&NewDataReady);
//     if (status) {
//         SerialPort.print("VL53L8CX check data ready failed\r\n");
//     }

//     if (NewDataReady) {
//         status = sensor_vl53l8cx_top.get_ranging_data(&Results);
//         if (status) {
//             SerialPort.print("VL53L8CX get ranging data failed\r\n");
//         }
//         print_result(&Results);
//     }
// }

// void print_result(VL53L8CX_ResultsData* Result)
// {
//     SerialPort.print("M");
//     SerialPort.print(millis());
//     SerialPort.print("\nD");
//     // Print distance_mm
//     for (int i = 0; i < res; i++) {
//         SerialPort.print(Result->distance_mm[i]);
//         SerialPort.print(",");
//     }
//     SerialPort.print("\nA");
//     for (int i = 0; i < VL53L8CX_RESOLUTION_8X8; i++) {
//         SerialPort.print(Result->ambient_per_spad[i]);
//         SerialPort.print(",");
//     }
//     SerialPort.print("\nT");
//     for (int i = 0; i < VL53L8CX_RESOLUTION_8X8; i++) {
//         SerialPort.print(Result->nb_target_detected[i]);
//         SerialPort.print(",");
//     }
//     // Print nb_spads_enabled
//     SerialPort.print("\nE");
//     for (int i = 0; i < VL53L8CX_RESOLUTION_8X8; i++) {
//         SerialPort.print(Result->nb_spads_enabled[i]);
//         SerialPort.print(",");
//     }
//     // Print signal_per_spad
//     SerialPort.print("\nG");
//     for (int i = 0; i < VL53L8CX_RESOLUTION_8X8 * VL53L8CX_NB_TARGET_PER_ZONE; i++) {
//         SerialPort.print(Result->signal_per_spad[i]);
//         SerialPort.print(",");
//     }
//     // Print range_sigma_mm
//     SerialPort.print("\nS");
//     for (int i = 0; i < VL53L8CX_RESOLUTION_8X8 * VL53L8CX_NB_TARGET_PER_ZONE; i++) {
//         SerialPort.print(Result->range_sigma_mm[i]);
//         SerialPort.print(",");
//     }
//     SerialPort.print("\nR");
//     for (int i = 0; i < VL53L8CX_RESOLUTION_8X8 * VL53L8CX_NB_TARGET_PER_ZONE; i++) {
//         SerialPort.print(Result->reflectance[i]);
//         SerialPort.print(",");
//     }
//     // Print target_status
//     SerialPort.print("\nU");
//     for (int i = 0; i < VL53L8CX_RESOLUTION_8X8 * VL53L8CX_NB_TARGET_PER_ZONE; i++) {
//         SerialPort.print(Result->target_status[i]);
//         SerialPort.print(",");
//     }
//     SerialPort.println();
// }
