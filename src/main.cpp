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

#include "point.h"

#include "lineSegmentation.h"
#include "ransacSegmentation.h"

#include <Arduino.h>

#include <Wifi.h>

// #define ALGORITHM 1 // tried basic thresholds, subtraction from ground zero, and convolution-based
// #define ALGORITHM 2 // plane finding, region growing segmentation https://pointclouds.org/documentation/tutorials/region_growing_segmentation.html, ai generated, just freezes
// #define ALGORITHM 3 // tried to fit line segments to each column, sets all point segmentIDs to 0, ai generated
// #define ALGORITHM 4 // performance optimized line fitting, handmade, kind of worked but still not that solid of a signal
// #define ALGORITHM 5 // calculating gradient from 3d points
#define ALGORITHM 6 // calculating gradient from 3d points from rolling averaging 4 distance measurements

// https://github.com/lorenwel/linefit_ground_segmentation

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
const uint32_t alarmSpeakerLoudestFrequency = 3000; // Hz, 3200 is the true loudest but it hurts my ears
const uint32_t alarmSpeakerHornFrequency = 440;

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

const uint8_t numToAverage = 4;
uint8_t frontSensorDataIndex = 0;
DistanceData distanceDataHistory[numToAverage][frontSensorDataHeight][frontSensorDataWidth];

const uint8_t lineSensor1Address = 0x49;
const uint8_t lineSensorSDA = 0;
const uint8_t lineSensorSCL = 1;
LineSensorADS1115 lineSensorFront(Wire, lineSensor1Address);

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

bool anythingNewFromFrontSensors = false;
#include "misc.h"

DistanceData convolutionOutput[frontSensorDataHeight][frontSensorDataWidth];

Point pointcloud[frontSensorDataHeight][frontSensorDataWidth];

void sensorToPointCloud(DistanceData* distanceData, Point* pointCloud, int distanceDataColOffset, int sensorNumCols, float focal_length_x, float focal_length_y, float sensorYaw)
{
    // x is forward, y is left, z is up
    float cyaw = cos(radians(sensorYaw));
    float syaw = sin(radians(sensorYaw));
    for (int y = 0; y < frontSensorDataHeight; y++) {
        for (int x = 0; x < sensorNumCols; x++) {
            int arrayIndex = y * frontSensorDataWidth + distanceDataColOffset + x;
            if (distanceData[arrayIndex].isValid) {
                pointCloud[arrayIndex].valid = true;
                float xcoord = distanceData[arrayIndex].distanceMm;
                float ycoord = (x - sensorNumCols / 2) * xcoord / focal_length_x;
                float zcoord = (y - frontSensorDataHeight / 2) * xcoord / focal_length_y;

                // rotate (yaw) aroound z axis by sensorYaw since there are 3 sensors with different yaws
                pointCloud[arrayIndex].x = xcoord * cyaw - ycoord * syaw;
                pointCloud[arrayIndex].y = ycoord * cyaw + xcoord * syaw;
                pointCloud[arrayIndex].z = zcoord;

            } else {
                pointCloud[arrayIndex].valid = false;
            }
        }
    }
}

void setup()
{
    powerControl.on();
    audioBoard.begin();
    powerControl.start();
    delay(15);
    Serial.begin(1000000);
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

    Wire.setSDA(lineSensorSDA);
    Wire.setSCL(lineSensorSCL);
    Wire.begin();
    Wire.setTimeout(25, false);
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

    lineSensorFront.run();
    // lineSensorBack.run();
    delay(5);
}

void ALGORITHM_1();
void ALGORITHM_2();
void ALGORITHM_3();
void ALGORITHM_4();
void ALGORITHM_5();

/**
 * @brief
 * @param  input[]:
 * @param  output[]:
 * @param  kernel[]:
 * @param  input_width:
 * @param  output_width:
 * @param  output_height:
 * @param  kernel_width:
 * @param  kernel_height:
 * @param  validThreshold:
 * @retval
 */
bool convolution(DistanceData input[], DistanceData output[], int32_t kernel[], int input_width, int input_height, int output_width, int output_height, int kernel_width, int kernel_height, int validThreshold, bool countZeroKernelAsValid)
{
    if (output_height < input_height - kernel_height + 1 || output_width < input_width - kernel_width + 1) {
        // Serial.println("Invalid output dimensions for convolution");
        return false;
    }
    if (validThreshold <= 0) {
        // Serial.println("validThreshold must be greater than 0");
        return false;
    }
    for (int inx = 0; inx < input_width - kernel_width + 1; inx++) {
        for (int iny = 0; iny < input_height - kernel_height + 1; iny++) {
            // for each output pixel
            int validCount = 0;
            int32_t sum = 0;
            for (int kx = 0; kx < kernel_width; kx++) {
                for (int ky = 0; ky < kernel_height; ky++) {
                    if (input[(iny + ky) * input_width + (inx + kx)].isValid || (countZeroKernelAsValid && kernel[ky * kernel_width + kx] == 0)) {
                        validCount++;
                        sum += input[(iny + ky) * input_width + (inx + kx)].distanceMm * kernel[ky * kernel_width + kx];
                    }
                }
            }
            if (validCount >= validThreshold) {
                output[iny * output_width + inx].isValid = true;
                output[iny * output_width + inx].distanceMm = sum / validCount;
            } else {
                output[iny * output_width + inx].isValid = false;
            }
        }
    }
    return true;
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
        if (!complainedAboutFrontPitchAngle && abs(frontSensorPitchAngle) > 35) {
            complainedAboutFrontPitchAngle = true;
            // audioBoard.playTrack(TRACK_FRONT_SENSOR_NOT_LEVEL);
            Serial.println("front sensor not level");
        }
        if (complainedAboutFrontPitchAngle && abs(frontSensorPitchAngle) < 20) {
            complainedAboutFrontPitchAngle = false;
        }
        Serial.print("DiSp,P,");
        Serial.println(frontSensorPitchAngle);
        Serial.print("DiSp,V,");
        Serial.println(analogRead(batMonPin) * voltsPerADCUnit);
    }

    // if (lineSensorBack.isMeasurementReady()) {
    //     int8_t linePos = lineSensorBack.getLinePosition();
    // }
    const int lineSensorPeriod = 1100;
    static unsigned int lineSensorLastMillis = 0;
    if (lineSensorFront.isMeasurementReady()) {
        int8_t linePos = lineSensorFront.getLinePosition();
        Serial.printf("line position: %d\n", linePos);
        bool isLineDetected = lineSensorFront.isLineDetected();
        Serial.printf("line detected: %s\n", isLineDetected ? "true" : "false");
        int16_t lineSensorReadings[4];
        lineSensorFront.getRawReadings(lineSensorReadings);
        Serial.print("Line sensor readings: ");
        for (int i = 0; i < 4; i++) {
            Serial.print(lineSensorReadings[i]);
            Serial.print("\t");
        }
        Serial.println();
        if (millis() - lineSensorLastMillis > lineSensorPeriod) {
            lineSensorLastMillis = millis();
            if (isLineDetected) {
                if (linePos < -50) {
                    audioBoard.playTrack(TRACK_LINE_LEFT);
                } else if (linePos > 50) {
                    audioBoard.playTrack(TRACK_LINE_RIGHT);
                } else {
                    audioBoard.playTrack(TRACK_LINE_CENTER);
                }
            }
        }
    }

    anythingNewFromFrontSensors = false;
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
#if ALGORITHM == 1 // ALGORITHM 1
    ALGORITHM_1();
#endif
    if (anythingNewFromFrontSensors) {
        Serial.print("DiSp,A,");
        for (int row = 0; row < frontSensorDataHeight; row++) {
            for (int col = 0; col < frontSensorDataWidth; col++) {
                if (distanceData[row][col].isValid) {
                    Serial.print(distanceData[row][col].distanceMm);
                } else {
                    Serial.print("nan");
                }
                Serial.print(",");
            }
        }
        Serial.println();

#if ALGORITHM == 6
        // save data to history
        for (int row = 0; row < frontSensorDataHeight; row++) {
            for (int col = 0; col < frontSensorDataWidth; col++) {
                distanceDataHistory[frontSensorDataIndex][row][col].distanceMm = distanceData[row][col].distanceMm;
                distanceDataHistory[frontSensorDataIndex][row][col].isValid = distanceData[row][col].isValid;
            }
        }
        // calculate average
        frontSensorDataIndex = (frontSensorDataIndex + 1) % numToAverage;
        DistanceData distanceDataAvg[frontSensorDataHeight][frontSensorDataWidth];
        for (int row = 0; row < frontSensorDataHeight; row++) {
            for (int col = 0; col < frontSensorDataWidth; col++) {
                int validCount = 0;
                int32_t distanceSum = 0;
                for (int i = 0; i < numToAverage; i++) {
                    if (distanceDataHistory[i][row][col].isValid) {
                        validCount++;
                        distanceSum += distanceDataHistory[i][row][col].distanceMm;
                    }
                }
                if (validCount == numToAverage) { // only consider it valid if all measurements are valid
                    distanceDataAvg[row][col].isValid = true;
                    distanceDataAvg[row][col].distanceMm = distanceSum / validCount;
                } else {
                    distanceDataAvg[row][col].isValid = false;
                }
            }
        }

        sensorToPointCloud((DistanceData*)distanceDataAvg, (Point*)pointcloud, 8, 8, 8.45, 8.45, 0);
        sensorToPointCloud((DistanceData*)distanceDataAvg, (Point*)pointcloud, 0, 8, 8.45, 8.45, -45);
        sensorToPointCloud((DistanceData*)distanceDataAvg, (Point*)pointcloud, 16, 8, 8.45, 8.45, 45);
#else
        sensorToPointCloud((DistanceData*)distanceData, (Point*)pointcloud, 8, 8, 8.45, 8.45, 0);
        sensorToPointCloud((DistanceData*)distanceData, (Point*)pointcloud, 0, 8, 8.45, 8.45, -45);
        sensorToPointCloud((DistanceData*)distanceData, (Point*)pointcloud, 16, 8, 8.45, 8.45, 45);

#endif

#if ALGORITHM == 2
        ALGORITHM_2();
#endif

#if ALGORITHM == 3
        ALGORITHM_3();
#endif

#if ALGORITHM == 4
        ALGORITHM_4();
        Serial.print("DiSp,I,");
        for (int row = 0; row < frontSensorDataHeight; row++) {
            for (int col = 0; col < frontSensorDataWidth; col++) {
                if (pointcloud[row][col].valid) {
                    Serial.print(pointcloud[row][col].segment);

                    Serial.print(",");
                }
            }
        }
        Serial.println();
        delay(50);
#endif

#if ALGORITHM == 5 || ALGORITHM == 6
        ALGORITHM_5();
        Serial.print("DiSp,B,");
        for (int row = 0; row < frontSensorDataHeight; row++) {
            for (int col = 0; col < frontSensorDataWidth; col++) {
                if (pointcloud[row][col].segment != INT16_MIN) {
                    Serial.print(pointcloud[row][col].segment);

                    Serial.print(",");
                } else {
                    Serial.print("nan,");
                }
            }
        }
        Serial.println();
        delay(20);

#endif

        Serial.print("DiSp,X,");
        for (int row = 0; row < frontSensorDataHeight; row++) {
            for (int col = 0; col < frontSensorDataWidth; col++) {
                if (pointcloud[row][col].valid) {
                    Serial.print(pointcloud[row][col].x);

                    Serial.print(",");
                }
            }
        }
        Serial.println();
        delay(50);
        Serial.print("DiSp,Y,");
        for (int row = 0; row < frontSensorDataHeight; row++) {
            for (int col = 0; col < frontSensorDataWidth; col++) {
                if (pointcloud[row][col].valid) {
                    Serial.print(pointcloud[row][col].y);

                    Serial.print(",");
                }
            }
        }
        Serial.println();
        delay(50);
        Serial.print("DiSp,Z,");
        for (int row = 0; row < frontSensorDataHeight; row++) {
            for (int col = 0; col < frontSensorDataWidth; col++) {
                if (pointcloud[row][col].valid) {
                    Serial.print(pointcloud[row][col].z);
                    Serial.print(",");
                }
            }
        }
        Serial.println();
        delay(50);

        // convolution((DistanceData*)distanceData, (DistanceData*)convolutionOutput, (int32_t[]) { -1, -2, -1, 0, 0, 0, 1, 2, 1 }, frontSensorDataWidth, frontSensorDataHeight, frontSensorDataWidth, frontSensorDataHeight, 3, 3, 9, true);
        // Serial.print("DiSp,C,");
        // for (int row = 0; row < frontSensorDataHeight; row++) {
        //     for (int col = 0; col < frontSensorDataWidth; col++) {
        //         if (convolutionOutput[row][col].isValid) {
        //             Serial.print(convolutionOutput[row][col].distanceMm);
        //         } else {
        //             Serial.print("nan");
        //         }
        //         Serial.print(",");
        //     }
        // }
        // Serial.println();
    }

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

void ALGORITHM_5()
{
    // calculate xz gradient of points
    for (int row = 0; row < frontSensorDataHeight - 2; row++) {
        for (int col = 0; col < frontSensorDataWidth; col++) {
            if (pointcloud[row][col].valid && pointcloud[row + 2][col].valid) {
                float dz = pointcloud[row + 2][col].z - pointcloud[row][col].z;
                float dx = pointcloud[row + 2][col].x - pointcloud[row][col].x;
                float gradient = atan2(dz, dx) * 180 / PI; // in degrees
                gradient += 180 + 45;
                if (gradient > 180) {
                    gradient -= 360;
                }
                pointcloud[row][col].segment = (int16_t)(gradient); // store gradient*1024 in segment field for now
            } else {
                pointcloud[row][col].segment = INT16_MIN; // invalid gradient
            }
        }
    }
    for (int col = 0; col < frontSensorDataWidth; col++) { // bottom row (invalid, since 7 rows of gradients can be calculated from 8 points)
        pointcloud[frontSensorDataHeight - 1][col].segment = INT16_MIN; // invalid gradient
    }
    for (int col = 0; col < frontSensorDataWidth; col++) { // bottom row (invalid, since 7 rows of gradients can be calculated from 8 points)
        pointcloud[frontSensorDataHeight - 2][col].segment = INT16_MIN; // invalid gradient
    }

    // calculate average segment value across all valid elements from ro 0 to row height-2
    int64_t segmentSum = 0;
    int validCount = 0;
    for (int row = 0; row < frontSensorDataHeight - 2; row++) {
        for (int col = 0; col < frontSensorDataWidth; col++) {
            if (pointcloud[row][col].segment != INT16_MIN) {
                segmentSum += pointcloud[row][col].segment;
                validCount++;
            }
        }
    }

    int16_t averageSegment = validCount > 0 ? segmentSum / validCount : 0;

    // for (int row = 0; row < frontSensorDataHeight - 2; row++) {
    //     for (int col = 0; col < frontSensorDataWidth; col++) {
    //         if (pointcloud[row][col].segment != INT16_MIN) {
    //             pointcloud[row][col].segment -= averageSegment;
    //         }
    //     }
    // }

    pointcloud[frontSensorDataHeight - 1][0].segment = averageSegment;

    const int LEFT = 0;
    const int CENTER = 1;
    const int RIGHT = 2;
    const int DROP = 0;
    const int OBJECT = 1;

    int detectionCounts[2][3] = { 0 }; // [object/drop][left/center/right]
    int32_t objectThreshold = -26;
    int32_t dropThreshold = -2;

    for (int row = 0; row < frontSensorDataHeight - 1; row++) {
        for (int col = 0; col < frontSensorDataWidth; col++) {
            if (pointcloud[row][col].valid && pointcloud[row][col].segment != INT16_MIN) {
                int sensorIndex = col / 8;
                int posIndex;
                if (sensorIndex == 0) {
                    posIndex = LEFT;
                } else if (sensorIndex == 1) {
                    posIndex = CENTER;
                } else {
                    posIndex = RIGHT;
                }
                if (pointcloud[row][col].segment <= objectThreshold) {
                    detectionCounts[OBJECT][posIndex]++;
                } else if (pointcloud[row][col].segment >= dropThreshold) {
                    detectionCounts[DROP][posIndex]++;
                }
            }
        }
    }

    pointcloud[frontSensorDataHeight - 2][10].segment = detectionCounts[1][0];
    pointcloud[frontSensorDataHeight - 2][11].segment = detectionCounts[1][1];
    pointcloud[frontSensorDataHeight - 2][12].segment = detectionCounts[1][2];
    pointcloud[frontSensorDataHeight - 1][10].segment = detectionCounts[0][0];
    pointcloud[frontSensorDataHeight - 1][11].segment = detectionCounts[0][1];
    pointcloud[frontSensorDataHeight - 1][12].segment = detectionCounts[0][2];

    static bool alertedYet[2][3] = { false }; // [object/drop][left/center/right]
    const int dropPixelDetectionThreshold = 4; // alert if above this
    const int objectPixelDetectionThreshold = 8;
    const int dropPixelDetectionThreshold_Low = 2; // reset if below this
    const int objectPixelDetectionThreshold_Low = 4;

    const int center_dropPixelDetectionThreshold = 3; // alert if above this
    const int center_objectPixelDetectionThreshold = 4;
    const int center_dropPixelDetectionThreshold_Low = 1; // reset if below this
    const int center_objectPixelDetectionThreshold_Low = 1;

    const int alertTracks[2][3] = {
        { TRACK_DROP_LEFT, TRACK_DROP_FRONT, TRACK_DROP_RIGHT },
        { TRACK_OBJECT_LEFT, TRACK_OBJECT_FRONT, TRACK_OBJECT_RIGHT },
    };

    for (int type = 0; type < 2; type++) {
        for (int pos = 0; pos < 3; pos += 2) { // 0,1
            if (!alertedYet[type][pos] && detectionCounts[type][pos] >= (type == OBJECT ? objectPixelDetectionThreshold : dropPixelDetectionThreshold)) {
                alertedYet[type][pos] = true;
                if (!audioBoard.isPlaying()) {
                    if (type == OBJECT && pos == RIGHT) { // Corey Case: mute alerts for objects on the right since Corey should be there.
                    } else {
                        audioBoard.playTrack(alertTracks[type][pos]);
                    }
                }
            } else if (alertedYet[type][pos] && detectionCounts[type][pos] < (type == OBJECT ? objectPixelDetectionThreshold_Low : dropPixelDetectionThreshold_Low)) {
                alertedYet[type][pos] = false;
                // audioBoard.stop();
            }
        }
    }
    for (int type = 0; type < 2; type++) {
        int pos = 1;
        if (!alertedYet[type][pos] && detectionCounts[type][pos] >= (type == OBJECT ? center_objectPixelDetectionThreshold : center_dropPixelDetectionThreshold)) {
            alertedYet[type][pos] = true;
            if (!audioBoard.isPlaying()) {
                if (type == OBJECT && pos == RIGHT) { // Corey Case: mute alerts for objects on the right since Corey should be there.
                } else {
                    audioBoard.playTrack(alertTracks[type][pos]);
                }
            }
        } else if (alertedYet[type][pos] && detectionCounts[type][pos] < (type == OBJECT ? center_objectPixelDetectionThreshold_Low : center_dropPixelDetectionThreshold_Low)) {
            alertedYet[type][pos] = false;
            // audioBoard.stop();
        }
    }
}

int32_t fastClosestLineDistance(Point pstart, Point pend, Point ptest, byte divisionPow2 = 4)
{
    int dx = pend.x - pstart.x;
    int dy = pend.y - pstart.y;
    int dz = pend.z - pstart.z;

    int stepx = dx >> divisionPow2;
    int stepy = dy >> divisionPow2;
    int stepz = dz >> divisionPow2;

    Point comparePoint = pstart;

    int32_t bestDistance = INT32_MAX;

    for (byte i = 0; i < (1 << divisionPow2) - 1; i++) {
        comparePoint.x += stepx;
        comparePoint.y += stepy;
        comparePoint.z += stepz;

        uint32_t distanceL1 = abs(comparePoint.x - ptest.x) + abs(comparePoint.y - ptest.y) + abs(comparePoint.z - ptest.z);
        if (distanceL1 < bestDistance) {
            bestDistance = distanceL1;
        }
    }
    return bestDistance;
}

void ALGORITHM_4()
{

    int segmentID = 1;
    for (int column = 0; column < frontSensorDataWidth; column++) {
        segmentID++;
        for (int row = 0; row < frontSensorDataHeight; row++) {
            pointcloud[row][column].segment = 0;
        }

        int startI = frontSensorDataHeight - 1;
        int midI = frontSensorDataHeight - 2;
        int endI = frontSensorDataHeight - 3;

        while (endI >= 0) {

            if (pointcloud[startI][column].valid == false) {
                endI--;
                midI--;
                startI--;
            }

            if (pointcloud[midI][column].valid == false) {
                endI--;
                midI--;
            }

            if (pointcloud[endI][column].valid == false) {
                endI--;
            }

            if (endI >= 0) {
                // we have three valid points to check
                int32_t dist = fastClosestLineDistance(pointcloud[startI][column], pointcloud[endI][column], pointcloud[midI][column]);
                // Serial.println(dist);
                if (dist < 35) {
                    pointcloud[startI][column].segment = segmentID;
                    pointcloud[endI][column].segment = segmentID;
                    pointcloud[midI][column].segment = segmentID;
                } else {
                    pointcloud[startI][column].segment = segmentID;
                    segmentID++;
                    pointcloud[endI][column].segment = segmentID;
                    pointcloud[midI][column].segment = segmentID;
                }
            }

            endI--;
            midI--;
            startI--;
        } // end while
    }
}

void ALGORITHM_3()
{
    // Per-column 3D line segmentation
    // For each of 24 sensor columns, detect discontinuities and fit 3D lines to segments
    // Minimum 3 points required per line segment

    const int DISCONTINUITY_THRESHOLD_MM = 50; // Gap threshold for detecting breaks
    const int RANSAC_TOLERANCE_MM = 25; // Distance tolerance for line inliers
    const int RANSAC_MAX_ITERATIONS = 30; // Max RANSAC iterations per segment
    const int MAX_SEGMENTS_PER_COLUMN = 8; // Max possible segments in a column

    static int nextSegmentId = 1; // Global segment ID counter

    // Initialize all points as unassigned
    for (int row = 0; row < 8; row++) {
        for (int col = 0; col < 24; col++) {
            pointcloud[row][col].segment = 0;
        }
    }

    // Process each column independently
    for (int col = 0; col < 24; col++) {
        // Extract single column into temporary array
        Point columnPoints[8];
        for (int row = 0; row < 8; row++) {
            columnPoints[row] = pointcloud[row][col];
        }

        // Detect segments within this column
        SegmentInfo segments[MAX_SEGMENTS_PER_COLUMN];
        int segmentCount = detectColumnSegments(columnPoints, 8, DISCONTINUITY_THRESHOLD_MM,
            segments, MAX_SEGMENTS_PER_COLUMN);

        // Fit line to each segment
        for (int segIdx = 0; segIdx < segmentCount; segIdx++) {
            SegmentInfo seg = segments[segIdx];

            // Collect points for this segment
            Point segmentPoints[8];
            int pointIdx = 0;
            for (int i = seg.startIdx; i <= seg.endIdx; i++) {
                if (columnPoints[i].valid) {
                    segmentPoints[pointIdx] = columnPoints[i];
                    pointIdx++;
                }
            }

            // Fit line via RANSAC
            int inlierCount = 0;
            Line bestLine = bestFitLine3D(segmentPoints, pointIdx, RANSAC_TOLERANCE_MM,
                RANSAC_MAX_ITERATIONS, inlierCount);

            // Assign segment IDs to points that are inliers to the best-fit line
            int currentSegmentId = nextSegmentId++;
            for (int i = seg.startIdx; i <= seg.endIdx; i++) {
                if (columnPoints[i].valid) {
                    float dist = pointToLineDistance(columnPoints[i], bestLine);
                    if (dist <= RANSAC_TOLERANCE_MM) {
                        // This point is an inlier - assign to segment
                        pointcloud[i][col].segment = currentSegmentId;
                    }
                }
            }
        }
    }
}

void ALGORITHM_2()
{
    // RANSAC-based flat surface segmentation
    // Segments the point cloud into regions containing flat surfaces
    // Each point gets assigned a unique segment ID based on plane fitting

    const int RANSAC_MAX_ITERATIONS = 100;
    const int RANSAC_TOLERANCE_MM = 50;
    const int MIN_SURFACE_SIZE = 9; // minimum points to form a valid surface

    // Initialize visited tracking for flood fill
    static bool visited[frontSensorDataHeight][frontSensorDataWidth];
    memset(visited, 0, sizeof(visited));

    static Point tempPoints[192];

    // Reset all segment IDs
    for (int row = 0; row < frontSensorDataHeight; row++) {
        for (int col = 0; col < frontSensorDataWidth; col++) {
            pointcloud[row][col].segment = 0;
        }
    }

    int nextSegmentId = 1;

    // Find connected components and segment each one
    for (int row = 0; row < frontSensorDataHeight; row++) {
        for (int col = 0; col < frontSensorDataWidth; col++) {
            if (!visited[row][col] && pointcloud[row][col].valid) {
                // Start a new connected component
                // First, gather all points in this component via flood fill
                int componentIndices[192]; // maximum 8*24 points
                int componentCount = 0;

                // Use a manual queue-based flood fill to avoid stack overflow
                int queue[192][2];
                int queueHead = 0, queueTail = 0;

                queue[queueTail][0] = row;
                queue[queueTail][1] = col;
                queueTail++;
                visited[row][col] = true;

                while (queueHead < queueTail) {
                    int r = queue[queueHead][0];
                    int c = queue[queueHead][1];
                    queueHead++;

                    // Add this point to component
                    componentIndices[componentCount++] = r * frontSensorDataWidth + c;

                    // Check 4-neighbors
                    int neighbors[4][2] = { { r - 1, c }, { r + 1, c }, { r, c - 1 }, { r, c + 1 } };
                    for (int n = 0; n < 4; n++) {
                        int nr = neighbors[n][0];
                        int nc = neighbors[n][1];

                        if (nr >= 0 && nr < frontSensorDataHeight && nc >= 0 && nc < frontSensorDataWidth && !visited[nr][nc] && pointcloud[nr][nc].valid) {
                            visited[nr][nc] = true;
                            queue[queueTail][0] = nr;
                            queue[queueTail][1] = nc;
                            queueTail++;
                        }
                    }
                }

                // Only process components with minimum surface size
                if (componentCount >= MIN_SURFACE_SIZE) {
                    // Create temporary Point array for this component
                    for (int i = 0; i < componentCount; i++) {
                        int idx = componentIndices[i];
                        int row = idx / frontSensorDataWidth;
                        int col = idx % frontSensorDataWidth;
                        pointcloud[row][col].segment = nextSegmentId;
                    }

                    // Run RANSAC to find best-fit plane for this component
                    int inlierCount = 0;
                    Plane bestPlane = bestFitPlane(tempPoints, componentCount,
                        RANSAC_TOLERANCE_MM,
                        RANSAC_MAX_ITERATIONS,
                        inlierCount);

                    // Assign points that are inliers to this segment
                    if (inlierCount >= MIN_SURFACE_SIZE) {
                        for (int i = 0; i < componentCount; i++) {
                            float dist = pointToPlaneDistance(tempPoints[i], bestPlane);
                            if (abs(dist) <= RANSAC_TOLERANCE_MM) {
                                int idx = componentIndices[i];
                                int row = idx / frontSensorDataWidth;
                                int col = idx % frontSensorDataWidth;
                                pointcloud[row][col].segment = nextSegmentId;
                            }
                        }
                        nextSegmentId++;
                    }
                }
            }
        }
    }
}

void ALGORITHM_1()
{

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
            // save data to history
            for (int row = 0; row < frontSensorDataHeight; row++) {
                for (int col = 0; col < frontSensorDataWidth; col++) {
                    // TODO: BE ABLE TO COPY with = operator
                    distanceDataHistory[frontSensorDataIndex][row][col].distanceMm = distanceData[row][col].distanceMm;
                    distanceDataHistory[frontSensorDataIndex][row][col].isValid = distanceData[row][col].isValid;
                }
            }
            // calculate average
            frontSensorDataIndex = (frontSensorDataIndex + 1) % numToAverage;
            DistanceData distanceDataAvg[frontSensorDataHeight][frontSensorDataWidth];
            for (int row = 0; row < frontSensorDataHeight; row++) {
                for (int col = 0; col < frontSensorDataWidth; col++) {
                    int validCount = 0;
                    int32_t distanceSum = 0;
                    for (int i = 0; i < numToAverage; i++) {
                        if (distanceDataHistory[i][row][col].isValid) {
                            validCount++;
                            distanceSum += distanceDataHistory[i][row][col].distanceMm;
                        }
                    }
                    if (validCount == numToAverage) { // only consider it valid if all measurements are valid
                        distanceDataAvg[row][col].isValid = true;
                        distanceDataAvg[row][col].distanceMm = distanceSum / validCount;
                    } else {
                        distanceDataAvg[row][col].isValid = false;
                    }
                }
            }

            const int LEFT = 0;
            const int CENTER = 1;
            const int RIGHT = 2;
            const int DROP = 0;
            const int OBJECT = 1;

            const int minDistanceToDetect = 300;
            int detectionCounts[2][3] = { 0 }; // [object/drop][left/center/right]
            int32_t objectThresholdPerThousand = -85;
            int32_t dropThresholdPerThousand = 85;

            // Serial.print("DiSp,B,");
            // for (int row = 0; row < frontSensorDataHeight; row++) {
            //     for (int col = 0; col < frontSensorDataWidth; col++) {
            //         if (distanceDataAvg[row][col].isValid && frontSensorZeros[row][col] != 0 && distanceDataAvg[row][col].distanceMm >= minDistanceToDetect) {
            //             int32_t adjustedDistance = distanceDataAvg[row][col].distanceMm - frontSensorZeros[row][col];
            //             Serial.print(adjustedDistance);
            //             if (adjustedDistance < frontSensorZeros[row][col] * objectThresholdPerThousand / 1000) {
            //                 detectionCounts[OBJECT][col / 8]++;
            //             } else if (adjustedDistance > frontSensorZeros[row][col] * dropThresholdPerThousand / 1000) {
            //                 detectionCounts[DROP][col / 8]++;
            //             }
            //         } else {
            //             Serial.print("nan");
            //         }
            //         Serial.print(",");
            //     }
            // }
            // Serial.println();

            // Serial.print("objectPixels:\t");
            // for (int i = 0; i < 3; i++) {
            //     Serial.print(detectionCounts[OBJECT][i]);
            //     Serial.print("\t");
            // }
            // Serial.print("\t");
            // Serial.print("dropPixels:\t");
            // for (int i = 0; i < 3; i++) {
            //     Serial.print(detectionCounts[DROP][i]);
            //     Serial.print("\t");
            // }
            // Serial.println();

            static bool alertedYet[2][3] = { false }; // [object/drop][left/center/right]
            const int dropPixelDetectionThreshold = 12; // alert if above this
            const int objectPixelDetectionThreshold = 12;
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
                            if (type == 0 && pos == 2) { // Corey Case: mute alerts for objects on the right since Corey should be there.
                            } else {
                                // audioBoard.playTrack(alertTracks[type][pos]);
                            }
                        }
                    } else if (alertedYet[type][pos] && detectionCounts[type][pos] < (type == OBJECT ? objectPixelDetectionThreshold_Low : dropPixelDetectionThreshold_Low)) {
                        alertedYet[type][pos] = false;
                        // audioBoard.stop();
                    }
                }
            }
        }
    }
}