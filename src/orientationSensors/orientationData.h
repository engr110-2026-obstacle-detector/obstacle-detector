#ifndef ORIENTATION_DATA_H
#define ORIENTATION_DATA_H
/**
 * @brief  x is forwards, y is left, z is up
 */
typedef struct {
    float Ax;
    float Ay;
    float Az;
    float Gx;
    float Gy;
    float Gz;
    float roll;
    float pitch;
    float yaw;
} OrientationData;
#endif // ORIENTATION_DATA_H
