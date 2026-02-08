#ifndef DISTANCE_DATA_H
#define DISTANCE_DATA_H
#include <stdint.h>
#include <stdbool.h>
typedef struct {
    int32_t distanceMm;
    bool isValid;
} DistanceData;

#endif // DISTANCE_DATA_H
