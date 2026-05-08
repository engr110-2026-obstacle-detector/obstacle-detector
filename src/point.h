#ifndef POINT_H
#define POINT_H
#include <Arduino.h>
typedef struct {
    boolean valid;
    int16_t x;
    int16_t y;
    int16_t z;
    int16_t segment;
} Point;

#endif // POINT_H
