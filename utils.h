#ifndef UTILS_H
#define UTILS_H
#include <iostream>
#include "define.h"
#include "common.h"
#include "math.h"

void getRi2bMatrix(float* matrix, RotationAngle angle);
void getLdoniMatrix(float* matrix, RotationAngle angle);
void getMMatrix(float* matrix, GPS gps);
GPS ECEF2Geoditic(Coordinate pos);
Coordinate Geoditic2ECEF(GPS gps);
float deg2rad(float deg);
float rad2deg(float rad);
#endif
