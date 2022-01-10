#ifndef UTILS_H
#define UTILS_H
#include <iostream>
#include "define.h"
#include "common.h"
#include "math.h"

void getRotationMatrix(float* matrix, RotationAngle angle);
GPS ECEF2Geoditic(Coordinate pos);
Coordinate Geoditic2ECEF(GPS gps);
#endif
