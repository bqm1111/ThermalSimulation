#ifndef UTILS_H
#define UTILS_H
#include <iostream>
#include "define.h"
#include "common.h"
#include "math.h"

void getRi2bMatrix(float* matrix, RotationAngle angle);
void getRb2cMatrix(float* matrix, RotationAngle angle);
void getRe2iMatrix(float* matrix, GPS gps);
GPS ECEF2Geoditic(Coordinate pos);
Coordinate Geoditic2ECEF(GPS gps);
float deg2rad(float deg);
float rad2deg(float rad);
void mul3x3(float *dst, float *src1, float *src2);
void mul3x3TransposeFirst(float *dst, float *src1, float *src2);
void mul3x3TransposeBoth(float *dst, float *src1, float *src2);
void mul3x3ToVec3x1(float *dst, float *mat, float *vec);
std::vector<std::pair<std::string, std::vector<float>>> read_csv(std::string filename);
bool readFromFile(std::string filename, float *arr, int numLine, int numPerLine);
void printObjStatus(ObjStatus * obj, int numLine);
void printSeekerInfo(SeekerInfo * obj, int numLine);
void genRandomMat(cv::Mat src);
#endif
