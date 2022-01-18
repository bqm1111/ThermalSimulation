#ifndef UTILS_H
#define UTILS_H
#include <iostream>
#include "define.h"
#include "common.h"
#include "math.h"

extern __host__ __device__ void getRi2bMatrix(float* matrix, RotationAngle angle);
extern __host__ __device__ void getRb2cMatrix(float* matrix, RotationAngle angle);
extern __host__ __device__ void getRe2iMatrix(float* matrix, GPS gps);
extern __host__ __device__ GPS ECEF2Geoditic(Coordinate pos);
extern __host__ __device__ Coordinate Geoditic2ECEF(GPS gps);
extern __host__ __device__ float deg2rad(float deg);
extern __host__ __device__ float rad2deg(float rad);
extern __host__ __device__ void mul3x3(float *dst, float *src1, float *src2);
extern __host__ __device__ void mul3x3TransposeFirst(float *dst, float *src1, float *src2);
extern __host__ __device__ void mul3x3TransposeBoth(float *dst, float *src1, float *src2);
extern __host__ __device__ void mul3x3ToVec3x1(float *dst, float *mat, float *vec);
bool readFromFile(std::string filename, float *arr, int numLine, int numPerLine);
void printObjStatus(ObjStatus * obj, int numLine);
void printSeekerInfo(SeekerInfo * obj, int numLine);
void genRandomMat(cv::Mat src);
#endif
