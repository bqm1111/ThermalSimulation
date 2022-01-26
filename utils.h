#ifndef UTILS_H
#define UTILS_H
#include <iostream>
#include "define.h"
#include "Simulator.h"
#include "math.h"

//extern __device__ void getRi2bMatrix(float* matrix, RotationAngle angle);
//extern __device__ void getRb2cMatrix(float* matrix, RotationAngle angle);
//extern __device__ void getRe2iMatrix(float* matrix, GPS gps);
//extern __device__ GPS ECEF2Geoditic(Coordinate pos);
//extern __device__ Coordinate Geoditic2ECEF(GPS gps);
//extern __device__ float deg2rad(float deg);
//extern __device__ float rad2deg(float rad);
//extern __device__ void mul3x3(float *dst, const float * __restrict__ src1, const float * __restrict__ src2);
//extern __device__ void mul3x3TransposeFirst(float *dst, const float * __restrict__ src1, const float * __restrict__ src2);
//extern __device__ void mul3x3TransposeBoth(float *dst, const float * __restrict__ src1, const float * __restrict__ src2);
//extern __device__ void mul3x3ToVec3x1(float *dst, float *mat, float *vec);
//extern __device__ float calcGainTransmittance(float distance);
//extern __device__ float norm(float2 input);
//extern __device__ float inner_product(float2 a, float2 b);
//extern __device__ Coordinate cross_product(Coordinate a, Coordinate b);
//extern __device__ Coordinate vectorCalculator(GPS src1, GPS src2);
//extern __device__ void pinv3x2(float *dst, float *src);
//extern __device__ float skyRadiance(float ifov, float sky_coeff, float path_coeff);
//extern __device__ float oceanRadiance(float ifov, float distance, float beta,
//                                               float solar_coeff, float ocean_coeff,
//                                               float sky_coeff, float object_coeff,
//                                               float path_coeff);
//extern __device__ float objectRadiance(float ifov, float distance, float beta,
//                                                float solar_coeff, float ocean_coeff,
//                                                float sky_coeff, float object_coeff,
//                                                float path_coeff);
//extern __device__ void CheckPoint(int idx);

bool readFromFile(std::string filename, float *arr, int numLine, int numPerLine);
void printObjStatus(ObjStatus * obj, int numLine);
void printSeekerInfo(SeekerInfo * obj, int numLine);
void genRandomMat(cv::Mat src);
void printDevicePtr(float *arr, int width, int height);
void printMat(float * data, int width, int height);
void dev2Host(float * dst, float *src, int size);
void host2Dev(float* dst, float *src, int size);

#endif
