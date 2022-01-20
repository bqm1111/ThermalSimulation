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
extern __host__ __device__ float calcGainTransmittance(float distance);
extern __host__ __device__ float norm(float2 input);
extern __host__ __device__ float inner_product(float2 a, float2 b);
extern __host__ __device__ Coordinate cross_product(Coordinate a, Coordinate b);
extern __host__ __device__ Coordinate vectorCalculator(GPS src1, GPS src2);
extern __host__ __device__ void pinv3x2(float *dst, float *src);
extern __host__ __device__ float skyRadiance(float ifov, float sky_coeff, float path_coeff);
extern __host__ __device__ float oceanRadiance(float ifov, float distance, float beta,
                                               float solar_coeff, float ocean_coeff,
                                               float sky_coeff, float object_coeff,
                                               float path_coeff);
extern __host__ __device__ float objectRadiance(float ifov, float distance, float beta,
                                                float solar_coeff, float ocean_coeff,
                                                float sky_coeff, float object_coeff,
                                                float path_coeff);

bool readFromFile(std::string filename, float *arr, int numLine, int numPerLine);
void printObjStatus(ObjStatus * obj, int numLine);
void printSeekerInfo(SeekerInfo * obj, int numLine);
void genRandomMat(cv::Mat src);
#endif
