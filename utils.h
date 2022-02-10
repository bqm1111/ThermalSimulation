#ifndef UTILS_H
#define UTILS_H
#include <iostream>
#include "define.h"
#include "Simulator.h"
#include "math.h"

//extern __device__ void getRi2bMatrix(double* matrix, RotationAngle angle);
//extern __device__ void getRb2cMatrix(double* matrix, RotationAngle angle);
//extern __device__ void getRe2iMatrix(double* matrix, GPS gps);
//extern __device__ GPS ECEF2Geoditic(Coordinate pos);
//extern __device__ Coordinate Geoditic2ECEF(GPS gps);
//extern __device__ double deg2rad(double deg);
//extern __device__ double rad2deg(double rad);
//extern __device__ void mul3x3(double *dst, const double * __restrict__ src1, const double * __restrict__ src2);
//extern __device__ void mul3x3TransposeFirst(double *dst, const double * __restrict__ src1, const double * __restrict__ src2);
//extern __device__ void mul3x3TransposeBoth(double *dst, const double * __restrict__ src1, const double * __restrict__ src2);
//extern __device__ void mul3x3ToVec3x1(double *dst, double *mat, double *vec);
//extern __device__ double calcGainTransmittance(double distance);
//extern __device__ double norm(double2 input);
//extern __device__ double inner_product(double2 a, double2 b);
//extern __device__ Coordinate cross_product(Coordinate a, Coordinate b);
//extern __device__ Coordinate vectorCalculator(GPS src1, GPS src2);
//extern __device__ void pinv3x2(double *dst, double *src);
//extern __device__ double skyRadiance(double ifov, double sky_coeff, double path_coeff);
//extern __device__ double oceanRadiance(double ifov, double distance, double beta,
//                                               double solar_coeff, double ocean_coeff,
//                                               double sky_coeff, double object_coeff,
//                                               double path_coeff);
//extern __device__ double objectRadiance(double ifov, double distance, double beta,
//                                                double solar_coeff, double ocean_coeff,
//                                                double sky_coeff, double object_coeff,
//                                                double path_coeff);
//extern __device__ void CheckPoint(int idx);

bool readFromFile(std::string filename, double *arr, int numLine, int numPerLine);
//void printObjStatus(ObjStatus * obj, int numLine);
//void printSeekerInfo(SeekerInfo * obj, int numLine);
void genRandomMat(cv::Mat src);
void printDevicePtr(double *arr, int width, int height);
void printMat(double * data, int width, int height);
//void printMat(double * data, int width, int height);
void dev2Host(double * dst, double *src, int size);
void host2Dev(double* dst, double *src, int size);
bool checkEqual(std::string name, double * gt, double *infer, int width, int height, bool show=true);
//bool checkEqual(std::string name, double * gt, double *infer, int width, int height, bool show=true);

void writeTofile(std::string filename, double *arr, int length);
void writeTofile(std::string filename, double *arr, int line, int numPerline);
#endif
