#ifndef UTILS_H
#define UTILS_H
#include <iostream>
#include "define.h"
#include "Simulator.h"
#include "math.h"
#include <fstream>

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
