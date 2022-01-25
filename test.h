#ifndef TEST_H
#define TEST_H
#include "utils.h"
#include "Simulator.h"
void testReadFromFile();
void testSimulator();
void testMul3x3();
bool checkEqual(std::string name, float * gt, float *infer, int width, int height);
void printTestName(std::string name);
void testCalcTransformationMatrices();
void testConvertToImage();
void calcTransMatrix(ObjStatus* missile_cur, ObjStatus* missile_prev,
                     ObjStatus *target_cur, ObjStatus* target_prev,
                     SeekerInfo* seeker_cur, SeekerInfo* seeker_prev,
                     float* Rb2c_cur, float* Ri2b_missile_cur,
                     float* Re2i_missile,
                     float* Ri2b_target,
                     float* Re2i_target,
                     float* Rb2c_prev,
                     float* Ri2b_missile_prev);
#endif
