#ifndef TEST_H
#define TEST_H
#include "utils.h"
#include "Simulator.h"
void testReadFromFile();
void testSimulator(int fps, int duration, int batch_size, int resume);
void testMul3x3();
void printTestName(std::string name);
void testConvertToImage();
void calcTransMatrix(ObjStatus* missile_cur, ObjStatus* missile_prev,
                     ObjStatus *target_cur, ObjStatus* target_prev,
                     SeekerInfo* seeker_cur, SeekerInfo* seeker_prev,
                     double* Rb2c_cur, double* Ri2b_missile_cur,
                     double* Re2i_missile,
                     double* Ri2b_target,
                     double* Re2i_target,
                     double* Rb2c_prev,
                     double* Ri2b_missile_prev);
void testRayInfo();

void testStruct();
#endif
