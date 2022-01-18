#ifndef TEST_H
#define TEST_H
#include "utils.h"
#include "Simulator.h"
void testReadFromFile();
void testSimulator();
void testMul3x3();
bool checkEqual(float * src1, float *src2, int width, int height);
void printTestName(std::string name);
#endif
