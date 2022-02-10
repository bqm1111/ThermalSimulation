#include <iostream>
#include "Simulator.h"
#include "test.h"

int main(int argc, char **argv)
{
    testSimulator(atoi(argv[1]), atoi(argv[2]), atoi(argv[3]));
//    testStruct();
//    getExeTime("test RayInfo time = ", testRayInfo());
//    testRayInfo();
//    testCUDATime("RayInfo Time = ", 100, testRayInfo());
//    testReadFromFile();
//    testMul3x3();
//    testCalcTransformationMatrices();
//    cv::Mat img = cv::imread("/home/martin/minhbq6/seeker/ThermalSimulation/img/0.jpg", 0);
//    cv::imshow("img", img);
//    cv::waitKey();
//    std::cout << img << std::endl;
}
