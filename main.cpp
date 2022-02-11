#include <iostream>
#include "Simulator.h"
#include "test.h"

int main(int argc, char **argv)
{
    int batch_size = 640;
    int fps = 30;
    int duration = 150;
    int resume = 0;
    int cuda_device = 0;
    std::cout << "========= DEFAULT PARAMETER =========" << std::endl;
    std::cout << "batch_size: " << batch_size << std::endl;
    std::cout << "fps: " << fps << std::endl;
    std::cout << "duration: " << duration << "(s)" << std::endl;
    std::cout << "resume: " << resume << std::endl;
    std::cout << "cuda_device: " << cuda_device << std::endl;
    std::cout << "=====================================" << std::endl;

    for(int i = 0; i < argc; i++)
    {
        if(strcmp(argv[i], "--batch_size") == 0)
        {
            batch_size = atoi(argv[i + 1]);
        }
        if(strcmp(argv[i], "--fps") == 0)
        {
            fps = atoi(argv[i + 1]);
        }
        if(strcmp(argv[i], "--duration") == 0)
        {
            duration = atoi(argv[i + 1]);
        }
        if(strcmp(argv[i], "--resume") == 0)
        {
            resume = atoi(argv[i + 1]);
        }
        if(strcmp(argv[i], "--cuda_device") == 0)
        {
            cuda_device = atoi(argv[i + 1]);
        }
    }
    gpuErrChk(cudaSetDevice(cuda_device));
    testSimulator(fps, duration, batch_size, resume);
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
