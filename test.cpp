#include "test.h"

void testSimulator()
{
    Simulator simulator;
    simulator.init();
    simulator.loadData();
    simulator.run();
}

void testReadFromFile()
{
        int numLine = 4500;
        ObjStatus *hPtr = (ObjStatus*)malloc(numLine * sizeof(ObjStatus));
        ObjStatus *dPtr;
        gpuErrChk(cudaMalloc((void**)&dPtr, numLine * sizeof(ObjStatus)));
        readFromFile("../data/Missile_30hz.txt", (float*)hPtr, numLine, 6);
        printObjStatus((ObjStatus*)hPtr, numLine);
        gpuErrChk(cudaMemcpy(dPtr, hPtr, numLine * sizeof(ObjStatus), cudaMemcpyHostToDevice));

        free(hPtr);
        gpuErrChk(cudaFree(dPtr));

}
bool checkEqual(float * src1, float *src2, int width, int height)
{
    for(int y = 0; y < height; y++)
    {
        for(int x = 0; x < width; x++)
        {
            int idx = y * width + x;
            if(src1[idx] - src2[idx] > 0.00000001)
            {
                std::cout << "!!! DO NOT MATCH !!!" << std::endl;
                return false;
            }
        }
    }
    std::cout << "*** MATCH ***" << std::endl;
    return true;
}


void testMul3x3()
{
    cv::Mat src1(3, 3, CV_32FC1);
    cv::Mat src2(3, 3, CV_32FC1);

    genRandomMat(src1);
    genRandomMat(src2);
    cv::Mat gt(3, 3, CV_32FC1);
    cv::Mat infer(3, 3, CV_32FC1);


    printTestName("Test mul3x3");
    gt = src1 * src2;
    mul3x3((float*)infer.data, (float*)src1.data, (float*)src2.data);
    if(!checkEqual((float*)gt.data, (float*)infer.data, 3, 3))
    {
        std::cout << "GT = \n" << gt << std::endl;
        std::cout << "INFER = \n" << infer << std::endl;
    }

    printTestName("Test mul3x3TransposeFirst");
    gt = src1.t() * src2;
    mul3x3TransposeFirst((float*)infer.data, (float*)src1.data, (float*)src2.data);
    if(!checkEqual((float*)gt.data, (float*)infer.data, 3, 3))
    {
        std::cout << "GT = \n" << gt << std::endl;
        std::cout << "INFER = \n" << infer << std::endl;
    }

    printTestName("Test mul3x3TranposeBoth");
    gt = src1.t() * src2.t();
    mul3x3TransposeBoth((float*)infer.data, (float*)src1.data, (float*)src2.data);
    if(!checkEqual((float*)gt.data, (float*)infer.data, 3, 3))
    {
        std::cout << "GT = \n" << gt << std::endl;
        std::cout << "INFER = \n" << infer << std::endl;
    }
}

void printTestName(std::string name)
{
    std::cout << "-------- " << name << "--------" << std::endl;
}
