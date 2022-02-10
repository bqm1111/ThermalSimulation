#include "test.h"

void testSimulator(int fps, int duration, int batch_size)
{
    Simulator simulator(fps, duration, batch_size);
    simulator.init();
    simulator.loadData();
//    simulator.run();
//    simulator.testFunc();
    simulator.test();
//    simulator.testFuck();
}

void test()
{
    GPS data(19.0982, 106.0423, 6.33);
    GPS *d_data;
    gpuErrChk(cudaMallocManaged((void**)&d_data, sizeof(GPS)));
    gpuErrChk(cudaMemcpy(d_data, &data, sizeof(GPS), cudaMemcpyHostToDevice));
    gpuErrChk(cudaFree(d_data));
}

void testConvertToImage()
{

}

void testReadFromFile()
{
//    int numLine = 4500;
//    ObjStatus *hPtr = (ObjStatus*)malloc(numLine * sizeof(ObjStatus));
//    ObjStatus *dPtr;
//    gpuErrChk(cudaMalloc((void**)&dPtr, numLine * sizeof(ObjStatus)));
//    readFromFile("../data/Missile_30hz.txt", (double*)hPtr, numLine, 6);
//    printObjStatus((ObjStatus*)hPtr, numLine);
//    gpuErrChk(cudaMemcpy(dPtr, hPtr, numLine * sizeof(ObjStatus), cudaMemcpyHostToDevice));

//    free(hPtr);
//    gpuErrChk(cudaFree(dPtr));
}


void testMul3x3()
{
//    cv::Mat src1(3, 3, CV_32FC1);
//    cv::Mat src2(3, 3, CV_32FC1);

//    genRandomMat(src1);
//    genRandomMat(src2);
//    cv::Mat gt(3, 3, CV_32FC1);
//    cv::Mat infer(3, 3, CV_32FC1);


//    gt = src1 * src2;
//    mul3x3((double*)infer.data, (double*)src1.data, (double*)src2.data);
//    if(!checkEqual("Test mul3x3", (double*)gt.data, (double*)infer.data, 3, 3))
//    {
//        std::cout << "GT = \n" << gt << std::endl;
//        std::cout << "INFER = \n" << infer << std::endl;
//    }

//    gt = src1.t() * src2;
//    mul3x3TransposeFirst((double*)infer.data, (double*)src1.data, (double*)src2.data);
//    checkEqual("Test mul3x3TransposeFirst", (double*)gt.data, (double*)infer.data, 3, 3);

//    gt = src1.t() * src2.t();
//    mul3x3TransposeBoth((double*)infer.data, (double*)src1.data, (double*)src2.data);
//    checkEqual("Test mul3x3TranposeBoth", (double*)gt.data, (double*)infer.data, 3, 3);

//    cv::Mat vec(3, 1, CV_32FC1);
//    genRandomMat(vec);
//    cv::Mat dst(3, 1, CV_32FC1);
//    cv::Mat vec_infer(3, 1, CV_32FC1);
//    dst = src1 * vec;
//    mul3x3ToVec3x1((double*)vec_infer.data, (double*)src1.data, (double*)vec.data);
//    checkEqual("Test mul3x3ToVec3x1", (double*)dst.data, (double*)vec_infer.data, 3, 1);

}

void printTestName(std::string name)
{
    std::cout << "-------- " << name << "--------" << std::endl;
}



