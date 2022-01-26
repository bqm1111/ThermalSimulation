#include "test.h"

void testSimulator()
{
    Simulator simulator(30, 150, 256);
    simulator.init();
    simulator.loadData();
    simulator.run();
}

void test()
{
    GPS data(19.0982, 106.0423, 6.33);
    GPS *d_data;
    gpuErrChk(cudaMallocManaged((void**)&d_data, sizeof(GPS)));
    gpuErrChk(cudaMemcpy(d_data, &data, sizeof(GPS), cudaMemcpyHostToDevice));
    gpuErrChk(cudaFree(d_data));
}

void testCalcTransformationMatrices()
{
//    ObjStatus *missile;
//    ObjStatus *target;
//    SeekerInfo *seeker;
//    float *Rb2c_cur;      // Matrix transform between body to camera
//    float *Rb2c_prev;       // Matrix transform between body to camera in the previous step
//    float *Ri2b_missile_cur;      // Matrix transform between inertial to body(missile)
//    float *Ri2b_missile_prev;     // Matrix transform between inertial to body(missile) in the previous step
//    float *Ri2b_target;           // Matrix transform between inertial to body(target)
//    float *Re2i_missile;      // Matrix transform between earth to inertial(missile)
//    float *Re2i_target;       // Matrix transform between earth to inertial(missile)

//    ObjStatus missile_data(GPS(19.0982, 106.0423, 6.33), RotationAngle(-0.000064, 0.0549, 1.2775));
//    ObjStatus target_data(GPS(19.1117, 106.0899, 6), RotationAngle(0, 0, 2.8483));
//    SeekerInfo seeker_data(0, -0.0549);

//    cv::Mat Ri2b_target_gt = (cv::Mat_<float>(3, 3) << -0.957297261243354,	-0.289105436849559,	0,
//                                                        0.289105436849559,	-0.957297261243354,	0,
//                                                        0,	0,	1);
//    cv::Mat Ri2b_missile_gt = (cv::Mat_<float>(3, 3) << 0.288669790415188,	-0.957298274792271,	0.0158039609399990,
//                                                        0.955854731683765,	0.289102073660829,	0.0525520972251927,
//                                                        -0.0548769898902011,	-6.39120480315225e-05,	0.998493120605165);

//    cv::Mat Rb2c_cur_gt = (cv::Mat_<float>(3, 3) << 0,	-0.0548769898902011,	0.998493122650622,
//                                                        1,	0,	0,
//                                                        0,	0.998493122650622,	0.0548769898902011);

//    cv::Mat Re2i_missile_gt = (cv::Mat_<float>(3, 3) << 0.0904175028400521,	-0.314446212923732,	0.944959392967810,
//                               -0.961057764657726,	-0.276347558323021,	0,
//                               0.261137220961058,	-0.908160561897965,	-0.327187630637082);
//    cv::Mat Re2i_target_gt = (cv::Mat_<float>(3, 3) << 0.0907407152381514,	-0.314585835730093,	0.944881936834474,
//                              -0.960827819646938,	-0.277145992199980,	0,
//                              0.261870241895830,	-0.907868851192444,	-0.327411248193968);

//    gpuErrChk(cudaMallocManaged((void**)&missile, sizeof(ObjStatus)));
//    gpuErrChk(cudaMallocManaged((void**)&target, sizeof(ObjStatus)));
//    gpuErrChk(cudaMallocManaged((void**)&seeker, sizeof(SeekerInfo)));
//    gpuErrChk(cudaMallocManaged((void**)&Rb2c_cur, 9 * sizeof(float)));
//    gpuErrChk(cudaMallocManaged((void**)&Rb2c_prev, 9 * sizeof(float)));
//    gpuErrChk(cudaMallocManaged((void**)&Ri2b_missile_cur, 9 * sizeof(float)));
//    gpuErrChk(cudaMallocManaged((void**)&Ri2b_missile_prev, 9 * sizeof(float)));
//    gpuErrChk(cudaMallocManaged((void**)&Ri2b_target, 9 * sizeof(float)));
//    gpuErrChk(cudaMallocManaged((void**)&Re2i_missile, 9 * sizeof(float)));
//    gpuErrChk(cudaMallocManaged((void**)&Re2i_target, 9 * sizeof(float)));
//    gpuErrChk(cudaMemcpy(missile, &missile_data, sizeof(ObjStatus), cudaMemcpyHostToDevice));
//    gpuErrChk(cudaMemcpy(target, &target_data, sizeof(ObjStatus), cudaMemcpyHostToDevice));
//    gpuErrChk(cudaMemcpy(seeker, &seeker_data, sizeof(SeekerInfo), cudaMemcpyHostToDevice));

//    calcTransMatrix(missile, missile,target, target,
//                    seeker, seeker,
//                    Rb2c_cur,Ri2b_missile_cur,
//                    Re2i_missile, Ri2b_target,Re2i_target,
//                    Rb2c_prev, Ri2b_missile_prev);

//    checkEqual("Test Rb2c", (float*)Rb2c_cur_gt.data, Rb2c_cur, 3, 3);
//    checkEqual("Test Ri2b_missile", (float*)Ri2b_missile_gt.data, Ri2b_missile_cur, 3, 3);
//    checkEqual("Test Ri2b_target", (float*)Ri2b_target_gt.data, Ri2b_target, 3, 3);
//    checkEqual("Test Re2i_missile", (float*)Re2i_missile_gt.data, Re2i_missile, 3, 3);
//    checkEqual("Test Re2i_target", (float*)Re2i_target_gt.data, Re2i_target, 3, 3);

//    gpuErrChk(cudaFree(missile));
//    gpuErrChk(cudaFree(target));
//    gpuErrChk(cudaFree(seeker));
//    gpuErrChk(cudaFree(Rb2c_cur));
//    gpuErrChk(cudaFree(Rb2c_prev));
//    gpuErrChk(cudaFree(Ri2b_missile_cur));
//    gpuErrChk(cudaFree(Ri2b_missile_prev));
//    gpuErrChk(cudaFree(Ri2b_target));
//    gpuErrChk(cudaFree(Re2i_missile));
//    gpuErrChk(cudaFree(Re2i_target));
}

void testConvertToImage()
{

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

bool checkEqual(std::string name, float * gt, float *infer, int width, int height)
{
    std::cout << name;
    for(int y = 0; y < height; y++)
    {
        for(int x = 0; x < width; x++)
        {
            int idx = y * width + x;
            if(gt[idx] - infer[idx] > 0.00001)
            {
                std::cout << ": !!! DO NOT MATCH !!!" << std::endl;
                std::cout << "GT = " << std::endl;
                printMat(gt, width, height);
                std::cout << "INFER = " << std::endl;
                printMat(infer, width, height);
                return false;
            }
        }
    }
    std::cout << ": *** MATCH ***" << std::endl;
    return true;
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
//    mul3x3((float*)infer.data, (float*)src1.data, (float*)src2.data);
//    if(!checkEqual("Test mul3x3", (float*)gt.data, (float*)infer.data, 3, 3))
//    {
//        std::cout << "GT = \n" << gt << std::endl;
//        std::cout << "INFER = \n" << infer << std::endl;
//    }

//    gt = src1.t() * src2;
//    mul3x3TransposeFirst((float*)infer.data, (float*)src1.data, (float*)src2.data);
//    checkEqual("Test mul3x3TransposeFirst", (float*)gt.data, (float*)infer.data, 3, 3);

//    gt = src1.t() * src2.t();
//    mul3x3TransposeBoth((float*)infer.data, (float*)src1.data, (float*)src2.data);
//    checkEqual("Test mul3x3TranposeBoth", (float*)gt.data, (float*)infer.data, 3, 3);

//    cv::Mat vec(3, 1, CV_32FC1);
//    genRandomMat(vec);
//    cv::Mat dst(3, 1, CV_32FC1);
//    cv::Mat vec_infer(3, 1, CV_32FC1);
//    dst = src1 * vec;
//    mul3x3ToVec3x1((float*)vec_infer.data, (float*)src1.data, (float*)vec.data);
//    checkEqual("Test mul3x3ToVec3x1", (float*)dst.data, (float*)vec_infer.data, 3, 1);

}

void printTestName(std::string name)
{
    std::cout << "-------- " << name << "--------" << std::endl;
}



