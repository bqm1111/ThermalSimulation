#include "test.h"

//__global__ void calcTransMatricesHelper(ObjStatus* missile_cur,
//                                        ObjStatus* missile_prev,
//                                        ObjStatus* target_cur,
//                                        ObjStatus* target_prev,
//                                        SeekerInfo* seeker_cur,
//                                        SeekerInfo* seeker_prev,
//                                        float* Rb2c_cur, float* Ri2b_missile_cur,
//                                        float* Re2i_missile,
//                                        float* Ri2b_target,
//                                        float* Re2i_target,
//                                        float* Rb2c_prev,
//                                        float* Ri2b_missile_prev)
//{
//    getRb2cMatrix(Rb2c_cur, RotationAngle(0, seeker_cur->elevation, seeker_cur->azimuth));
//    getRb2cMatrix(Rb2c_prev, RotationAngle(0, seeker_prev->elevation, seeker_prev->azimuth));
//    getRi2bMatrix(Ri2b_missile_cur, missile_cur->angle);
//    getRi2bMatrix(Ri2b_missile_prev, missile_prev->angle);
//    getRi2bMatrix(Ri2b_target, target_cur->angle);
//    getRe2iMatrix(Re2i_missile, missile_cur->gps);
//    getRe2iMatrix(Re2i_target, target_cur->gps);
//}

//void calcTransMatrix(ObjStatus* missile_cur, ObjStatus* missile_prev,
//                     ObjStatus *target_cur, ObjStatus* target_prev,
//                     SeekerInfo* seeker_cur, SeekerInfo* seeker_prev,
//                     float* Rb2c_cur, float* Ri2b_missile_cur,
//                     float* Re2i_missile,
//                     float* Ri2b_target,
//                     float* Re2i_target,
//                     float* Rb2c_prev,
//                     float* Ri2b_missile_prev)
//{
//    calcTransMatricesHelper<<<1, 1>>>(missile_cur, missile_prev,target_cur, target_prev,
//                                              seeker_cur, seeker_prev,
//                                              Rb2c_cur,Ri2b_missile_cur,
//                                              Re2i_missile, Ri2b_target,Re2i_target,
//                                              Rb2c_prev, Ri2b_missile_prev);
//}


//__global__ void cudaTestRayInfo(RayInfo* dst, int width, int height)
//{
//    int row = threadIdx.y + IMUL(blockIdx.y, blockDim.y);
//    int col = threadIdx.x + IMUL(blockIdx.x, blockDim.x);

//    if(row < height && col < width)
//    {
//        int idx = row * width + height;
//        dst[idx].angle = -1;
//        dst[idx].distance = -1;
//        dst[idx].objIdx = 0;
//    }
//}

//void testRayInfo()
//{
//    RayInfo * ray;
//    int width = 640;
//    int height = 480;
//    gpuErrChk(cudaMalloc((void**)&ray, width * height * sizeof(RayInfo)));

//    dim3 blockDim(16, 16);
//    dim3 gridDim(width / 16, height /16);
//    cudaTestRayInfo<<<gridDim, blockDim>>>(ray, width, height);
//    gpuErrChk(cudaFree(ray));
//}
