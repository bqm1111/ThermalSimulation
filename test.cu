#include "test.h"
__device__ void mul3x3ToVec3x1Test(double *dst, double *mat, double *vec)
{
#pragma unroll
    for(int y = 0; y < 3; y++)
    {
        for(int x = 0; x < 1; x++)
        {
            int idx = y + x;
            dst[idx] = 0;
            for(int k = 0; k < 3; k++)
            {
                dst[idx] = dst[idx] + mat[y * 3 + k] * vec[k + x];
            }
        }
    }
}

__device__ void mul3x3Test(double *dst, const double *__restrict__ src1, const double * __restrict__ src2)
{
#pragma unroll
    for(int y = 0; y < 3; y++)
    {
        for(int x = 0; x < 3; x++)
        {
            int idx = y * 3 + x;
            dst[idx] = 0;
            for(int k = 0; k < 3; k++)
            {
                dst[idx] = dst[idx] + src1[y * 3 + k] * src2[k * 3 + x];
            }
        }
    }
}

__device__ void mul3x3TransposeFirstTest(double *dst, const double * __restrict__ src1,
                                         const double * __restrict__ src2)
{
#pragma unroll
    for(int y = 0; y < 3; y++)
    {
        for(int x = 0; x < 3; x++)
        {
            int idx = y * 3 + x;
            dst[idx] = 0;
            for(int k = 0; k < 3; k++)
            {
                dst[idx] = dst[idx] + src1[k * 3 + y] * src2[k * 3 + x];
            }
        }
    }
}

__device__ void mul3x3TransposeBothTest(double *dst, const double *__restrict__ src1, const double * __restrict__ src2)
{
#pragma unroll
    for(int y = 0; y < 3; y++)
    {
        for(int x = 0; x < 3; x++)
        {
            int idx = y * 3 + x;
            dst[idx] = 0;
            for(int k = 0; k < 3; k++)
            {
                dst[idx] = dst[idx] + src1[k * 3 + y] * src2[x * 3 + k];
            }
        }
    }
}

//__global__ void calcTransMatricesHelper(ObjStatus* missile_cur,
//                                        ObjStatus* missile_prev,
//                                        ObjStatus* target_cur,
//                                        ObjStatus* target_prev,
//                                        SeekerInfo* seeker_cur,
//                                        SeekerInfo* seeker_prev,
//                                        double* Rb2c_cur, double* Ri2b_missile_cur,
//                                        double* Re2i_missile,
//                                        double* Ri2b_target,
//                                        double* Re2i_target,
//                                        double* Rb2c_prev,
//                                        double* Ri2b_missile_prev)
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
//                     double* Rb2c_cur, double* Ri2b_missile_cur,
//                     double* Re2i_missile,
//                     double* Ri2b_target,
//                     double* Re2i_target,
//                     double* Rb2c_prev,
//                     double* Ri2b_missile_prev)
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

__device__ double func(Coordinate pos)
{
    double x = pos.x;
    double y = pos.y;
    double z = pos.z;

    double a = 6378137;
    double b = 6356752;
    double f = (a - b) / a;
    double e_sq = f * (2 - f);
    double eps = e_sq / (1.0 - e_sq);
    double p = sqrt(x * x + y * y);
    double q = atan2(z * a, p * b);
    double sinf_q = sin(q);
    double cosf_q = cos(q);
    double sinf_q_3 = sinf_q *sinf_q * sinf_q;
    double cosf_q_3 = cosf_q * cosf_q * cosf_q;
    double phi = atan2(z + eps * b * sinf_q_3, p - e_sq * a * cosf_q_3);
    double lambda = atan2(y, x);
    double v = a / sqrt(1.0 - e_sq * sin(phi) * sin(phi));
    double res = p / cos(phi) - v;
    return res;
}
__device__ GPS ECEF2GeoditicTest(Coordinate pos)
{
    GPS result;
    double x = pos.x;
    double y = pos.y;
    double z = pos.z;

    double a = 6378137;
    double b = 6356752;
    double f = (a - b) / a;
    double e_sq = f * (2 - f);
    double eps = e_sq / (1.0 - e_sq);
    double p = sqrt(x * x + y * y);
    double q = atan2(z * a, p * b);
    double sinf_q = sin(q);
    double cosf_q = cos(q);
    double sinf_q_3 = sinf_q *sinf_q * sinf_q;
    double cosf_q_3 = cosf_q * cosf_q * cosf_q;
    double phi = atan2(z + eps * b * sinf_q_3, p - e_sq * a * cosf_q_3);
    double lambda = atan2(y, x);
    double v = a / sqrt(1.0 - e_sq * sin(phi) * sin(phi));
    result.height = p / cos(phi) - v;
    result.latitude = phi / M_PI * 180;
    result.longtitude = lambda / M_PI * 180;

    return result;
}

__device__ Coordinate Geoditic2ECEFTest(GPS gps)
{
    Coordinate result;

    double a = 6378137;
    double b = 6356752;
    double f = (a - b) / a;
    double e_sq = f * (2 - f);

    double lambda = gps.latitude / 180 * M_PI;
    double phi = gps.longtitude / 180 * M_PI;

    double N = a / sqrt(1 - e_sq * sin(lambda) * sin(lambda));
    result.x = (gps.height + N) * cos(lambda) * cos(phi);
    result.y = (gps.height + N) * cos(lambda) * sin(phi);
    result.z = (gps.height + (1 - e_sq) * N) * sin(lambda);
    return result;
}

__device__ void calcNEDTest(double *__restrict__ NED, double2 imgPos,
                            const double * __restrict__ Rb2c_cur, const double * __restrict__ Ri2b_missile_cur,
                            const double * __restrict__ Re2i_missile, double fov_pixel)
{
    double Ldonic[3];
    double normTmp = sqrt(imgPos.x * imgPos.x + imgPos.y * imgPos.y + fov_pixel * fov_pixel);
    Ldonic[0] = imgPos.x /normTmp;
    Ldonic[1] = imgPos.y /normTmp;
    Ldonic[2] = fov_pixel / normTmp;

    double tmp[9];
    double tmp1[9];
    mul3x3TransposeFirstTest(&tmp[0], (double*)Re2i_missile, (double*)Ri2b_missile_cur);
    mul3x3Test(&tmp1[0], &tmp[0], (double*)Rb2c_cur);
    mul3x3ToVec3x1Test(&NED[0], &tmp1[0], &Ldonic[0]);
}

__global__ void testCalcDistance(double * distance, double* angle, int* objIdx,
                                 ObjStatus *  missile_cur,
                                 double *  Rb2c_cur, double *  Ri2b_missile_cur,
                                 double *  Re2i_missile,
                                 double fov_pixel,
                                 int num_faces, int num_partialPix,
                                 int offset, int batch_size, int width, int height)
{
    int partialPixIdx = threadIdx.x + IMUL(blockIdx.x, blockDim.x);
    int faceIdx = threadIdx.y + IMUL(blockIdx.y, blockDim.y);

    // index of result array to be calculated(double* distance)
    int resultIdx = faceIdx * batch_size * PIXEL_GRID_SIZE * PIXEL_GRID_SIZE + partialPixIdx;
    //     pixel index in the output image(640 * 480)
    int pixIdx = offset * batch_size + partialPixIdx / (PIXEL_GRID_SIZE * PIXEL_GRID_SIZE);

    // row, col index of current thread in the output width * height (640 * 480) image
    int row = pixIdx / width;
    int col = pixIdx % width;

    // index of subpixel inside the subgrid for each pixel
    int partialPixIdx_X = partialPixIdx / (batch_size * PIXEL_GRID_SIZE);
    int partialPixIdx_Y = partialPixIdx % (batch_size * PIXEL_GRID_SIZE);

    if(faceIdx < num_faces && partialPixIdx < num_partialPix)
    {
//        Coordinate missile_pos;
//        missile_pos = Geoditic2ECEFTest(missile_cur->gps);
//        double u = col + (double)(partialPixIdx_X - 0.5)/ PIXEL_GRID_SIZE - width / 2 - 0.5;
//        double w = row + (double)(partialPixIdx_Y - 0.5)/ PIXEL_GRID_SIZE - height / 2 - 0.5;
//        double2 imgPos;
//        imgPos.x = u;
//        imgPos.y = w;

//        // TODO: The following code performs a uncoalescing memory access
//        // It is not easy to avoid since it require random data access pattern
//        // A possible solution is to reorganize input data
//        double2 surface_imgPos[3];
//        //        surface_imgPos[0] = ship_surface_imgPos[faceIdx].x;
//        //        surface_imgPos[1] = ship_surface_imgPos[faceIdx].y;
//        //        surface_imgPos[2] = ship_surface_imgPos[faceIdx].z;

//        //        GPS surface_gps[3];
//        //        surface_gps[0] = ship_surface_gps[faceIdx].x;
//        //        surface_gps[1] = ship_surface_gps[faceIdx].y;
//        //        surface_gps[2] = ship_surface_gps[faceIdx].z;

//        double NED[3];
//        calcNEDTest(&NED[0], imgPos, Rb2c_cur, Ri2b_missile_cur, Re2i_missile, fov_pixel);

//        Coordinate NED1;

//        double missile_norm = missile_pos.norm();
//        NED1.x = missile_pos.x /missile_norm;
//        NED1.y = missile_pos.y / missile_norm;
//        NED1.z = missile_pos.z / missile_norm;
//        GPS missile_gps = ECEF2GeoditicTest(missile_pos);
//        Coordinate tmp(NED[0], NED[1], NED[2]);
//        double L = missile_norm * fabs(tmp * NED1);
//        GPS cutting_point_gps = ECEF2GeoditicTest(missile_pos + NED1 * L);

//        //        GPS cutting_point_gps = ECEF2GeoditicTest(Coordinate(3, 4, 5));
//        if(cutting_point_gps.height > 0)
        {
            distance[resultIdx] = -1;
            angle[resultIdx] = -1;
            objIdx[resultIdx] = 0;
        }
    }
}

void helper(dim3 gridDim, dim3 blockDim, double * distance, double* angle, int* objIdx,
            ObjStatus* missile_cur,
            double* Rb2c_cur, double* Ri2b_missile_cur,
            double* Re2i_missile,
            double fov_pixel,
            int num_faces, int num_partialPix, int offset, int batch_size, int width, int height)
{
    testCalcDistance<<<gridDim, blockDim>>>(distance, angle, objIdx,
                                            missile_cur,
                                            Rb2c_cur, Ri2b_missile_cur, Re2i_missile,
                                            fov_pixel,
                                            num_faces, num_partialPix,
                                            offset, batch_size, width, height);
    gpuErrChk(cudaDeviceSynchronize());
}

void testStruct()
{
    int offset = 0;
    int batch_size = 256;
    int num_partialPix = batch_size * PIXEL_GRID_SIZE * PIXEL_GRID_SIZE;
    int num_surfaces = 4563;
    int width = 640;
    int height = 480;
    double fov_pixel = 3.7;
    int grid_size = num_surfaces * batch_size * PIXEL_GRID_SIZE * PIXEL_GRID_SIZE;

    double *angle, *distance;
    int *objIdx;
    double *Ri2b_target, *Ri2b_missile, *Rb2c_cur, *Re2i_missile, *Re2i_target;
    ObjStatus * missile_cur;
    ObjStatus missile(GPS(3.0, 4.0, 5.0), RotationAngle(1.0, 2.0, 3.0));
    gpuErrChk(cudaMalloc((void**)&missile_cur, sizeof(ObjStatus)));
    gpuErrChk(cudaMalloc((void**)&(angle), grid_size * sizeof(double)));
    gpuErrChk(cudaMalloc((void**)&(distance), grid_size * sizeof(double)));
    gpuErrChk(cudaMalloc((void**)&(objIdx), grid_size * sizeof(int)));
    gpuErrChk(cudaMemcpy(missile_cur, &missile, sizeof(ObjStatus), cudaMemcpyHostToDevice));

    gpuErrChk(cudaMalloc((void**)&Ri2b_target, 9 * sizeof(double)));
    gpuErrChk(cudaMalloc((void**)&Ri2b_missile, 9 * sizeof(double)));
    gpuErrChk(cudaMalloc((void**)&Rb2c_cur, 9 * sizeof(double)));
    gpuErrChk(cudaMalloc((void**)&Re2i_target, 9 * sizeof(double)));
    gpuErrChk(cudaMalloc((void**)&Re2i_missile, 9 * sizeof(double)));

    cv::Mat Ri2b_target_gt = (cv::Mat_<double>(3, 3) << -0.957297261243354,	-0.289105436849559,	0,
                              0.289105436849559,	-0.957297261243354,	0,
                              0,	0,	1);
    cv::Mat Ri2b_missile_gt = (cv::Mat_<double>(3, 3) << 0.288669790415188,	-0.957298274792271,	0.0158039609399990,
                               0.955854731683765,	0.289102073660829,	0.0525520972251927,
                               -0.0548769898902011,	-6.39120480315225e-05,	0.998493120605165);

    cv::Mat Rb2c_cur_gt = (cv::Mat_<double>(3, 3) << 0,	-0.0548769898902011,	0.998493122650622,
                           1,	0,	0,
                           0,	0.998493122650622,	0.0548769898902011);

    cv::Mat Re2i_missile_gt = (cv::Mat_<double>(3, 3) << 0.0904175028400521,	-0.314446212923732,	0.944959392967810,
                               -0.961057764657726,	-0.276347558323021,	0,
                               0.261137220961058,	-0.908160561897965,	-0.327187630637082);
    cv::Mat Re2i_target_gt = (cv::Mat_<double>(3, 3) << 0.0907407152381514,	-0.314585835730093,	0.944881936834474,
                              -0.960827819646938,	-0.277145992199980,	0,
                              0.261870241895830,	-0.907868851192444,	-0.327411248193968);

    gpuErrChk(cudaMemcpy(Ri2b_target, (double*)Ri2b_target_gt.data, 9 * sizeof(double), cudaMemcpyHostToDevice));
    gpuErrChk(cudaMemcpy(Ri2b_missile, (double*)Ri2b_missile_gt.data, 9 * sizeof(double), cudaMemcpyHostToDevice));
    gpuErrChk(cudaMemcpy(Rb2c_cur, (double*)Rb2c_cur_gt.data, 9 * sizeof(double), cudaMemcpyHostToDevice));
    gpuErrChk(cudaMemcpy(Re2i_target, (double*)Re2i_target_gt.data, 9 * sizeof(double), cudaMemcpyHostToDevice));
    gpuErrChk(cudaMemcpy(Re2i_missile, (double*)Re2i_target_gt.data, 9 * sizeof(double), cudaMemcpyHostToDevice));

    dim3 blockDim(threadsPerBlock, threadsPerBlock);
    dim3 gridDim(ceil((double)num_partialPix / threadsPerBlock),
                 ceil((double)num_surfaces / threadsPerBlock));
    printf("gridDim = %d - %d\n", gridDim.x, gridDim.y);

    testCUDATime("TestFunc time = ", 10, helper(gridDim, blockDim, distance, angle, objIdx,
                                                 missile_cur,
                                                 Rb2c_cur, Ri2b_missile, Re2i_missile,
                                                 fov_pixel,
                                                 num_surfaces, num_partialPix, offset, batch_size,
                                                 width, height));
    gpuErrChk(cudaFree(missile_cur));
    gpuErrChk(cudaFree(angle));
    gpuErrChk(cudaFree(distance));
    gpuErrChk(cudaFree(objIdx));

    gpuErrChk(cudaFree(Ri2b_target));
    gpuErrChk(cudaFree(Ri2b_missile));
    gpuErrChk(cudaFree(Rb2c_cur));
    gpuErrChk(cudaFree(Re2i_target));
    gpuErrChk(cudaFree(Re2i_missile));
}
