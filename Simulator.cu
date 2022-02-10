#include "Simulator.h"


__device__ double deg2rad(double deg)
{
    return deg * M_PI / 180.0;
}

__device__ double rad2deg(double rad)
{
    return rad * 180.0 / M_PI;
}

__device__ double calcGainTransmittance(double distance)
{
    return expf(-0.26 * distance / 1000);
}

__device__ double norm(double2 input)
{
    return sqrt(input.x * input.x + input.y * input.y);
}

__device__ double inner_product(double2 a, double2 b)
{
    return (a.x * b.x + a.y * b.y);
}

__device__ Coordinate cross_product(Coordinate a, Coordinate b)
{
    return Coordinate(a.y * b.z - a.z * b.y,
                      a.z * b.x - a.x * b.z,
                      a.x * b.y - a.y * b.x);
}

__device__ void mul3x3ToVec3x1(double *dst, double *mat, double *vec)
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

__device__ void mul3x3(double *dst, const double *__restrict__ src1, const double * __restrict__ src2)
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

__device__ void mul3x3TransposeFirst(double *dst, const double * __restrict__ src1,
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

__device__ void mul3x3TransposeBoth(double *dst, const double *__restrict__ src1, const double * __restrict__ src2)
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

__device__ void pinv3x2(double *dst, double *src)
{
    double tmp[4];
#pragma unroll
    for(int y = 0; y < 2; y++)
    {
        for(int x = 0; x < 2; x++)
        {
            int idx = y * 2 + x;
            tmp[idx] = 0;
            for(int k = 0; k < 3; k++)
            {
                tmp[idx] += src[k * 2 + y] * src[k * 2 + x];
            }
        }
    }

    double det = tmp[0] * tmp[3] - tmp[1] * tmp[2];
    double inv[4];
    inv[0] = tmp[3] /det;
    inv[1] = -tmp[2] /det;
    inv[2] = -tmp[3] /det;
    inv[3] = tmp[0] /det;
#pragma unroll
    for(int y = 0; y < 2; y++)
        for(int x = 0; x< 3; x++)
        {
            int idx = y * 3 + x;
            dst[idx] = 0;
            for(int k = 0; k < 2; k++)
            {
                dst[idx] += inv[y * 2 + k] * src[x * 2 + k];
            }
        }
}

// Matrix Ri2b in matlab code
__host__ __device__ void getRi2bMatrix(double* matrix, RotationAngle angle)
{
    matrix[0] = cos(angle.pitch) * cos(angle.yaw);
    matrix[1] = sin(angle.roll) * sin(angle.pitch) * cos(angle.yaw) -
            cos(angle.roll) * sin(angle.yaw);
    matrix[2] = cos(angle.roll)*sin(angle.pitch)*cos(angle.yaw) +
            sin(angle.roll)*sin(angle.yaw);
    matrix[3] = cos(angle.pitch) * sin(angle.yaw);
    matrix[4] = sin(angle.roll) * sin(angle.pitch) * sin(angle.yaw) +
            cos(angle.roll) * cos(angle.yaw);
    matrix[5] = cos(angle.roll) * sin(angle.pitch) * sin(angle.yaw) -
            sin(angle.roll) * cos(angle.yaw);
    matrix[6] = -sin(angle.pitch);
    matrix[7] = sin(angle.roll) * cos(angle.pitch);
    matrix[8] = cos(angle.roll) * cos(angle.pitch);
}

// Matrix Ldoni in matlab code
__host__ __device__ void getRb2cMatrix(double* matrix, RotationAngle angle)
{
    matrix[0] = -sin(angle.yaw);
    matrix[1] = sin(angle.pitch) * cos(angle.yaw);
    matrix[2] = cos(angle.pitch) * cos(angle.yaw);
    matrix[3] = cos(angle.yaw);
    matrix[4] = sin(angle.pitch) * sin(angle.yaw);
    matrix[5] = cos(angle.pitch) * sin(angle.yaw);
    matrix[6] = 0;
    matrix[7] = cos(angle.pitch);
    matrix[8] =-sin(angle.pitch);
}
// Matrix M in matlab code
__host__ __device__ void getRe2iMatrix(double* matrix, GPS gps)
{
    double lambda = (double)gps.latitude / 180.0 * M_PI;
    double phi = (double)gps.longtitude / 180.0 * M_PI;

    matrix[0] = -cos(phi) * sin(lambda);;
    matrix[1] = -sin(lambda) * sin(phi);
    matrix[2] = cos(lambda);
    matrix[3] = -sin(phi);
    matrix[4] = cos(phi);
    matrix[5] = 0;
    matrix[6] = -cos(lambda) * cos(phi);
    matrix[7] = -cos(lambda) * sin(phi);
    matrix[8] = -sin(lambda);
}

__device__ GPS ECEF2Geoditic(Coordinate pos)
{
    GPS result;
    double x = (double)pos.x;
    double y = (double)pos.y;
    double z = (double)pos.z;

    double a = 6378137.0;
    double b = 6356752.0;
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

__device__ Coordinate Geoditic2ECEF(GPS gps)
{
    Coordinate result;

    double a = 6378137;
    double b = 6356752;
    double f = (a - b) / a;
    double e_sq = f * (2 - f);

    double lambda = (double)gps.latitude / 180 * M_PI;
    double phi = (double)gps.longtitude / 180 * M_PI;

    double N = a / sqrt(1 - e_sq * sin(lambda) * sin(lambda));
    result.x = (gps.height + N) * cos(lambda) * cos(phi);
    result.y = (gps.height + N) * cos(lambda) * sin(phi);
    result.z = (gps.height + (1 - e_sq) * N) * sin(lambda);
    return result;
}

__device__ Coordinate vectorCalculator(GPS src1, GPS src2)
{
    Coordinate coor1 = Geoditic2ECEF(src1);
    Coordinate coor2 = Geoditic2ECEF(src2);
    return (coor2 - coor1) / (coor2 - coor1).norm();
}

__device__ double objectRadiance(double ifov, double distance, double beta,
                                 double solar_coeff, double ocean_coeff,
                                 double sky_coeff, double object_coeff,
                                 double path_coeff)
{
    double transmiss = calcGainTransmittance(distance);
//    double ifov_rad_sqr = powf(deg2rad(ifov), 2);
    double e = 0.02 * (1 - powf(1 - cos(beta), 5));
    double solar_ifov = solar_coeff / M_PI * (1 - e) * transmiss;
    double ocean_ifov = ocean_coeff / M_PI * 0.9 * (1 - e) * transmiss ;
    double sky_ifov = sky_coeff / M_PI * (1 - e) * transmiss;
    double object_ifov = object_coeff / M_PI * e * transmiss;
    double path_ifov = path_coeff / M_PI;

    return (object_ifov + solar_ifov + ocean_ifov + sky_ifov + path_ifov);
}

__device__ double oceanRadiance(double ifov, double distance, double beta,
                                double solar_coeff, double ocean_coeff,
                                double sky_coeff, double object_coeff,
                                double path_coeff)
{
    double transmiss = calcGainTransmittance(distance);
//    double ifov_rad_sqr = powf(deg2rad(ifov), 2);
    double e = 0.9 * (1 - powf(1 - cos(beta), 5));

    double solar_ifov = solar_coeff / M_PI * (1 - e) * transmiss;
    double ocean_ifov = ocean_coeff / M_PI * e * transmiss ;
    double sky_ifov = sky_coeff / M_PI * (1 - e) * transmiss;
    double path_ifov = path_coeff / M_PI * (1 - expf(-0.25 * distance /1000));

    return (solar_ifov + ocean_ifov + sky_ifov + path_ifov);
}

__device__ double skyRadiance(double ifov, double sky_coeff, double path_coeff)
{
//    double ifov_rad_sqr = powf(deg2rad(ifov), 2);
    return (sky_coeff / M_PI + path_coeff /M_PI);
}

__device__ bool isInsideSurface(double2* data, int length_data, double2 imgPos)
{
    double sum = 0;
    for(int i = 0; i < length_data; i++)
    {
        double2 vector1;
        double2 vector2;

        vector1.x = data[i].x - imgPos.x;
        vector1.y = data[i].y - imgPos.y;
        if(i == length_data - 1)
        {
            vector2.x = data[0].x - imgPos.x;
            vector2.y = data[0].y - imgPos.y;
        }
        else
        {
            vector2.x = data[i + 1].x - imgPos.x;
            vector2.y = data[i + 1].y - imgPos.y;
        }

        double cos_phi = inner_product(vector1, vector2) / norm(vector1) / norm(vector2);

        if(norm(vector1) * norm(vector2) < 0.00001)
        {
            return true;
        }
        sum = sum + acosf(cos_phi);
    }

    if(sum > 2 * M_PI - 0.01)
    {
        return true;
    }
    else
    {
        return false;
    }
}

__global__ void testFuckingFunc(Coordinate coor)
{
    GPS result = ECEF2Geoditic(coor);
    printf("Result = %f - %f - %f\n", result.latitude, result.longtitude, result.height);
}

void Simulator::testFuck()
{
    Coordinate data(-1670848.125000, 5792593.000000, 2075039.375000);
    testFuckingFunc<<<1, 1>>>(data);
    gpuErrChk(cudaDeviceSynchronize());
}
__global__ void testInsideSurface(double2* data, int length_data, double2 imgPos)
{
    printf("Result = %d\n", isInsideSurface(data, length_data, imgPos));
}


void Simulator::testFunc()
{
    double2 hData[3];
    hData[0] = {0, -1};
    hData[1] = {1, 0};
    hData[2] = {-1, 1};
    double2 *dData;
    double2 pt = {1, 0};
    double2 *dPt;

    gpuErrChk(cudaMalloc((void**)&dPt, sizeof(double2)));
    gpuErrChk(cudaMemcpy(dPt, &pt, sizeof(double2), cudaMemcpyHostToDevice));
    gpuErrChk(cudaMalloc((void**)&dData, 3 * sizeof(double2)));
    gpuErrChk(cudaMemcpy(dData, &hData[0], 3 * sizeof(double2), cudaMemcpyHostToDevice));

    testInsideSurface<<<1, 1>>>(dData, 3, pt);

    gpuErrChk(cudaDeviceSynchronize());

    gpuErrChk(cudaFree(dData));
    gpuErrChk(cudaFree(dPt));
}

__device__ double2 cudaImageModel(ObjStatus *missile, GPS target_gps,
                                  double * Rb2c_cur, double *Ri2b_missile_cur,
                                  double * Re2i_missile, double fov_pixel)
{
    double2 result;
    Coordinate target_pos = Geoditic2ECEF(target_gps);
    Coordinate missile_pos = Geoditic2ECEF(missile->gps);
    double distance = (missile_pos - target_pos).norm();
    //            sqrt((missile_pos.x - target_pos.x) * (missile_pos.x - target_pos.x) +
    //                           (missile_pos.y - target_pos.y) * (missile_pos.y - target_pos.y) +
    //                           (missile_pos.z - target_pos.z) * (missile_pos.z - target_pos.z));

    double NED[3];
    NED[0] = (missile_pos.x - target_pos.x) / distance;
    NED[1] = (missile_pos.y - target_pos.y) / distance;
    NED[2] = (missile_pos.z - target_pos.z) / distance;

    double Ldonic[3];
    double temp1[9];
    double temp2[9];

    mul3x3TransposeBoth(&temp1[0], Rb2c_cur, Ri2b_missile_cur);
    mul3x3(&temp2[0], &temp1[0], Re2i_missile);
    mul3x3ToVec3x1(&Ldonic[0], &temp2[0], &NED[0]);

    if(Ldonic[2] < 0)
    {
        Ldonic[2] = - Ldonic[2];
        Ldonic[1] = - Ldonic[1];
        Ldonic[0] = - Ldonic[0];
    }

    result.x = Ldonic[0] / Ldonic[2] * fov_pixel;
    result.y = Ldonic[1] / Ldonic[2] * fov_pixel;
    //    printf("res = %f - %f\n", result.x, result.y);
    return result;
}

__global__ void cudaConvertToImage(GPS * ship_gps, double2 * shipImgPos,
                                   double3 * ship_vertices, ObjStatus *missile_cur, ObjStatus *target_cur,
                                   double *Rb2c_cur, double *Ri2b_missile_cur,
                                   double *Re2i_missile,
                                   double *Ri2b_target, double * Re2i_target,
                                   int num_vertices, double fov_pixel)
{
    int idx = threadIdx.x + IMUL(blockDim.x, blockIdx.x);
    if(idx < num_vertices)
    {
        double vertex[3];
        vertex[0] = ship_vertices[idx].x;
        vertex[1] = ship_vertices[idx].y;
        vertex[2] = ship_vertices[idx].z;

        double temp[9];
        double NED[3];

        mul3x3TransposeFirst(&temp[0], Re2i_target, Ri2b_target);
        mul3x3ToVec3x1(&NED[0], temp, vertex);
        Coordinate tmp = Geoditic2ECEF(target_cur->gps) + Coordinate(NED[0], NED[1], NED[2]);


        ship_gps[idx] = ECEF2Geoditic(tmp);
        shipImgPos[idx] = cudaImageModel(missile_cur,ship_gps[idx], Rb2c_cur,
                                         Ri2b_missile_cur, Re2i_missile, fov_pixel);
    }
}

void Simulator::convertToImage(ObjStatus* missile_cur, ObjStatus *target_cur)
{
    int numBlock = (m_ship.num_vertices + threadsPerBlock - 1) / threadsPerBlock;
    cudaConvertToImage<<<numBlock, threadsPerBlock>>>(m_ship.gps, m_ship.imgPos,
                                                      m_ship.vertices, missile_cur,
                                                      target_cur,
                                                      m_Rb2c_cur, m_Ri2b_missile_cur,
                                                      m_Re2i_missile,
                                                      m_Ri2b_target,
                                                      m_Re2i_target,
                                                      m_ship.num_vertices, m_fov_pixel);
    gpuErrChk(cudaDeviceSynchronize());
}

__device__ void calcNED(double *__restrict__ NED, double2 imgPos,
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
    mul3x3TransposeFirst(&tmp[0], (double*)Re2i_missile, (double*)Ri2b_missile_cur);
    mul3x3(&tmp1[0], &tmp[0], (double*)Rb2c_cur);
    mul3x3ToVec3x1(&NED[0], &tmp1[0], &Ldonic[0]);
}

__device__ void calcDistanceToOcean(double *dist, double*angle, int* objIdx,
                                    double * __restrict__ NED, double2 imgPos, Coordinate missile_pos)
{
    Coordinate NED1;
    double missile_norm = missile_pos.norm();
    //    NED1 = missile_pos / missile_pos.norm();
    NED1.x = missile_pos.x /missile_norm;
    NED1.y = missile_pos.y / missile_norm;
    NED1.z = missile_pos.z / missile_norm;
    GPS missile_gps = ECEF2Geoditic(missile_pos);
    Coordinate tmp(NED[0], NED[1], NED[2]);
    double L = missile_pos.norm() * fabs(tmp * NED1);

    GPS cutting_point_gps = ECEF2Geoditic(missile_pos + NED1 * L);
    if(cutting_point_gps.height > 0)
    {
        dist[0] = -1;
        angle[0] = -1;
        objIdx[0] = 0;
    }
    else
    {
        int iter = 0;
        while(fabs(cutting_point_gps.height) > 0.05 && iter < 2000)
        {
            iter++;
            L = L * missile_gps.height / (missile_gps.height - cutting_point_gps.height);
            cutting_point_gps = ECEF2Geoditic(missile_pos + NED1 * L);
        }
        Coordinate NEDtmp(NED[0], NED[1], NED[2]);
        NED1 = (missile_pos + NEDtmp * L) / (missile_pos + NEDtmp * L).norm();
        dist[0] = L;
        angle[0] = acosf(fabs(NEDtmp * NED1));
        objIdx[0] = 1;
    }
}

__device__ void averageDistance(double * dist, double*angle, int* objIdx,
                                const double2 * __restrict__ surfaceImgPos, double2 imgPos,
                                const GPS * __restrict__ surface_gps,
                                const double * __restrict__ NED, Coordinate missile_pos)
{
    double distance = 1000000000;
    for(int i = 0; i < 3; i++)
    {
        int idx1 = i;
        int idx2;
        if(i == 2)
        {
            idx2 = 0;
        }
        else
        {
            idx2 = i + 1;
        }

        double2 vector1;
        double2 vector2;
        vector1.x = surfaceImgPos[idx1].x - imgPos.x;
        vector1.y = surfaceImgPos[idx1].y - imgPos.y;

        vector2.x = surfaceImgPos[idx2].x - imgPos.x;
        vector2.y = surfaceImgPos[idx2].y - imgPos.y;

        Coordinate tmp1, tmp2;
        double d;
        double norm1 = norm(vector1);
        double norm2 = norm(vector2);
        tmp1 = Geoditic2ECEF(surface_gps[idx1]);
        tmp2 = Geoditic2ECEF(surface_gps[idx2]);

        if(norm1 == 0 || norm2 == 0)
        {
            if(norm1 == 0)
            {
                d = (tmp1 - missile_pos).norm();
            }

            if(norm2 == 0)
            {
                d = (tmp2 - missile_pos).norm();
            }
        }
        else
        {
            if(inner_product(vector1, vector2) / (norm1 * norm2) < 0)
            {
                Coordinate NED1 = (tmp1 - tmp2) / (tmp1 - tmp2).norm();

                double A[6];
                double B[3];

                B[0] = tmp1.x - missile_pos.x;
                B[1] = tmp1.y - missile_pos.y;
                B[2] = tmp1.z - missile_pos.z;

                A[0] = NED[0];
                A[1] = -NED1.x;
                A[2] = NED[1];
                A[3] = -NED1.y;
                A[4] = NED[2];
                A[5] = -NED1.z;

                double pinvA[6];
                pinv3x2(&pinvA[0], &A[0]);

                double t = pinvA[0] * B[0] + pinvA[1] * B[1] + pinvA[2] * B[2];
                d = t * sqrt(NED[0] * NED[0] + NED[1] * NED[1] + NED[2] * NED[2]);
            }
        }
        if(d < distance)
        {
            distance = d;
        }
    }
    dist[0] = distance;
    angle[0] = M_PI / 2;
    objIdx[0] = 2;
}

__device__ void calcDistanceToFace(double * dist, double *angle, int*objIdx, const double * __restrict__ NED, const double2 * __restrict__ surfaceImgPos,
                                   double2 imgPos, Coordinate missile_pos, const GPS *__restrict__ surface_gps)
{
    Coordinate vertex1 = Geoditic2ECEF(surface_gps[0]);

    Coordinate vector1 = vectorCalculator(surface_gps[0], surface_gps[1]);
    Coordinate vector2 = vectorCalculator(surface_gps[1], surface_gps[2]);

    Coordinate n = cross_product(vector1, vector2);
    double d = vertex1 * n;
    double coeff = missile_pos * n - d;
    if(coeff == 0)
    {
        //        averageDistance(dist, angle, objIdx, surfaceImgPos, imgPos, surface_gps, &NED[0], missile_pos);
        double distance = 1000000000;
        for(int i = 0; i < 3; i++)
        {
            int idx1 = i;
            int idx2;
            if(i == 2)
            {
                idx2 = 0;
            }
            else
            {
                idx2 = i + 1;
            }

            double2 vector1;
            double2 vector2;
            vector1.x = surfaceImgPos[idx1].x - imgPos.x;
            vector1.y = surfaceImgPos[idx1].y - imgPos.y;

            vector2.x = surfaceImgPos[idx2].x - imgPos.x;
            vector2.y = surfaceImgPos[idx2].y - imgPos.y;

            Coordinate tmp1, tmp2;
            double d;
            double norm1 = norm(vector1);
            double norm2 = norm(vector2);
            tmp1 = Geoditic2ECEF(surface_gps[idx1]);
            tmp2 = Geoditic2ECEF(surface_gps[idx2]);

            if(norm1 == 0 || norm2 == 0)
            {
                if(norm1 == 0)
                {
                    d = (tmp1 - missile_pos).norm();
                }

                if(norm2 == 0)
                {
                    d = (tmp2 - missile_pos).norm();
                }
            }
            else
            {
                if(inner_product(vector1, vector2) / (norm1 * norm2) < 0)
                {
                    Coordinate NED1 = (tmp1 - tmp2) / (tmp1 - tmp2).norm();

                    double A[6];
                    double B[3];

                    B[0] = tmp1.x - missile_pos.x;
                    B[1] = tmp1.y - missile_pos.y;
                    B[2] = tmp1.z - missile_pos.z;

                    A[0] = NED[0];
                    A[1] = -NED1.x;
                    A[2] = NED[1];
                    A[3] = -NED1.y;
                    A[4] = NED[2];
                    A[5] = -NED1.z;

                    double pinvA[6];
                    pinv3x2(&pinvA[0], &A[0]);

                    double t = pinvA[0] * B[0] + pinvA[1] * B[1] + pinvA[2] * B[2];
                    d = t * sqrt(NED[0] * NED[0] + NED[1] * NED[1] + NED[2] * NED[2]);
                }
            }
            if(d < distance)
            {
                distance = d;
            }
        }
        dist[0] = distance;
        angle[0] = M_PI / 2;
        objIdx[0] = 2;
    }
    else
    {
        Coordinate NEDtmp(NED[0], NED[1], NED[2]);
        double distance = NEDtmp.norm() / fabs(NEDtmp * n) * fabs(coeff);
        double beta = acosf((NEDtmp * n) / NEDtmp.norm() / n.norm());
        dist[0] = distance;
        angle[0] = beta;
        objIdx[0] = 2;
        //        printf("distance = %f\n", distance);
    }
}

__global__ void cudaCalcDistance(double * __restrict__ distance, double* __restrict__ angle, int* __restrict__ objIdx,
                                 GPS3 *  ship_surface_gps,
                                 double6 *  ship_surface_imgPos,
                                 ObjStatus *  missile_cur,
                                 double *  Rb2c_cur, double *  Ri2b_missile_cur,
                                 double *  Re2i_missile, double fov_pixel,
                                 int num_faces, int num_partialPix, int offset,
                                 int batch_size, int width, int height)
{
    int partialPixIdx = threadIdx.x + IMUL(blockIdx.x, blockDim.x);
    int faceIdx = threadIdx.y + IMUL(blockIdx.y, blockDim.y);

    if(faceIdx < num_faces && partialPixIdx < num_partialPix)
    {
        // index of result array to be calculated(double* distance)
        int resultIdx = faceIdx * batch_size * PIXEL_GRID_SIZE * PIXEL_GRID_SIZE + partialPixIdx;

        // pixel index in the output image(640 * 480)
        int pixIdx = offset * batch_size + partialPixIdx / (PIXEL_GRID_SIZE * PIXEL_GRID_SIZE);

        // row, col index of current thread in the output width * height (640 * 480) image
        int row = pixIdx / width;
        int col = pixIdx % width;
        // index of subpixel inside the subgrid for each pixel
        int partialPixIdx_X = (partialPixIdx % (PIXEL_GRID_SIZE * PIXEL_GRID_SIZE)) % PIXEL_GRID_SIZE;
        int partialPixIdx_Y = (partialPixIdx % (PIXEL_GRID_SIZE * PIXEL_GRID_SIZE)) / PIXEL_GRID_SIZE;

        Coordinate missile_pos;
        missile_pos = Geoditic2ECEF(missile_cur->gps);

        double u = col + (double)(partialPixIdx_X - 0.5)/ PIXEL_GRID_SIZE - width / 2 - 0.5;
        double w = row + (double)(partialPixIdx_Y - 0.5)/ PIXEL_GRID_SIZE - height / 2 - 0.5;
        double2 imgPos;
        imgPos.x = u;
        imgPos.y = w;

        double2 surface_imgPos[3];
        surface_imgPos[0] = ship_surface_imgPos[faceIdx].x;
        surface_imgPos[1] = ship_surface_imgPos[faceIdx].y;
        surface_imgPos[2] = ship_surface_imgPos[faceIdx].z;
        //        printf("%f - %f\n", surface_imgPos[0].x, surface_imgPos[0].y);
        GPS surface_gps[3];
        surface_gps[0] = ship_surface_gps[faceIdx].x;
        surface_gps[1] = ship_surface_gps[faceIdx].y;
        surface_gps[2] = ship_surface_gps[faceIdx].z;

        double NED[3];
        calcNED(&NED[0], imgPos, Rb2c_cur, Ri2b_missile_cur, Re2i_missile, fov_pixel);

        if(isInsideSurface(&surface_imgPos[0], 3, imgPos))
        {
            //            printf("Jump into object\n");
            //            objIdx[resultIdx]  = 2;
            //            calcDistanceToFace(&distance[resultIdx], &angle[resultIdx], &objIdx[resultIdx],
            //                               &NED[0], &surface_imgPos[0], imgPos,
            //                                missile_pos, &surface_gps[0]);

            Coordinate vertex1 = Geoditic2ECEF(surface_gps[0]);

            Coordinate vector1 = vectorCalculator(surface_gps[0], surface_gps[1]);
            Coordinate vector2 = vectorCalculator(surface_gps[1], surface_gps[2]);

            Coordinate n = cross_product(vector1, vector2);
            double d = vertex1 * n;
            double coeff = missile_pos * n - d;
            //            if(coeff == 0)
            //            {
            //                averageDistance(dist, angle, objIdx, surfaceImgPos, imgPos, surface_gps, &NED[0], missile_pos);
            //            }
            //            else
            {
                Coordinate NEDtmp(NED[0], NED[1], NED[2]);
                double dist = NEDtmp.norm() / fabs(NEDtmp * n) * fabs(coeff);
                double beta = acosf((NEDtmp * n) / NEDtmp.norm() / n.norm());
                //                                printf("dist = %f\n", dist);
                distance[resultIdx] = dist;
                angle[resultIdx] = beta;
                objIdx[resultIdx] = 2;
            }
        }
        else
        {
//            objIdx[resultIdx]  = 0;
            //            printf("Jump into ocean\n");
            //            calcDistanceToOcean(&distance[resultIdx], &angle[resultIdx], &objIdx[resultIdx],
            //                                &NED[0], imgPos, missile_pos);

            Coordinate NED1;
            double missile_norm = missile_pos.norm();

            //    NED1 = missile_pos / missile_pos.norm();
            NED1.x = missile_pos.x /missile_norm;
            NED1.y = missile_pos.y / missile_norm;
            NED1.z = missile_pos.z / missile_norm;
            GPS missile_gps = ECEF2Geoditic(missile_pos);
            Coordinate tmp(NED[0], NED[1], NED[2]);
            double L = missile_pos.norm() * fabs(tmp * NED1);

            GPS cutting_point_gps = ECEF2Geoditic(missile_pos + tmp * L);
            if(cutting_point_gps.height > 0)
            {
                distance[resultIdx] = -1;
                angle[resultIdx] = -1;
                objIdx[resultIdx] = 0;
            }
            else
            {
                int iter = 0;
                while(fabs(cutting_point_gps.height) > 0.05 && iter < 1000)
                {
                    iter++;
                    L = L * missile_gps.height / (missile_gps.height - cutting_point_gps.height);
                    cutting_point_gps = ECEF2Geoditic(missile_pos + NED1 * L);
                }
                Coordinate NEDtmp(NED[0], NED[1], NED[2]);
                NED1 = (missile_pos + NEDtmp * L) / (missile_pos + NEDtmp * L).norm();
                distance[resultIdx] = L;
                angle[resultIdx] = acosf(fabs(NEDtmp * NED1));
                objIdx[resultIdx] = 1;
            }
        }
    }
}

void Simulator::calcDistance(int offset, ObjStatus * missile_cur)
{
    int num_partialPix = m_batch_size * PIXEL_GRID_SIZE * PIXEL_GRID_SIZE;
    int num_surfaces = m_ship.num_surfaces;

    dim3 blockDim(threadsPerBlock, threadsPerBlock);
    dim3 gridDim(ceil((double)num_partialPix / threadsPerBlock),
                 ceil((double)num_surfaces / threadsPerBlock));

    cudaCalcDistance<<<gridDim, blockDim>>>(m_ray.distance, m_ray.angle, m_ray.objIdx, m_ship.surface_gps,
                                            m_ship.surface_imgPos, missile_cur,
                                            m_Rb2c_cur, m_Ri2b_missile_cur, m_Re2i_missile, m_fov_pixel,
                                            num_surfaces, num_partialPix, offset, m_batch_size, m_width, m_height);
    gpuErrChk(cudaDeviceSynchronize());
}

__global__ void cudaCalcMask(const int *__restrict__ objIdx,
                             unsigned char * maskImg,
                             int num_surfaces, int batch_size, int offset)
{
    int idx = threadIdx.x + IMUL(blockIdx.x, blockDim.x);
    int grid_size = batch_size * PIXEL_GRID_SIZE * PIXEL_GRID_SIZE;
    if(idx < grid_size)
    {
        for(int i = 0; i < num_surfaces; i++)
        {
            int distIdx = i * grid_size + idx;
            if(objIdx[distIdx] == 2)
            {
                maskImg[offset * batch_size + idx] = 255;
                break;
            }
        }
    }
}

void Simulator::mask(int offset)
{
    int numBlock = (m_batch_size * PIXEL_GRID_SIZE * PIXEL_GRID_SIZE + threadsPerBlock * threadsPerBlock - 1) /(threadsPerBlock * threadsPerBlock);

    cudaCalcMask<<<numBlock, threadsPerBlock * threadsPerBlock>>>(m_ray.objIdx, m_maskImg, m_ship.num_surfaces, m_batch_size, offset);
    gpuErrChk(cudaDeviceSynchronize());
}

__global__ void cudaCalcRadiance(const double * __restrict__ distance,
                                 const double * __restrict__ angle,
                                 const int *__restrict__ objIdx,
                                 double * __restrict__ radiance,
                                 int num_surfaces, int batch_size, double ifov,
                                 double solar_coeff, double ocean_coeff,
                                 double sky_coeff, double object_coeff,
                                 double path_coeff, int offset)
{
    int idx = threadIdx.x + IMUL(blockDim.x, blockIdx.x);
    int grid_size = batch_size * PIXEL_GRID_SIZE * PIXEL_GRID_SIZE;
    if(idx < grid_size)
    {
        bool hitObj = false;
        double minDist = 100000000;
        int hitSurfaceIdx;
        for(int i = 0; i < num_surfaces; i++)
        {
            int distIdx = i * grid_size + idx;
            if(objIdx[distIdx] == 2)
            {
                hitObj = true;
                if(distance[distIdx] < minDist)
                {
                    minDist = distance[distIdx];
                    hitSurfaceIdx = distIdx;
                }
            }
        }

        if(hitObj)
        {
//            radiance[idx] = objectRadiance(ifov, minDist, angle[hitSurfaceIdx],
//                                           solar_coeff, ocean_coeff,
//                                           sky_coeff, object_coeff,
//                                           path_coeff);

            double transmiss = calcGainTransmittance(minDist);
//            double ifov_rad_sqr = powf(deg2rad(ifov), 2);
            double e = 0.02 * (1 - powf(1 - cos(angle[hitSurfaceIdx]), 5));
            double solar_ifov = solar_coeff / M_PI * (1 - e) * transmiss;
            double ocean_ifov = ocean_coeff / M_PI * 0.9 * (1 - e) * transmiss;
            double sky_ifov = sky_coeff / M_PI * (1 - e) * transmiss;
            double object_ifov = object_coeff / M_PI * e * transmiss;
            double path_ifov = path_coeff / M_PI;

            radiance[idx] = (object_ifov + solar_ifov + ocean_ifov + sky_ifov + path_ifov);
//            printf("%f - %f - %f - %f - %f\n", solar_ifov, ocean_ifov, sky_ifov, object_ifov, path_ifov);
//            printf("%f - %f - %f - %f - %f\n", solar_ifov, ocean_ifov, sky_ifov, object_ifov, path_ifov);

//            printf("Object radiance - minDist - beta = %f - %f - %f - %f - %f - %f - %f - %f - %f\n",
//                                                                        radiance[idx], ifov, minDist, angle[hitSurfaceIdx],
//                                                                        solar_coeff, ocean_coeff,
//                                                                        sky_coeff, object_coeff,
//                                                                        path_coeff);
        }
        else
        {
            if(objIdx[idx] == 0)
            {
                radiance[idx] = skyRadiance(ifov, sky_coeff, path_coeff);
//                printf("Sky = %f\n", radiance[idx]);
            }
            else if(objIdx[idx] == 1)
            {
                radiance[idx] = oceanRadiance(ifov, distance[idx], angle[idx],
                                              solar_coeff, ocean_coeff,
                                              sky_coeff, object_coeff,
                                              path_coeff);
            }
        }
    }
}

void Simulator::calcRadiance(int offset)
{
    double ifov = m_fov / m_width;
    int numBlock = (m_batch_size * PIXEL_GRID_SIZE * PIXEL_GRID_SIZE + threadsPerBlock * threadsPerBlock - 1) /(threadsPerBlock * threadsPerBlock);

    cudaCalcRadiance<<<numBlock, threadsPerBlock * threadsPerBlock>>>(m_ray.distance, m_ray.angle, m_ray.objIdx, m_partialRadiance, m_ship.num_surfaces, m_batch_size, ifov,
                                                                      m_solar_coeff, m_ocean_coeff, m_sky_coeff, m_object_coeff, m_horizon_coeff, offset);
    gpuErrChk(cudaDeviceSynchronize());
}

__global__ void cudaRenderPartialImg(unsigned char * renderedImg, double * radiance, int fov, float ifov, int offset, int batch_size)
{
    int idx = threadIdx.x + IMUL(blockDim.x, blockIdx.x);
    if(idx < batch_size)
    {
        double tmp = 0;
#pragma unroll
        for(int i = 0; i < PIXEL_GRID_SIZE * PIXEL_GRID_SIZE; i++)
        {
            tmp = tmp + radiance[idx * PIXEL_GRID_SIZE * PIXEL_GRID_SIZE + i];
        }
        int imgIdx = offset * batch_size + idx;
        double ifov_rad_sqr = powf(deg2rad(ifov), 2);
        renderedImg[imgIdx] = (unsigned char)(tmp / (PIXEL_GRID_SIZE * PIXEL_GRID_SIZE) * 0.5 * ifov_rad_sqr * powf(10, 8) * 255/ powf(fov, 2));
//        printf("num = %f\n",  ifov_rad_sqr * powf(10, 8) * 255/ powf(fov, 2));

        if(renderedImg[imgIdx] > 255)
        {
            renderedImg[imgIdx] = 255;
        }
    }
}

void Simulator::renderPartialImg(int offset)
{
    double ifov = m_fov / m_width;
    cudaRenderPartialImg<<<m_batch_size, PIXEL_GRID_SIZE * PIXEL_GRID_SIZE>>>(m_renderedImg, m_partialRadiance, m_fov, ifov, offset, m_batch_size);
    gpuErrChk(cudaDeviceSynchronize());
}

__global__ void calcTranformationMatricesHelper(ObjStatus* missile_cur,
                                                ObjStatus* missile_prev,
                                                ObjStatus* target_cur,
                                                ObjStatus* target_prev,
                                                SeekerInfo* seeker_cur,
                                                SeekerInfo* seeker_prev,
                                                double* Rb2c_cur, double* Ri2b_missile_cur,
                                                double* Re2i_missile,
                                                double* Ri2b_target,
                                                double* Re2i_target,
                                                double* Rb2c_prev,
                                                double* Ri2b_missile_prev)
{
    getRb2cMatrix(Rb2c_cur, RotationAngle(0, seeker_cur->elevation, seeker_cur->azimuth));
    getRb2cMatrix(Rb2c_prev, RotationAngle(0, seeker_prev->elevation, seeker_prev->azimuth));
    getRi2bMatrix(Ri2b_missile_cur, missile_cur->angle);
    getRi2bMatrix(Ri2b_missile_prev, missile_prev->angle);
    getRi2bMatrix(Ri2b_target, target_cur->angle);
    getRe2iMatrix(Re2i_missile, missile_cur->gps);
    getRe2iMatrix(Re2i_target, target_cur->gps);
}

void Simulator::calcTranformationMatrices(ObjStatus* missile_cur, ObjStatus* missile_prev,
                                          ObjStatus *target_cur, ObjStatus* target_prev,
                                          SeekerInfo* seeker_cur, SeekerInfo* seeker_prev)
{
    calcTranformationMatricesHelper<<<1, 1>>>(missile_cur, missile_prev,target_cur, target_prev,
                                              seeker_cur, seeker_prev,
                                              m_Rb2c_cur,m_Ri2b_missile_cur,
                                              m_Re2i_missile, m_Ri2b_target,m_Re2i_target,
                                              m_Rb2c_prev, m_Ri2b_missile_prev);
    gpuErrChk(cudaDeviceSynchronize());
}
