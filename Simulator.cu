#include "Simulator.h"


__device__ float deg2rad(float deg)
{
    return deg * M_PI / 180.0;
}

__device__ float rad2deg(float rad)
{
    return rad * 180.0 / M_PI;
}

__device__ float calcGainTransmittance(float distance)
{
    return expf(-0.26 * distance / 1000);
}

__device__ float norm(float2 input)
{
    return sqrtf(input.x * input.x + input.y * input.y);
}

__device__ float inner_product(float2 a, float2 b)
{
    return (a.x * b.x + a.y * b.y);
}

__device__ Coordinate cross_product(Coordinate a, Coordinate b)
{
    return Coordinate(a.y * b.z - a.z * b.y,
                      a.z * b.x - a.x * b.z,
                      a.x * b.y - a.y * b.x);
}

__device__ void mul3x3ToVec3x1(float *dst, float *mat, float *vec)
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

__device__ void mul3x3(float *dst, const float *__restrict__ src1, const float * __restrict__ src2)
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

__device__ void mul3x3TransposeFirst(float *dst, const float * __restrict__ src1,
                                              const float * __restrict__ src2)
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

__device__ void mul3x3TransposeBoth(float *dst, const float *__restrict__ src1, const float * __restrict__ src2)
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

__device__ void pinv3x2(float *dst, float *src)
{
    float tmp[4];
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

    float det = tmp[0] * tmp[3] - tmp[1] * tmp[2];
    float inv[4];
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
__device__ void getRi2bMatrix(float* matrix, RotationAngle angle)
{
    matrix[0] = cosf(angle.pitch) * cosf(angle.yaw);
    matrix[1] = sinf(angle.roll) * sinf(angle.pitch) * cosf(angle.yaw) -
            cosf(angle.roll) * sinf(angle.yaw);
    matrix[2] = cosf(angle.roll)*sinf(angle.pitch)*cosf(angle.yaw) +
            sinf(angle.roll)*sinf(angle.yaw);
    matrix[3] = cosf(angle.pitch) * sinf(angle.yaw);
    matrix[4] = sinf(angle.roll) * sinf(angle.pitch) * sinf(angle.yaw) +
            cosf(angle.roll) * cosf(angle.yaw);
    matrix[5] = cosf(angle.roll) * sinf(angle.pitch) * sinf(angle.yaw) -
            sinf(angle.roll) * cosf(angle.yaw);
    matrix[6] = -sinf(angle.pitch);
    matrix[7] = sinf(angle.roll) * cosf(angle.pitch);
    matrix[8] = cosf(angle.roll) * cosf(angle.pitch);
}
// Matrix Ldoni in matlab code
__device__ void getRb2cMatrix(float* matrix, RotationAngle angle)
{
    matrix[0] = -sinf(angle.yaw);
    matrix[1] = sinf(angle.pitch) * cosf(angle.yaw);
    matrix[2] = cosf(angle.pitch) * cosf(angle.yaw);
    matrix[3] = cosf(angle.yaw);
    matrix[4] = sinf(angle.pitch) * sinf(angle.yaw);
    matrix[5] = cosf(angle.pitch) * sinf(angle.yaw);
    matrix[6] = 0;
    matrix[7] = cosf(angle.pitch);
    matrix[8] =-sinf(angle.pitch);
}
// Matrix M in matlab code
__device__ void getRe2iMatrix(float* matrix, GPS gps)
{
    float lambda = gps.latitude / 180.0 * M_PI;
    float phi = gps.longtitude / 180.0 * M_PI;

    matrix[0] = -cosf(phi) * sinf(lambda);;
    matrix[1] = -sinf(lambda) * sinf(phi);
    matrix[2] = cosf(lambda);
    matrix[3] = -sinf(phi);
    matrix[4] = cosf(phi);
    matrix[5] = 0;
    matrix[6] = -cosf(lambda) * cosf(phi);
    matrix[7] = -cosf(lambda) * sinf(phi);
    matrix[8] = -sinf(lambda);
}

__device__ GPS ECEF2Geoditic(Coordinate pos)
{
    GPS result;
    float x = pos.x;
    float y = pos.y;
    float z = pos.z;

    float a = 6378137;
    float b = 6356752;
    float f = (a - b) / a;
    float e_sq = f * (2 - f);
    float eps = e_sq / (1.0 - e_sq);
    float p = sqrtf(x * x + y * y);
    float q = atan2(z * a, p * b);
    float sinf_q = sinf(q);
    float cosf_q = cosf(q);
    float sinf_q_3 = sinf_q *sinf_q * sinf_q;
    float cosf_q_3 = cosf_q * cosf_q * cosf_q;
    float phi = atan2(z + eps * b * sinf_q_3, p - e_sq * a * cosf_q_3);
    float lambda = atan2(y, x);
    float v = a / sqrtf(1.0 - e_sq * sinf(phi) * sinf(phi));
    result.height = p / cosf(phi) - v;
    result.latitude = phi / M_PI * 180;
    result.longtitude = lambda / M_PI * 180;

    return result;
}

__device__ Coordinate Geoditic2ECEF(GPS gps)
{
    Coordinate result;

    float a = 6378137;
    float b = 6356752;
    float f = (a - b) / a;
    float e_sq = f * (2 - f);

    float lambda = gps.latitude / 180 * M_PI;
    float phi = gps.longtitude / 180 * M_PI;

    float N = a / sqrtf(1 - e_sq * sinf(lambda) * sinf(lambda));
    result.x = (gps.height + N) * cosf(lambda) * cosf(phi);
    result.y = (gps.height + N) * cosf(lambda) * sinf(phi);
    result.z = (gps.height + (1 - e_sq) * N) * sinf(lambda);
    return result;
}
__device__ Coordinate vectorCalculator(GPS src1, GPS src2)
{
    Coordinate coor1 = Geoditic2ECEF(src1);
    Coordinate coor2 = Geoditic2ECEF(src2);
    return (coor2 - coor1) / (coor2 - coor1).norm();
}
__device__ float objectRadiance(float ifov, float distance, float beta,
                                         float solar_coeff, float ocean_coeff,
                                         float sky_coeff, float object_coeff,
                                         float path_coeff)
{
    float transmiss = calcGainTransmittance(distance);
    float ifov_rad_sqr = powf(deg2rad(ifov), 2);
    float e = 0.02 * (1 - powf(1 - cosf(beta), 5));
    float solar_ifov = solar_coeff / M_PI * (1 - e) * transmiss * ifov_rad_sqr;
    float ocean_ifov = ocean_coeff / M_PI * 0.9 * (1 - e) * transmiss * ifov_rad_sqr;
    float sky_ifov = sky_coeff / M_PI * (1 - e) * transmiss * ifov_rad_sqr;
    float object_ifov = object_coeff / M_PI * e * transmiss * ifov_rad_sqr;
    float path_ifov = path_coeff / M_PI * ifov_rad_sqr;

    return (object_ifov + solar_ifov + ocean_ifov + sky_ifov + path_ifov);
}

__device__ float oceanRadiance(float ifov, float distance, float beta,
                                               float solar_coeff, float ocean_coeff,
                                               float sky_coeff, float object_coeff,
                                               float path_coeff)
{
    float transmiss = calcGainTransmittance(distance);
    float ifov_rad_sqr = powf(deg2rad(ifov), 2);
    float e = 0.9 * (1 - powf(1 - cosf(beta), 5));

    float solar_ifov = solar_coeff / M_PI * (1 - e) * transmiss * ifov_rad_sqr;
    float ocean_ifov = ocean_coeff / M_PI * e * transmiss * ifov_rad_sqr;
    float sky_ifov = sky_coeff / M_PI * (1 - e) * transmiss * ifov_rad_sqr;
    float path_ifov = path_coeff / M_PI * ifov_rad_sqr * (1 - expf(-0.25 * distance /1000));

    return (solar_ifov + ocean_ifov + sky_ifov + path_ifov);
}
__device__ float skyRadiance(float ifov, float sky_coeff, float path_coeff)
{
    float ifov_rad_sqr = powf(deg2rad(ifov), 2);
    return (sky_coeff / M_PI * ifov_rad_sqr + path_coeff /M_PI * ifov_rad_sqr);
}

__device__ bool isInsideSurface(float2* data, int length_data, float2 imgPos)
{
    float sum = 0;
    for(int i = 0; i < length_data; i++)
    {
        float2 vector1;
        float2 vector2;

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

        float cos_phi = inner_product(vector1, vector2) / norm(vector1) / norm(vector2);
        sum = sum + acosf(cos_phi);
    }

    if(sum > 2 * M_PI - 0.001)
    {
        return true;
    }
    else
    {
        return false;
    }
}

__device__ float2 cudaImageModel(ObjStatus *missile, GPS target_gps,
                                 float * Rb2c_cur, float *Ri2b_missile_cur,
                                 float * Re2i_missile, float fov_pixel)
{
    float2 result;
    Coordinate target_pos = Geoditic2ECEF(target_gps);
    Coordinate missile_pos = Geoditic2ECEF(missile->gps);
    float distance = sqrtf((missile_pos.x - target_pos.x) * (missile_pos.x - target_pos.x) +
                           (missile_pos.y - target_pos.y) * (missile_pos.y - target_pos.y) +
                           (missile_pos.z - target_pos.z) * (missile_pos.z - target_pos.z));

    float NED[3];
    NED[0] = (missile_pos.x - target_pos.x) / distance;
    NED[1] = (missile_pos.y - target_pos.y) / distance;
    NED[2] = (missile_pos.z - target_pos.z) / distance;

    float Ldonic[3];
    float temp1[9];
    float temp2[9];
    mul3x3TransposeBoth(&temp1[0], Rb2c_cur, Ri2b_missile_cur);
    mul3x3(&temp2[0], &temp1[0], Re2i_missile);
    mul3x3ToVec3x1(&Ldonic[0], &temp2[0], &NED[0]);
    if(Ldonic[2] < 0)
    {
        Ldonic[2] = - Ldonic[2];
    }
    result.x = Ldonic[0] / Ldonic[2] * fov_pixel;
    result.y = Ldonic[1] / Ldonic[2] * fov_pixel;
    return  result;
}

//__device__ bool Simulator::isShipAppear()
//{
//    ObjStatus missile = m_missile[m_current_img_id];
//    ObjStatus target = m_target[m_current_img_id];
//    float2 target_imgPos = cudaImageModel(missile, target.gps, m_Rb2c_cur,
//                                          m_Ri2b_missile_cur, m_Re2i_missile, m_fov_pixel);
//    Coordinate target_pos = Geoditic2ECEF(target.gps);
//    Coordinate missile_pos = Geoditic2ECEF(missile.gps);

//    float distance = (target_pos - missile_pos).norm();

//    float delta_az = asinf(target_imgPos.x / sqrtf(target_imgPos.x * target_imgPos.x + m_fov_pixel * m_fov_pixel));
//    float delta_el = asinf(target_imgPos.y / Coordinate(target_imgPos.x, target_imgPos.y, m_fov_pixel).norm());
//    //    float delta_u0 = atan(m_ship.length / distance) * f;
//    //    float delta_w0 = atan(m_ship.height / distance) * f;

//    return (std::fabs(delta_az) < deg2rad(m_fov / 2) + atan(15 / distance)) &&
//            (std::fabs(delta_el) < deg2rad(3 / 4 * m_fov /2 ) + atan(6 / distance));
//}


__global__ void cudaConvertToImage(GPS * ship_gps, float2 * shipImgPos,
                                   float3 * ship_vertices, ObjStatus *missile_cur, ObjStatus *target_cur,
                                   float *Rb2c_cur, float *Ri2b_missile_cur,
                                   float *Re2i_missile,
                                   float *Ri2b_target, float * Re2i_target,
                                   int num_vertices, float fov_pixel)
{
    int idx = threadIdx.x + IMUL(blockDim.x, blockIdx.x);
    if(idx < num_vertices)
    {
        float vertex[3];
        vertex[0] = ship_vertices[idx].x;
        vertex[1] = ship_vertices[idx].y;
        vertex[2] = ship_vertices[idx].z;

        float temp[9];
        float NED[3];
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

__device__ void calcNED(float *__restrict__ NED, float2 imgPos,
                        const float * __restrict__ Rb2c_cur, const float * __restrict__ Ri2b_missile_cur,
                        const float * __restrict__ Re2i_missile, float fov_pixel)
{
    float Ldonic[3];
    float normTmp = sqrtf(imgPos.x * imgPos.x + imgPos.y * imgPos.y + fov_pixel * fov_pixel);
    Ldonic[0] = imgPos.x /normTmp;
    Ldonic[1] = imgPos.y /normTmp;
    Ldonic[2] = fov_pixel / normTmp;

    float tmp[9];
    float tmp1[9];
    mul3x3TransposeFirst(&tmp[0], (float*)Re2i_missile, (float*)Ri2b_missile_cur);
    mul3x3(&tmp1[0], &tmp[0], (float*)Rb2c_cur);
    mul3x3ToVec3x1(&NED[0], &tmp1[0], &Ldonic[0]);
}

__device__ RayInfo calcDistanceToOcean(float * __restrict__ NED, float2 imgPos, Coordinate missile_pos)
{
    Coordinate NED1 = missile_pos / missile_pos.norm();
    printf("cutting point height = %f - %f - %f\n", NED1.x, NED1.y, NED1.z);
    GPS missile_gps = ECEF2Geoditic(missile_pos);
    Coordinate tmp(NED[0], NED[1], NED[2]);
    float L = missile_pos.norm() * fabs(tmp * NED1);
//    printf("cutting point height = %f\n", L);

    GPS cutting_point_gps = ECEF2Geoditic(missile_pos + NED1 * L);
    if(cutting_point_gps.height > 0)
    {
        return RayInfo(-1, -1, 0);
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
    }

    Coordinate NEDtmp(NED[0], NED[1], NED[2]);
    NED1 = (missile_pos + NEDtmp * L) / (missile_pos + NEDtmp * L).norm();
    return RayInfo(L, acosf(fabs(NEDtmp * NED1)), 1);
}

__device__ RayInfo averageDistance(const float2 * __restrict__ surfaceImgPos, float2 imgPos,
                                   const GPS * __restrict__ surface_gps,
                                   const float * __restrict__ NED, Coordinate missile_pos)
{
    float distance = 1000000000;
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

        float2 vector1;
        float2 vector2;
        vector1.x = surfaceImgPos[idx1].x - imgPos.x;
        vector1.y = surfaceImgPos[idx1].y - imgPos.y;

        vector2.x = surfaceImgPos[idx2].x - imgPos.x;
        vector2.y = surfaceImgPos[idx2].y - imgPos.y;

        Coordinate tmp1, tmp2;
        float d;
        float norm1 = norm(vector1);
        float norm2 = norm(vector2);
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

                float A[6];
                float B[3];

                B[0] = tmp1.x - missile_pos.x;
                B[1] = tmp1.y - missile_pos.y;
                B[2] = tmp1.z - missile_pos.z;

                A[0] = NED[0];
                A[1] = -NED1.x;
                A[2] = NED[1];
                A[3] = -NED1.y;
                A[4] = NED[2];
                A[5] = -NED1.z;

                float pinvA[6];
                pinv3x2(&pinvA[0], &A[0]);

                float t = pinvA[0] * B[0] + pinvA[1] * B[1] + pinvA[2] * B[2];
                d = t * sqrtf(NED[0] * NED[0] + NED[1] * NED[1] + NED[2] * NED[2]);
            }
        }
        if(d < distance)
        {
            distance = d;
        }
    }
    return RayInfo(distance, M_PI / 2, 2);
}

__device__ RayInfo calcDistanceToFace(const float * __restrict__ NED, const float2 * __restrict__ surfaceImgPos,
                                      float2 imgPos, Coordinate missile_pos, const GPS *__restrict__ surface_gps)
{
    Coordinate vertex1 = Geoditic2ECEF(surface_gps[0]);

    Coordinate vector1 = vectorCalculator(surface_gps[0], surface_gps[1]);
    Coordinate vector2 = vectorCalculator(surface_gps[1], surface_gps[2]);

    Coordinate n = cross_product(vector1, vector2);
    float d = vertex1 * n;
    float coeff = missile_pos * n - d;
    if(coeff == 0)
    {
        return averageDistance(surfaceImgPos, imgPos, surface_gps, &NED[0], missile_pos);
    }
    else
    {
        Coordinate NEDtmp(NED[0], NED[1], NED[2]);
        float distance = NEDtmp.norm() / fabs(NEDtmp * n) * fabs(coeff);
        float beta = acosf((NEDtmp * n) / NEDtmp.norm() / n.norm());
        return RayInfo(distance, beta, 2);
    }
}

__global__ void cudaCalcDistance(RayInfo * __restrict__ distance,
                                 const GPS3 * __restrict__ ship_surface_gps,
                                 const float6 * __restrict__ ship_surface_imgPos,
                                 const ObjStatus * __restrict__ missile_cur,
                                 const float * __restrict__ Rb2c_cur, const float * __restrict__ Ri2b_missile_cur,
                                 const float * __restrict__ Re2i_missile, float fov_pixel,
                                 int num_faces, int num_partialPix, int offset,
                                 int batch_size, int width, int height)
{
    int partialPixIdx = threadIdx.x + IMUL(blockIdx.x, blockDim.x);
    int faceIdx = threadIdx.y + IMUL(blockIdx.y, blockDim.y);

    // index of result array to be calculated(float* distance)
    int resultIdx = faceIdx * batch_size * PIXEL_GRID_SIZE * PIXEL_GRID_SIZE + partialPixIdx;

    // pixel index in the output image(640 * 480)
    int pixIdx = offset * batch_size + partialPixIdx / (PIXEL_GRID_SIZE * PIXEL_GRID_SIZE);

    // row, col index of current thread in the output width * height (640 * 480) image
    int row = pixIdx / width;
    int col = pixIdx % width;

    // index of subpixel inside the subgrid for each pixel
    int partialPixIdx_X = partialPixIdx / (batch_size * PIXEL_GRID_SIZE);
    int partialPixIdx_Y = partialPixIdx % (batch_size * PIXEL_GRID_SIZE);

    if(faceIdx < num_faces && partialPixIdx < num_partialPix)
    {
        Coordinate missile_pos;
        missile_pos = Geoditic2ECEF(missile_cur->gps);
        float u = col + (float)(partialPixIdx_X - 0.5)/ PIXEL_GRID_SIZE - width / 2 - 0.5;
        float w = row + (float)(partialPixIdx_Y - 0.5)/ PIXEL_GRID_SIZE - height / 2 - 0.5;
        float2 imgPos;
        imgPos.x = u;
        imgPos.y = w;

        // TODO: The following code performs a uncoalescing memory access
        // It is not easy to avoid since it require random data access pattern
        // A possible solution is to reorganize input data
        float2 surface_imgPos[3];
        surface_imgPos[0] = ship_surface_imgPos[faceIdx].x;
        surface_imgPos[1] = ship_surface_imgPos[faceIdx].y;
        surface_imgPos[2] = ship_surface_imgPos[faceIdx].z;

        GPS surface_gps[3];
        surface_gps[0] = ship_surface_gps[faceIdx].x;
        surface_gps[1] = ship_surface_gps[faceIdx].y;
        surface_gps[2] = ship_surface_gps[faceIdx].z;

        float NED[3];
        calcNED(&NED[0], imgPos, Rb2c_cur, Ri2b_missile_cur, Re2i_missile, fov_pixel);

//        isInsideSurface(&surface_imgPos[0], 3, imgPos);
//                    distance[resultIdx] = calcDistanceToFace(&NED[0], &surface_imgPos[0], imgPos,
//                            missile_pos, &surface_gps[0]);

        Coordinate NED1;
        float missile_pos_norm = missile_pos.norm();
//        NED1 = missile_pos / missile_pos.norm();
        NED1.x = missile_pos.x / missile_pos_norm;
        NED1.y = missile_pos.y / missile_pos_norm;
        NED1.z = missile_pos.z / missile_pos_norm;


//        printf("cutting point height = %f - %f - %f\n", NED1.x, NED1.y, NED1.z);
        GPS missile_gps = ECEF2Geoditic(missile_pos);
        Coordinate tmp(NED[0], NED[1], NED[2]);
//        float L = missile_pos.norm() * fabs(tmp * NED1);
//    //    printf("cutting point height = %f\n", L);

//        GPS cutting_point_gps = ECEF2Geoditic(missile_pos + NED1 * L);
//        if(cutting_point_gps.height > 0)
//        {
//            distance[resultIdx] =  RayInfo(-1, -1, 0);
//        }
//        else
//        {
//            int iter = 0;
//            while(fabs(cutting_point_gps.height) > 0.05 && iter < 2000)
//            {
//                iter++;
//                L = L * missile_gps.height / (missile_gps.height - cutting_point_gps.height);
//                cutting_point_gps = ECEF2Geoditic(missile_pos + NED1 * L);
//            }
//        }

//        Coordinate NEDtmp(NED[0], NED[1], NED[2]);
//        NED1 = (missile_pos + NEDtmp * L) / (missile_pos + NEDtmp * L).norm();
//        distance[resultIdx] = RayInfo(L, acosf(fabs(NEDtmp * NED1)), 1);

//        distance[resultIdx] = calcDistanceToOcean(&NED[0], imgPos, missile_pos);

//        if(isInsideSurface(&surface_imgPos[0], 3, imgPos))
//        {
//            distance[resultIdx] = calcDistanceToFace(&NED[0], &surface_imgPos[0], imgPos,
//                    missile_pos, &surface_gps[0]);
//        }
//        else
//        {
//            distance[resultIdx] = calcDistanceToOcean(&NED[0], imgPos, missile_pos);
//        }
    }
}

void Simulator::calcDistance(int offset, ObjStatus * missile_cur)
{
    int num_partialPix = m_batch_size * PIXEL_GRID_SIZE * PIXEL_GRID_SIZE;
    int num_surfaces = m_ship.num_surfaces;


    dim3 blockDim(threadsPerBlock, threadsPerBlock);
    dim3 gridDim(ceil((float)num_partialPix / threadsPerBlock),
                 ceil((float)num_surfaces / threadsPerBlock));

    cudaCalcDistance<<<gridDim, blockDim>>>(m_ray,m_ship.surface_gps,
                                            m_ship.surface_imgPos, missile_cur,
                                            m_Rb2c_cur, m_Ri2b_missile_cur, m_Re2i_missile, m_fov_pixel,
                                            num_surfaces, num_partialPix, offset, m_batch_size, m_width, m_height);
    gpuErrChk(cudaDeviceSynchronize());
}

__global__ void cudaCalcRadiance(const RayInfo * __restrict__ distance,
                                 float * __restrict__ radiance,
                                 int num_surfaces, int batch_size, float ifov,
                                 float solar_coeff, float ocean_coeff,
                                 float sky_coeff, float object_coeff,
                                 float path_coeff, int offset)
{
    int idx = threadIdx.x + IMUL(blockDim.x, blockIdx.x);
    int grid_size = batch_size * PIXEL_GRID_SIZE * PIXEL_GRID_SIZE;
    if(idx < grid_size)
    {
        bool hitObj = false;
        float minDist = 100000000;
        int hitSurfaceIdx;
        for(int i = 0; i < num_surfaces; i++)
        {
            int distIdx = i * grid_size + idx;
            if(distance[distIdx].objIdx == 2)
            {
                hitObj = true;
                if(distance[distIdx].distance < minDist)
                {
                    minDist = distance[distIdx].distance;
                    hitSurfaceIdx = distIdx;
                }
            }
        }
        if(hitObj)
        {
            radiance[idx] = objectRadiance(ifov, minDist, distance[hitSurfaceIdx].angle,
                                           solar_coeff, ocean_coeff,
                                           sky_coeff, object_coeff,
                                           path_coeff);
        }
        else
        {
            if(distance[idx].objIdx == 0)
            {
                radiance[idx] = skyRadiance(ifov, sky_coeff, path_coeff);
            }
            else if(distance[idx].objIdx == 1)
            {
//                printf("Jumping to ocean\n");
                radiance[idx] = oceanRadiance(ifov, distance[idx].distance, distance[idx].angle,
                                              solar_coeff, ocean_coeff,
                                              sky_coeff, object_coeff,
                                              path_coeff);
            }
        }
    }
}

void Simulator::calcRadiance(int offset)
{
    float ifov = m_fov / m_width;
    int numBlock = (m_batch_size * PIXEL_GRID_SIZE * PIXEL_GRID_SIZE + threadsPerBlock * threadsPerBlock - 1) /(threadsPerBlock * threadsPerBlock);
    cudaCalcRadiance<<<numBlock, threadsPerBlock * threadsPerBlock>>>(m_ray, m_partialRadiance, m_ship.num_surfaces, m_batch_size, ifov,
                     m_solar_coeff, m_ocean_coeff, m_sky_coeff, m_object_coeff, m_horizon_coeff, offset);
    gpuErrChk(cudaDeviceSynchronize());

}

__global__ void cudaRenderPartialImg(unsigned char * renderedImg, float * radiance, int fov, int offset, int batch_size)
{
    int idx = threadIdx.x + IMUL(blockDim.x, blockIdx.x);
    if(idx < batch_size)
    {
        float tmp = 0;
#pragma unroll
        for(int i = 0; i < PIXEL_GRID_SIZE * PIXEL_GRID_SIZE; i++)
        {
            tmp = tmp + radiance[idx * PIXEL_GRID_SIZE * PIXEL_GRID_SIZE + i];
        }
        int imgIdx = offset * batch_size + idx;
        renderedImg[imgIdx] = (unsigned char)(tmp / (PIXEL_GRID_SIZE * PIXEL_GRID_SIZE) * 0.6 * powf(10, 8) * 255/ powf(fov, 2));

        if(renderedImg[imgIdx] > 255)
        {
            renderedImg[imgIdx] = 255;
        }
    }
}

void Simulator::renderPartialImg(int offset)
{

    cudaRenderPartialImg<<<m_batch_size, PIXEL_GRID_SIZE * PIXEL_GRID_SIZE>>>(m_renderedImg, m_partialRadiance, m_fov, offset, m_batch_size);
    gpuErrChk(cudaDeviceSynchronize());
}

__global__ void calcTranformationMatricesHelper(ObjStatus* missile_cur,
                                                ObjStatus* missile_prev,
                                                ObjStatus* target_cur,
                                                ObjStatus* target_prev,
                                                SeekerInfo* seeker_cur,
                                                SeekerInfo* seeker_prev,
                                                float* Rb2c_cur, float* Ri2b_missile_cur,
                                                float* Re2i_missile,
                                                float* Ri2b_target,
                                                float* Re2i_target,
                                                float* Rb2c_prev,
                                                float* Ri2b_missile_prev)
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
}

//void Simulator::calcTranformationMatrices()
//{
//    ObjStatus missile_cur = m_missile[m_current_img_id];
//    ObjStatus target_cur = m_target[m_current_img_id];
//    SeekerInfo seeker_cur = m_seeker[m_current_img_id];
//    ObjStatus missile_prev;
//    ObjStatus target_prev;
//    SeekerInfo seeker_prev;
//    CheckPoint(1);
//    if(m_current_img_id > 0)
//    {
//        missile_prev = m_missile[m_current_img_id - 1];
//        target_prev = m_target[m_current_img_id - 1];
//        seeker_prev = m_seeker[m_current_img_id - 1];
//    }
//    else {
//        missile_prev = m_missile[m_current_img_id];
//        target_prev = m_target[m_current_img_id];
//        seeker_prev = m_seeker[m_current_img_id];
//    }
//    getRb2cMatrix(m_Rb2c_cur, RotationAngle(0, seeker_cur.elevation, seeker_cur.azimuth));
//    getRb2cMatrix(m_Rb2c_prev, RotationAngle(0, seeker_prev.elevation, seeker_prev.azimuth));
//    getRi2bMatrix(m_Ri2b_missile_cur, missile_cur.angle);
//    getRi2bMatrix(m_Ri2b_missile_prev, missile_prev.angle);
//    getRi2bMatrix(m_Ri2b_target, target_cur.angle);
//    getRe2iMatrix(m_Re2i_missile, missile_cur.gps);
//    getRe2iMatrix(m_Re2i_target, target_cur.gps);
//}
