#include "utils.h"

__host__ __device__ float deg2rad(float deg)
{
    return deg * M_PI / 180.0;
}

__host__ __device__ float rad2deg(float rad)
{
    return rad * 180.0 / M_PI;
}

__host__ __device__ void mul3x3ToVec3x1(float *dst, float *mat, float *vec)
{
#pragma unroll
    for(int y = 0; y < 3; y++)
    {
        for(int x = 0; x < 1; x++)
        {
            int idx = y * 3 + x;
            dst[idx] = 0;
            for(int k = 0; k < 3; k++)
            {
                dst[idx] = dst[idx] + mat[y * 3 + k] * vec[k * 3 + x];
            }
        }
    }
}

__host__ __device__ void mul3x3(float *dst, float *src1, float *src2)
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

__host__ __device__ void mul3x3TransposeFirst(float *dst, float *src1, float *src2)
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

__host__ __device__ void mul3x3TransposeBoth(float *dst, float *src1, float *src2)
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


__host__ __device__ void getRi2bMatrix(float* matrix, RotationAngle angle)
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

__host__ __device__ void getRe2iMatrix(float* matrix, GPS gps)
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

__host__ __device__ GPS ECEF2Geoditic(Coordinate pos)
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
    float p = std::sqrt(x * x + y * y);
    float q = atan2(z * a, p * b);
    float sinf_q = sinf(q);
    float cosf_q = cosf(q);
    float sinf_q_3 = sinf_q *sinf_q * sinf_q;
    float cosf_q_3 = cosf_q * cosf_q * cosf_q;
    float phi = atan2(z + eps * b * sinf_q_3, p - e_sq * a * cosf_q_3);
    float lambda = atan2(y, x);
    float v = a / std::sqrt(1.0 - e_sq * sinf(phi) * sinf(phi));
    result.height = p / cosf(phi) - v;
    result.latitude = phi / M_PI * 180;
    result.longtitude = lambda / M_PI * 180;

    return result;
}

__host__ __device__ Coordinate Geoditic2ECEF(GPS gps)
{
    Coordinate result;

    float a = 6378137;
    float b = 6356752;
    float f = (a - b) / a;
    float e_sq = f * (2 - f);

    float lambda = gps.latitude / 180 * M_PI;
    float phi = gps.longtitude / 180 * M_PI;

    float N = a / std::sqrt(1 - e_sq * sinf(lambda) * sinf(lambda));
    result.x = (gps.height + N) * cosf(lambda) * cosf(phi);
    result.y = (gps.height + N) * cosf(lambda) * sinf(phi);
    result.z = (gps.height + (1 - e_sq) * N) * sinf(lambda);
    return result;
}
