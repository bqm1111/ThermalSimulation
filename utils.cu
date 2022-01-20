#include "utils.h"

__host__ __device__ float deg2rad(float deg)
{
    return deg * M_PI / 180.0;
}

__host__ __device__ float rad2deg(float rad)
{
    return rad * 180.0 / M_PI;
}

__host__ __device__ float calcGainTransmittance(float distance)
{
    return expf(-0.26 * distance / 1000);
}

__host__ __device__ float norm(float2 input)
{
    return sqrtf(input.x * input.x + input.y * input.y);
}

__host__ __device__ float inner_product(float2 a, float2 b)
{
    return (a.x * b.x + a.y * b.y);
}

__host__ __device__ Coordinate cross_product(Coordinate a, Coordinate b)
{
    return Coordinate(a.y * b.z - a.z * b.y,
                      a.z * b.x - a.x * b.z,
                      a.x * b.y - a.y * b.x);
}

__host__ __device__ Coordinate vectorCalculator(GPS src1, GPS src2)
{
    Coordinate coor1 = Geoditic2ECEF(src1);
    Coordinate coor2 = Geoditic2ECEF(src2);
    return (coor2 - coor1) / (coor2 - coor1).norm();
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

__host__ __device__ void pinv3x2(float *dst, float *src)
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
// Matrix Ldoni in matlab code
__host__ __device__ void getRb2cMatrix(float* matrix, RotationAngle angle)
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

__host__ __device__ float objectRadiance(float ifov, float distance, float beta,
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
__host__ __device__ float oceanRadiance(float ifov, float distance, float beta,
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
__host__ __device__ float skyRadiance(float ifov, float sky_coeff, float path_coeff)
{
    float ifov_rad_sqr = powf(deg2rad(ifov), 2);

    return (sky_coeff / M_PI * ifov_rad_sqr + path_coeff /M_PI * ifov_rad_sqr);
}
