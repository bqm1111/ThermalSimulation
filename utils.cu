#include "utils.h"
__host__ __device__ void getRi2bMatrix(float* matrix, RotationAngle angle)
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

__host__ __device__ void getRb2cMatrix(float* matrix, RotationAngle angle)
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

__host__ __device__ void getRe2iMatrix(float* matrix, GPS gps)
{
    float lambda = gps.latitude / 180.0 * M_PI;
    float phi = gps.longtitude / 180.0 * M_PI;

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
    float sin_q = sin(q);
    float cos_q = cos(q);
    float sin_q_3 = sin_q *sin_q * sin_q;
    float cos_q_3 = cos_q * cos_q * cos_q;
    float phi = atan2(z + eps * b * sin_q_3, p - e_sq * a * cos_q_3);
    float lambda = atan2(y, x);
    float v = a / std::sqrt(1.0 - e_sq * sin(phi) * sin(phi));
    result.height = p / cos(phi) - v;
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

    float N = a / std::sqrt(1 - e_sq * sin(lambda) * sin(lambda));
    result.x = (gps.height + N) * cos(lambda) * cos(phi);
    result.y = (gps.height + N) * cos(lambda) * sin(phi);
    result.z = (gps.height + (1 - e_sq) * N) * sin(lambda);
    return result;
}