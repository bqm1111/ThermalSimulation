#ifndef COMMON_H
#define COMMON_H
#include <iostream>
#include <vector_types.h>
#include <math.h>
enum DataType
{
    UCHAR,
    INT,
    FLOAT,
    FLOAT3,
    DOUBLE
};

struct GPS {
    __device__ GPS(float lat = 0, float lon = 0, float h = 0)
    {
        latitude = lat;
        longtitude = lon;
        height = h;
    }
    __device__ void print(char *str)
    {
        printf("%s: %f - %f - %f\n", str, latitude, longtitude, height);
    }

    float latitude;
    float longtitude;
    float height;
};

struct GPS3 {
    GPS x;
    GPS y;
    GPS z;
};

struct float6
{
    float2 x;
    float2 y;
    float2 z;
};

struct RotationAngle
{
    __device__ RotationAngle(float r = 0, float p = 0, float y = 0)
    {
        roll = r;
        pitch = p;
        yaw = y;
    }
    __device__ void print(char *str)
    {
        printf("%s: %f - %f - %f\n", str, roll, pitch, yaw);
    }

    float roll;
    float pitch;
    float yaw;
};

struct Coordinate
{
    __device__ Coordinate(float x_ = 0, float y_ = 0, float z_= 0)
    {
        x = x_;
        y = y_;
        z = z_;
    }
    __device__ float norm()
    {
        return sqrtf(x * x + y * y + z * z);
    }

    __device__ Coordinate operator -(const Coordinate & coor)
    {
        return Coordinate(x - coor.x, y - coor.y, z - coor.z);
    }

    __device__ Coordinate operator +(const Coordinate & coor)
    {
        return Coordinate(x + coor.x, y + coor.y, z + coor.z);
    }

    __device__ float operator *(const Coordinate & coor)
    {
        return (x * coor.x + y * coor.y + z * coor.z);
    }

    __device__ Coordinate operator *(const float s)
    {
        return Coordinate(x * s, y * s, z * s);
    }

    __device__ Coordinate operator /(const float s)
    {
        if(s==0)
        {
            printf("Invalid division\n");
        }
        return Coordinate(x/s, y/s, z/s);
    }

    float x;
    float y;
    float z;
};

struct ObjStatus
{
    __device__ ObjStatus(GPS gps_, RotationAngle angle_)
    {
        gps = gps_;
        angle = angle_;
    }

    GPS gps;
    RotationAngle angle;
};

struct RayInfo
{
    float *distance;
    float *angle;
    int *objIdx;    // 0: sky  1: ocean 2: object
};

struct SeekerInfo
{
    __device__ SeekerInfo(float az, float ele)
    {
        azimuth = az;
        elevation = ele;
    }
    float azimuth;
    float elevation;
};

struct ShipInfo
{
    // surfaces and vertices are given
    int num_surfaces;       // get from configuration of target model (predefined)
    int num_vertices;       // get from configuration of target model (predefined)

    float3 *surfaces;       // num_surfaces * sizeof(float3) bytes allocated
    // vertice coordinate with reference to the target
    float3 *vertices;       // num_vertices * sizeof(float3) bytes allocated
    // gps and imgPos is calculated in simulation time
    GPS *gps;               // gps data of each vertex
    float2 *imgPos;          // pixel position of each vertex when projecting onto the image

    GPS3 * surface_gps;
    float6 * surface_imgPos;

    float length;      // length of the ship
    float height;      // height of the ship
};
#endif
