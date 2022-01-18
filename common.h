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
    float latitude;
    float longtitude;
    float height;
};
struct RotationAngle
{
    RotationAngle(float r = 0, float p = 0, float y = 0)
    {
        roll = r;
        pitch = p;
        yaw = y;
    }
    float roll;
    float pitch;
    float yaw;
};

struct Coordinate
{
    Coordinate(float x_ = 0, float y_ = 0, float z_= 0)
    {
        x = x_;
        y = y_;
        z = z_;
    }
    float norm()
    {
        return std::sqrt(x * x + y * y + z * z);
    }
    Coordinate operator -(const Coordinate & coor)
    {
        return Coordinate(x - coor.x, y - coor.y, z - coor.z);
    }
    Coordinate operator +(const Coordinate & coor)
    {
        return Coordinate(x + coor.x, y + coor.y, z + coor.z);
    }

    float x;
    float y;
    float z;
};

struct ObjStatus
{
    GPS gps;
    RotationAngle angle;
};

struct RayInfo
{
    float distance;
    float angle;
};

struct SeekerInfo
{
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

    float length;      // length of the ship
    float height;      // height of the ship
};

#endif
