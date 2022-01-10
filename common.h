#ifndef COMMON_H
#define COMMON_H
#include <vector_types.h>
struct GPS {
    float latitude;
    float longtitude;
    float height;
};
struct RotationAngle
{
    float roll;
    float pitch;
    float yaw;
};
struct Coordinate
{
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
    float fov;
    float azimuth;
    float elevation;
};

struct ShipInfo
{
    float3 *surfaces;
    float3 *vertices;
    GPS *gps;
    uint2 *imgPos;
};

#endif
