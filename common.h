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
    __host__ __device__ GPS(double lat = 0, double lon = 0, double h = 0)
    {
        latitude = lat;
        longtitude = lon;
        height = h;
    }
    __device__ void print(char *str)
    {
        printf("%s: %f - %f - %f\n", str, latitude, longtitude, height);
    }
    __device__ double getHeight()
    {
        return height;
    }

    double latitude;
    double longtitude;
    double height;
};

struct GPS3 {
    GPS x;
    GPS y;
    GPS z;
};

struct double6
{
    double2 x;
    double2 y;
    double2 z;
};

struct RotationAngle
{
    __host__ __device__ RotationAngle(double r = 0, double p = 0, double y = 0)
    {
        roll = r;
        pitch = p;
        yaw = y;
    }
    __device__ void print(char *str)
    {
        printf("%s: %f - %f - %f\n", str, roll, pitch, yaw);
    }

    double roll;
    double pitch;
    double yaw;
};

struct Coordinate
{
    __host__ __device__ Coordinate(double x_ = 0, double y_ = 0, double z_= 0)
    {
        x = x_;
        y = y_;
        z = z_;
    }
    __device__ double norm()
    {
        return sqrt(x * x + y * y + z * z);
    }

    __device__ Coordinate operator -(const Coordinate & coor)
    {
        return Coordinate(x - coor.x, y - coor.y, z - coor.z);
    }

    __device__ Coordinate operator +(const Coordinate & coor)
    {
        return Coordinate(x + coor.x, y + coor.y, z + coor.z);
    }

    __device__ double operator *(const Coordinate & coor)
    {
        return (x * coor.x + y * coor.y + z * coor.z);
    }

    __device__ Coordinate operator *(const double s)
    {
        return Coordinate(x * s, y * s, z * s);
    }

    __device__ Coordinate operator /(const double s)
    {
        if(s==0)
        {
            printf("Invalid division\n");
        }
        return Coordinate(x/s, y/s, z/s);
    }

    double x;
    double y;
    double z;
};

struct ObjStatus
{
    __host__ __device__ ObjStatus(GPS gps_, RotationAngle angle_)
    {
        gps = gps_;
        angle = angle_;
    }

    GPS gps;
    RotationAngle angle;
};

struct RayInfo
{
    double *distance;
    double *angle;
    int *objIdx;    // 0: sky  1: ocean 2: object
};

struct SeekerInfo
{
    __host__ __device__ SeekerInfo(double az, double ele)
    {
        azimuth = az;
        elevation = ele;
    }
    double azimuth;
    double elevation;
};

struct ShipInfo
{
    // surfaces and vertices are given
    int num_surfaces;       // get from configuration of target model (predefined)
    int num_vertices;       // get from configuration of target model (predefined)

    double3 *surfaces;       // num_surfaces * sizeof(double3) bytes allocated
    // vertice coordinate with reference to the target
    double3 *vertices;       // num_vertices * sizeof(double3) bytes allocated
    // gps and imgPos is calculated in simulation time
    GPS *gps;               // gps data of each vertex
    double2 *imgPos;          // pixel position of each vertex when projecting onto the image

    GPS3 * surface_gps;
    double6 * surface_imgPos;

    double length;      // length of the ship
    double height;      // height of the ship
};
#endif
