#ifndef SIMULATOR_H
#define SIMULATOR_H
#include "define.h"
#include "common.h"
#include "utils.h"
#include "vector_types.h"
#include "opencv2/opencv.hpp"
class Simulator
{
public:
    Simulator(int fps=100, int duration = 153);
    ~Simulator(){}

    ObjStatus * m_missile;          // (fps * duration * sizeof(ObjStatus) bytes allocated
    ObjStatus * m_target;           // (fps * duration * sizeof(ObjStatus) bytes allocated
    SeekerInfo * m_seeker;          // (fps * duration * sizeof(SeekerInfo) bytes allocated
    ShipInfo m_ship;

private:
    int m_fps;
    int m_duration;
    void convertToImage(ShipInfo &ship, ObjStatus &missile, ObjStatus &target, SeekerInfo &seeker);
    uint2 imageModel(ObjStatus missile, ObjStatus target, SeekerInfo seeker);
    bool isShipAppear();
    float m_ocean_coeff;           // radiance coefficient of the ocean
    float m_horizon_coeff;         // radiance coefficient of horizone line
    float m_sky_coeff;             // radiance coefficient of the sky
    float m_solar_coeff;           // radiance coefficient of the sun
    float m_object_coeff;          // radiance coefficient of the object (in this case, ship)
    int m_current_img_id;       // id of current image being simulated
};

#endif
