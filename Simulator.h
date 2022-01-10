#ifndef SIMULATOR_H
#define SIMULATOR_H
#include "define.h"
#include "common.h"
#include "utils.h"
class Simulator
{
public:
    Simulator(int fps=100, int duration = 153);
    ~Simulator(){}

    float3 * m_surface;
    ObjStatus * m_missile;
    ObjStatus * m_target;
    SeekerInfo * m_seeker;

private:
    int m_fps;
    int m_duration;
    void convertToImage(ShipInfo &ship, ObjStatus &missile, ObjStatus &target, SeekerInfo &seeker);
    uint2 imageModel(ObjStatus missile, ObjStatus target, SeekerInfo seeke);
};

#endif
