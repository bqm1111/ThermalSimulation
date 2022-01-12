#ifndef SIMULATOR_H
#define SIMULATOR_H
#include "define.h"
#include "common.h"
#include "utils.h"
#include "vector_types.h"
#include "opencv2/opencv.hpp"

#define PIXEL_GRID_SIZE     16

class Simulator
{
public:
    Simulator(int fps=100, int duration = 153, int batch_size = 32);
    ~Simulator();

    ObjStatus * m_missile;          // (fps * duration * sizeof(ObjStatus) bytes allocated
    ObjStatus * m_target;           // (fps * duration * sizeof(ObjStatus) bytes allocated
    SeekerInfo * m_seeker;          // (fps * duration * sizeof(SeekerInfo) bytes allocated
    ShipInfo m_ship;
    int m_current_img_id;       // id of current image being simulated
    void run();
    void init();
    void loadData();

private:
    int m_fps;
    int m_duration;
    void convertToImage(ShipInfo &ship, ObjStatus &missile, ObjStatus &target);
    uint2 imageModel(ObjStatus missile, GPS target_gps);
    void calcTranformationMatrices();
    RayInfo calcDistance(ObjStatus missile, uint2 particlePix);
    bool isShipAppear();

    // thermal parameter
    float m_ocean_coeff;           // radiance coefficient of the ocean
    float m_horizon_coeff;         // radiance coefficient of horizone line
    float m_sky_coeff;             // radiance coefficient of the sky
    float m_solar_coeff;           // radiance coefficient of the sun
    float m_object_coeff;          // radiance coefficient of the object (in this case, ship)

    // camera parameter
    int m_width;
    int m_height;
    float m_fov;
    float m_fov_pixel;

    // transformation matrices between coordinate systems
    float *m_Rb2c_cur;      // Matrix transform between body to camera
    float *m_Rb2c_prev;       // Matrix transform between body to camera in the previous step
    float *m_Ri2b_missile_cur;      // Matrix transform between inertial to body(missile)
    float *m_Ri2b_missile_prev;     // Matrix transform between inertial to body(missile) in the previous step
    float *m_Ri2b_target;           // Matrix transform between inertial to body(target)
    float *m_Re2i_missile;      // Matrix transform between earth to inertial(missile)
    float *m_Re2i_target;       // Matrix transform between earth to inertial(missile)

    // Core data for rendering image
    float * m_distanceToFace;
    bool * m_InorOut;
    float * m_partialRadiance;
    float * m_radiance;
    unsigned char * m_renderedImg;
    int m_batch_size;
};

#endif
