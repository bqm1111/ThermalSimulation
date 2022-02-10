#ifndef SIMULATOR_H
#define SIMULATOR_H
#include "define.h"
#include "common.h"
#include "utils.h"
#include "vector_types.h"
#include "opencv2/opencv.hpp"

#define PIXEL_GRID_SIZE     4

class Simulator
{
public:
    Simulator(int fps=30, int duration = 150, int batch_size = 256);
    ~Simulator();

    ObjStatus * m_missile;          // (fps * duration * sizeof(ObjStatus) bytes allocated
    ObjStatus * m_target;           // (fps * duration * sizeof(ObjStatus) bytes allocated
    SeekerInfo * m_seeker;          // (fps * duration * sizeof(SeekerInfo) bytes allocated
    ShipInfo m_ship;
    int m_current_img_id;       // id of current image being simulated
    void init();
    void loadData();
    void run(int resume);
    void test();
    void testFunc();
    void testFuck();

private:
    int m_fps;
    int m_duration;
    int m_totalFrame;
    void convertToImage(ObjStatus* missile_cur, ObjStatus *target_cur);
    void calcSurfaceData();
    void calcTranformationMatrices(ObjStatus* missile_cur, ObjStatus* missile_prev,
                                   ObjStatus *target_cur, ObjStatus* target_prev,
                                   SeekerInfo* seeker_cur, SeekerInfo* seeker_prev);
    void calcDistance(int offset, ObjStatus * missile_cur);
    void calcRadiance(int offset);
    void renderPartialImg(int offset);
    bool isShipAppear();
    void renderSingleImg();
    void testCalcShipData(ObjStatus * missile, ObjStatus * target);
    void testStruct();
    void mask(int offset);

    // thermal parameter
    double m_ocean_coeff;           // radiance coefficient of the ocean
    double m_horizon_coeff;         // radiance coefficient of horizone line
    double m_sky_coeff;             // radiance coefficient of the sky
    double m_solar_coeff;           // radiance coefficient of the sun
    double m_object_coeff;          // radiance coefficient of the object (in this case, ship)

    // camera parameter
    int m_width;
    int m_height;
    double m_fov;
    double m_fov_pixel;

    // transformation matrices between coordinate systems
    double *m_Rb2c_cur;      // Matrix transform between body to camera
    double *m_Rb2c_prev;       // Matrix transform between body to camera in the previous step
    double *m_Ri2b_missile_cur;      // Matrix transform between inertial to body(missile)
    double *m_Ri2b_missile_prev;     // Matrix transform between inertial to body(missile) in the previous step
    double *m_Ri2b_target;           // Matrix transform between inertial to body(target)
    double *m_Re2i_missile;      // Matrix transform between earth to inertial(missile)
    double *m_Re2i_target;       // Matrix transform between earth to inertial(missile)

    // Core data for rendering image
    RayInfo m_ray;
    double * m_partialRadiance;
    unsigned char * m_renderedImg;
    unsigned char * m_maskImg;
    int m_batch_size;

    // data file source
    std::string m_missile_data_filename;
    std::string m_target_data_filename;
    std::string m_seeker_data_filename;
    std::string m_ship_surfaces_data_filename;
    std::string m_ship_vertices_data_filename;
};

#endif
