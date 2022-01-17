#include "Simulator.h"

Simulator::Simulator(int fps, int duration, int batch_size):
    m_fps(fps), m_duration(duration), m_batch_size(batch_size)
{
    //!!
    //! Common Parameter
    //!

    // camera parameter
    m_width = 640;
    m_height = 480;
    m_fov = 3.7;
    m_fov_pixel = m_width / 2 / tan(deg2rad(m_fov / 2));
    m_current_img_id = 0;
    // thermal parameter
    m_ocean_coeff= 4.5468;
    m_horizon_coeff  = 40.0641;
    m_sky_coeff   = 1.0555;
    m_solar_coeff = 27.7758;
    m_object_coeff = 8.5245;
}

Simulator::~Simulator()
{
    gpuErrChk(cudaFree(m_distanceToFace));
    gpuErrChk(cudaFree(m_InorOut));
    gpuErrChk(cudaFree(m_radiance));
    gpuErrChk(cudaFree(m_partialRadiance));
    gpuErrChk(cudaFree(m_radiance));
    gpuErrChk(cudaFree(m_renderedImg));

    // free mem transformation matrices
    gpuErrChk(cudaFree(m_Rb2c_cur));
    gpuErrChk(cudaFree(m_Rb2c_prev));
    gpuErrChk(cudaFree(m_Ri2b_missile_cur));
    gpuErrChk(cudaFree(m_Ri2b_missile_prev));
    gpuErrChk(cudaFree(m_Ri2b_target));
    gpuErrChk(cudaFree(m_Re2i_missile));
    gpuErrChk(cudaFree(m_Re2i_target));


}

void Simulator::loadData()
{

}

void Simulator::init()
{
    int grid_size = m_ship.num_surfaces * m_batch_size * PIXEL_GRID_SIZE * PIXEL_GRID_SIZE;

    gpuErrChk(cudaMalloc((void**)&m_distanceToFace, grid_size * sizeof(float)));
    gpuErrChk(cudaMalloc((void**)&m_InorOut, grid_size * sizeof(bool)));
    gpuErrChk(cudaMalloc((void**)&m_partialRadiance, grid_size * sizeof(float)));
    gpuErrChk(cudaMalloc((void**)&m_radiance, m_width * m_height * sizeof(float)));
    gpuErrChk(cudaMalloc((void**)&m_renderedImg, m_width * m_height * sizeof(unsigned char)));

    // Allocate memory for transformation matrices
    gpuErrChk(cudaMalloc((void**)&m_Rb2c_cur, 9 * sizeof(float)));
    gpuErrChk(cudaMalloc((void**)&m_Rb2c_prev, 9 * sizeof(float)));
    gpuErrChk(cudaMalloc((void**)&m_Ri2b_missile_cur, 9 * sizeof(float)));
    gpuErrChk(cudaMalloc((void**)&m_Ri2b_missile_prev, 9 * sizeof(float)));
    gpuErrChk(cudaMalloc((void**)&m_Ri2b_target, 9 * sizeof(float)));
    gpuErrChk(cudaMalloc((void**)&m_Re2i_missile, 9 * sizeof(float)));
    gpuErrChk(cudaMalloc((void**)&m_Re2i_target, 9 * sizeof(float)));
}
//!
//! \brief Simulator::convertToImage
//! \param ship
//! \param missile
//! \param target
//! \param seeker
//!
void Simulator::convertToImage(ShipInfo &ship, ObjStatus &missile, ObjStatus &target)
{
    Coordinate target_pos = Geoditic2ECEF(target.gps);
    cv::Mat Ri2b_target(3, 3, CV_32FC1, m_Ri2b_target);
    cv::Mat Re2i_target(3, 3, CV_32FC1, m_Re2i_target);

    for(int i = 0; i < ship.num_vertices; i++)
    {
        cv::Mat vertex(3, 1, CV_32FC1, (float*)(ship.vertices + i));
        cv::Mat NED(3, 1, CV_32FC1);
        NED = Re2i_target.t() * Ri2b_target * vertex;
        Coordinate temp;
        temp.x = target_pos.x + NED.at<float>(0,0);
        temp.y = target_pos.y + NED.at<float>(1,0);
        temp.z = target_pos.z + NED.at<float>(2,0);
        ship.gps[i] = ECEF2Geoditic(temp);
        ship.imgPos[i] = imageModel(missile, ship.gps[i]);
    }
}

float2 Simulator::imageModel(ObjStatus missile, GPS target_gps)
{
    float2 result;
    Coordinate target_pos = Geoditic2ECEF(target_gps);
    Coordinate missile_pos = Geoditic2ECEF(missile.gps);
    float distance = std::sqrt((missile_pos.x - target_pos.x) * (missile_pos.x - target_pos.x) +
                               (missile_pos.y - target_pos.y) * (missile_pos.y - target_pos.y) +
                               (missile_pos.z - target_pos.z) * (missile_pos.z - target_pos.z));

    cv::Mat NED(3, 1, CV_32FC1);
    NED.at<float>(0, 0) = (missile_pos.x - target_pos.x) / distance;
    NED.at<float>(1, 0) = (missile_pos.y - target_pos.y) / distance;
    NED.at<float>(2, 0) = (missile_pos.z - target_pos.z) / distance;
    cv::Mat Rb2c_cur(3, 3, CV_32FC1, m_Rb2c_cur);
    cv::Mat Ri2b_missile_cur(3, 3, CV_32FC1, m_Ri2b_missile_cur);
    cv::Mat Re2i_missile(3,3, CV_32FC1, m_Re2i_missile);


    cv::Mat Ldonic(3, 1, CV_32FC1);
    Ldonic = Rb2c_cur.t() * Ri2b_missile_cur.t() * Re2i_missile * NED;

    if(Ldonic.at<float>(2, 0) < 0)
        Ldonic.at<float>(2, 0) = - Ldonic.at<float>(2, 0);

    result.x = Ldonic.at<float>(0, 0) / Ldonic.at<float>(2, 0) * m_fov_pixel;
    result.y = Ldonic.at<float>(1, 0) / Ldonic.at<float>(2, 0) * m_fov_pixel;
    return  result;
}

bool Simulator::isShipAppear()
{
    ObjStatus missile = m_missile[m_current_img_id];
    ObjStatus target = m_target[m_current_img_id];
    float2 target_imgPos = imageModel(missile, target.gps);
    Coordinate target_pos = Geoditic2ECEF(target.gps);
    Coordinate missile_pos = Geoditic2ECEF(missile.gps);

    float distance = (target_pos - missile_pos).norm();

    float delta_az = asin(target_imgPos.x / std::sqrt(target_imgPos.x * target_imgPos.x + m_fov_pixel * m_fov_pixel));
    float delta_el = asin(target_imgPos.y / Coordinate(target_imgPos.x, target_imgPos.y, m_fov_pixel).norm());
    //    float delta_u0 = atan(m_ship.length / distance) * f;
    //    float delta_w0 = atan(m_ship.height / distance) * f;

    return (std::fabs(delta_az) < deg2rad(m_fov / 2) + atan(15 / distance)) &&
            (std::fabs(delta_el) < deg2rad(3 / 4 * m_fov /2 ) + atan(6 / distance));
}

void Simulator::calcTranformationMatrices()
{
    ObjStatus missile_cur = m_missile[m_current_img_id];
    ObjStatus target_cur = m_target[m_current_img_id];
    SeekerInfo seeker_cur = m_seeker[m_current_img_id];
    ObjStatus missile_prev;
    ObjStatus target_prev;
    SeekerInfo seeker_prev;

    if(m_current_img_id > 0)
    {
        missile_prev = m_missile[m_current_img_id - 1];
        target_prev = m_target[m_current_img_id - 1];
        seeker_prev = m_seeker[m_current_img_id - 1];
    }
    cv::Mat temp(3, 3, CV_32FC1);
    getRb2cMatrix((float*)temp.data, RotationAngle(0, seeker_cur.elevation, seeker_cur.azimuth));
    gpuErrChk(cudaMemcpy(m_Rb2c_cur, (float*)temp.data, 9 * sizeof(float), cudaMemcpyHostToDevice));

    getRb2cMatrix((float*)temp.data, RotationAngle(0, seeker_prev.elevation, seeker_prev.azimuth));
    gpuErrChk(cudaMemcpy(m_Rb2c_prev, (float*)temp.data, 9 * sizeof(float), cudaMemcpyHostToDevice));

    getRi2bMatrix((float*)temp.data, missile_cur.angle);
    gpuErrChk(cudaMemcpy(m_Ri2b_missile_cur, (float*)temp.data, 9 * sizeof(float), cudaMemcpyHostToDevice));

    getRi2bMatrix((float*)temp.data, missile_prev.angle);
    gpuErrChk(cudaMemcpy(m_Ri2b_missile_prev, (float*)temp.data, 9 * sizeof(float), cudaMemcpyHostToDevice));

    getRi2bMatrix((float*)temp.data, target_cur.angle);
    gpuErrChk(cudaMemcpy(m_Ri2b_target, (float*)temp.data, 9 * sizeof(float), cudaMemcpyHostToDevice));

    getRe2iMatrix((float*)temp.data, missile_cur.gps);
    gpuErrChk(cudaMemcpy(m_Re2i_missile, (float*)temp.data, 9 * sizeof(float), cudaMemcpyHostToDevice));

    getRe2iMatrix((float*)temp.data, target_cur.gps);
    gpuErrChk(cudaMemcpy(m_Re2i_target, (float*)temp.data, 9 * sizeof(float), cudaMemcpyHostToDevice));
}

RayInfo Simulator::calcDistance(ObjStatus missile, uint2 particlePix)
{
    cv::Mat Ldonic(3, 1, CV_32FC1);
    float temp = Coordinate(particlePix.x, particlePix.y, m_fov_pixel).norm();
    Ldonic.at<float>(0,0) = particlePix.x / temp;
    Ldonic.at<float>(1, 0) = particlePix.y / temp;
    Ldonic.at<float>(2, 0) = m_fov_pixel / temp;
    cv::Mat NED(3, 1, CV_32FC1);
    cv::Mat Re2i_missile(3, 1, CV_32FC1, m_Re2i_missile);
    cv::Mat Ri2b_missile(3, 1, CV_32FC1, m_Ri2b_missile_cur);
    cv::Mat Rb2c_cur(3, 1, CV_32FC1, m_Rb2c_cur);

    NED = Re2i_missile.t() * Ri2b_missile * Rb2c_cur * Ldonic;
    Coordinate missile_pos = Geoditic2ECEF(missile.gps);
    cv::Mat NED1(3, 1, CV_32FC1);
    NED1.at<float>(0, 0) = missile_pos.x / missile_pos.norm();
    NED1.at<float>(1, 0) = missile_pos.y / missile_pos.norm();
    NED1.at<float>(2, 0) = missile_pos.z / missile_pos.norm();

    cv::Mat res = NED.t()* NED1;
    float L = missile_pos.norm() * std::fabs(res.at<float>(0, 0));

    GPS cutting_point_gps = ECEF2Geoditic(Coordinate(missile_pos.x + L * NED1.at<float>(0, 0),
                                                     missile_pos.y + L * NED1.at<float>(1, 0),
                                                     missile_pos.z + L * NED1.at<float>(2, 0)));

    RayInfo result;
    if(cutting_point_gps.height > 0)
    {
        result.distance = -1;
        result.angle = -1;
    }
    else
    {
        int iter = 0;
        while(std::fabs(cutting_point_gps.height > 0.05 && iter < 2000))
        {
            iter++;
            L = L * missile.gps.height / (missile.gps.height - cutting_point_gps.height);
            cutting_point_gps = ECEF2Geoditic(Coordinate(missile_pos.x + L * NED1.at<float>(0, 0),
                                                                 missile_pos.y + L * NED1.at<float>(1, 0),
                                                                 missile_pos.z + L * NED1.at<float>(2, 0)));

        }
    }
    return result;
}

void Simulator::run()
{
    for(int i = 0; i < m_fps * m_duration; i++)
    {
        for(int idx = 0; idx < m_width * m_height / m_batch_size; idx++)
        {

        }
        m_current_img_id++;
    }
}
