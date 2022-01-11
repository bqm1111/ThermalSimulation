#include "Simulator.h"

Simulator::Simulator(int fps, int duration):
    m_fps(fps), m_duration(duration)
{
    m_ocean_coeff= 4.5468;
    m_horizon_coeff  = 40.0641;
    m_sky_coeff   = 1.0555;
    m_solar_coeff = 27.7758;
    m_object_coeff = 8.5245;
}

//!
//! \brief Simulator::convertToImage
//! \param ship
//! \param missile
//! \param target
//! \param seeker
//!
void Simulator::convertToImage(ShipInfo &ship, ObjStatus &missile, ObjStatus &target, SeekerInfo &seeker)
{
    Coordinate target_pos = Geoditic2ECEF(target.gps);
    cv::Mat Ri2b0(3, 3, CV_32FC1);
    cv::Mat M(3, 3, CV_32FC1);
    getRi2bMatrix((float*)Ri2b0.data, target.angle);
    getMMatrix((float*)M.data, target.gps);

    for(int i = 0; i < ship.num_vertices; i++)
    {
        cv::Mat vertex(3, 1, CV_32FC1, (float*)(ship.vertices + i));
        cv::Mat NED(3, 1, CV_32FC1);
        NED = M.t() * Ri2b0 * vertex;
        Coordinate temp;
        temp.x = target_pos.x + NED.at<float>(0,0);
        temp.y = target_pos.y + NED.at<float>(1,0);
        temp.z = target_pos.z + NED.at<float>(2,0);
        ship.gps[i] = ECEF2Geoditic(temp);
        ship.imgPos[i] = imageModel(missile, target, seeker);
    }
}

uint2 Simulator::imageModel(ObjStatus missile, ObjStatus target, SeekerInfo seeker)
{
    uint2 result;
    Coordinate target_pos = Geoditic2ECEF(target.gps);
    Coordinate missile_pos = Geoditic2ECEF(missile.gps);
    float distance = std::sqrt((missile_pos.x - target_pos.x) * (missile_pos.x - target_pos.x) +
                               (missile_pos.y - target_pos.y) * (missile_pos.y - target_pos.y) +
                               (missile_pos.z - target_pos.z) * (missile_pos.z - target_pos.z));

    cv::Mat NED(3, 1, CV_32FC1);
    NED.at<float>(0, 0) = (missile_pos.x - target_pos.x) / distance;
    NED.at<float>(1, 0) = (missile_pos.y - target_pos.y) / distance;
    NED.at<float>(2, 0) = (missile_pos.z - target_pos.z) / distance;
    cv::Mat Ldoni(3, 3, CV_32FC1);
    cv::Mat Ri2b(3, 3, CV_32FC1);
    cv::Mat M(3,3, CV_32FC1);
    getLdoniMatrix((float*)Ldoni.data, RotationAngle(0, seeker.elevation, seeker.azimuth));
    getRi2bMatrix((float*)Ri2b.data, missile.angle);
    getMMatrix((float*)M.data, missile.gps);

    cv::Mat Ldonic(3, 1, CV_32FC1);
    Ldonic = Ldoni.t() * Ri2b.t() * M * NED;

    if(Ldonic.at<float>(2, 0) < 0)
        Ldonic.at<float>(2, 0) = - Ldonic.at<float>(2, 0);

    float f = 320 / tan(deg2rad(seeker.fov / 2));
    result.x = Ldonic.at<float>(0, 0) / Ldonic.at<float>(2, 0) * f;
    result.y = Ldonic.at<float>(1, 0) / Ldonic.at<float>(2, 0) * f;
    return  result;
}

bool Simulator::isShipAppear()
{

}



//azimuth = seeker.az;
//elevator = seeker.el;
//fov = seeker.fov;
//f = 320/tan(deg2rad(fov/2));
//[u0,w0]  = imageModel(lat,lon,h,lat0,lon0,h0,roll,pitch,yaw,azimuth,elevator,fov);
//[ x0, y0, z0 ] = Geoditic2ECEF( lat0, lon0, h0 );
//[ x, y, z ] = Geoditic2ECEF( lat, lon, h );
//distance0 = norm([x-x0,y-y0,z-z0]);

//L = (max(Data.vertices(:,1))-min(Data.vertices(:,1)))/2;
//if round(L) < L
//    L = round(L) + 1;
//else
//    L = round(L);
//end

//height = (max(Data.vertices(:,2))-min(Data.vertices(:,2)))/2;
//if round(height) < height
//    height = round(height) + 1;
//else
//    height = round(height);
//end

//delta_az = asin(u0/norm([u0 320/tan(deg2rad(fov/2))]));
//delta_el = asin(w0/norm([u0 w0 320/tan(deg2rad(fov/2))]));

//delta_u0 = atan(L/distance0)*320/tan(deg2rad(fov/2));
//delta_w0 = atan(height/distance0)*320/tan(deg2rad(fov/2));

//ImageMatrix = zeros(480,640);
//if (abs(delta_az) < deg2rad(fov/2)+atan(15/distance0)) && (abs(delta_el) < deg2rad(3/4*fov/2)+atan(6/distance0))
//    [Data] = convert2image (Data,lat,lon,h,lat0,lon0,h0,roll,pitch,yaw,azimuth,elevator,roll0,pitch0,yaw0,fov);
//end
