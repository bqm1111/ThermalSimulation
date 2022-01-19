#include "Simulator.h"


__device__ bool isInsideObject(float2* data, int length_data, float2 imgPos)
{
    float sum = 0;
    for(int i = 0; i < length_data; i++)
    {
        float2 vector1;
        float2 vector2;

        vector1.x = data[i].x - imgPos.x;
        vector1.y = data[i].y - imgPos.y;
        if(i == length_data - 1)
        {
            vector2.x = data[0].x - imgPos.x;
            vector2.y = data[0].y - imgPos.y;
        }
        else
        {
            vector2.x = data[i + 1].x - imgPos.x;
            vector2.y = data[i + 1].y - imgPos.y;
        }

        float cos_phi = inner_product(vector1, vector2) / norm(vector1) / norm(vector2);
        sum = sum + acosf(cos_phi);
    }

    if(sum > 2 * M_PI - 0.001)
    {
        return true;
    }
    else
    {
        return false;
    }
}

__device__ float2 cudaImageModel(ObjStatus missile, GPS target_gps,
                                 float * Rb2c_cur, float *Ri2b_missile_cur,
                                 float * Re2i_missile, float fov_pixel)
{
    float2 result;
    Coordinate target_pos = Geoditic2ECEF(target_gps);
    Coordinate missile_pos = Geoditic2ECEF(missile.gps);
    float distance = sqrtf((missile_pos.x - target_pos.x) * (missile_pos.x - target_pos.x) +
                           (missile_pos.y - target_pos.y) * (missile_pos.y - target_pos.y) +
                           (missile_pos.z - target_pos.z) * (missile_pos.z - target_pos.z));

    float NED[3];
    NED[0] = (missile_pos.x - target_pos.x) / distance;
    NED[1] = (missile_pos.y - target_pos.y) / distance;
    NED[2] = (missile_pos.z - target_pos.z) / distance;

    float Ldonic[3];
    float temp1[9];
    float temp2[9];
    mul3x3TransposeBoth(&temp1[0], Rb2c_cur, Ri2b_missile_cur);
    mul3x3(&temp2[0], &temp1[0], Re2i_missile);
    mul3x3ToVec3x1(&Ldonic[0], &temp2[0], &NED[0]);
    if(Ldonic[2] < 0)
    {
        Ldonic[2] = - Ldonic[2];
    }
    result.x = Ldonic[0] / Ldonic[2] * fov_pixel;
    result.y = Ldonic[1] / Ldonic[2] * fov_pixel;
    return  result;
}

__device__ bool Simulator::isShipAppear()
{
    ObjStatus missile = m_missile[m_current_img_id];
    ObjStatus target = m_target[m_current_img_id];
    float2 target_imgPos = cudaImageModel(missile, target.gps, m_Rb2c_cur,
                                             m_Ri2b_missile_cur, m_Re2i_missile, m_fov_pixel);
    Coordinate target_pos = Geoditic2ECEF(target.gps);
    Coordinate missile_pos = Geoditic2ECEF(missile.gps);

    float distance = (target_pos - missile_pos).norm();

    float delta_az = asinf(target_imgPos.x / sqrtf(target_imgPos.x * target_imgPos.x + m_fov_pixel * m_fov_pixel));
    float delta_el = asinf(target_imgPos.y / Coordinate(target_imgPos.x, target_imgPos.y, m_fov_pixel).norm());
    //    float delta_u0 = atan(m_ship.length / distance) * f;
    //    float delta_w0 = atan(m_ship.height / distance) * f;

    return (std::fabs(delta_az) < deg2rad(m_fov / 2) + atan(15 / distance)) &&
            (std::fabs(delta_el) < deg2rad(3 / 4 * m_fov /2 ) + atan(6 / distance));
}

__global__ void cudaConvertToImage(GPS * ship_gps, float2 * shipImgPos,
                                   float3 * ship_vertices, ObjStatus missile, ObjStatus target,
                                   float *Rb2c_cur, float *Ri2b_missile_cur,
                                   float *Re2i_missile,
                                   float *Ri2b_target, float * Re2i_target,
                                   int num_vertices, float fov_pixel)
{
    int idx = threadIdx.x + IMUL(blockDim.x, blockIdx.x);
    if(idx < num_vertices)
    {
        float vertex[3];
        vertex[0] = ship_vertices[idx].x;
        vertex[1] = ship_vertices[idx].y;
        vertex[2] = ship_vertices[idx].z;

        float temp[9];
        float NED[3];
        mul3x3TransposeFirst(&temp[0], Re2i_target, Ri2b_target);
        mul3x3ToVec3x1(&NED[0], temp, vertex);
        Coordinate tmp = Geoditic2ECEF(target.gps) + Coordinate(NED[0], NED[1], NED[2]);
        ship_gps[idx] = ECEF2Geoditic(tmp);
        shipImgPos[idx] = cudaImageModel(missile,ship_gps[idx], Rb2c_cur,
                                         Ri2b_missile_cur, Re2i_missile, fov_pixel);
    }
}

void Simulator::convertToImage()
{
    int numBlock = (m_ship.num_vertices + threadsPerBlock - 1) / threadsPerBlock;
    cudaConvertToImage<<<numBlock, threadsPerBlock>>>(m_ship.gps, m_ship.imgPos,
                                                      m_ship.vertices, m_missile[m_current_img_id],
                                                      m_target[m_current_img_id],
                                                      m_Rb2c_cur, m_Ri2b_missile_cur,
                                                      m_Re2i_missile,
                                                      m_Ri2b_target,
                                                      m_Re2i_target,
                                                      m_ship.num_vertices, m_fov_pixel);
    gpuErrChk(cudaDeviceSynchronize());
}

__device__ RayInfo calcDistanceToOcean(float * Rb2c_cur, float *Ri2b_missile_cur,
                                       float * Re2i_missile, float fov_pixel)
{

}
//function [d,beta] = Distance (lat,lon,h,roll,pitch,yaw,azimuth,elevator,fov,i_pixel,j_pixel)

//f = 640/2/tan(deg2rad(fov/2));
//u = i_pixel;
//w = j_pixel;

//Ldonic = [u w f]'/norm([u w f]);

//Ldoni = [-sin(azimuth) sin(elevator)*cos(azimuth) cos(elevator)*cos(azimuth);
//    cos(azimuth) sin(elevator)*sin(azimuth) cos(elevator)*sin(azimuth);
//    0          cos(elevator)              -sin(elevator)];
//Ri2b = [cos(pitch)*cos(yaw) sin(roll)*sin(pitch)*cos(yaw)-cos(roll)*sin(yaw) cos(roll)*sin(pitch)*cos(yaw)+sin(roll)*sin(yaw);
//    cos(pitch)*sin(yaw) sin(roll)*sin(pitch)*sin(yaw)+cos(roll)*cos(yaw) cos(roll)*sin(pitch)*sin(yaw)-sin(roll)*cos(yaw);
//    -sin(pitch)          sin(roll)*cos(pitch)                             cos(roll)*cos(pitch)];

//lambda = lat / 180.0 * pi;
//phi = lon / 180.0 * pi;

//sin_lambda = sin(lambda);
//cos_lambda = cos(lambda);
//sin_phi = sin(phi);
//cos_phi = cos(phi);

//M = [-cos_phi * sin_lambda -sin_lambda * sin_phi cos_lambda;
//    -sin_phi              cos_phi               0;
//    -cos_lambda * cos_phi -cos_lambda * sin_phi -sin_lambda];
//NED = M' * Ri2b * Ldoni * Ldonic;

//[ xA, yA, zA ] = Geoditic2ECEF( lat, lon, h );
//NED1 = [xA yA zA]'/norm([xA,yA,zA]);
//% fprintf('\n%f',NED'*NED1)
//L = norm([xA, yA, zA])*abs(NED'*NED1);
//[~,~,h0] = ECEF2Geoditic( xA+L*NED(1,1), yA+L*NED(2,1), zA+L*NED(3,1) );

//if h0 > 0
//    d = -1;
//    beta = -1;
//else
//    [d,beta] = calculator ( xA,yA,zA,NED,L,h,h0 );
//end
//end

__device__ RayInfo averageDistance(float2 *surfaceImgPos, float2 imgPos,
                                   GPS *surface_gps,
                                   float *NED, GPS missile_gps,
                                   Coordinate missile_pos)
{
    float distance = 1000000000;
    for(int i = 0; i < 3; i++)
    {
        int idx1 = i;
        int idx2;
        if(i == 2)
        {
            idx2 = 0;
        }
        else
        {
            idx2 = i + 1;
        }

        float2 vector1;
        float2 vector2;
        vector1.x = surfaceImgPos[idx1].x - imgPos.x;
        vector1.y = surfaceImgPos[idx1].y - imgPos.y;

        vector2.x = surfaceImgPos[idx2].x - imgPos.x;
        vector2.y = surfaceImgPos[idx2].y - imgPos.y;

        Coordinate tmp1, tmp2;
        float d;
        float norm1 = norm(vector1);
        float norm2 = norm(vector2);
        tmp1 = Geoditic2ECEF(surface_gps[idx1]);
        tmp2 = Geoditic2ECEF(surface_gps[idx2]);

        if(norm1 == 0 || norm2 == 0)
        {
            if(norm1 == 0)
            {
                d = (tmp1 - missile_pos).norm();
                if(d < distance)
                {
                    distance = d;
                }
            }

            if(norm2 == 0)
            {
                d = (tmp2 - missile_pos).norm();
                if(d < distance)
                {
                    distance = d;
                }
            }
        }
        else
        {
            if(inner_product(vector1, vector2) / (norm1 * norm2) < 0)
            {
                Coordinate NED1 = (tmp1 - tmp2) / (tmp1 - tmp2).norm();

                float A[6];
                float B[3];

                B[0] = tmp1.x - missile_pos.x;
                B[1] = tmp1.y - missile_pos.y;
                B[2] = tmp1.z - missile_pos.z;

                A[0] = NED[0];
                A[1] = -NED1.x;
                A[2] = NED[1];
                A[3] = -NED1.y;
                A[4] = NED[2];
                A[5] = -NED1.z;

            }
        }
    }
    return RayInfo(distance, M_PI / 2, 0);
}

__device__ RayInfo calcDistanceToFace(float2 *surfaceImgPos, float2 imgPos, GPS missile_gps, GPS *surface_gps,
                                      float * Rb2c_cur, float *Ri2b_missile_cur,
                                      float * Re2i_missile, float fov_pixel)
{
    float NED[3];
    float Ldonic[3];
    float normTmp = sqrtf(imgPos.x * imgPos.x + imgPos.y * imgPos.y + fov_pixel * fov_pixel);
    Ldonic[0] = imgPos.x /normTmp;
    Ldonic[1] = imgPos.y /normTmp;
    Ldonic[2] = fov_pixel / normTmp;

    float tmp[9];
    float tmp1[9];
    mul3x3TransposeFirst(&tmp[0], Re2i_missile, Ri2b_missile_cur);
    mul3x3(&tmp1[0], &tmp[0], Rb2c_cur);
    mul3x3ToVec3x1(&NED[0], &tmp1[0], &Ldonic[0]);

    Coordinate missile_pos = Geoditic2ECEF(missile_gps);
    Coordinate vertex1 = Geoditic2ECEF(surface_gps[0]);

    Coordinate vector1 = vectorCalculator(surface_gps[0], surface_gps[1]);
    Coordinate vector2 = vectorCalculator(surface_gps[1], surface_gps[2]);

    Coordinate n = cross_product(vector1, vector2);
    float d = vertex1 * n;
    float coeff = missile_pos * n - d;
    if(coeff == 0)
    {
//        return averageDistance(&NED[0], missile_gps, missile_pos);
    }
    else
    {
        Coordinate NEDtmp(NED[0], NED[1], NED[2]);
        float distance = NEDtmp.norm() / fabs(NEDtmp * n) * fabs(coeff);
        float beta = acosf((NEDtmp * n) / NEDtmp.norm() / n.norm());
        return RayInfo(distance, beta, 0);
    }
}

__global__ void cudaCalcDistance(RayInfo *distance,
                                 float3 *ship_faces,
                                 float2 *shipImgPos,
                                 GPS *ship_gps,
                                 GPS missile_gps,
                                 float * Rb2c_cur, float *Ri2b_missile_cur,
                                 float * Re2i_missile, float fov_pixel,
                                 int num_faces, int num_partialPix, int offset,
                                 int batch_size, int width, int height)
{
    int partialPixIdx = threadIdx.x + IMUL(blockIdx.x, blockDim.x);
    int faceIdx = threadIdx.y + IMUL(blockIdx.y, blockDim.y);

    // index of result array to be calculated(float* distance)
    int resultIdx = faceIdx * batch_size * PIXEL_GRID_SIZE * PIXEL_GRID_SIZE + partialPixIdx;

    // pixel index in the output image(640 * 480)
    int pixIdx = offset * batch_size + partialPixIdx / (PIXEL_GRID_SIZE * PIXEL_GRID_SIZE);

    // row, col index of current thread in the output width * height (640 * 480) image
    int row = pixIdx / width;
    int col = pixIdx % width;

    // index of subpixel inside the subgrid for each pixel
    int partialPixIdx_X = partialPixIdx / (batch_size * PIXEL_GRID_SIZE);
    int partialPixIdx_Y = partialPixIdx % (batch_size * PIXEL_GRID_SIZE);

    if(faceIdx < num_faces && pixIdx < num_partialPix)
    {
        float u = col + (float)(partialPixIdx_X - 0.5)/ PIXEL_GRID_SIZE - width / 2 - 0.5;
        float w = row + (float)(partialPixIdx_Y - 0.5)/ PIXEL_GRID_SIZE - height / 2 - 0.5;
        float2 imgPos;
        imgPos.x = u;
        imgPos.y = w;

        // TODO: The following code performs a uncoalescing memory access
        // It is not easy to avoid since it require random data access pattern
        // A possible solution is to reorganize input data
        float2 surface_imgPos[3];
        surface_imgPos[0] = shipImgPos[(int)ship_faces[faceIdx].x];
        surface_imgPos[1] = shipImgPos[(int)ship_faces[faceIdx].y];
        surface_imgPos[2] = shipImgPos[(int)ship_faces[faceIdx].z];

        GPS surface_gps[3];
        surface_gps[0] = ship_gps[(int)ship_faces[faceIdx].x];
        surface_gps[1] = ship_gps[(int)ship_faces[faceIdx].y];
        surface_gps[2] = ship_gps[(int)ship_faces[faceIdx].z];

        if(isInsideObject(&surface_imgPos[0], 3, imgPos))
        {

        }
        else
        {
//            distance[resultIdx] = 0;
        }
    }
}

void Simulator::gpuCalcDistanceToFace()
{
    dim3 blockDim(threadsPerBlock, threadsPerBlock);
    dim3 gridDim(ceil((float)m_batch_size * PIXEL_GRID_SIZE * PIXEL_GRID_SIZE / threadsPerBlock),
                 ceil((float)m_ship.num_surfaces / threadsPerBlock));

    gpuErrChk(cudaDeviceSynchronize());
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
    else {
        missile_prev = m_missile[m_current_img_id];
        target_prev = m_target[m_current_img_id];
        seeker_prev = m_seeker[m_current_img_id];
    }

    getRb2cMatrix(m_Rb2c_cur, RotationAngle(0, seeker_cur.elevation, seeker_cur.azimuth));
    getRb2cMatrix(m_Rb2c_prev, RotationAngle(0, seeker_prev.elevation, seeker_prev.azimuth));
    getRi2bMatrix(m_Ri2b_missile_cur, missile_cur.angle);
    getRi2bMatrix(m_Ri2b_missile_prev, missile_prev.angle);
    getRi2bMatrix(m_Ri2b_target, target_cur.angle);
    getRe2iMatrix(m_Re2i_missile, missile_cur.gps);
    getRe2iMatrix(m_Re2i_target, target_cur.gps);
}
