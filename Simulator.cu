#include "Simulator.h"

__device__ float calcGainTransmittance(float distance)
{
    return expf(-0.26 * distance / 1000);
}

__device__ float norm(float2 input)
{
    return sqrtf(input.x * input.x + input.y * input.y);
}

__device__ float inner_product(float2 a, float2 b)
{
    return (a.x * b.x + a.y * b.y);
}

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


__device__ float calcDistanceToFaceHelper()
{

}
__global__ void cudaConvertToImage(GPS * ship_gps, float2 * shipImgPos,
                                   float3 * ship_vertices, ObjStatus missile, ObjStatus target,
                                   float *Rb2c_cur, float *Ri2b_missile_cur,
                                   float *Re2i_missile,
                                   float * Ri2b_target, float * Re2i_target,
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

__global__ void cudaCalcDistanceToFace(float *distance,
                                       int num_faces, int num_partialPix, int offset,
                                       int batch_size, int width, int height)
{
    int partialPixIdx = threadIdx.x + IMUL(blockIdx.x, blockDim.x);
    int faceIdx = threadIdx.y + IMUL(blockIdx.y, blockDim.y);
    int resultIdx = faceIdx * batch_size * PIXEL_GRID_SIZE * PIXEL_GRID_SIZE + partialPixIdx;

    int pixIdx = offset * batch_size + partialPixIdx / (PIXEL_GRID_SIZE * PIXEL_GRID_SIZE);
    int row = pixIdx / width;
    int col = pixIdx % width;
    int partialPixIdx_X = partialPixIdx / (batch_size * PIXEL_GRID_SIZE);
    int partialPixIdx_Y = partialPixIdx % (batch_size * PIXEL_GRID_SIZE);

    if(faceIdx < num_faces && pixIdx < num_partialPix)
    {
        float u = col + (float)(partialPixIdx_X - 0.5)/ PIXEL_GRID_SIZE - width / 2 - 0.5;
        float w = row + (float)(partialPixIdx_Y - 0.5)/ PIXEL_GRID_SIZE - height / 2 - 0.5;
        if(isInsideObject())
        {

        }
        else
        {
            distance[resultIdx] = 0;
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
