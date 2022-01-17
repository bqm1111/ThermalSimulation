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

__device__ float calcDistanceToFaceHelper()
{

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
//        if(isInsideObject())
//        {

//        }
//        else
//        {
//            distance[resultIdx] = 0;
//        }
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

    cudaGetRb2cMatrix(m_Rb2c_cur, RotationAngle(0, seeker_cur.elevation, seeker_cur.azimuth));
    cudaGetRb2cMatrix(m_Rb2c_prev, RotationAngle(0, seeker_prev.elevation, seeker_prev.azimuth));
    cudaGetRi2bMatrix(m_Ri2b_missile_cur, missile_cur.angle);
    cudaGetRi2bMatrix(m_Ri2b_missile_prev, missile_prev.angle);
    cudaGetRi2bMatrix(m_Ri2b_target, target_cur.angle);
    cudaGetRe2iMatrix(m_Re2i_missile, missile_cur.gps);
    cudaGetRe2iMatrix(m_Re2i_target, target_cur.gps);
}
