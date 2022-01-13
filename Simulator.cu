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
}

__global__ void cudaCalcDistanceToFace(int num_faces, int num_partialPix, int offset, int batch_size, int width, int height)
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
    }

}


void Simulator::gpuCalcDistanceToFace()
{
    dim3 blockDim(threadsPerBlock, threadsPerBlock);
    dim3 gridDim(ceil((float)m_batch_size * PIXEL_GRID_SIZE * PIXEL_GRID_SIZE / threadsPerBlock),
                 ceil((float)m_ship.num_surfaces / threadsPerBlock));


    gpuErrChk(cudaDeviceSynchronize());

}
