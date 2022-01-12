#include "Simulator.h"

__device__ float calcGainTransmittance(float distance)
{
    return expf(-0.26 * distance / 1000);
}
