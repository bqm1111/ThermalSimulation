#include "buffer.h"

GenericData::GenericData(size_t size, DataType dataType, AllocateType allocType)
{
    create(size, dataType, allocType);
}

void *GenericData::hPtr()
{
    if(allocType_ == Device)
    {
        std::cerr << "This pointer is allocated in device. Cant access from host" << std::endl;
    }
    return hData;
}

void *GenericData::dPtr()
{
    if(allocType_ == Host)
    {
         std::cerr << "This pointer is allocated in host. Cant access from device" << std::endl;
    }
    return dData;
}

GenericData::~GenericData()
{
    switch (allocType_) {
    case Host:
        free(hData);
        break;
    case Device:
        gpuErrChk(cudaFree(dData));
        break;
    case Dual:
        gpuErrChk(cudaFreeHost(hData));
        break;
    }
}

void GenericData::create(size_t size, DataType dataType, AllocateType allocType)
{
    type_ = dataType;
    switch (allocType) {
    case Host:
        switch (dataType) {
        case UCHAR:
            hData = (unsigned char*)malloc(size);
            break;
        case INT:
            hData = (int*)malloc(size);
            break;
        case FLOAT:
            hData = (double*)malloc(size);
            break;
        case FLOAT3:
            hData = (double3*)malloc(size);
            break;
        case DOUBLE:
            hData = (double*)malloc(size);
            break;
        default:
            std::cerr << "Invalid Data Type" << std::endl;
        }
        break;
    case Device:
        gpuErrChk(cudaMalloc((void**)&dData, size));
        break;
    case Dual:
        gpuErrChk(cudaHostAlloc((void**)&hData, size, cudaHostAllocMapped));
        gpuErrChk(cudaHostGetDevicePointer(reinterpret_cast<void**>(&dData), hData, 0));
        break;
    default:
        std::cerr << "Invalid allocate Type" << std::endl;
    }
}
