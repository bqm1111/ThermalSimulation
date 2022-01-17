#ifndef BUFFER_H
#define BUFFER_H
#include "common.h"
#include "define.h"
class GenericData
{
public:
    enum AllocateType{
        Host,
        Device,
        Dual
    };
    GenericData(){}
    ~GenericData();
    GenericData(size_t size, DataType dataType, AllocateType allocType=AllocateType::Host);
    void create(size_t size, DataType dataType, AllocateType allocType=AllocateType::Host);
    void *dPtr();
    void *hPtr();
private:
    DataType type_;
    AllocateType allocType_;
    size_t size_;
    void *dData;
    void *hData;

};

#endif
