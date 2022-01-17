#include <iostream>
#include "Simulator.h"
int main(int argc, char **argv)
{
    Simulator simulator;
    simulator.init();
    simulator.loadData();
//    simulator.run();
//    int numLine = 4500;
//    ObjStatus *hPtr = (ObjStatus*)malloc(numLine * sizeof(ObjStatus));
//    ObjStatus *dPtr;
//    gpuErrChk(cudaMalloc((void**)&dPtr, numLine * sizeof(ObjStatus)));

//    readFromFile("../data/Missile_30hz.txt", (float*)hPtr, numLine, 6);
////    readFromFile("../data/Target_30hz.txt", arr, 20, 6);

//    printObjStatus((ObjStatus*)hPtr, numLine);
//    gpuErrChk(cudaMemcpy(dPtr, hPtr, numLine * sizeof(ObjStatus), cudaMemcpyHostToDevice));

//    free(hPtr);
//    gpuErrChk(cudaFree(dPtr));
}
