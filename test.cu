#include "test.h"

__global__ void calcTransMatricesHelper(ObjStatus* missile_cur,
                                        ObjStatus* missile_prev,
                                        ObjStatus* target_cur,
                                        ObjStatus* target_prev,
                                        SeekerInfo* seeker_cur,
                                        SeekerInfo* seeker_prev,
                                        float* Rb2c_cur, float* Ri2b_missile_cur,
                                        float* Re2i_missile,
                                        float* Ri2b_target,
                                        float* Re2i_target,
                                        float* Rb2c_prev,
                                        float* Ri2b_missile_prev)
{
    getRb2cMatrix(Rb2c_cur, RotationAngle(0, seeker_cur->elevation, seeker_cur->azimuth));
    getRb2cMatrix(Rb2c_prev, RotationAngle(0, seeker_prev->elevation, seeker_prev->azimuth));
    getRi2bMatrix(Ri2b_missile_cur, missile_cur->angle);
    getRi2bMatrix(Ri2b_missile_prev, missile_prev->angle);
    getRi2bMatrix(Ri2b_target, target_cur->angle);
    getRe2iMatrix(Re2i_missile, missile_cur->gps);
    getRe2iMatrix(Re2i_target, target_cur->gps);
}

void calcTransMatrix(ObjStatus* missile_cur, ObjStatus* missile_prev,
                     ObjStatus *target_cur, ObjStatus* target_prev,
                     SeekerInfo* seeker_cur, SeekerInfo* seeker_prev,
                     float* Rb2c_cur, float* Ri2b_missile_cur,
                     float* Re2i_missile,
                     float* Ri2b_target,
                     float* Re2i_target,
                     float* Rb2c_prev,
                     float* Ri2b_missile_prev)
{
    calcTransMatricesHelper<<<1, 1>>>(missile_cur, missile_prev,target_cur, target_prev,
                                              seeker_cur, seeker_prev,
                                              Rb2c_cur,Ri2b_missile_cur,
                                              Re2i_missile, Ri2b_target,Re2i_target,
                                              Rb2c_prev, Ri2b_missile_prev);
}
