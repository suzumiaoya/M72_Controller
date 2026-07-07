// SPDX-License-Identifier: AGPL-3.0-only
#ifndef ALG_DH_MODEL_H
#define ALG_DH_MODEL_H

#include "config.h"
#include <string.h>

class Class_DH_Model
{
public:
    void Init();
    void Set_Joint_Angles(const float *__Joint_Angles, uint8_t __Joint_Num = CONTROLLER_JOINT_NUM);
    void Calculate();

    inline float Get_End_Effector_X();
    inline float Get_End_Effector_Y();
    inline float Get_End_Effector_Z();

protected:
    float Joint_Angle[CONTROLLER_JOINT_NUM] = {0.0f};
    float End_Effector_Position[3] = {0.0f};
};

inline float Class_DH_Model::Get_End_Effector_X()
{
    return (End_Effector_Position[0]);
}

inline float Class_DH_Model::Get_End_Effector_Y()
{
    return (End_Effector_Position[1]);
}

inline float Class_DH_Model::Get_End_Effector_Z()
{
    return (End_Effector_Position[2]);
}

#endif
