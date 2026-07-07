// SPDX-License-Identifier: AGPL-3.0-only
#include "alg_dh_model.h"

void Class_DH_Model::Init()
{
    memset(Joint_Angle, 0, sizeof(Joint_Angle));
    memset(End_Effector_Position, 0, sizeof(End_Effector_Position));
}

void Class_DH_Model::Set_Joint_Angles(const float *__Joint_Angles, uint8_t __Joint_Num)
{
    uint8_t Joint_Num = __Joint_Num > CONTROLLER_JOINT_NUM ? CONTROLLER_JOINT_NUM : __Joint_Num;
    memcpy(Joint_Angle, __Joint_Angles, Joint_Num * sizeof(float));
}

void Class_DH_Model::Calculate()
{
    End_Effector_Position[0] = Joint_Angle[0];
    End_Effector_Position[1] = Joint_Angle[1];
    End_Effector_Position[2] = Joint_Angle[2];
}
