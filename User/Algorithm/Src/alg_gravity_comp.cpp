#include "alg_gravity_comp.h"

void Class_Gravity_Comp::Init()
{
    memset(Joint_Angle, 0, sizeof(Joint_Angle));
    memset(Output_Torque, 0, sizeof(Output_Torque));
}

void Class_Gravity_Comp::Set_Joint_Angles(const float *__Joint_Angles, uint8_t __Joint_Num)
{
    uint8_t Joint_Num = __Joint_Num > CONTROLLER_JOINT_NUM ? CONTROLLER_JOINT_NUM : __Joint_Num;
    memcpy(Joint_Angle, __Joint_Angles, Joint_Num * sizeof(float));
}

void Class_Gravity_Comp::Calculate()
{
    memset(Output_Torque, 0, sizeof(Output_Torque));
}
