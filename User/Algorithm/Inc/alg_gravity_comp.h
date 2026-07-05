#ifndef ALG_GRAVITY_COMP_H
#define ALG_GRAVITY_COMP_H

#include "config.h"
#include <string.h>

class Class_Gravity_Comp
{
public:
    void Init();
    void Set_Joint_Angles(const float *__Joint_Angles, uint8_t __Joint_Num = CONTROLLER_JOINT_NUM);
    void Calculate();

    inline float Get_Output_Torque(uint8_t Joint_ID);

protected:
    float Joint_Angle[CONTROLLER_JOINT_NUM] = {0.0f};
    float Output_Torque[CONTROLLER_JOINT_NUM] = {0.0f};
};

inline float Class_Gravity_Comp::Get_Output_Torque(uint8_t Joint_ID)
{
    return (Joint_ID < CONTROLLER_JOINT_NUM ? Output_Torque[Joint_ID] : 0.0f);
}

#endif
