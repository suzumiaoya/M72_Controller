// SPDX-License-Identifier: AGPL-3.0-only
#ifndef ALG_KINEMATICS_H
#define ALG_KINEMATICS_H

#include "config.h"
#include "mdh_model.h"
#include <string.h>

/**
 * @brief 机械臂运动学, 目前仅实现正运动学
 *
 * MDH参数表见mdh_model.h, 末端位姿在基座系下表达
 */
class Class_Kinematics
{
public:
    void Init();
    void Set_Joint_Angles(const float *__Joint_Angles, uint8_t __Joint_Num = CONTROLLER_JOINT_NUM);
    void Fkine();

    inline float Get_End_Effector_X();
    inline float Get_End_Effector_Y();
    inline float Get_End_Effector_Z();
    inline const float *Get_End_Effector_Rotation();

protected:
    float Joint_Angle[CONTROLLER_JOINT_NUM] = {0.0f};
    float End_Effector_Position[3] = {0.0f};
    float End_Effector_Rotation[9] = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
};

inline float Class_Kinematics::Get_End_Effector_X()
{
    return (End_Effector_Position[0]);
}

inline float Class_Kinematics::Get_End_Effector_Y()
{
    return (End_Effector_Position[1]);
}

inline float Class_Kinematics::Get_End_Effector_Z()
{
    return (End_Effector_Position[2]);
}

// 行主序3x3旋转矩阵
inline const float *Class_Kinematics::Get_End_Effector_Rotation()
{
    return (End_Effector_Rotation);
}

#endif
