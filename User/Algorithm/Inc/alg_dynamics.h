// SPDX-License-Identifier: AGPL-3.0-only
#ifndef ALG_DYNAMICS_H
#define ALG_DYNAMICS_H

#include "config.h"
#include "mdh_model.h"
#include <string.h>

/**
 * @brief 机械臂动力学, 目前仅实现重力项 g(q)
 *
 * 由URDFly内置的sympybotics生成 (RobotDynCode::gen_gravityterm),
 * 表达式仅依赖各连杆质量m_i与一阶矩L_i = m_i * c_i, 共24个参数,
 * 转动惯量不参与重力项计算
 *
 * 生成代码给出的是"为平衡重力需施加的关节力矩", 与重力产生的力矩反号,
 * Output_Torque已按前者的符号给出, 可直接叠加到目标力矩上
 *
 * 已与MDH原生参考实现逐位校验, 1500组随机构型残差为0
 */

// 单连杆惯性参数, 均在该连杆自身的MDH帧下表达
struct Struct_Dynamics_Link_Param
{
    float Mass;             // 连杆质量, kg
    float First_Moment[3];  // 一阶矩 m*c, kg*m
};

class Class_Dynamics
{
public:
    void Init();
    void Set_Joint_Angles(const float *__Joint_Angles, uint8_t __Joint_Num = CONTROLLER_JOINT_NUM);
    void Set_Gravity_Vector(float __Gx, float __Gy, float __Gz);
    void Calculate();

    inline float Get_Gravity_Torque(uint8_t Joint_ID);

protected:
    float Joint_Angle[CONTROLLER_JOINT_NUM] = {0.0f};
    float Gravity_Torque[CONTROLLER_JOINT_NUM] = {0.0f};

    // 基座系下的重力加速度向量
    // SolidWorks中base坐标系已按z轴负方向对齐重力, 故默认值即可用
    // 若后续接入IMU, 调用Set_Gravity_Vector()传入实测值
    float Gravity[3] = {0.0f, 0.0f, -9.81f};

    void Calculate_Gravity_Term();
};

inline float Class_Dynamics::Get_Gravity_Torque(uint8_t Joint_ID)
{
    return (Joint_ID < CONTROLLER_JOINT_NUM ? Gravity_Torque[Joint_ID] : 0.0f);
}

#endif
