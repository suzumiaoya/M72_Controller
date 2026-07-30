// SPDX-License-Identifier: AGPL-3.0-only
#ifndef MDH_MODEL_H
#define MDH_MODEL_H

#include "config.h"

/**
 * @brief 机械臂MDH运动学模型 (Modified/Craig DH)
 *
 * 变换约定: A_i = Rx(alpha_i) * Tx(a_i) * Rz(theta_i + q_i) * Tz(d_i)
 *
 * 参数来源: URDFly由Manipulator_Left.urdf导出的
 *           base_to_Link6_mdh_parameters.txt
 * 已将alpha/theta吸附到pi/2的整数倍, 并将1e-7量级的a归零
 * (URDFly输出仅8位小数, alpha写作1.57079265而非pi/2, 差3.7e-6 rad)
 *
 * MDH各帧z轴与URDF关节轴同向, 已逐关节验证 dot = +1.000000
 * 注意J0/J2的URDF axis为[0,0,-1], 其MDH子帧x/y轴相对URDF翻转,
 * 但转动正方向一致, 故关节角无需取反
 */

#define MDH_JOINT_NUM  CONTROLLER_JOINT_NUM

struct Struct_MDH_Link
{
    float Theta;    // 绕z_i的常量偏置, rad
    float D;        // 沿z_i的平移, m
    float A;        // 沿x_{i-1}的平移, m
    float Alpha;    // 绕x_{i-1}的旋转, rad
};

extern const Struct_MDH_Link MDH_Model[MDH_JOINT_NUM];

#endif
