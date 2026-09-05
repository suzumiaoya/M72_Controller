// SPDX-License-Identifier: AGPL-3.0-only
/**
 * @file alg_dynamics_urdf.cpp
 * @brief 机械臂动力学 - URDF坐标系实现
 * @version 0.3
 * @date 2025-01-XX
 *
 * 编译条件: USE_URDF_COORDS == 1
 *
 * 实现: 递归牛顿-欧拉算法 (RNEA) 计算重力项
 *
 * 特点:
 * - 质心和质量可在Ozone中实时调整
 * - 直接使用URDF坐标系，参数物理意义明确
 * - 性能: 单臂约254μs (H723@550MHz)
 *
 * @copyright ZLLC 2027
 */

#include "alg_dynamics.h"

#if USE_URDF_COORDS

#include "urdf_model.h"
#include <math.h>
#include <string.h>

void Class_Dynamics::Init()
{
    memset(Joint_Angle, 0, sizeof(Joint_Angle));
    memset(Gravity_Torque, 0, sizeof(Gravity_Torque));
    Gravity[0] = 0.0f;
    Gravity[1] = 0.0f;
    Gravity[2] = -9.81f;
}

void Class_Dynamics::Set_Joint_Angles(const float *__Joint_Angles, uint8_t __Joint_Num)
{
    uint8_t Joint_Num = __Joint_Num > CONTROLLER_JOINT_NUM ? CONTROLLER_JOINT_NUM : __Joint_Num;
    memcpy(Joint_Angle, __Joint_Angles, Joint_Num * sizeof(float));
}

void Class_Dynamics::Set_Gravity_Vector(float __Gx, float __Gy, float __Gz)
{
    Gravity[0] = __Gx;
    Gravity[1] = __Gy;
    Gravity[2] = __Gz;
}

void Class_Dynamics::Calculate()
{
    Calculate_Gravity_Term();
}

/**
 * @brief 重力项 g(q) - 递归牛顿-欧拉算法
 *
 * 前向递推:
 * - 计算每个连杆坐标系的变换矩阵
 * - 将基座系重力向量变换到各连杆坐标系: g_i = R_i^T * g_{i-1}
 *
 * 后向递推:
 * - 从末端到基座，累加每个连杆的重力产生的力和力矩
 * - f_i = m_i * g_i + R_{i+1} * f_{i+1}
 * - n_i = c_i x (m_i * g_i) + R_{i+1} * n_{i+1} + p_{i+1} x R_{i+1} * f_{i+1}
 * - tau_i = n_i^T * axis_i (投影到关节轴)
 *
 * 输出: 平衡重力所需施加的关节力矩（与重力产生的力矩反号）
 */
void Class_Dynamics::Calculate_Gravity_Term()
{
    // 选择当前机械臂的URDF模型
    const Struct_URDF_Link *Model = Right_URDF_Model;
    // const Struct_URDF_Link *Model = (CURRENT_ARM_ID == Manipulator_ID_LEFT) ?
    //                                  Left_URDF_Model : Right_URDF_Model;

    // 各连杆Frame的旋转与平移
    float Rotation[CONTROLLER_JOINT_NUM][3][3];
    float Translation[CONTROLLER_JOINT_NUM][3];
    float Local_Gravity[CONTROLLER_JOINT_NUM][3];

    float Propagated_G[3] = {Gravity[0], Gravity[1], Gravity[2]};

    // ========================================================================
    // 前向递推: 计算运动学和重力传播
    // ========================================================================

    for (uint8_t i = 0; i < CONTROLLER_JOINT_NUM; i++)
    {
        // 固定变换
        const float *Rf = Model[i].R;
        const float *Pf = Model[i].xyz;

        // 关节变换
        const float q = Joint_Angle[i];
        const float *axis = Model[i].axis;
        const float c = cosf(q);
        const float s = sinf(q);
        const float t = 1.0f - c;
        const float x = axis[0], y = axis[1], z = axis[2];

        float Rj[3][3];
        Rj[0][0] = t*x*x + c;     Rj[0][1] = t*x*y - s*z;   Rj[0][2] = t*x*z + s*y;
        Rj[1][0] = t*x*y + s*z;   Rj[1][1] = t*y*y + c;     Rj[1][2] = t*y*z - s*x;
        Rj[2][0] = t*x*z - s*y;   Rj[2][1] = t*y*z + s*x;   Rj[2][2] = t*z*z + c;

        // 组合变换: R_i = Rf * Rj
        float R_i[3][3];
        for (uint8_t r = 0; r < 3; r++)
        {
            for (uint8_t cc = 0; cc < 3; cc++)
            {
                // Rf以行主序存储，Rf[r][c] = Rf[r*3+c]
                R_i[r][cc] = Rf[r*3+0]*Rj[0][cc] + Rf[r*3+1]*Rj[1][cc] + Rf[r*3+2]*Rj[2][cc];
            }
        }

        // 保存变换
        memcpy(Rotation[i], R_i, sizeof(R_i));
        memcpy(Translation[i], Pf, sizeof(float)*3);

        // 重力向量变换: g_i = R_i^T * g_{i-1}
        float Local[3];
        for (uint8_t r = 0; r < 3; r++)
        {
            Local[r] = R_i[0][r] * Propagated_G[0]
                     + R_i[1][r] * Propagated_G[1]
                     + R_i[2][r] * Propagated_G[2];
        }
        Local_Gravity[i][0] = Propagated_G[0] = Local[0];
        Local_Gravity[i][1] = Propagated_G[1] = Local[1];
        Local_Gravity[i][2] = Propagated_G[2] = Local[2];
    }

    // ========================================================================
    // 后向递推: 计算力和力矩
    // ========================================================================

    float Force[3] = {0.0f, 0.0f, 0.0f};
    float Moment[3] = {0.0f, 0.0f, 0.0f};

    for (int8_t i = CONTROLLER_JOINT_NUM - 1; i >= 0; i--)
    {
        const float Mass = Model[i].mass;
        const float *COM = Model[i].com;
        const float *G = Local_Gravity[i];

        // 本连杆自身贡献
        float Next_Force[3];
        float Next_Moment[3];

        // f = m * g
        Next_Force[0] = Mass * G[0];
        Next_Force[1] = Mass * G[1];
        Next_Force[2] = Mass * G[2];

        // n = c x (m * g)
        Next_Moment[0] = COM[1] * Next_Force[2] - COM[2] * Next_Force[1];
        Next_Moment[1] = COM[2] * Next_Force[0] - COM[0] * Next_Force[2];
        Next_Moment[2] = COM[0] * Next_Force[1] - COM[1] * Next_Force[0];

        // 累加子连杆的贡献
        if (i + 1 < CONTROLLER_JOINT_NUM)
        {
            const uint8_t child = i + 1;

            // 子连杆力变换到本Frame: f_child_local = R_child * f_child
            float Child_Force[3];
            for (uint8_t r = 0; r < 3; r++)
            {
                Child_Force[r] = Rotation[child][r][0] * Force[0]
                               + Rotation[child][r][1] * Force[1]
                               + Rotation[child][r][2] * Force[2];
            }

            // 子连杆力矩变换
            float Child_Moment[3];
            for (uint8_t r = 0; r < 3; r++)
            {
                Child_Moment[r] = Rotation[child][r][0] * Moment[0]
                                + Rotation[child][r][1] * Moment[1]
                                + Rotation[child][r][2] * Moment[2];
            }

            // 力矩贡献: n += n_child + p_child x f_child
            const float *P = Translation[child];
            Next_Moment[0] += Child_Moment[0] + P[1] * Child_Force[2] - P[2] * Child_Force[1];
            Next_Moment[1] += Child_Moment[1] + P[2] * Child_Force[0] - P[0] * Child_Force[2];
            Next_Moment[2] += Child_Moment[2] + P[0] * Child_Force[1] - P[1] * Child_Force[0];

            // 力累加
            Next_Force[0] += Child_Force[0];
            Next_Force[1] += Child_Force[1];
            Next_Force[2] += Child_Force[2];
        }

        memcpy(Force, Next_Force, sizeof(Force));
        memcpy(Moment, Next_Moment, sizeof(Moment));

        // 关节力矩 = 力矩在关节轴上的投影
        const float *axis = Model[i].axis;
        float tau = Moment[0]*axis[0] + Moment[1]*axis[1] + Moment[2]*axis[2];

        // 取反，得到需要施加的力矩（平衡重力）
        Gravity_Torque[i] = -tau;
    }
}

#endif // USE_URDF_COORDS
