// SPDX-License-Identifier: AGPL-3.0-only
#include "alg_dynamics.h"
#include <math.h>

/**
 * @brief 左臂各连杆惯性参数, 在对应连杆的MDH帧下表达
 *
 * 质量和质心来源: Manipulator-Left.urdf的<inertial>, Link1依次对应MDH1,
 * Link6对应MDH6. base不随机械臂关节运动, 不计入关节重力矩.
 * URDF质心通过c_mdh = R_mdh^T * (p_com_base - p_mdh)转换到MDH帧,
 * 表中保存质量m与一阶矩L = m*c_mdh.
 */
static const Struct_Dynamics_Link_Param Left_Dynamics_Link_Param[CONTROLLER_JOINT_NUM] =
{
    {0.758220778634767f, { 0.000756164398497296f, -0.00150616966659868f,  0.00792639613989271f}},
    {0.614430000000000f, { 0.064502861400000000f,  0.000366180111593166f,  0.02344635304902510f}},
    {0.616098841447018f, {-0.000001073984380438f, -0.00848852439977156f, -0.00448005996913223f}},
    {0.355335860914443f, {-0.000088677662703699f,  0.01264007061927230f, -0.00187787984203345f}},
    {0.370856186126271f, {-0.000216231031980213f, -0.01823554571865870f,  0.000815420234088653f}},
    {0.003465972785661f, { 0.000000568495769010f, -0.000000150060768404f,  0.000039525461202507f}},
};

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
 * @brief 重力项g(q), Newton-Euler递推
 *
 * 前向: 将基座系重力向量逐帧变换到各连杆帧, g_i = R_i^T * g_{i-1}
 * 反向: 累加力与力矩, tau_i取力矩的z分量(MDH中关节轴恒为z_i)
 *
 * 与sympybotics生成的g_func逐位一致(2000组随机构型残差为0)
 * 输出为平衡重力所需施加的关节力矩
 */
void Class_Dynamics::Calculate_Gravity_Term()
{
    // 各连杆帧的旋转与平移, 及重力向量在各帧下的表达
    float Rotation[CONTROLLER_JOINT_NUM][3][3];
    float Translation[CONTROLLER_JOINT_NUM][3];
    float Local_Gravity[CONTROLLER_JOINT_NUM][3];

    float Propagated[3] = {Gravity[0], Gravity[1], Gravity[2]};

    for (uint8_t i = 0; i < CONTROLLER_JOINT_NUM; i++)
    {
        const float Theta = MDH_Model[i].Theta + Joint_Angle[i];
        const float Ct = cosf(Theta);
        const float St = sinf(Theta);
        const float Ca = cosf(MDH_Model[i].Alpha);
        const float Sa = sinf(MDH_Model[i].Alpha);

        // A_i = Rx(alpha)*Tx(a)*Rz(theta)*Tz(d)
        Rotation[i][0][0] =  Ct;      Rotation[i][0][1] = -St;      Rotation[i][0][2] = 0.0f;
        Rotation[i][1][0] =  St * Ca; Rotation[i][1][1] =  Ct * Ca; Rotation[i][1][2] = -Sa;
        Rotation[i][2][0] =  St * Sa; Rotation[i][2][1] =  Ct * Sa; Rotation[i][2][2] =  Ca;

        Translation[i][0] =  MDH_Model[i].A;
        Translation[i][1] = -MDH_Model[i].D * Sa;
        Translation[i][2] =  MDH_Model[i].D * Ca;

        // g_i = R_i^T * g_{i-1}
        float Local[3];
        for (uint8_t r = 0; r < 3; r++)
        {
            Local[r] = Rotation[i][0][r] * Propagated[0]
                     + Rotation[i][1][r] * Propagated[1]
                     + Rotation[i][2][r] * Propagated[2];
        }
        Local_Gravity[i][0] = Propagated[0] = Local[0];
        Local_Gravity[i][1] = Propagated[1] = Local[1];
        Local_Gravity[i][2] = Propagated[2] = Local[2];
    }

    float Force[3] = {0.0f, 0.0f, 0.0f};
    float Moment[3] = {0.0f, 0.0f, 0.0f};

    for (int8_t i = CONTROLLER_JOINT_NUM - 1; i >= 0; i--)
    {
        const float Mass = Left_Dynamics_Link_Param[i].Mass;
        const float *L = Left_Dynamics_Link_Param[i].First_Moment;
        const float *G = Local_Gravity[i];

        // 本连杆自身贡献: f = m*g, n = (m*c) x g
        float Next_Force[3];
        float Next_Moment[3];
        Next_Force[0] = Mass * G[0];
        Next_Force[1] = Mass * G[1];
        Next_Force[2] = Mass * G[2];
        Next_Moment[0] = L[1] * G[2] - L[2] * G[1];
        Next_Moment[1] = L[2] * G[0] - L[0] * G[2];
        Next_Moment[2] = L[0] * G[1] - L[1] * G[0];

        if (i + 1 < CONTROLLER_JOINT_NUM)
        {
            // 子连杆的力与力矩变换到本帧
            const uint8_t c = i + 1;
            float Child_Force[3];
            for (uint8_t r = 0; r < 3; r++)
            {
                Child_Force[r] = Rotation[c][r][0] * Force[0]
                               + Rotation[c][r][1] * Force[1]
                               + Rotation[c][r][2] * Force[2];
            }

            float Child_Moment[3];
            for (uint8_t r = 0; r < 3; r++)
            {
                Child_Moment[r] = Rotation[c][r][0] * Moment[0]
                                + Rotation[c][r][1] * Moment[1]
                                + Rotation[c][r][2] * Moment[2];
            }

            const float *P = Translation[c];
            Next_Moment[0] += Child_Moment[0] + P[1] * Child_Force[2] - P[2] * Child_Force[1];
            Next_Moment[1] += Child_Moment[1] + P[2] * Child_Force[0] - P[0] * Child_Force[2];
            Next_Moment[2] += Child_Moment[2] + P[0] * Child_Force[1] - P[1] * Child_Force[0];

            Next_Force[0] += Child_Force[0];
            Next_Force[1] += Child_Force[1];
            Next_Force[2] += Child_Force[2];
        }

        memcpy(Force, Next_Force, sizeof(Force));
        memcpy(Moment, Next_Moment, sizeof(Moment));

        // 关节轴恒为z_i, 取力矩z分量并反号得到需施加的力矩
        Gravity_Torque[i] = -Moment[2];
    }
}
