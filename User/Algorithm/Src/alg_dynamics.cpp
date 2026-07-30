// SPDX-License-Identifier: AGPL-3.0-only
#include "alg_dynamics.h"
#include <math.h>

/**
 * @brief 各连杆惯性参数, 在该连杆自身的MDH帧下表达
 *
 * 质量来源: Manipulator_Left.urdf的<inertial>, 但URDF中三个张大头ZDT电机的
 *          质量覆盖未被sw2urdf正确导出(Link3/4/5的整节质量低于其所含单个
 *          电机的标称值), 故按STL连通体体积识别出电机后, 用标称质量替换:
 *          L60 = 487g @ 103.99cm^3, L40 = 285g @ 70.01cm^3
 *          其余结构件(打印件+玻纤板)按1400 kg/m^3估算
 *          Link1/Link2的AK80质量原本正确, 未改动
 *
 * TODO 待实测修正: 机械师给出单侧臂(不含Unitree)约2.3kg, 此表对应2.64kg,
 *      相差15%。称一个L40和一个L60即可锁定, 只需改本表数值, 代码无需改动
 */
static const Struct_Dynamics_Link_Param Dynamics_Link_Param[CONTROLLER_JOINT_NUM] =
{
    {0.6572444f, { 0.00104418f, -0.00463105f,  0.00671625f}},
    {0.6020305f, { 0.06429749f,  0.00044138f,  0.02584005f}},
    {0.6354614f, {-0.00000100f, -0.00915234f, -0.00646763f}},
    {0.3401152f, { 0.00110941f,  0.01269233f, -0.00192894f}},
    {0.4020753f, {-0.00169213f, -0.01922481f,  0.00119474f}},
    {0.0017924f, { 0.00000659f, -0.00000008f,  0.00002045f}},
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
        const float Mass = Dynamics_Link_Param[i].Mass;
        const float *L = Dynamics_Link_Param[i].First_Moment;
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
