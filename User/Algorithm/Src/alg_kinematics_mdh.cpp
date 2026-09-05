// SPDX-License-Identifier: AGPL-3.0-only
/**
 * @file alg_kinematics.cpp
 * @author hsl
 * @brief 机械臂运动学算法
 * @version 0.1
 * @date 2025-07-31 0.1 27赛季定稿
 *
 * @copyright ZLLC 2027
 *
 */
#include "alg_kinematics.h"
#include <math.h>

#if !USE_URDF_COORDS
void Class_Kinematics::Init()
{
    memset(Joint_Angle, 0, sizeof(Joint_Angle));
    memset(End_Effector_Position, 0, sizeof(End_Effector_Position));
    memset(End_Effector_Rotation, 0, sizeof(End_Effector_Rotation));
    End_Effector_Rotation[0] = 1.0f;
    End_Effector_Rotation[4] = 1.0f;
    End_Effector_Rotation[8] = 1.0f;
}

void Class_Kinematics::Set_Joint_Angles(const float *__Joint_Angles, uint8_t __Joint_Num)
{
    uint8_t Joint_Num = __Joint_Num > CONTROLLER_JOINT_NUM ? CONTROLLER_JOINT_NUM : __Joint_Num;
    memcpy(Joint_Angle, __Joint_Angles, Joint_Num * sizeof(float));
}

/**
 * @brief 正运动学, 逐帧累乘MDH变换
 *
 * A_i = Rx(alpha_i)*Tx(a_i)*Rz(theta_i + q_i)*Tz(d_i)
 */
void Class_Kinematics::Fkine()
{
    float R[3][3] = {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
    float P[3] = {0.0f, 0.0f, 0.0f};

    for (uint8_t i = 0; i < CONTROLLER_JOINT_NUM; i++)
    {
        const float Theta = MDH_Model[i].Theta + Joint_Angle[i];
        const float Ct = cosf(Theta);
        const float St = sinf(Theta);
        const float Ca = cosf(MDH_Model[i].Alpha);
        const float Sa = sinf(MDH_Model[i].Alpha);

        const float Ri[3][3] =
        {
            { Ct,      -St,       0.0f},
            { St * Ca,  Ct * Ca, -Sa  },
            { St * Sa,  Ct * Sa,  Ca  },
        };
        const float Pi[3] =
        {
             MDH_Model[i].A,
            -MDH_Model[i].D * Sa,
             MDH_Model[i].D * Ca,
        };

        // P = P + R * Pi
        for (uint8_t r = 0; r < 3; r++)
        {
            P[r] += R[r][0] * Pi[0] + R[r][1] * Pi[1] + R[r][2] * Pi[2];
        }

        // R = R * Ri
        float Next[3][3];
        for (uint8_t r = 0; r < 3; r++)
        {
            for (uint8_t c = 0; c < 3; c++)
            {
                Next[r][c] = R[r][0] * Ri[0][c] + R[r][1] * Ri[1][c] + R[r][2] * Ri[2][c];
            }
        }
        memcpy(R, Next, sizeof(R));
    }

    End_Effector_Position[0] = P[0];
    End_Effector_Position[1] = P[1];
    End_Effector_Position[2] = P[2];
    for (uint8_t r = 0; r < 3; r++)
    {
        for (uint8_t c = 0; c < 3; c++)
        {
            End_Effector_Rotation[r * 3 + c] = R[r][c];
        }
    }
}

#endif