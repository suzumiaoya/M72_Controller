// SPDX-License-Identifier: AGPL-3.0-only
#include "config.h"
#include "mdh_model.h"
#include <float.h>

// 左右臂限位，此处为相对于mdh模型的限位，非相对于电机控制时的限位
const Struct_Joint_Limit Left_Arm_Joint_Limit[CONTROLLER_JOINT_NUM] =
{
    {-1.570796f, 1.134464f},
    { 0.000000f, 1.570796f},
    {-1.570796f, 1.570796f},
    {-2.617994f, 2.617994f},
    {-1.570796f, 1.570796f},
    {-FLT_MAX, FLT_MAX},
};

const Struct_Joint_Limit Right_Arm_Joint_Limit[CONTROLLER_JOINT_NUM] =
{
    {-1.134464f, 1.570796f},
    {-1.570796f, 0.000000f},
    {-1.570796f, 1.570796f},
    {-2.617994f, 2.617994f},
    {-1.570796f, 1.570796f},
    {-FLT_MAX, FLT_MAX},
};

// 左右臂对齐配置
const Struct_Joint_Angle_Alignment Left_Arm_Joint_Alignment[CONTROLLER_JOINT_NUM] =
{
    {Joint_Angle_Direction_INVERTED, -0.307038f},
    {Joint_Angle_Direction_SAME,     -0.494201f},
    {Joint_Angle_Direction_INVERTED,  0.000000f},
    {Joint_Angle_Direction_SAME,      0.000000f},
    {Joint_Angle_Direction_SAME,      0.000000f},
    {Joint_Angle_Direction_SAME,      0.000000f},
};

const Struct_Joint_Angle_Alignment Right_Arm_Joint_Alignment[CONTROLLER_JOINT_NUM] =
{
    {Joint_Angle_Direction_INVERTED, 1.739424f},
    {Joint_Angle_Direction_SAME, 0.023841f},
    {Joint_Angle_Direction_SAME, -0.243191f},
    {Joint_Angle_Direction_SAME, 0.0000000f},
    {Joint_Angle_Direction_SAME, 0.0000000f},
    {Joint_Angle_Direction_SAME, 0.0000000f},
};

// AK80电机由于结构安装时没按照CAN_ID顺序安装，所以这里反一下

const Struct_Joint_Binding Left_Arm_Joint_Binding[CONTROLLER_JOINT_NUM] =
{
    {Controller_Motor_Type_UNITREE, Bus_ID_RS485_USART2, 0x0000},
    {Controller_Motor_Type_AK80, Bus_ID_CAN_1, 0x0002},
    {Controller_Motor_Type_AK80, Bus_ID_CAN_1, 0x0001},
    {Controller_Motor_Type_ZDT, Bus_ID_CAN_1, 0x0003},
    {Controller_Motor_Type_ZDT, Bus_ID_CAN_1, 0x0004},
    {Controller_Motor_Type_ZDT, Bus_ID_CAN_1, 0x0005},
};

const Struct_Joint_Binding Right_Arm_Joint_Binding[CONTROLLER_JOINT_NUM] =
{
    {Controller_Motor_Type_UNITREE, Bus_ID_RS485_USART3, 0x0000},
    {Controller_Motor_Type_AK80, Bus_ID_CAN_2, 0x0002},
    {Controller_Motor_Type_AK80, Bus_ID_CAN_2, 0x0001},
    {Controller_Motor_Type_ZDT, Bus_ID_CAN_2, 0x0003},
    {Controller_Motor_Type_ZDT, Bus_ID_CAN_2, 0x0004},
    {Controller_Motor_Type_ZDT, Bus_ID_CAN_2, 0x0005},
};

// MDH运动学模型参数，来源见mdh_model.h
const Struct_MDH_Link MDH_Model[MDH_JOINT_NUM] =
{
    // Theta          D              A          Alpha
    { 3.14159265f, -0.11560009f, 0.00000000f,  1.57079633f},
    { 3.14159265f,  0.02368700f, 0.00000000f,  1.57079633f},
    { 1.57079633f,  0.02470000f, 0.11500000f,  3.14159265f},
    {-1.57079633f,  0.13249998f, 0.00000000f,  1.57079633f},
    { 3.14159265f,  0.00019408f, 0.00000000f,  1.57079633f},
    {-3.14159265f,  0.07700000f, 0.00000000f,  1.57079633f},
};
