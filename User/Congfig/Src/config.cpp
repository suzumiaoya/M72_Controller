// SPDX-License-Identifier: AGPL-3.0-only
#include "config.h"
#include <float.h>

// 左右臂限位，此处为相对于mdh模型的限位，非相对于电机控制时的限位
const Struct_Joint_Limit Left_Arm_Joint_Limit[CONTROLLER_JOINT_NUM] =
{
    {-1.134464f, 1.570796f},
    { 0.000000f, 1.570796f},
    {-1.570796f, 1.570796f},
    {-2.617994f, 2.617994f},
    {-1.570796f, 1.570796f},
    {-FLT_MAX, FLT_MAX},
};

const Struct_Joint_Limit Right_Arm_Joint_Limit[CONTROLLER_JOINT_NUM] =
{
    {-1.134464f, 1.570796f},
    { 0.000000f, 1.570796f},
    {-1.570796f, 1.570796f},
    {-2.617994f, 2.617994f},
    {-1.570796f, 1.570796f},
    {-FLT_MAX, FLT_MAX},
};

// 左右臂对齐配置
const Struct_Joint_Angle_Alignment Left_Arm_Joint_Alignment[CONTROLLER_JOINT_NUM] =
{
    {Joint_Angle_Direction_SAME, -0.307071f},
    {Joint_Angle_Direction_SAME, -0.494201f},
    {Joint_Angle_Direction_SAME, -0.462291f},
    {Joint_Angle_Direction_SAME, 0.0000000f},
    {Joint_Angle_Direction_SAME, 0.0000000f},
    {Joint_Angle_Direction_SAME, 0.0000000f},
};

const Struct_Joint_Angle_Alignment Right_Arm_Joint_Alignment[CONTROLLER_JOINT_NUM] =
{
    {Joint_Angle_Direction_SAME, 1.736424f},
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
