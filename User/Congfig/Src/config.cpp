// SPDX-License-Identifier: AGPL-3.0-only
#include "config.h"

const Struct_Joint_Limit Left_Arm_Joint_Limit[CONTROLLER_JOINT_NUM] =
{
    {-3.141593f, 3.141593f},
    {-2.967060f, 2.967060f},
    {-2.617994f, 2.617994f},
    {-2.268928f, 2.268928f},
    {-2.094395f, 2.094395f},
    {-1.745329f, 1.745329f},
};

const Struct_Joint_Limit Right_Arm_Joint_Limit[CONTROLLER_JOINT_NUM] =
{
    {-3.141593f, 3.141593f},
    {-2.792527f, 2.792527f},
    {-2.617994f, 2.617994f},
    {-2.268928f, 2.268928f},
    {-1.919862f, 1.919862f},
    {-1.570796f, 1.570796f},
};

const Struct_Joint_Binding Left_Arm_Joint_Binding[CONTROLLER_JOINT_NUM] =
{
    {Controller_Motor_Type_UNITREE, Bus_ID_RS485_USART2, 0x0000},
    {Controller_Motor_Type_AK80, Bus_ID_CAN_1, 0x0001},
    {Controller_Motor_Type_AK80, Bus_ID_CAN_1, 0x0002},
    {Controller_Motor_Type_ZDT, Bus_ID_CAN_1, 0x0003},
    {Controller_Motor_Type_ZDT, Bus_ID_CAN_1, 0x0004},
    {Controller_Motor_Type_ZDT, Bus_ID_CAN_1, 0x0005},
};

const Struct_Joint_Binding Right_Arm_Joint_Binding[CONTROLLER_JOINT_NUM] =
{
    {Controller_Motor_Type_UNITREE, Bus_ID_RS485_USART3, 0x0000},
    {Controller_Motor_Type_AK80, Bus_ID_CAN_2, 0x0001},
    {Controller_Motor_Type_AK80, Bus_ID_CAN_2, 0x0002},
    {Controller_Motor_Type_ZDT, Bus_ID_CAN_2, 0x0003},
    {Controller_Motor_Type_ZDT, Bus_ID_CAN_2, 0x0004},
    {Controller_Motor_Type_ZDT, Bus_ID_CAN_2, 0x0005},
};
