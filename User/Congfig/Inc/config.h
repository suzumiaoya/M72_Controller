// SPDX-License-Identifier: AGPL-3.0-only
#ifndef CONFIG_H
#define CONFIG_H

#include "main.h"
#include <stdint.h>

#define CONTROLLER_ARM_COUNT      2U
#define CONTROLLER_JOINT_NUM      6U
#define CONTROLLER_CAN_JOINT_NUM  5U

#ifndef USE_URDF_COORDS
#define USE_URDF_COORDS 1  // 1=URDF, 0=MDH
#endif

#ifndef CURRENT_ARM_ID
#define CURRENT_ARM_ID Manipulator_ID_RIGHT
#endif

enum Enum_Manipulator_ID
{
    Manipulator_ID_LEFT = 0,
    Manipulator_ID_RIGHT,
};

enum Enum_Controller_Joint_ID
{
    Controller_Joint_ID_J0 = 0,
    Controller_Joint_ID_J1,
    Controller_Joint_ID_J2,
    Controller_Joint_ID_J3,
    Controller_Joint_ID_J4,
    Controller_Joint_ID_J5,
};

enum Enum_Controller_Motor_Type
{
    Controller_Motor_Type_UNITREE = 0,
    Controller_Motor_Type_AK80,
    Controller_Motor_Type_ZDT,
};

enum Enum_Bus_ID
{
    Bus_ID_RS485_USART2 = 0,
    Bus_ID_RS485_USART3,
    Bus_ID_CAN_1,
    Bus_ID_CAN_2,
};

// 电机转动方向到MDH关节方向的映射
enum Enum_Joint_Angle_Direction
{
    Joint_Angle_Direction_SAME = 1,
    Joint_Angle_Direction_INVERTED = -1,
};

// 关节限位
struct Struct_Joint_Limit
{
    float Min_Angle;
    float Max_Angle;
};

// 电机角度到MDH关节角度的对齐配置
struct Struct_Joint_Angle_Alignment
{
    Enum_Joint_Angle_Direction Direction;
    // 统一参考位姿下的电机反馈角度，单位rad
    float Motor_Angle_At_Reference;
};

// 各关节电机通信配置
struct Struct_Joint_Binding
{
    Enum_Controller_Motor_Type Motor_Type;
    Enum_Bus_ID Bus_ID;
    uint16_t Device_ID;
};

extern const Struct_Joint_Limit Left_Arm_Joint_Limit[CONTROLLER_JOINT_NUM];
extern const Struct_Joint_Limit Right_Arm_Joint_Limit[CONTROLLER_JOINT_NUM];

extern const Struct_Joint_Angle_Alignment Left_Arm_Joint_Alignment[CONTROLLER_JOINT_NUM];
extern const Struct_Joint_Angle_Alignment Right_Arm_Joint_Alignment[CONTROLLER_JOINT_NUM];

extern const Struct_Joint_Binding Left_Arm_Joint_Binding[CONTROLLER_JOINT_NUM];
extern const Struct_Joint_Binding Right_Arm_Joint_Binding[CONTROLLER_JOINT_NUM];

#endif
