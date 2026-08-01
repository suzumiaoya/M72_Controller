// SPDX-License-Identifier: AGPL-3.0-only
#ifndef CONFIG_H
#define CONFIG_H

#include "main.h"
#include <stdint.h>

#define CONTROLLER_ARM_COUNT      2U
#define CONTROLLER_JOINT_NUM      6U
#define CONTROLLER_CAN_JOINT_NUM  5U
#define STATIC_IDENTIFY_POSE_COUNT 12U

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

#ifndef STRUCT_DYNAMICS_LINK_PARAM_DEFINED
#define STRUCT_DYNAMICS_LINK_PARAM_DEFINED
struct Struct_Dynamics_Link_Param
{
    float Mass;
    float First_Moment[3];
};
#endif

struct Struct_Static_Identify_Pose
{
    float Joint_Angle[CONTROLLER_JOINT_NUM];
};

struct Struct_Static_Identify_Runtime
{
    volatile uint8_t Manipulator_Control_Status_Request;
    volatile uint8_t Identify_Entry_Request;
    volatile float Tuning_Target_Joint_Angle[3];
    volatile float MIT_K_P[3];
    volatile float MIT_K_D[3];
    volatile float J0_J2_Profile_Omega;
    volatile float ZDT_Target_Omega;
    volatile float Gravity_Compensation_Ratio;
};

struct Struct_Static_Identify_Monitor
{
    volatile uint32_t Millisecond;
    volatile uint16_t Pose_Index;
    volatile uint8_t State;
    volatile float Target_Joint_Angle[CONTROLLER_JOINT_NUM];
    volatile float Current_Joint_Angle[CONTROLLER_JOINT_NUM];
    volatile float Current_Joint_Omega[CONTROLLER_JOINT_NUM];
    volatile float Current_Joint_Torque[3];
    volatile float Gravity_Compensation_Torque[3];
};

extern const Struct_Joint_Limit Left_Arm_Joint_Limit[CONTROLLER_JOINT_NUM];
extern const Struct_Joint_Limit Right_Arm_Joint_Limit[CONTROLLER_JOINT_NUM];

extern const Struct_Joint_Angle_Alignment Left_Arm_Joint_Alignment[CONTROLLER_JOINT_NUM];
extern const Struct_Joint_Angle_Alignment Right_Arm_Joint_Alignment[CONTROLLER_JOINT_NUM];

extern const Struct_Joint_Binding Left_Arm_Joint_Binding[CONTROLLER_JOINT_NUM];
extern const Struct_Joint_Binding Right_Arm_Joint_Binding[CONTROLLER_JOINT_NUM];

extern const Struct_Dynamics_Link_Param Left_Dynamics_Link_Param[CONTROLLER_JOINT_NUM];
extern const Struct_Dynamics_Link_Param Right_Dynamics_Link_Param[CONTROLLER_JOINT_NUM];
extern const Struct_Static_Identify_Pose Static_Identify_Left_Pose[STATIC_IDENTIFY_POSE_COUNT];
extern const Struct_Static_Identify_Pose Static_Identify_Right_Pose[STATIC_IDENTIFY_POSE_COUNT];
extern volatile Struct_Static_Identify_Runtime Static_Identify_Runtime;
extern volatile Struct_Static_Identify_Monitor Static_Identify_Monitor;

static constexpr Enum_Manipulator_ID Static_Identify_Active_Arm = Manipulator_ID_LEFT;

#endif
