#ifndef CONFIG_H
#define CONFIG_H

#include "main.h"
#include <stdint.h>

#define CONTROLLER_ARM_COUNT      2U
#define CONTROLLER_JOINT_NUM      6U
#define CONTROLLER_CAN_JOINT_NUM  5U

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

struct Struct_Joint_Limit
{
    float Min_Angle;
    float Max_Angle;
};

struct Struct_Joint_Binding
{
    Enum_Controller_Motor_Type Motor_Type;
    Enum_Bus_ID Bus_ID;
    uint16_t Device_ID;
};

extern const Struct_Joint_Limit Left_Arm_Joint_Limit[CONTROLLER_JOINT_NUM];
extern const Struct_Joint_Limit Right_Arm_Joint_Limit[CONTROLLER_JOINT_NUM];

extern const Struct_Joint_Binding Left_Arm_Joint_Binding[CONTROLLER_JOINT_NUM];
extern const Struct_Joint_Binding Right_Arm_Joint_Binding[CONTROLLER_JOINT_NUM];

#endif
