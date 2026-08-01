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
    {Joint_Angle_Direction_SAME,     -0.432787f},
    {Joint_Angle_Direction_SAME,      0.000000f},
    {Joint_Angle_Direction_SAME,      0.000000f},
    {Joint_Angle_Direction_SAME,      0.000000f},
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

// MDH运动学模型参数，来源见mdh_model.h
const Struct_Dynamics_Link_Param Left_Dynamics_Link_Param[CONTROLLER_JOINT_NUM] =
{
    {0.758220778634767f, { 0.000756164398497296f, -0.00150616966659868f,  0.00792639613989271f}},
    {0.614430000000000f, { 0.064502861400000000f,  0.000366180111593166f,  0.02344635304902510f}},
    {0.616098841447018f, {-0.000001073984380438f, -0.00848852439977156f, -0.00448005996913223f}},
    {0.355335860914443f, {-0.000088677662703699f,  0.01264007061927230f, -0.00187787984203345f}},
    {0.370856186126271f, {-0.000216231031980213f, -0.01823554571865870f,  0.000815420234088653f}},
    {0.003465972785661f, { 0.000000568495769010f, -0.000000150060768404f,  0.000039525461202507f}},
};

const Struct_Dynamics_Link_Param Right_Dynamics_Link_Param[CONTROLLER_JOINT_NUM] =
{
    {0.0f, {0.0f, 0.0f, 0.0f}},
    {0.0f, {0.0f, 0.0f, 0.0f}},
    {0.0f, {0.0f, 0.0f, 0.0f}},
    {0.0f, {0.0f, 0.0f, 0.0f}},
    {0.0f, {0.0f, 0.0f, 0.0f}},
    {0.0f, {0.0f, 0.0f, 0.0f}},
};

const Struct_Static_Identify_Pose Static_Identify_Left_Pose[STATIC_IDENTIFY_POSE_COUNT] =
{
    {{-0.218166f, 0.785398f,  0.000000f,  0.000000f,  0.000000f,  0.000000f}},
    {{-0.800000f, 0.450000f, -0.700000f, -1.100000f,  0.600000f,  0.500000f}},
    {{ 0.350000f, 1.100000f,  0.650000f,  1.000000f, -0.650000f, -0.450000f}},
    {{-0.650000f, 1.150000f,  0.450000f, -0.900000f, -0.500000f,  0.650000f}},
    {{ 0.450000f, 0.400000f, -0.500000f,  1.100000f,  0.550000f, -0.600000f}},
    {{-0.940000f, 0.850000f,  0.750000f,  0.650000f, -0.750000f,  0.300000f}},
    {{ 0.150000f, 1.170000f, -0.750000f, -0.700000f,  0.700000f, -0.300000f}},
    {{-0.450000f, 0.400000f,  0.850000f,  1.300000f, -0.350000f,  0.700000f}},
    {{ 0.500000f, 0.950000f, -0.850000f, -1.300000f,  0.350000f, -0.700000f}},
    {{-0.700000f, 1.170000f, -0.300000f,  0.400000f, -0.850000f,  0.450000f}},
    {{ 0.250000f, 0.500000f,  0.300000f, -0.400000f,  0.850000f, -0.450000f}},
    {{-0.218166f, 0.785398f,  0.000000f,  0.000000f,  0.000000f,  0.000000f}},
};

// Populate the right-arm excitation poses after its mass/COM seed is available.
// The neutral placeholder keeps a config-only arm switch mechanically bounded.
const Struct_Static_Identify_Pose Static_Identify_Right_Pose[STATIC_IDENTIFY_POSE_COUNT] =
{
    {{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}},
    {{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}},
    {{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}},
    {{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}},
    {{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}},
    {{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}},
    {{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}},
    {{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}},
    {{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}},
    {{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}},
    {{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}},
    {{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}},
};

volatile Struct_Static_Identify_Runtime Static_Identify_Runtime =
{
    0U,
    0U,
    {-0.218166f, 0.785398f, 0.0f},
    {0.0f, 0.0f, 0.0f},
    {0.0f, 0.0f, 0.0f},
    0.30f,
    4.00f,
    0.0f,
};

volatile Struct_Static_Identify_Monitor Static_Identify_Monitor = {};

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
