// SPDX-License-Identifier: AGPL-3.0-only
#ifndef CTL_MANIPULATOR_H
#define CTL_MANIPULATOR_H

#include "config.h"
#include "drv_math.h"
#include "drv_can.h"
#include "dvc_unitree_motor.h"
#include "dvc_ak80_motor.h"
#include "dvc_zdt_motor.h"
#include "alg_kinematics.h"
#include "alg_dynamics.h"
#include "alg_static_identify.h"

enum Enum_Manipulator_Control_Status
{
    Manipulator_Control_Status_DISABLE = 0,
    Manipulator_Control_Status_ENABLE,
};

class Class_Manipulator
{
public:
    Class_Unitree_Motor Motor_J0;
    Class_AK_Motor_80_6 Motor_J1;
    Class_AK_Motor_80_6 Motor_J2;
    Class_ZDT_Motor Motor_J3;
    Class_ZDT_Motor Motor_J4;
    Class_ZDT_Motor Motor_J5;

    void Init(Enum_Manipulator_ID __Manipulator_ID);

    inline Enum_Manipulator_Control_Status Get_Manipulator_Control_Status();
    inline float Get_Target_Joint_Angle(uint8_t Joint_ID);
    inline float Get_Target_Joint_Omega(uint8_t Joint_ID);
    inline float Get_Current_Joint_Angle(uint8_t Joint_ID);
    inline float Get_Current_Joint_Omega(uint8_t Joint_ID);
    inline float Get_Current_Joint_Torque(uint8_t Joint_ID);
    inline float Get_Target_Joint_Torque(uint8_t Joint_ID);
    inline Enum_Manipulator_ID Get_Manipulator_ID();

    inline float Get_Gravity_Compensation_Torque(uint8_t Joint_ID);
    inline float Get_Gravity_Compensation_Ratio(uint8_t Joint_ID);
    inline float Get_End_Effector_Position(uint8_t Axis_ID);

    inline void Set_Manipulator_Control_Status(Enum_Manipulator_Control_Status __Manipulator_Control_Status);
    inline void Set_Gravity_Compensation_Ratio(uint8_t Joint_ID, float __Ratio);
    inline void Set_Target_Joint_Angle(uint8_t Joint_ID, float __Target_Joint_Angle);
    inline void Set_Target_Joint_Omega(uint8_t Joint_ID, float __Target_Joint_Omega);
    inline void Set_Target_Joint_Torque(uint8_t Joint_ID, float __Target_Joint_Torque);

    void CAN_RxCpltCallback(Struct_CAN_Rx_Buffer *CAN_RxMessage);
    void UART_RxCpltCallback(uint8_t *Buffer, uint16_t Length);
    void TIM_Calculate_PeriodElapsedCallback();
    void TIM_CAN_PeriodElapsedCallback();
    void TIM1msMod50_Alive_PeriodElapsedCallback();

protected:
    Enum_Manipulator_ID Manipulator_ID = Manipulator_ID_LEFT;
    const Struct_Joint_Limit *Joint_Limit = 0;
    const Struct_Joint_Angle_Alignment *Joint_Angle_Alignment = 0;
    const Struct_Joint_Binding *Joint_Binding = 0;
    Class_Kinematics Kinematics;
    Class_Dynamics Dynamics;

    Enum_Manipulator_Control_Status Manipulator_Control_Status = Manipulator_Control_Status_DISABLE;
    float Target_Joint_Angle[CONTROLLER_JOINT_NUM] = {0.0f};
    float Target_Joint_Omega[CONTROLLER_JOINT_NUM] = {0.0f};
    float Current_Joint_Angle[CONTROLLER_JOINT_NUM] = {0.0f};
    float Current_Joint_Omega[CONTROLLER_JOINT_NUM] = {0.0f};
    float Current_Joint_Torque[CONTROLLER_JOINT_NUM] = {0.0f};
    float Target_Joint_Torque[CONTROLLER_JOINT_NUM] = {0.0f};

    // 重力补偿力矩, 由Dynamics按当前关节角算出, 叠加到目标力矩上输出
    float Gravity_Compensation_Torque[CONTROLLER_JOINT_NUM] = {0.0f};

    // 左臂重力补偿比例, 0关闭, 1使用完整模型输出; 右臂始终不输出重力补偿
    float Gravity_Compensation_Ratio[CONTROLLER_JOINT_NUM] = {0.0f};

    uint8_t CAN_Schedule_Slot = 0U;
    Class_Static_Identify_FSM Static_Identify_FSM;
    uint32_t Static_Identify_Millisecond = 0U;
    float Static_Identify_Target[CONTROLLER_JOINT_NUM] = {0.0f};

    void Calculate_Model();
    void Output();
    void Update_Current_State();
    void Static_Identify_PeriodElapsedCallback();
    uint8_t Static_Identify_At_Target();
    void Static_Identify_Update_Monitor();
    float Motor_Angle_To_Joint_Angle(uint8_t Joint_ID, float Motor_Angle);
    float Joint_Angle_To_Motor_Angle(uint8_t Joint_ID, float Joint_Angle);
    float Motor_Omega_To_Joint_Omega(uint8_t Joint_ID, float Motor_Omega);
    float Joint_Omega_To_Motor_Omega(uint8_t Joint_ID, float Joint_Omega);
    float Motor_Torque_To_Joint_Torque(uint8_t Joint_ID, float Motor_Torque);
    float Joint_Torque_To_Motor_Torque(uint8_t Joint_ID, float Joint_Torque);
};

inline Enum_Manipulator_Control_Status Class_Manipulator::Get_Manipulator_Control_Status()
{
    return (Manipulator_Control_Status);
}

inline float Class_Manipulator::Get_Target_Joint_Angle(uint8_t Joint_ID)
{
    return (Joint_ID < CONTROLLER_JOINT_NUM ? Target_Joint_Angle[Joint_ID] : 0.0f);
}

inline float Class_Manipulator::Get_Target_Joint_Omega(uint8_t Joint_ID)
{
    return (Joint_ID < CONTROLLER_JOINT_NUM ? Target_Joint_Omega[Joint_ID] : 0.0f);
}

inline float Class_Manipulator::Get_Current_Joint_Angle(uint8_t Joint_ID)
{
    return (Joint_ID < CONTROLLER_JOINT_NUM ? Current_Joint_Angle[Joint_ID] : 0.0f);
}

inline float Class_Manipulator::Get_Current_Joint_Omega(uint8_t Joint_ID)
{
    return (Joint_ID < CONTROLLER_JOINT_NUM ? Current_Joint_Omega[Joint_ID] : 0.0f);
}

inline float Class_Manipulator::Get_Current_Joint_Torque(uint8_t Joint_ID)
{
    return (Joint_ID < CONTROLLER_JOINT_NUM ? Current_Joint_Torque[Joint_ID] : 0.0f);
}

inline float Class_Manipulator::Get_Target_Joint_Torque(uint8_t Joint_ID)
{
    return (Joint_ID < CONTROLLER_JOINT_NUM ? Target_Joint_Torque[Joint_ID] : 0.0f);
}

inline Enum_Manipulator_ID Class_Manipulator::Get_Manipulator_ID()
{
    return (Manipulator_ID);
}

inline float Class_Manipulator::Get_Gravity_Compensation_Torque(uint8_t Joint_ID)
{
    return (Joint_ID < CONTROLLER_JOINT_NUM ? Gravity_Compensation_Torque[Joint_ID] : 0.0f);
}

inline float Class_Manipulator::Get_Gravity_Compensation_Ratio(uint8_t Joint_ID)
{
    return (Joint_ID < CONTROLLER_JOINT_NUM ? Gravity_Compensation_Ratio[Joint_ID] : 0.0f);
}

inline float Class_Manipulator::Get_End_Effector_Position(uint8_t Axis_ID)
{
    if (Axis_ID == 0U)
    {
        return (Kinematics.Get_End_Effector_X());
    }
    else if (Axis_ID == 1U)
    {
        return (Kinematics.Get_End_Effector_Y());
    }
    else if (Axis_ID == 2U)
    {
        return (Kinematics.Get_End_Effector_Z());
    }
    return (0.0f);
}

inline void Class_Manipulator::Set_Manipulator_Control_Status(Enum_Manipulator_Control_Status __Manipulator_Control_Status)
{
    Manipulator_Control_Status = __Manipulator_Control_Status;
}

inline void Class_Manipulator::Set_Gravity_Compensation_Ratio(uint8_t Joint_ID, float __Ratio)
{
    if ((Manipulator_ID == Manipulator_ID_LEFT) && (Joint_ID < CONTROLLER_JOINT_NUM))
    {
        Gravity_Compensation_Ratio[Joint_ID] = __Ratio;
    }
}

inline void Class_Manipulator::Set_Target_Joint_Angle(uint8_t Joint_ID, float __Target_Joint_Angle)
{
    if (Joint_ID < CONTROLLER_JOINT_NUM)
    {
        Target_Joint_Angle[Joint_ID] = __Target_Joint_Angle;
    }
}

inline void Class_Manipulator::Set_Target_Joint_Omega(uint8_t Joint_ID, float __Target_Joint_Omega)
{
    if (Joint_ID < CONTROLLER_JOINT_NUM)
    {
        Target_Joint_Omega[Joint_ID] = __Target_Joint_Omega;
    }
}

inline void Class_Manipulator::Set_Target_Joint_Torque(uint8_t Joint_ID, float __Target_Joint_Torque)
{
    if (Joint_ID < CONTROLLER_JOINT_NUM)
    {
        Target_Joint_Torque[Joint_ID] = __Target_Joint_Torque;
    }
}

#endif
