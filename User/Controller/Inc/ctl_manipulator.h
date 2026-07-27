// SPDX-License-Identifier: AGPL-3.0-only
#ifndef CTL_MANIPULATOR_H
#define CTL_MANIPULATOR_H

#include "config.h"
#include "drv_math.h"
#include "drv_can.h"
#include "dvc_unitree_motor.h"
#include "dvc_ak80_motor.h"
#include "dvc_zdt_motor.h"
#include "alg_dh_model.h"
#include "alg_gravity_comp.h"

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
    inline float Get_Current_Joint_Angle(uint8_t Joint_ID);
    inline float Get_Current_Joint_Torque(uint8_t Joint_ID);
    inline float Get_Target_Joint_Torque(uint8_t Joint_ID);
    inline Enum_Manipulator_ID Get_Manipulator_ID();

    inline void Set_Manipulator_Control_Status(Enum_Manipulator_Control_Status __Manipulator_Control_Status);
    inline void Set_Target_Joint_Angle(uint8_t Joint_ID, float __Target_Joint_Angle);
    inline void Set_Target_Joint_Torque(uint8_t Joint_ID, float __Target_Joint_Torque);

    void CAN_RxCpltCallback(Struct_CAN_Rx_Buffer *CAN_RxMessage);
    void UART_RxCpltCallback(uint8_t *Buffer, uint16_t Length);
    void TIM_Calculate_PeriodElapsedCallback();
    void TIM_CAN_PeriodElapsedCallback();
    void TIM1msMod50_Alive_PeriodElapsedCallback();

protected:
    Enum_Manipulator_ID Manipulator_ID = Manipulator_ID_LEFT;
    const Struct_Joint_Limit *Joint_Limit = 0;
    const Struct_Joint_Binding *Joint_Binding = 0;
    Class_DH_Model DH_Model;
    Class_Gravity_Comp Gravity_Comp;

    Enum_Manipulator_Control_Status Manipulator_Control_Status = Manipulator_Control_Status_DISABLE;
    float Target_Joint_Angle[CONTROLLER_JOINT_NUM] = {0.0f};
    float Current_Joint_Angle[CONTROLLER_JOINT_NUM] = {0.0f};
    float Current_Joint_Torque[CONTROLLER_JOINT_NUM] = {0.0f};
    float Target_Joint_Torque[CONTROLLER_JOINT_NUM] = {0.0f};

    Enum_Manipulator_Control_Status CAN_Applied_Control_Status = Manipulator_Control_Status_DISABLE;
    Enum_Manipulator_Control_Status CAN_Transition_Target = Manipulator_Control_Status_DISABLE;
    uint8_t CAN_Transition_Active = 0U;
    volatile uint8_t CAN_Transition_Joint = Controller_Joint_ID_J1;
    volatile uint8_t CAN_Transition_Retry = 0U;
    uint8_t CAN_Schedule_Slot = 0U;
    uint8_t CAN_Superframe_Count = 0U;

    volatile uint8_t CAN_Transaction_Pending = 0U;
    volatile uint8_t CAN_Pending_Joint = 0U;
    volatile uint8_t CAN_Pending_Function = 0xffU;
    volatile uint8_t CAN_Pending_Is_Transition = 0U;
    volatile uint8_t CAN_Response_Timeout_Count = 0U;

    void Output();
    void Update_Current_State();
    void CAN_Set_Pending_Transaction(uint8_t Joint_ID, uint8_t Expected_Function, uint8_t Is_Transition);
    void CAN_Complete_Pending_Transaction(uint8_t Joint_ID, uint8_t Response_Function);
    void CAN_Advance_Schedule();
    uint8_t CAN_Send_Transition_Request();
    uint8_t CAN_Send_Scheduled_Request();
};

inline Enum_Manipulator_Control_Status Class_Manipulator::Get_Manipulator_Control_Status()
{
    return (Manipulator_Control_Status);
}

inline float Class_Manipulator::Get_Target_Joint_Angle(uint8_t Joint_ID)
{
    return (Joint_ID < CONTROLLER_JOINT_NUM ? Target_Joint_Angle[Joint_ID] : 0.0f);
}

inline float Class_Manipulator::Get_Current_Joint_Angle(uint8_t Joint_ID)
{
    return (Joint_ID < CONTROLLER_JOINT_NUM ? Current_Joint_Angle[Joint_ID] : 0.0f);
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

inline void Class_Manipulator::Set_Manipulator_Control_Status(Enum_Manipulator_Control_Status __Manipulator_Control_Status)
{
    Manipulator_Control_Status = __Manipulator_Control_Status;
}

inline void Class_Manipulator::Set_Target_Joint_Angle(uint8_t Joint_ID, float __Target_Joint_Angle)
{
    if (Joint_ID < CONTROLLER_JOINT_NUM)
    {
        Target_Joint_Angle[Joint_ID] = __Target_Joint_Angle;
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
