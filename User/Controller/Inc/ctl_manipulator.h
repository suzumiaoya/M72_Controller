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

    void Output();
    void Update_Current_State();
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
