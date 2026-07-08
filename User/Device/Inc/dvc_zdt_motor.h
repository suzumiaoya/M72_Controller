// SPDX-License-Identifier: AGPL-3.0-only
#ifndef DVC_ZDT_MOTOR_H
#define DVC_ZDT_MOTOR_H

/**
 * @file dvc_zdt_motor.h
 * @brief ZDT motor configuration and operation
 */

/* Includes ------------------------------------------------------------------*/

#include "drv_can.h"
#include "drv_math.h"

/* Exported macros -----------------------------------------------------------*/

#define ZDT_MOTOR_DEFAULT_MAX_TORQUE  (0.43f)
#define ZDT_MOTOR_DEFAULT_MAX_CURRENT (2.0f)
#define ZDT_MOTOR_DEFAULT_MAX_OMEGA   (314.15927f)
#define ZDT_MOTOR_DEFAULT_CURRENT_RAMP (20000.0f)

/* Exported types ------------------------------------------------------------*/

enum Enum_ZDT_Motor_Status
{
    ZDT_Motor_Status_DISABLE = 0,
    ZDT_Motor_Status_ENABLE,
};

enum Enum_ZDT_Motor_Control_Status
{
    ZDT_Motor_Control_Status_DISABLE = 0,
    ZDT_Motor_Control_Status_ENABLE,
};

enum Enum_ZDT_Motor_Control_Method
{
    ZDT_Motor_Control_Method_POSITION_OMEGA = 0,
    ZDT_Motor_Control_Method_OMEGA,
    ZDT_Motor_Control_Method_TORQUE,
};

struct Struct_ZDT_Motor_Rx_Data
{
    uint16_t CAN_ID;
    float Now_Angle;
    float Now_Omega;
    float Now_Torque;
    float Now_Current;
};

class Class_ZDT_Motor
{
public:
    void Init(FDCAN_HandleTypeDef *hcan, uint16_t __CAN_ID = 0x0001,
              Enum_ZDT_Motor_Control_Method __Control_Method = ZDT_Motor_Control_Method_POSITION_OMEGA,
              float __Max_Torque = ZDT_MOTOR_DEFAULT_MAX_TORQUE,
              float __Max_Current = ZDT_MOTOR_DEFAULT_MAX_CURRENT,
              float __Max_Omega = ZDT_MOTOR_DEFAULT_MAX_OMEGA);

    inline Enum_ZDT_Motor_Status Get_ZDT_Motor_Status();
    inline Enum_ZDT_Motor_Control_Status Get_ZDT_Motor_Control_Status();
    inline Enum_ZDT_Motor_Control_Method Get_ZDT_Motor_Control_Method();
    inline float Get_Now_Angle();
    inline float Get_Now_Omega();
    inline float Get_Now_Torque();
    inline float Get_Now_Current();
    inline float Get_Target_Angle();
    inline float Get_Target_Omega();
    inline float Get_Target_Torque();
    inline float Get_Target_Accel();
    inline float Get_Target_Current_Ramp();
    inline float Get_Max_Omega();
    inline uint8_t Get_Torque_Feedback_Enable();
    inline uint8_t Get_Torque_Feedback_Query_Divider();

    inline void Set_ZDT_Motor_Control_Status(Enum_ZDT_Motor_Control_Status __Control_Status);
    inline void Set_ZDT_Motor_Control_Method(Enum_ZDT_Motor_Control_Method __Control_Method);
    inline void Set_Target_Angle(float __Target_Angle);
    inline void Set_Target_Omega(float __Target_Omega);
    inline void Set_Target_Torque(float __Target_Torque);
    inline void Set_Target_Accel(float __Target_Accel);
    inline void Set_Target_Current_Ramp(float __Target_Current_Ramp);
    inline void Set_Torque_Feedback_Enable(uint8_t __Enable);
    inline void Set_Torque_Feedback_Query_Divider(uint8_t __Query_Divider);

    void CAN_RxCpltCallback(Struct_CAN_Rx_Buffer *CAN_RxMessage);
    void TIM_Alive_PeriodElapsedCallback();
    void TIM_Process_PeriodElapsedCallback();

protected:
    Struct_CAN_Manage_Object *CAN_Manage_Object = 0;
    uint16_t CAN_ID = 0x0001;

    Enum_ZDT_Motor_Status ZDT_Motor_Status = ZDT_Motor_Status_DISABLE;
    Enum_ZDT_Motor_Control_Status ZDT_Motor_Control_Status = ZDT_Motor_Control_Status_DISABLE;
    Enum_ZDT_Motor_Control_Status Pre_ZDT_Motor_Control_Status = ZDT_Motor_Control_Status_DISABLE;
    Enum_ZDT_Motor_Control_Method ZDT_Motor_Control_Method = ZDT_Motor_Control_Method_POSITION_OMEGA;

    uint32_t Flag = 0;
    uint32_t Pre_Flag = 0;
    uint32_t Position_Query_Counter = 0;
    uint32_t Last_Position_Query_Counter = 0;
    uint8_t Angle_Valid_Flag = 0;

    uint8_t Torque_Feedback_Enable = 0;
    uint8_t Torque_Feedback_Query_Divider = 1;
    uint8_t Torque_Feedback_Query_Counter = 0;

    float Max_Torque = ZDT_MOTOR_DEFAULT_MAX_TORQUE;
    float Max_Current = ZDT_MOTOR_DEFAULT_MAX_CURRENT;
    float Max_Omega = ZDT_MOTOR_DEFAULT_MAX_OMEGA;
    float Torque_Constant = ZDT_MOTOR_DEFAULT_MAX_TORQUE / ZDT_MOTOR_DEFAULT_MAX_CURRENT;

    Struct_ZDT_Motor_Rx_Data Data = {0x0001, 0.0f, 0.0f, 0.0f, 0.0f};

    float Target_Angle = 0.0f;
    float Target_Omega = 0.0f;
    float Target_Torque = 0.0f;
    float Target_Accel = 0.0f;
    float Target_Current_Ramp = ZDT_MOTOR_DEFAULT_CURRENT_RAMP;

    void Data_Process(const Struct_CAN_Rx_Buffer *CAN_RxMessage);
    void Send_Command(uint8_t *Command, uint16_t Length);
    void Send_Enable_Command();
    void Send_Stop_Command();
    void Send_Position_Command();
    void Send_Omega_Command();
    void Send_Torque_Command();
    void Send_Position_Query_Command();
    void Send_Torque_Query_Command();
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

inline Enum_ZDT_Motor_Status Class_ZDT_Motor::Get_ZDT_Motor_Status()
{
    return (ZDT_Motor_Status);
}

inline Enum_ZDT_Motor_Control_Status Class_ZDT_Motor::Get_ZDT_Motor_Control_Status()
{
    return (ZDT_Motor_Control_Status);
}

inline Enum_ZDT_Motor_Control_Method Class_ZDT_Motor::Get_ZDT_Motor_Control_Method()
{
    return (ZDT_Motor_Control_Method);
}

inline float Class_ZDT_Motor::Get_Now_Angle()
{
    return (Data.Now_Angle);
}

inline float Class_ZDT_Motor::Get_Now_Omega()
{
    return (Data.Now_Omega);
}

inline float Class_ZDT_Motor::Get_Now_Torque()
{
    return (Data.Now_Torque);
}

inline float Class_ZDT_Motor::Get_Now_Current()
{
    return (Data.Now_Current);
}

inline float Class_ZDT_Motor::Get_Target_Angle()
{
    return (Target_Angle);
}

inline float Class_ZDT_Motor::Get_Target_Omega()
{
    return (Target_Omega);
}

inline float Class_ZDT_Motor::Get_Target_Torque()
{
    return (Target_Torque);
}

inline float Class_ZDT_Motor::Get_Target_Accel()
{
    return (Target_Accel);
}

inline float Class_ZDT_Motor::Get_Target_Current_Ramp()
{
    return (Target_Current_Ramp);
}

inline float Class_ZDT_Motor::Get_Max_Omega()
{
    return (Max_Omega);
}

inline uint8_t Class_ZDT_Motor::Get_Torque_Feedback_Enable()
{
    return (Torque_Feedback_Enable);
}

inline uint8_t Class_ZDT_Motor::Get_Torque_Feedback_Query_Divider()
{
    return (Torque_Feedback_Query_Divider);
}

inline void Class_ZDT_Motor::Set_ZDT_Motor_Control_Status(Enum_ZDT_Motor_Control_Status __Control_Status)
{
    ZDT_Motor_Control_Status = __Control_Status;
}

inline void Class_ZDT_Motor::Set_ZDT_Motor_Control_Method(Enum_ZDT_Motor_Control_Method __Control_Method)
{
    ZDT_Motor_Control_Method = __Control_Method;
}

inline void Class_ZDT_Motor::Set_Target_Angle(float __Target_Angle)
{
    Target_Angle = __Target_Angle;
}

inline void Class_ZDT_Motor::Set_Target_Omega(float __Target_Omega)
{
    Target_Omega = __Target_Omega;
}

inline void Class_ZDT_Motor::Set_Target_Torque(float __Target_Torque)
{
    Target_Torque = __Target_Torque;
}

inline void Class_ZDT_Motor::Set_Target_Accel(float __Target_Accel)
{
    Target_Accel = __Target_Accel;
}

inline void Class_ZDT_Motor::Set_Target_Current_Ramp(float __Target_Current_Ramp)
{
    Target_Current_Ramp = __Target_Current_Ramp;
}

inline void Class_ZDT_Motor::Set_Torque_Feedback_Enable(uint8_t __Enable)
{
    Torque_Feedback_Enable = (__Enable == 0U) ? 0U : 1U;
}

inline void Class_ZDT_Motor::Set_Torque_Feedback_Query_Divider(uint8_t __Query_Divider)
{
    Torque_Feedback_Query_Divider = (__Query_Divider == 0U) ? 1U : __Query_Divider;
}

#endif
