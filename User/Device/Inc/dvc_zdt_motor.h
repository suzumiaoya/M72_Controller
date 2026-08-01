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
#include "alg_pid.h"

/* Exported macros -----------------------------------------------------------*/

#define ZDT_L40_MAX_TORQUE              (0.43f)
#define ZDT_L60_MAX_TORQUE              (0.7f)
#define ZDT_MOTOR_DEFAULT_MAX_CURRENT   (2.0f)
#define ZDT_MOTOR_DEFAULT_MAX_OMEGA     (314.15927f)
#define ZDT_MOTOR_DEFAULT_CURRENT_RAMP  (20000.0f)
#define ZDT_EMMX_PULSES_PER_REVOLUTION  (3200.0f)
#define ZDT_EMMX_MAX_RPM                (5000U)
#define ZDT_EMMX_DEFAULT_RPM            (100U)
#define ZDT_EMMX_DEFAULT_ACCELERATION   (10U)

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
    ZDT_Motor_Control_Method_TORQUE_MIT = 0,
    ZDT_Motor_Control_Method_TORQUE_OMEGA,
    ZDT_Motor_Control_Method_TORQUE_POSITION_OMEGA,
    ZDT_Motor_Control_Method_EMMX_POSITION,
};

enum Enum_ZDT_Motor_Query_Type
{
    ZDT_Motor_Query_Type_CURRENT = 0x27,
    ZDT_Motor_Query_Type_OMEGA = 0x35,
    ZDT_Motor_Query_Type_POSITION = 0x36,
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
    Class_PID PID_Angle;
    Class_PID PID_Omega;

    void Init(FDCAN_HandleTypeDef *hcan, uint16_t __CAN_ID = 0x0001,
              Enum_ZDT_Motor_Control_Method __Control_Method = ZDT_Motor_Control_Method_TORQUE_MIT,
              float __Max_Torque = ZDT_L40_MAX_TORQUE,
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
    inline float Get_Target_Current_Ramp();
    inline float Get_MIT_K_P();
    inline float Get_MIT_K_D();
    inline float Get_Max_Omega();

    inline void Set_ZDT_Motor_Control_Status(Enum_ZDT_Motor_Control_Status __Control_Status);
    inline void Set_ZDT_Motor_Control_Method(Enum_ZDT_Motor_Control_Method __Control_Method);
    inline void Set_Target_Angle(float __Target_Angle);
    inline void Set_Target_Omega(float __Target_Omega);
    inline void Set_Target_Torque(float __Target_Torque);
    inline void Set_Target_Current_Ramp(float __Target_Current_Ramp);
    inline void Set_MIT_K_P(float __MIT_K_P);
    inline void Set_MIT_K_D(float __MIT_K_D);
    inline void Set_Emmx_Position_Config(float __Pulses_Per_Revolution, uint16_t __Max_RPM,
                                         uint8_t __Acceleration, uint16_t __Default_RPM);

    void CAN_RxCpltCallback(Struct_CAN_Rx_Buffer *CAN_RxMessage);
    void TIM_Alive_PeriodElapsedCallback();
    void TIM_PID_PeriodElapsedCallback();
    uint8_t Send_Control_Command();
    uint8_t Send_Position_Command();
    uint8_t Send_Emmx_Position_Command();
    void Send_Query_Command(Enum_ZDT_Motor_Query_Type Query_Type);
    void Send_Timed_Query_Command(Enum_ZDT_Motor_Query_Type Query_Type, uint16_t Period_ms);

protected:
    Struct_CAN_Manage_Object *CAN_Manage_Object = 0;
    uint16_t CAN_ID = 0x0001;

    Enum_ZDT_Motor_Status ZDT_Motor_Status = ZDT_Motor_Status_DISABLE;
    Enum_ZDT_Motor_Control_Status ZDT_Motor_Control_Status = ZDT_Motor_Control_Status_DISABLE;
    Enum_ZDT_Motor_Control_Status Pre_ZDT_Motor_Control_Status = ZDT_Motor_Control_Status_DISABLE;
    Enum_ZDT_Motor_Control_Method ZDT_Motor_Control_Method = ZDT_Motor_Control_Method_TORQUE_MIT;
    Enum_ZDT_Motor_Control_Method Pre_ZDT_Motor_Control_Method = ZDT_Motor_Control_Method_TORQUE_MIT;

    uint32_t Flag = 0;
    uint32_t Pre_Flag = 0;
    uint8_t Angle_Valid_Flag = 0;
    uint8_t Omega_Valid_Flag = 0;
    uint16_t Position_Feedback_Period_ms = 0;
    uint8_t Emmx_Position_Command_Valid = 0U;

    float Max_Torque = ZDT_L40_MAX_TORQUE;
    float Max_Current = ZDT_MOTOR_DEFAULT_MAX_CURRENT;
    float Max_Omega = ZDT_MOTOR_DEFAULT_MAX_OMEGA;
    float Torque_Constant = ZDT_L40_MAX_TORQUE / ZDT_MOTOR_DEFAULT_MAX_CURRENT;

    Struct_ZDT_Motor_Rx_Data Data = {0x0001, 0.0f, 0.0f, 0.0f, 0.0f};

    float Target_Angle = 0.0f;
    float Target_Omega = 0.0f;
    float Target_Torque = 0.0f;
    float Target_Current_Ramp = ZDT_MOTOR_DEFAULT_CURRENT_RAMP;
    float MIT_K_P = 0.0f;
    float MIT_K_D = 0.0f;
    float Output_Torque = 0.0f;
    float Emmx_Pulses_Per_Revolution = ZDT_EMMX_PULSES_PER_REVOLUTION;
    uint16_t Emmx_Max_RPM = ZDT_EMMX_MAX_RPM;
    uint8_t Emmx_Acceleration = ZDT_EMMX_DEFAULT_ACCELERATION;
    uint16_t Emmx_Default_RPM = ZDT_EMMX_DEFAULT_RPM;
    float Emmx_Last_Target_Angle = 0.0f;

    void Data_Process(const Struct_CAN_Rx_Buffer *CAN_RxMessage);
    uint8_t Send_Command(uint8_t *Command, uint16_t Length);
    uint8_t Send_Torque_Command(float Torque);
    uint8_t Send_Emmx_Enable_Command(uint8_t Enable);
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

inline float Class_ZDT_Motor::Get_Target_Current_Ramp()
{
    return (Target_Current_Ramp);
}

inline float Class_ZDT_Motor::Get_MIT_K_P()
{
    return (MIT_K_P);
}

inline float Class_ZDT_Motor::Get_MIT_K_D()
{
    return (MIT_K_D);
}

inline float Class_ZDT_Motor::Get_Max_Omega()
{
    return (Max_Omega);
}

inline void Class_ZDT_Motor::Set_ZDT_Motor_Control_Status(Enum_ZDT_Motor_Control_Status __Control_Status)
{
    if (ZDT_Motor_Control_Status != __Control_Status)
    {
        PID_Angle.Set_Integral_Error(0.0f);
        PID_Omega.Set_Integral_Error(0.0f);
        Output_Torque = 0.0f;
    }
    ZDT_Motor_Control_Status = __Control_Status;
}

inline void Class_ZDT_Motor::Set_ZDT_Motor_Control_Method(Enum_ZDT_Motor_Control_Method __Control_Method)
{
    if (ZDT_Motor_Control_Method != __Control_Method)
    {
        PID_Angle.Set_Integral_Error(0.0f);
        PID_Omega.Set_Integral_Error(0.0f);
        Output_Torque = 0.0f;
        Omega_Valid_Flag = 0U;
        Emmx_Position_Command_Valid = 0U;
    }
    ZDT_Motor_Control_Method = __Control_Method;
    Pre_ZDT_Motor_Control_Method = __Control_Method;
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

inline void Class_ZDT_Motor::Set_Target_Current_Ramp(float __Target_Current_Ramp)
{
    Target_Current_Ramp = __Target_Current_Ramp;
}

inline void Class_ZDT_Motor::Set_MIT_K_P(float __MIT_K_P)
{
    MIT_K_P = __MIT_K_P;
}

inline void Class_ZDT_Motor::Set_MIT_K_D(float __MIT_K_D)
{
    MIT_K_D = __MIT_K_D;
}

inline void Class_ZDT_Motor::Set_Emmx_Position_Config(float __Pulses_Per_Revolution,
                                                       uint16_t __Max_RPM, uint8_t __Acceleration,
                                                       uint16_t __Default_RPM)
{
    Emmx_Pulses_Per_Revolution = __Pulses_Per_Revolution > 0.0f ? __Pulses_Per_Revolution : ZDT_EMMX_PULSES_PER_REVOLUTION;
    Emmx_Max_RPM = __Max_RPM > 0U ? __Max_RPM : ZDT_EMMX_MAX_RPM;
    Emmx_Acceleration = __Acceleration;
    Emmx_Default_RPM = __Default_RPM > 0U ? __Default_RPM : ZDT_EMMX_DEFAULT_RPM;
    Emmx_Position_Command_Valid = 0U;
}

#endif
