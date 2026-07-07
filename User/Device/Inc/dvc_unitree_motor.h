#ifndef DVC_UNITREE_MOTOR_H
#define DVC_UNITREE_MOTOR_H

/**
 * @file dvc_unitree_motor.h
 * @brief Unitree GO-M8010-6 motor configuration and operation
 */

/* Includes ------------------------------------------------------------------*/

#include "drv_uart.h"
#include "drv_math.h"

/* Exported macros -----------------------------------------------------------*/

#define UNITREE_MOTOR_DEFAULT_GEAR_RATIO (6.33f)

/* Exported types ------------------------------------------------------------*/

enum Enum_Unitree_Motor_Status
{
    Unitree_Motor_Status_DISABLE = 0,
    Unitree_Motor_Status_ENABLE,
};

enum Enum_Unitree_Motor_Control_Status
{
    Unitree_Motor_Control_Status_DISABLE = 0,
    Unitree_Motor_Control_Status_ENABLE,
};

enum Enum_Unitree_Motor_Working_Status : uint8_t
{
    Unitree_Motor_Working_Status_LOCK = 0,
    Unitree_Motor_Working_Status_FOC = 1,
    Unitree_Motor_Working_Status_CALIBRATE = 2,
};

enum Enum_Unitree_Motor_Error_Status : uint8_t
{
    Unitree_Motor_Error_Status_NONE = 0,
    Unitree_Motor_Error_Status_OVER_TEMPERATURE,
    Unitree_Motor_Error_Status_OVER_CURRENT,
    Unitree_Motor_Error_Status_OVER_VOLTAGE,
    Unitree_Motor_Error_Status_ENCODER,
    Unitree_Motor_Error_Status_UNDER_VOLTAGE,
    Unitree_Motor_Error_Status_WINDING_OVER_TEMPERATURE,
    Unitree_Motor_Error_Status_RESERVED,
};

struct Struct_Unitree_Motor_Rx_Data
{
    uint16_t Node_ID;
    Enum_Unitree_Motor_Working_Status Working_Status;
    float Now_Angle;
    float Now_Omega;
    float Now_Torque;
    float Now_Rotor_Temperature;
    Enum_Unitree_Motor_Error_Status Error_Status;
};

class Class_Unitree_Motor
{
public:
    void Init(Struct_UART_Manage_Object *__UART_Manage_Object, uint16_t __Node_ID = 0x0001,
              float __Gear_Ratio = UNITREE_MOTOR_DEFAULT_GEAR_RATIO, float __MIT_K_P = 0.0f, float __MIT_K_D = 0.0f);

    inline Enum_Unitree_Motor_Status Get_Unitree_Motor_Status();
    inline Enum_Unitree_Motor_Control_Status Get_Unitree_Motor_Control_Status();
    inline Enum_Unitree_Motor_Working_Status Get_Working_Status();
    inline Enum_Unitree_Motor_Error_Status Get_Error_Status();
    inline float Get_Now_Angle();
    inline float Get_Now_Omega();
    inline float Get_Now_Torque();
    inline float Get_Now_Rotor_Temperature();
    inline float Get_Target_Angle();
    inline float Get_Target_Omega();
    inline float Get_Target_Torque();
    inline float Get_MIT_K_P();
    inline float Get_MIT_K_D();

    inline void Set_Unitree_Motor_Control_Status(Enum_Unitree_Motor_Control_Status __Control_Status);
    inline void Set_Target_Angle(float __Target_Angle);
    inline void Set_Target_Omega(float __Target_Omega);
    inline void Set_Target_Torque(float __Target_Torque);
    inline void Set_MIT_K_P(float __MIT_K_P);
    inline void Set_MIT_K_D(float __MIT_K_D);

    void UART_RxCpltCallback(uint8_t *Rx_Data, uint16_t Length);
    void TIM_Alive_PeriodElapsedCallback();
    void TIM_Process_PeriodElapsedCallback();

protected:
    Struct_UART_Manage_Object *UART_Manage_Object = 0;
    uint16_t Node_ID = 0;
    float Gear_Ratio = UNITREE_MOTOR_DEFAULT_GEAR_RATIO;

    Enum_Unitree_Motor_Status Unitree_Motor_Status = Unitree_Motor_Status_DISABLE;
    Enum_Unitree_Motor_Control_Status Unitree_Motor_Control_Status = Unitree_Motor_Control_Status_DISABLE;
    uint32_t Flag = 0;
    uint32_t Pre_Flag = 0;

    Struct_Unitree_Motor_Rx_Data Data = {0, Unitree_Motor_Working_Status_LOCK, 0.0f, 0.0f, 0.0f, 0.0f, Unitree_Motor_Error_Status_NONE};

    float MIT_K_P = 0.0f;
    float MIT_K_D = 0.0f;
    float Target_Angle = 0.0f;
    float Target_Omega = 0.0f;
    float Target_Torque = 0.0f;

    uint8_t Data_Process(uint8_t *Rx_Data);
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

inline Enum_Unitree_Motor_Status Class_Unitree_Motor::Get_Unitree_Motor_Status()
{
    return (Unitree_Motor_Status);
}

inline Enum_Unitree_Motor_Control_Status Class_Unitree_Motor::Get_Unitree_Motor_Control_Status()
{
    return (Unitree_Motor_Control_Status);
}

inline Enum_Unitree_Motor_Working_Status Class_Unitree_Motor::Get_Working_Status()
{
    return (Data.Working_Status);
}

inline Enum_Unitree_Motor_Error_Status Class_Unitree_Motor::Get_Error_Status()
{
    return (Data.Error_Status);
}

inline float Class_Unitree_Motor::Get_Now_Angle()
{
    return (Data.Now_Angle);
}

inline float Class_Unitree_Motor::Get_Now_Omega()
{
    return (Data.Now_Omega);
}

inline float Class_Unitree_Motor::Get_Now_Torque()
{
    return (Data.Now_Torque);
}

inline float Class_Unitree_Motor::Get_Now_Rotor_Temperature()
{
    return (Data.Now_Rotor_Temperature);
}

inline float Class_Unitree_Motor::Get_Target_Angle()
{
    return (Target_Angle);
}

inline float Class_Unitree_Motor::Get_Target_Omega()
{
    return (Target_Omega);
}

inline float Class_Unitree_Motor::Get_Target_Torque()
{
    return (Target_Torque);
}

inline float Class_Unitree_Motor::Get_MIT_K_P()
{
    return (MIT_K_P);
}

inline float Class_Unitree_Motor::Get_MIT_K_D()
{
    return (MIT_K_D);
}

inline void Class_Unitree_Motor::Set_Unitree_Motor_Control_Status(Enum_Unitree_Motor_Control_Status __Control_Status)
{
    Unitree_Motor_Control_Status = __Control_Status;
}

inline void Class_Unitree_Motor::Set_Target_Angle(float __Target_Angle)
{
    Target_Angle = __Target_Angle;
}

inline void Class_Unitree_Motor::Set_Target_Omega(float __Target_Omega)
{
    Target_Omega = __Target_Omega;
}

inline void Class_Unitree_Motor::Set_Target_Torque(float __Target_Torque)
{
    Target_Torque = __Target_Torque;
}

inline void Class_Unitree_Motor::Set_MIT_K_P(float __MIT_K_P)
{
    MIT_K_P = __MIT_K_P;
}

inline void Class_Unitree_Motor::Set_MIT_K_D(float __MIT_K_D)
{
    MIT_K_D = __MIT_K_D;
}

#endif
