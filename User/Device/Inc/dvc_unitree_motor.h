#ifndef DVC_UNITREE_MOTOR_H
#define DVC_UNITREE_MOTOR_H

#include "drv_uart.h"
#include "drv_math.h"

// 宇树电机在线状态
enum Enum_Unitree_Motor_Status
{
    Unitree_Motor_Status_DISABLE = 0,
    Unitree_Motor_Status_ENABLE,
};

// 宇树电机使能/失能状态
enum Enum_Unitree_Motor_Control_Status
{
    Unitree_Motor_Control_Status_DISABLE = 0,
    Unitree_Motor_Control_Status_ENABLE,
};

class Class_Unitree_Motor
{
public:
    void Init(Struct_UART_Manage_Object *__UART_Manage_Object, uint16_t __Node_ID = 0x0001);

    inline Enum_Unitree_Motor_Status Get_Unitree_Motor_Status();
    inline Enum_Unitree_Motor_Control_Status Get_Unitree_Motor_Control_Status();
    inline float Get_Now_Angle();
    inline float Get_Now_Omega();
    inline float Get_Now_Torque();
    inline float Get_Target_Angle();
    inline float Get_Target_Torque();

    inline void Set_Unitree_Motor_Control_Status(Enum_Unitree_Motor_Control_Status __Control_Status);
    inline void Set_Target_Angle(float __Target_Angle);
    inline void Set_Target_Torque(float __Target_Torque);

    void UART_RxCpltCallback(uint8_t *Rx_Data, uint16_t Length);
    void TIM_Alive_PeriodElapsedCallback();
    void TIM_Process_PeriodElapsedCallback();

protected:
    Struct_UART_Manage_Object *UART_Manage_Object = 0;
    uint16_t Node_ID = 0;
    Enum_Unitree_Motor_Status Unitree_Motor_Status = Unitree_Motor_Status_DISABLE;
    Enum_Unitree_Motor_Control_Status Unitree_Motor_Control_Status = Unitree_Motor_Control_Status_DISABLE;
    uint32_t Flag = 0;
    uint32_t Pre_Flag = 0;
    float Now_Angle = 0.0f;
    float Now_Omega = 0.0f;
    float Now_Torque = 0.0f;
    float Target_Angle = 0.0f;
    float Target_Torque = 0.0f;
};

inline Enum_Unitree_Motor_Status Class_Unitree_Motor::Get_Unitree_Motor_Status()
{
    return (Unitree_Motor_Status);
}

inline Enum_Unitree_Motor_Control_Status Class_Unitree_Motor::Get_Unitree_Motor_Control_Status()
{
    return (Unitree_Motor_Control_Status);
}

inline float Class_Unitree_Motor::Get_Now_Angle()
{
    return (Now_Angle);
}

inline float Class_Unitree_Motor::Get_Now_Omega()
{
    return (Now_Omega);
}

inline float Class_Unitree_Motor::Get_Now_Torque()
{
    return (Now_Torque);
}

inline float Class_Unitree_Motor::Get_Target_Angle()
{
    return (Target_Angle);
}

inline float Class_Unitree_Motor::Get_Target_Torque()
{
    return (Target_Torque);
}

inline void Class_Unitree_Motor::Set_Unitree_Motor_Control_Status(Enum_Unitree_Motor_Control_Status __Control_Status)
{
    Unitree_Motor_Control_Status = __Control_Status;
}

inline void Class_Unitree_Motor::Set_Target_Angle(float __Target_Angle)
{
    Target_Angle = __Target_Angle;
}

inline void Class_Unitree_Motor::Set_Target_Torque(float __Target_Torque)
{
    Target_Torque = __Target_Torque;
}

#endif
