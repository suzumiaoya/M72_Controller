#ifndef DVC_AK80_MOTOR_H
#define DVC_AK80_MOTOR_H

#include "drv_can.h"
#include "drv_math.h"

enum Enum_AK80_Motor_Status
{
    AK80_Motor_Status_DISABLE = 0,
    AK80_Motor_Status_ENABLE,
};

enum Enum_AK80_Motor_Control_Status
{
    AK80_Motor_Control_Status_DISABLE = 0,
    AK80_Motor_Control_Status_ENABLE,
};

class Class_AK80_Motor
{
public:
    void Init(FDCAN_HandleTypeDef *hcan, uint16_t __CAN_ID = 0x0001);

    inline Enum_AK80_Motor_Status Get_AK80_Motor_Status();
    inline float Get_Now_Angle();
    inline float Get_Now_Omega();
    inline float Get_Now_Torque();
    inline float Get_Target_Angle();
    inline float Get_Target_Torque();

    inline void Set_AK80_Motor_Control_Status(Enum_AK80_Motor_Control_Status __Control_Status);
    inline void Set_Target_Angle(float __Target_Angle);
    inline void Set_Target_Torque(float __Target_Torque);

    void CAN_RxCpltCallback(uint8_t *Rx_Data);
    void TIM_Alive_PeriodElapsedCallback();
    void TIM_Process_PeriodElapsedCallback();

protected:
    Struct_CAN_Manage_Object *CAN_Manage_Object = 0;
    uint16_t CAN_ID = 0;
    Enum_AK80_Motor_Status AK80_Motor_Status = AK80_Motor_Status_DISABLE;
    Enum_AK80_Motor_Control_Status AK80_Motor_Control_Status = AK80_Motor_Control_Status_DISABLE;
    uint32_t Flag = 0;
    uint32_t Pre_Flag = 0;
    float Now_Angle = 0.0f;
    float Now_Omega = 0.0f;
    float Now_Torque = 0.0f;
    float Target_Angle = 0.0f;
    float Target_Torque = 0.0f;
};

inline Enum_AK80_Motor_Status Class_AK80_Motor::Get_AK80_Motor_Status()
{
    return (AK80_Motor_Status);
}

inline float Class_AK80_Motor::Get_Now_Angle()
{
    return (Now_Angle);
}

inline float Class_AK80_Motor::Get_Now_Omega()
{
    return (Now_Omega);
}

inline float Class_AK80_Motor::Get_Now_Torque()
{
    return (Now_Torque);
}

inline float Class_AK80_Motor::Get_Target_Angle()
{
    return (Target_Angle);
}

inline float Class_AK80_Motor::Get_Target_Torque()
{
    return (Target_Torque);
}

inline void Class_AK80_Motor::Set_AK80_Motor_Control_Status(Enum_AK80_Motor_Control_Status __Control_Status)
{
    AK80_Motor_Control_Status = __Control_Status;
}

inline void Class_AK80_Motor::Set_Target_Angle(float __Target_Angle)
{
    Target_Angle = __Target_Angle;
}

inline void Class_AK80_Motor::Set_Target_Torque(float __Target_Torque)
{
    Target_Torque = __Target_Torque;
}

#endif
