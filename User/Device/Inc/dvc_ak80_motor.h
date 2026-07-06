#ifndef DVC_AK80_MOTOR_H
#define DVC_AK80_MOTOR_H

/**
 * @file dvc_ak80_motor.h
 * @author cjw by yssickjgd
 * @brief AK80电机配置与操作
 * @version 0.1
 * @date 2025-07-1 0.1 26赛季定稿
 *
 * @copyright ZLLC 2026
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "drv_math.h"
#include "drv_can.h"
#include "alg_pid.h"
#include "alg_slope.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief AK电机状态
 *
 */
enum Enum_AK_Motor_Status
{
    AK_Motor_Status_DISABLE = 0,
    AK_Motor_Status_ENABLE,
};

/**
 * @brief AK电机的ID枚举类型
 *
 */
enum Enum_AK_Motor_ID : uint8_t
{
    AK_Motor_ID_0x01 = 0x01,
    AK_Motor_ID_0x02,
    AK_Motor_ID_0x03,
    AK_Motor_ID_0x04,
    AK_Motor_ID_0x05,
    AK_Motor_ID_0x06,
    AK_Motor_ID_0x07,
    AK_Motor_ID_0x08,
    AK_Motor_ID_0x09,
    AK_Motor_ID_0x10,
    AK_Motor_ID_0x11,
    AK_Motor_ID_0x12,
};

/**
 * @brief AK电机错误码
 *
 */
enum ERROR_STATUE_TYPE_T : uint8_t
{
    NONE_ERROR = 0,
    OVER_TEMPERATURE_ERROR,
    OVER_CURRENT_ERROR,
    OVER_VOLTAGE_ERROR,
    UNDER_VOLTAGE_ERROR,
    ENCODER_ERROR,
    CURRENT_UNBALANCE_ERROR,
};

/**
 * @brief AK电机控制状态
 *
 */
enum Enum_AK_Motor_Control_Status
{
    AK_Motor_Control_Status_DISABLE = 0,
    AK_Motor_Control_Status_ENABLE,
};

/**
 * @brief AK电机控制方式
 *
 */
enum Enum_AK_Motor_Control_Method
{
    AK_CONTROL_METHOD_MIT = 0,
};

/**
 * @brief AK电机经过处理后的数据
 *
 */
struct Struct_AK_Motor_Rx_Data
{
    Enum_AK_Motor_ID CAN_ID;
    float Now_Angle;
    float Now_Omega;
    float Now_Torque;
    float Now_Rotor_Temperature;
    ERROR_STATUE_TYPE_T error_statue;
    uint16_t Pre_Position;
    int32_t Total_Position;
    int32_t Total_Round;
};

/**
 * @brief AK80-6无刷电机
 *
 */
class Class_AK_Motor_80_6
{
public:
    // PID角度环控制
    Class_PID PID_Angle;
    // PID角速度环控制
    Class_PID PID_Omega;
    // 斜坡函数加减速速度X
    Class_Slope Slope_Joint_Angle;

    void Init(FDCAN_HandleTypeDef *hcan, Enum_AK_Motor_ID __CAN_ID, Enum_AK_Motor_Control_Method __Control_Method = AK_CONTROL_METHOD_MIT,
              float __MIT_K_P = 0.0f, float __MIT_K_D = 0.0f, int32_t __Position_Offset = 0,
              float __Angle_Max = 12.5f, float __Omega_Max = 76.0f, float __Torque_Max = 12.0f,
              float __Slope_Angle = 0.1f);

    inline Enum_AK_Motor_Control_Status Get_AK_Motor_Control_Status();
    inline Enum_AK_Motor_Status Get_AK_Motor_Status();
    inline float Get_Now_Angle();
    inline float Get_Now_Omega();
    inline float Get_Now_Torque();
    inline float Get_Now_Rotor_Temperature();
    inline Enum_AK_Motor_Control_Method Get_Control_Method();
    inline float Get_MIT_K_P();
    inline float Get_MIT_K_D();
    inline float Get_Target_Angle();
    inline float Get_Target_Omega();
    inline float Get_Target_Torque();
    inline float get_Max_Omega();

    inline void Set_AK_Control_Status(Enum_AK_Motor_Control_Status __AK_Motor_Control_Status);
    inline void Set_AK_Motor_Control_Method(Enum_AK_Motor_Control_Method __AK_Motor_Control_Method);
    inline void Set_MIT_K_P(float __MIT_K_P);
    inline void Set_MIT_K_D(float __MIT_K_D);
    inline void Set_Target_Angle(float __Target_Angle);
    inline void Set_Target_Omega(float __Target_Omega);
    inline void Set_Target_Torque(float __Target_Torque);

    void CAN_RxCpltCallback(uint8_t *Rx_Data);
    void Task_Alive_PeriodElapsedCallback();
    void Task_PID_PeriodElapsedCallback();
    void Task_Process_PeriodElapsedCallback();

protected:
    // 初始化相关变量

    // 绑定的CAN
    Struct_CAN_Manage_Object *CAN_Manage_Object = 0;
    // 收数据绑定的CAN ID
    Enum_AK_Motor_ID CAN_ID = AK_Motor_ID_0x01;
    // 发送缓冲区
    uint8_t *CAN_Tx_Data = 0;
    // 位置反馈偏移
    int32_t Position_Offset = 0;
    // 位置最大值
    float Angle_Max = 12.5f;
    // 最大速度
    float Omega_Max = 76.0f;
    // 最大扭矩
    float Torque_Max = 12.0f;

    // 当前时刻的电机接收Flag
    uint32_t Flag = 0;
    // 前一时刻的电机接收Flag
    uint32_t Pre_Flag = 0;

    // 电机状态
    Enum_AK_Motor_Status AK_Motor_Status = AK_Motor_Status_DISABLE;
    // 电机对外接口信息
    Struct_AK_Motor_Rx_Data Data = {AK_Motor_ID_0x01, 0.0f, 0.0f, 0.0f, 0.0f, NONE_ERROR, 0, 0, 0};

    // 电机控制状态
    Enum_AK_Motor_Control_Status AK_Motor_Control_Status = AK_Motor_Control_Status_DISABLE;
    // 上一拍的电机控制状态
    Enum_AK_Motor_Control_Status Pre_AK_Motor_Control_Status = AK_Motor_Control_Status_DISABLE;
    // 电机控制方式
    Enum_AK_Motor_Control_Method AK_Motor_Control_Method = AK_CONTROL_METHOD_MIT;
    // 上一拍的电机控制方式
    // MIT的Kp值
    float MIT_K_P = 12.0f;
    // MIT的Kd值
    float MIT_K_D = 0.8f;
    // 目标角度
    float Target_Angle = 0.0f;
    // 目标速度
    float Target_Omega = 0.0f;
    // 目标扭矩
    float Target_Torque = 0.0f;

    void Data_Process(uint8_t *Rx_Data);
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

Enum_AK_Motor_Control_Status Class_AK_Motor_80_6::Get_AK_Motor_Control_Status()
{
    return (AK_Motor_Control_Status);
}

Enum_AK_Motor_Status Class_AK_Motor_80_6::Get_AK_Motor_Status()
{
    return (AK_Motor_Status);
}

float Class_AK_Motor_80_6::Get_Now_Angle()
{
    return (Data.Now_Angle);
}

float Class_AK_Motor_80_6::Get_Now_Omega()
{
    return (Data.Now_Omega);
}

float Class_AK_Motor_80_6::Get_Now_Torque()
{
    return (Data.Now_Torque);
}

float Class_AK_Motor_80_6::Get_Now_Rotor_Temperature()
{
    return (Data.Now_Rotor_Temperature);
}

Enum_AK_Motor_Control_Method Class_AK_Motor_80_6::Get_Control_Method()
{
    return (AK_Motor_Control_Method);
}

float Class_AK_Motor_80_6::Get_MIT_K_P()
{
    return (MIT_K_P);
}

float Class_AK_Motor_80_6::Get_MIT_K_D()
{
    return (MIT_K_D);
}

float Class_AK_Motor_80_6::Get_Target_Angle()
{
    return (Target_Angle);
}

float Class_AK_Motor_80_6::Get_Target_Omega()
{
    return (Target_Omega);
}

float Class_AK_Motor_80_6::Get_Target_Torque()
{
    return (Target_Torque);
}

float Class_AK_Motor_80_6::get_Max_Omega()
{
    return (Omega_Max);
}

void Class_AK_Motor_80_6::Set_AK_Control_Status(Enum_AK_Motor_Control_Status __AK_Motor_Control_Status)
{
    AK_Motor_Control_Status = __AK_Motor_Control_Status;
}

void Class_AK_Motor_80_6::Set_AK_Motor_Control_Method(Enum_AK_Motor_Control_Method __Control_Method)
{
    UNUSED(__Control_Method);
    AK_Motor_Control_Method = AK_CONTROL_METHOD_MIT;
}

void Class_AK_Motor_80_6::Set_MIT_K_P(float __MIT_K_P)
{
    MIT_K_P = __MIT_K_P;
}

void Class_AK_Motor_80_6::Set_MIT_K_D(float __MIT_K_D)
{
    MIT_K_D = __MIT_K_D;
}

void Class_AK_Motor_80_6::Set_Target_Angle(float __Target_Angle)
{
    Target_Angle = __Target_Angle;
}

void Class_AK_Motor_80_6::Set_Target_Omega(float __Target_Omega)
{
    Target_Omega = __Target_Omega;
}

void Class_AK_Motor_80_6::Set_Target_Torque(float __Target_Torque)
{
    Target_Torque = __Target_Torque;
}

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
