// SPDX-License-Identifier: AGPL-3.0-only
/**
 * @file dvc_ak80_motor.cpp
 * @author hsl by suzumiaoya
 * @brief AK80电机配置与操作
 * @version 0.1
 * @date 2025-07-1 0.1 26赛季定稿
 *
 * @copyright ZLLC 2026
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "dvc_ak80_motor.h"

/* Private macros ------------------------------------------------------------*/

static constexpr float AK80_SERVO_GEAR_RATIO = 6.0f;
static constexpr float AK80_SERVO_POLE_PAIRS = 7.0f;
static constexpr float AK80_SERVO_POSITION_SCALE = 10000.0f;
static constexpr float AK80_SERVO_SPEED_SCALE = 10.0f;
static constexpr float AK80_SERVO_CURRENT_SCALE = 1000.0f;
static constexpr float AK80_SERVO_FEEDBACK_POSITION_SCALE = 0.1f;
static constexpr float AK80_SERVO_FEEDBACK_CURRENT_SCALE = 0.01f;
static constexpr float AK80_SERVO_POSITION_MAX_DEG = 36000.0f;
static constexpr float AK80_SERVO_CURRENT_PROTOCOL_MAX = 60.0f;

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

// 使能电机
uint8_t AK_Motor_CAN_Message_Enter[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc};
// 失能电机
uint8_t AK_Motor_CAN_Message_Exit[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd};
// 伺服模式失能时发送0A电流命令
uint8_t AK_Motor_CAN_Message_Zero_Current[4] = {0x00, 0x00, 0x00, 0x00};
// 保存当前位置为零点
uint8_t AK_Motor_CAN_Message_Save_Zero[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe};

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 分配CAN发送缓冲区
 *
 * @param hcan CAN编号
 * @param __CAN_ID CAN ID
 * @return uint8_t* 缓冲区指针
 */
uint8_t *allocate_tx_data(FDCAN_HandleTypeDef *hcan, Enum_AK_Motor_ID __CAN_ID)
{
    uint8_t *tmp_tx_data_ptr = 0;
    if (hcan == &hfdcan1)
    {
        switch (__CAN_ID)
        {
        case (AK_Motor_ID_0x01):
        {
            tmp_tx_data_ptr = CAN1_0xx01_Tx_Data;
        }
        break;
        case (AK_Motor_ID_0x02):
        {
            tmp_tx_data_ptr = CAN1_0xx02_Tx_Data;
        }
        break;
        case (AK_Motor_ID_0x03):
        {
            tmp_tx_data_ptr = CAN1_0xx03_Tx_Data;
        }
        break;
        case (AK_Motor_ID_0x04):
        {
            tmp_tx_data_ptr = CAN1_0xx04_Tx_Data;
        }
        break;
        case (AK_Motor_ID_0x05):
        {
            tmp_tx_data_ptr = CAN1_0xx05_Tx_Data;
        }
        break;
        case (AK_Motor_ID_0x06):
        {
            tmp_tx_data_ptr = CAN1_0xx06_Tx_Data;
        }
        break;
        case (AK_Motor_ID_0x07):
        {
            tmp_tx_data_ptr = CAN1_0xx07_Tx_Data;
        }
        break;
        case (AK_Motor_ID_0x08):
        {
            tmp_tx_data_ptr = CAN1_0xx08_Tx_Data;
        }
        break;
        default:
        {
        }
        break;
        }
    }
    else if (hcan == &hfdcan2)
    {
        switch (__CAN_ID)
        {
        case (AK_Motor_ID_0x01):
        {
            tmp_tx_data_ptr = CAN2_0xx01_Tx_Data;
        }
        break;
        case (AK_Motor_ID_0x02):
        {
            tmp_tx_data_ptr = CAN2_0xx02_Tx_Data;
        }
        break;
        case (AK_Motor_ID_0x03):
        {
            tmp_tx_data_ptr = CAN2_0xx03_Tx_Data;
        }
        break;
        case (AK_Motor_ID_0x04):
        {
            tmp_tx_data_ptr = CAN2_0xx04_Tx_Data;
        }
        break;
        case (AK_Motor_ID_0x05):
        {
            tmp_tx_data_ptr = CAN2_0xx05_Tx_Data;
        }
        break;
        case (AK_Motor_ID_0x06):
        {
            tmp_tx_data_ptr = CAN2_0xx06_Tx_Data;
        }
        break;
        case (AK_Motor_ID_0x07):
        {
            tmp_tx_data_ptr = CAN2_0xx07_Tx_Data;
        }
        break;
        case (AK_Motor_ID_0x08):
        {
            tmp_tx_data_ptr = CAN2_0xx08_Tx_Data;
        }
        break;
        default:
        {
        }
        break;
        }
    }
    return (tmp_tx_data_ptr);
}

/**
 * @brief 电机初始化
 *
 * @param hcan 绑定的CAN总线
 * @param __CAN_ID 绑定的CAN ID
 * @param __Control_Method 电机控制方式
 * @param __Position_Offset 位置反馈偏移
 * @param __Angle_Max 角度最大值
 * @param __Omega_Max 速度最大值
 * @param __Torque_Max 扭矩最大值
 */
void Class_AK_Motor_80_6::Init(FDCAN_HandleTypeDef *hcan, Enum_AK_Motor_ID __CAN_ID, Enum_AK_Motor_Control_Method __Control_Method, float __MIT_K_P, float __MIT_K_D,
                               int32_t __Position_Offset, float __Angle_Max, float __Omega_Max, float __Torque_Max, float __Slope_Angle, float __Current_Max)
{
    if (hcan->Instance == FDCAN1)
    {
        CAN_Manage_Object = &CAN1_Manage_Object;
    }
    else if (hcan->Instance == FDCAN2)
    {
        CAN_Manage_Object = &CAN2_Manage_Object;
    }

    CAN_ID = __CAN_ID;
    AK_Motor_Control_Method = __Control_Method;
    Position_Offset = __Position_Offset;
    Angle_Max = __Angle_Max;
    Omega_Max = __Omega_Max;
    Torque_Max = __Torque_Max;
    Current_Max = Math_Abs(__Current_Max);
    Math_Constrain(&Current_Max, 0.0f, AK80_SERVO_CURRENT_PROTOCOL_MAX);
    MIT_K_P = __MIT_K_P;
    MIT_K_D = __MIT_K_D;
    Target_Current = 0.0f;
    Target_Acceleration = 1.0f;
    Slope_Joint_Angle.Init(__Slope_Angle, __Slope_Angle);
    CAN_Tx_Data = allocate_tx_data(hcan, __CAN_ID);

    Flag = 0;
    Pre_Flag = 0;
    AK_Motor_Status = AK_Motor_Status_DISABLE;
    AK_Motor_Control_Status = AK_Motor_Control_Status_DISABLE;
    Pre_AK_Motor_Control_Status = AK_Motor_Control_Status_DISABLE;
    Clear_Enable_Confirm();
    Clear_Torque_Delta_Data();
    Data.CAN_ID = __CAN_ID;
    Data.Now_Angle = 0.0f;
    Data.Now_Omega = 0.0f;
    Data.Now_Torque = 0.0f;
    Data.Now_Current = 0.0f;
    Data.Now_Rotor_Temperature = 0.0f;
    Data.error_statue = NONE_ERROR;
    Data.Pre_Position = 0;
    Data.Total_Position = 0;
    Data.Total_Round = 0;
}

/**
 * @brief 数据处理过程
 *
 * @param Rx_Data 接收的数据
 */
void Class_AK_Motor_80_6::Data_Process(uint8_t *Rx_Data)
{
    uint8_t AK_Rx_Data[8];

    if (CAN_Manage_Object == 0)
    {
        return;
    }

    memcpy(AK_Rx_Data, Rx_Data, 8);

    switch (AK_Motor_Control_Method)
    {
    case (AK_CONTROL_METHOD_MIT):
    {
        if ((CAN_Manage_Object->Rx_Buffer.Header.IdType != FDCAN_STANDARD_ID) ||
            (CAN_Manage_Object->Rx_Buffer.Header.DataLength != FDCAN_DLC_BYTES_8))
        {
            return;
        }

        uint16_t tmp_position = (AK_Rx_Data[1] << 8) | (AK_Rx_Data[2]);
        uint16_t tmp_omega = (AK_Rx_Data[3] << 4) | (AK_Rx_Data[4] >> 4);
        uint16_t tmp_torque = ((AK_Rx_Data[4] & 0x0f) << 8) | (AK_Rx_Data[5]);

        Data.CAN_ID = (Enum_AK_Motor_ID)(AK_Rx_Data[0]);
        Data.Now_Angle = Math_Int_To_Float(tmp_position, 0, (1 << 16) - 1, -Angle_Max, Angle_Max);
        Data.Now_Omega = Math_Int_To_Float(tmp_omega, 0, (1 << 12) - 1, -Omega_Max, Omega_Max);
        Data.Now_Torque = Math_Int_To_Float(tmp_torque, 0, (1 << 12) - 1, -Torque_Max, Torque_Max);
        Data.Now_Current = 0.0f;
        Data.Now_Rotor_Temperature = AK_Rx_Data[6];
        Data.error_statue = (ERROR_STATUE_TYPE_T)(AK_Rx_Data[7]);
        Data.Pre_Position = tmp_position;
        Data.Total_Position = tmp_position + Position_Offset;
        Data.Total_Round = 0;

        if (AK_Motor_Control_Status == AK_Motor_Control_Status_ENABLE)
        {
            Torque_Delta_Data_Process();
        }
    }
    break;
    case (AK_CONTROL_METHOD_CURRENT):
    case (AK_CONTROL_METHOD_POSITION_OMEGA):
    {
        if ((CAN_Manage_Object->Rx_Buffer.Header.IdType != FDCAN_EXTENDED_ID) ||
            (CAN_Manage_Object->Rx_Buffer.Header.DataLength != FDCAN_DLC_BYTES_8) ||
            ((CAN_Manage_Object->Rx_Buffer.Header.Identifier & 0xffU) != static_cast<uint32_t>(CAN_ID)))
        {
            return;
        }

        int16_t tmp_position = static_cast<int16_t>((static_cast<uint16_t>(AK_Rx_Data[0]) << 8) | AK_Rx_Data[1]);
        int16_t tmp_omega = static_cast<int16_t>((static_cast<uint16_t>(AK_Rx_Data[2]) << 8) | AK_Rx_Data[3]);
        int16_t tmp_current = static_cast<int16_t>((static_cast<uint16_t>(AK_Rx_Data[4]) << 8) | AK_Rx_Data[5]);

        Data.CAN_ID = static_cast<Enum_AK_Motor_ID>(CAN_Manage_Object->Rx_Buffer.Header.Identifier & 0xffU);
        Data.Now_Angle = static_cast<float>(tmp_position) * AK80_SERVO_FEEDBACK_POSITION_SCALE * DEG_TO_RAD;
        Data.Now_Omega = static_cast<float>(tmp_omega) * AK80_SERVO_SPEED_SCALE * RPM_TO_RADPS /
                         (AK80_SERVO_GEAR_RATIO * AK80_SERVO_POLE_PAIRS);
        Data.Now_Torque = 0.0f;
        Data.Now_Current = static_cast<float>(tmp_current) * AK80_SERVO_FEEDBACK_CURRENT_SCALE;
        Data.Now_Rotor_Temperature = static_cast<float>(static_cast<int8_t>(AK_Rx_Data[6]));
        Data.error_statue = static_cast<ERROR_STATUE_TYPE_T>(AK_Rx_Data[7]);
        Data.Pre_Position = static_cast<uint16_t>(tmp_position);
        Data.Total_Position = static_cast<int32_t>(tmp_position) + Position_Offset;
        Data.Total_Round = 0;
    }
    break;
    default:
    {
    }
    break;
    }
}

/**
 * @brief CAN通信接收回调函数
 *
 * @param Rx_Data 接收的数据
 */
void Class_AK_Motor_80_6::CAN_RxCpltCallback(uint8_t *Rx_Data)
{
    Flag += 1;

    Data_Process(Rx_Data);
}

/**
 * @brief TIM定时器中断定期检测电机是否存活
 *
 */
void Class_AK_Motor_80_6::Task_Alive_PeriodElapsedCallback()
{
    if ((Flag == Pre_Flag) || (Data.error_statue != NONE_ERROR))
    {
        AK_Motor_Status = AK_Motor_Status_DISABLE;
    }
    else
    {
        AK_Motor_Status = AK_Motor_Status_ENABLE;
    }

    if (CAN_Manage_Object == 0)
    {
        Pre_Flag = Flag;
        return;
    }

    switch (AK_Motor_Control_Method)
    {
    case (AK_CONTROL_METHOD_MIT):
    {
        switch (AK_Motor_Control_Status)
        {
        case (AK_Motor_Control_Status_DISABLE):
        {
            CAN_Send_Data(CAN_Manage_Object->CAN_Handler, (uint16_t)CAN_ID, AK_Motor_CAN_Message_Exit, 8);
        }
        break;
        case (AK_Motor_Control_Status_ENABLE):
        {
            // 上升沿触发一次请求使能帧
            if (Pre_AK_Motor_Control_Status != AK_Motor_Control_Status_ENABLE)
            {
                CAN_Send_Data(CAN_Manage_Object->CAN_Handler, (uint16_t)CAN_ID, AK_Motor_CAN_Message_Enter, 8);
                break;
            }

            // 在设置为使能状态下检测电机是否已经使能，若未使能则重发请求使能帧
            if(Enable_Confirmed == 0)
            {
                CAN_Send_Data(CAN_Manage_Object->CAN_Handler, (uint16_t)CAN_ID, AK_Motor_CAN_Message_Enter, 8);
            }
        }
        break;
        default:
        {
        }
        break;
        }
    }
    break;
    case (AK_CONTROL_METHOD_CURRENT):
    case (AK_CONTROL_METHOD_POSITION_OMEGA):
    {
        if (AK_Motor_Control_Status == AK_Motor_Control_Status_DISABLE)
        {
            CAN_Send_Extended_Data(CAN_Manage_Object->CAN_Handler,
                                   (static_cast<uint32_t>(AK_CONTROL_METHOD_CURRENT) << 8) |
                                       static_cast<uint32_t>(CAN_ID),
                                   AK_Motor_CAN_Message_Zero_Current, 4);
        }
    }
    break;
    default:
    {
    }
    break;
    }

    Pre_AK_Motor_Control_Status = AK_Motor_Control_Status;
    Pre_Flag = Flag;
}

void Class_AK_Motor_80_6::Task_PID_PeriodElapsedCallback()
{
}

/**
 * @brief TIM定时器中断发送回调函数
 *
 */
void Class_AK_Motor_80_6::Task_Process_PeriodElapsedCallback()
{
    switch (AK_Motor_Control_Method)
    {
    case (AK_CONTROL_METHOD_MIT):
    {
        if ((AK_Motor_Control_Status == AK_Motor_Control_Status_DISABLE) || (CAN_Manage_Object == 0) || (CAN_Tx_Data == 0))
        {
            break;
        }

        uint16_t tmp_position = Math_Float_To_Int(Target_Angle, -Angle_Max, Angle_Max, 0, (1 << 16) - 1);
        uint16_t tmp_velocity = Math_Float_To_Int(Target_Omega, -Omega_Max, Omega_Max, 0, (1 << 12) - 1);
        uint16_t tmp_k_p = Math_Float_To_Int(MIT_K_P, 0.0f, 500.0f, 0, (1 << 12) - 1);
        uint16_t tmp_k_d = Math_Float_To_Int(MIT_K_D, 0.0f, 5.0f, 0, (1 << 12) - 1);
        uint16_t tmp_torque = Math_Float_To_Int(Target_Torque, -Torque_Max, Torque_Max, 0, (1 << 12) - 1);

        Math_Endian_Reverse_16(&tmp_position);
        memcpy(&CAN_Tx_Data[0], &tmp_position, sizeof(uint16_t));

        uint8_t tmp_velocity_11_4 = tmp_velocity >> 4;
        memcpy(&CAN_Tx_Data[2], &tmp_velocity_11_4, sizeof(uint8_t));

        uint8_t tmp_velocity_3_0_k_p_11_8 = ((tmp_velocity & 0x0f) << 4) | (tmp_k_p >> 8);
        memcpy(&CAN_Tx_Data[3], &tmp_velocity_3_0_k_p_11_8, sizeof(uint8_t));

        uint8_t tmp_k_p_7_0 = tmp_k_p;
        memcpy(&CAN_Tx_Data[4], &tmp_k_p_7_0, sizeof(uint8_t));

        uint8_t tmp_k_d_11_4 = tmp_k_d >> 4;
        memcpy(&CAN_Tx_Data[5], &tmp_k_d_11_4, sizeof(uint8_t));

        uint8_t tmp_k_d_3_0_torque_11_8 = ((tmp_k_d & 0x0f) << 4) | (tmp_torque >> 8);
        memcpy(&CAN_Tx_Data[6], &tmp_k_d_3_0_torque_11_8, sizeof(uint8_t));

        uint8_t tmp_torque_7_0 = tmp_torque;
        memcpy(&CAN_Tx_Data[7], &tmp_torque_7_0, sizeof(uint8_t));

        CAN_Send_Data(CAN_Manage_Object->CAN_Handler, (uint16_t)CAN_ID, CAN_Tx_Data, 8);
    }
    break;
    case (AK_CONTROL_METHOD_CURRENT):
    {
        if ((AK_Motor_Control_Status == AK_Motor_Control_Status_DISABLE) || (CAN_Manage_Object == 0) || (CAN_Tx_Data == 0))
        {
            break;
        }

        float tmp_current = Target_Current;
        Math_Constrain(&tmp_current, -Current_Max, Current_Max);
        int32_t tmp_current_raw = static_cast<int32_t>(tmp_current * AK80_SERVO_CURRENT_SCALE);
        Math_Endian_Reverse_32(&tmp_current_raw);
        memcpy(&CAN_Tx_Data[0], &tmp_current_raw, sizeof(int32_t));

        CAN_Send_Extended_Data(CAN_Manage_Object->CAN_Handler,
                               (static_cast<uint32_t>(AK_CONTROL_METHOD_CURRENT) << 8) | static_cast<uint32_t>(CAN_ID),
                               CAN_Tx_Data, 4);
    }
    break;
    case (AK_CONTROL_METHOD_POSITION_OMEGA):
    {
        if ((AK_Motor_Control_Status == AK_Motor_Control_Status_DISABLE) || (CAN_Manage_Object == 0) || (CAN_Tx_Data == 0))
        {
            break;
        }

        float tmp_position_deg = Target_Angle * RAD_TO_DEG;
        Math_Constrain(&tmp_position_deg, -AK80_SERVO_POSITION_MAX_DEG, AK80_SERVO_POSITION_MAX_DEG);
        int32_t tmp_position_raw = static_cast<int32_t>(tmp_position_deg * AK80_SERVO_POSITION_SCALE);
        Math_Endian_Reverse_32(&tmp_position_raw);
        memcpy(&CAN_Tx_Data[0], &tmp_position_raw, sizeof(int32_t));

        float tmp_speed_raw = Target_Omega * AK80_SERVO_GEAR_RATIO * AK80_SERVO_POLE_PAIRS / RPM_TO_RADPS / AK80_SERVO_SPEED_SCALE;
        Math_Constrain(&tmp_speed_raw, -32768.0f, 32767.0f);
        int16_t tmp_speed_raw_16 = static_cast<int16_t>(tmp_speed_raw);
        Math_Endian_Reverse_16(&tmp_speed_raw_16);
        memcpy(&CAN_Tx_Data[4], &tmp_speed_raw_16, sizeof(int16_t));

        float tmp_acceleration_raw = Math_Abs(Target_Acceleration) * AK80_SERVO_GEAR_RATIO * AK80_SERVO_POLE_PAIRS / RPM_TO_RADPS / AK80_SERVO_SPEED_SCALE;
        Math_Constrain(&tmp_acceleration_raw, 0.0f, 32767.0f);
        int16_t tmp_acceleration_raw_16 = static_cast<int16_t>(tmp_acceleration_raw);
        Math_Endian_Reverse_16(&tmp_acceleration_raw_16);
        memcpy(&CAN_Tx_Data[6], &tmp_acceleration_raw_16, sizeof(int16_t));

        CAN_Send_Extended_Data(CAN_Manage_Object->CAN_Handler,
                               (static_cast<uint32_t>(AK_CONTROL_METHOD_POSITION_OMEGA) << 8) | static_cast<uint32_t>(CAN_ID),
                               CAN_Tx_Data, 8);
    }
    break;
    default:
    {
    }
    break;
    }
}

void Class_AK_Motor_80_6::Clear_Enable_Confirm()
{
    Enable_Confirmed = 0;
    Enable_Confirm_Count = 0;
}

void Class_AK_Motor_80_6::Clear_Torque_Delta_Data()
{
    Torque_Delta_Data.Valid_Flag = 0;
    Torque_Delta_Data.Pre_Now_Torque = 0.0f;
    memset(Torque_Delta_Data.Delta_Buffer, 0, sizeof(Torque_Delta_Data.Delta_Buffer));
    Torque_Delta_Data.Delta_Index = 0;
    Torque_Delta_Data.Delta_Count = 0;
    Torque_Delta_Data.Delta_Sum = 0.0f;
    Torque_Delta_Data.Delta_Mean = 0.0f;
}

// 对电机返回的扭矩进行差分求均值，判断电机是否使能
void Class_AK_Motor_80_6::Torque_Delta_Data_Process()
{
    if (Torque_Delta_Data.Valid_Flag == 0)
    {
        Torque_Delta_Data.Pre_Now_Torque = Data.Now_Torque;
        Torque_Delta_Data.Valid_Flag = 1;
    }
    else
    {
        float tmp_delta = Math_Abs(Data.Now_Torque - Torque_Delta_Data.Pre_Now_Torque);
        Torque_Delta_Data.Pre_Now_Torque = Data.Now_Torque;

        if (Torque_Delta_Data.Delta_Count < 4)
        {
            Torque_Delta_Data.Delta_Buffer[Torque_Delta_Data.Delta_Index] = tmp_delta;
            Torque_Delta_Data.Delta_Sum += tmp_delta;
            Torque_Delta_Data.Delta_Count++;
        }
        else
        {
            Torque_Delta_Data.Delta_Sum -= Torque_Delta_Data.Delta_Buffer[Torque_Delta_Data.Delta_Index];
            Torque_Delta_Data.Delta_Buffer[Torque_Delta_Data.Delta_Index] = tmp_delta;
            Torque_Delta_Data.Delta_Sum += tmp_delta;
        }

        Torque_Delta_Data.Delta_Index++;
        if (Torque_Delta_Data.Delta_Index >= 4)
        {
            Torque_Delta_Data.Delta_Index = 0;
        }

        Torque_Delta_Data.Delta_Mean = Torque_Delta_Data.Delta_Sum / (float)Torque_Delta_Data.Delta_Count;

        if (Torque_Delta_Data.Delta_Count >= 4)
        {
            if (Torque_Delta_Data.Delta_Mean >= Enable_Delta_Threshold)
            {
                if (Enable_Confirm_Count < 2)
                {
                    Enable_Confirm_Count++;
                }
                if (Enable_Confirm_Count >= 2)
                {
                    Enable_Confirmed = 1;
                }
            }
            else
            {
                Enable_Confirm_Count = 0;
            }
        }
    }
}

/************************ COPYRIGHT(C) NEUQ-ZLLC **************************/
