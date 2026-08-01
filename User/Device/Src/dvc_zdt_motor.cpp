// SPDX-License-Identifier: AGPL-3.0-only
/**
 * @file dvc_zdt_motor.cpp
 * @brief ZDT motor configuration and operation
 */

/* Includes ------------------------------------------------------------------*/

#include "dvc_zdt_motor.h"
#include "dvc_dwt.h"

/* Private macros ------------------------------------------------------------*/

#define ZDT_MOTOR_TORQUE_COMMAND          (0xF5U)
#define ZDT_MOTOR_POSITION_COMMAND        (0xFBU)
#define ZDT_MOTOR_POSITION_LIMIT_COMMAND  (0xCBU)
#define ZDT_MOTOR_TIMED_QUERY_COMMAND     (0x11U)
#define ZDT_EMMX_ENABLE_COMMAND            (0xF3U)
#define ZDT_EMMX_ENABLE_AUX_CODE           (0xABU)
#define ZDT_EMMX_POSITION_COMMAND          (0xFDU)

#define ZDT_MOTOR_TIMED_QUERY_AUX_CODE    (0x18U)
#define ZDT_MOTOR_CHECKSUM                (0x6BU)
#define ZDT_MOTOR_ABSOLUTE_POSITION       (0x01U)
#define ZDT_MOTOR_RELATIVE_CURRENT_POS    (0x02U)
#define ZDT_MOTOR_SYNC_DISABLE            (0x00U)

#define ZDT_MOTOR_PID_PERIOD              (0.001f)

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

static uint16_t ZDT_Motor_Get_Rx_Length(const FDCAN_RxHeaderTypeDef *Header);
static uint16_t ZDT_Motor_Current_A_To_mA(float Current_A);
static uint16_t ZDT_Motor_Omega_Radps_To_Raw(float Omega_Radps);
static uint16_t ZDT_Motor_Current_Ramp_To_Raw(float Current_Ramp);
static uint32_t ZDT_Motor_Position_Rad_To_Raw(float Position_Rad);
static uint16_t ZDT_Motor_Torque_To_Current_mA(float Torque, float Torque_Constant, float Max_Current);
static uint16_t ZDT_Motor_Omega_Radps_To_RPM(float Omega_Radps);
static uint32_t ZDT_Motor_Position_Rad_To_Pulses(float Position_Rad, float Pulses_Per_Revolution);

/* Function prototypes -------------------------------------------------------*/

void Class_ZDT_Motor::Init(FDCAN_HandleTypeDef *hcan, uint16_t __CAN_ID,
                           Enum_ZDT_Motor_Control_Method __Control_Method,
                           float __Max_Torque, float __Max_Current, float __Max_Omega)
{
    if (hcan->Instance == FDCAN1)
    {
        CAN_Manage_Object = &CAN1_Manage_Object;
    }
    else if (hcan->Instance == FDCAN2)
    {
        CAN_Manage_Object = &CAN2_Manage_Object;
    }
    else
    {
        CAN_Manage_Object = 0;
    }

    CAN_ID = __CAN_ID;
    ZDT_Motor_Status = ZDT_Motor_Status_DISABLE;
    ZDT_Motor_Control_Status = ZDT_Motor_Control_Status_DISABLE;
    Pre_ZDT_Motor_Control_Status = ZDT_Motor_Control_Status_DISABLE;
    ZDT_Motor_Control_Method = __Control_Method;
    Pre_ZDT_Motor_Control_Method = __Control_Method;

    Flag = 0;
    Pre_Flag = 0;
    Angle_Valid_Flag = 0;
    Omega_Valid_Flag = 0;
    Position_Feedback_Period_ms = 0;
    Emmx_Position_Command_Valid = 0U;

    Max_Torque = Math_Abs(__Max_Torque);
    if (Max_Torque <= FLT_EPSILON)
    {
        Max_Torque = ZDT_L40_MAX_TORQUE;
    }

    Max_Current = Math_Abs(__Max_Current);
    if (Max_Current <= FLT_EPSILON)
    {
        Max_Current = ZDT_MOTOR_DEFAULT_MAX_CURRENT;
    }

    Max_Omega = Math_Abs(__Max_Omega);
    if (Max_Omega <= FLT_EPSILON)
    {
        Max_Omega = ZDT_MOTOR_DEFAULT_MAX_OMEGA;
    }

    Torque_Constant = Max_Torque / Max_Current;

    Data.CAN_ID = CAN_ID;
    Data.Now_Angle = 0.0f;
    Data.Now_Omega = 0.0f;
    Data.Now_Torque = 0.0f;
    Data.Now_Current = 0.0f;

    Target_Angle = 0.0f;
    Target_Omega = 0.0f;
    Target_Torque = 0.0f;
    Target_Current_Ramp = ZDT_MOTOR_DEFAULT_CURRENT_RAMP;
    MIT_K_P = 0.0f;
    MIT_K_D = 0.0f;
    Output_Torque = 0.0f;
    Emmx_Pulses_Per_Revolution = ZDT_EMMX_PULSES_PER_REVOLUTION;
    Emmx_Max_RPM = ZDT_EMMX_MAX_RPM;
    Emmx_Acceleration = ZDT_EMMX_DEFAULT_ACCELERATION;
    Emmx_Default_RPM = ZDT_EMMX_DEFAULT_RPM;
    Emmx_Last_Target_Angle = 0.0f;

    PID_Angle.Init(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, Max_Omega,
                   0.0f, 0.0f, 0.0f, ZDT_MOTOR_PID_PERIOD);
    PID_Omega.Init(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, Max_Torque,
                   0.0f, 0.0f, 0.0f, ZDT_MOTOR_PID_PERIOD);
}

uint8_t Class_ZDT_Motor::Send_Command(uint8_t *Command, uint16_t Length)
{
    if ((CAN_Manage_Object == 0) || (CAN_Manage_Object->CAN_Handler == 0) ||
        (Command == 0) || (Length < 3U))
    {
        return (uint8_t)HAL_ERROR;
    }

    uint8_t tmp_addr = Command[0];
    uint8_t tmp_function = Command[1];
    uint8_t *tmp_payload = &Command[2];
    uint16_t tmp_payload_length = Length - 2U;
    uint32_t tmp_pack_index = 0U;
    uint32_t tmp_pack_count = (tmp_payload_length + 6U) / 7U;

    if (HAL_FDCAN_GetTxFifoFreeLevel(CAN_Manage_Object->CAN_Handler) < tmp_pack_count)
    {
        return (uint8_t)HAL_BUSY;
    }

    while (tmp_payload_length > 0U)
    {
        uint8_t tmp_tx_data[8] = {0};
        uint16_t tmp_copy_length = (tmp_payload_length > 7U) ? 7U : tmp_payload_length;

        tmp_tx_data[0] = tmp_function;
        memcpy(&tmp_tx_data[1], tmp_payload, tmp_copy_length);

        uint8_t tmp_status = CAN_Send_Extended_Data(CAN_Manage_Object->CAN_Handler,
                                                    ((static_cast<uint32_t>(tmp_addr) & 0xffU) << 8) | tmp_pack_index,
                                                    tmp_tx_data, tmp_copy_length + 1U);
        if (tmp_status != (uint8_t)HAL_OK)
        {
            return tmp_status;
        }

        tmp_payload += tmp_copy_length;
        tmp_payload_length -= tmp_copy_length;
        tmp_pack_index++;
    }

    return (uint8_t)HAL_OK;
}

uint8_t Class_ZDT_Motor::Send_Position_Command()
{
    if (Angle_Valid_Flag == 0U)
    {
        return (uint8_t)HAL_ERROR;
    }

    float tmp_delta_angle = Target_Angle - Data.Now_Angle;
    float tmp_target_omega = Math_Abs(Target_Omega);
    Math_Constrain(&tmp_target_omega, 0.0f, Max_Omega);

    uint8_t tmp_direction = (tmp_delta_angle >= 0.0f) ? 0U : 1U;
    uint16_t tmp_velocity = ZDT_Motor_Omega_Radps_To_Raw(tmp_target_omega);
    uint32_t tmp_position = ZDT_Motor_Position_Rad_To_Raw(tmp_delta_angle);
    uint16_t tmp_limit_current = ZDT_Motor_Torque_To_Current_mA(Target_Torque, Torque_Constant, Max_Current);

    if (Target_Torque == 0.0f)
    {
        uint8_t tmp_command[12] = {
            static_cast<uint8_t>(CAN_ID),
            ZDT_MOTOR_POSITION_COMMAND,
            tmp_direction,
            static_cast<uint8_t>(tmp_velocity >> 8),
            static_cast<uint8_t>(tmp_velocity),
            static_cast<uint8_t>(tmp_position >> 24),
            static_cast<uint8_t>(tmp_position >> 16),
            static_cast<uint8_t>(tmp_position >> 8),
            static_cast<uint8_t>(tmp_position),
            ZDT_MOTOR_RELATIVE_CURRENT_POS,
            ZDT_MOTOR_SYNC_DISABLE,
            ZDT_MOTOR_CHECKSUM,
        };

        return Send_Command(tmp_command, sizeof(tmp_command));
    }

    uint8_t tmp_command[14] = {
        static_cast<uint8_t>(CAN_ID),
        ZDT_MOTOR_POSITION_LIMIT_COMMAND,
        tmp_direction,
        static_cast<uint8_t>(tmp_velocity >> 8),
        static_cast<uint8_t>(tmp_velocity),
        static_cast<uint8_t>(tmp_position >> 24),
        static_cast<uint8_t>(tmp_position >> 16),
        static_cast<uint8_t>(tmp_position >> 8),
        static_cast<uint8_t>(tmp_position),
        ZDT_MOTOR_RELATIVE_CURRENT_POS,
        ZDT_MOTOR_SYNC_DISABLE,
        static_cast<uint8_t>(tmp_limit_current >> 8),
        static_cast<uint8_t>(tmp_limit_current),
        ZDT_MOTOR_CHECKSUM,
    };

    return Send_Command(tmp_command, sizeof(tmp_command));
}

uint8_t Class_ZDT_Motor::Send_Emmx_Enable_Command(uint8_t Enable)
{
    uint8_t tmp_command[6] = {
        static_cast<uint8_t>(CAN_ID),
        ZDT_EMMX_ENABLE_COMMAND,
        ZDT_EMMX_ENABLE_AUX_CODE,
        static_cast<uint8_t>(Enable != 0U),
        ZDT_MOTOR_SYNC_DISABLE,
        ZDT_MOTOR_CHECKSUM,
    };

    return Send_Command(tmp_command, sizeof(tmp_command));
}

uint8_t Class_ZDT_Motor::Send_Emmx_Position_Command()
{
    float tmp_pulse_resolution = Emmx_Pulses_Per_Revolution;
    if (tmp_pulse_resolution <= FLT_EPSILON)
    {
        return static_cast<uint8_t>(HAL_ERROR);
    }

    float tmp_command_epsilon = PI / tmp_pulse_resolution;
    if ((Emmx_Position_Command_Valid != 0U) &&
        (Math_Abs(Target_Angle - Emmx_Last_Target_Angle) < tmp_command_epsilon))
    {
        return static_cast<uint8_t>(HAL_OK);
    }

    uint8_t tmp_direction = Target_Angle >= 0.0f ? 0U : 1U;
    uint32_t tmp_pulses =
        ZDT_Motor_Position_Rad_To_Pulses(Target_Angle, tmp_pulse_resolution);

    float tmp_target_omega = Math_Abs(Target_Omega);
    uint16_t tmp_rpm = tmp_target_omega <= FLT_EPSILON ? Emmx_Default_RPM :
                       ZDT_Motor_Omega_Radps_To_RPM(tmp_target_omega);
    if (tmp_rpm == 0U)
    {
        tmp_rpm = 1U;
    }
    if (tmp_rpm > Emmx_Max_RPM)
    {
        tmp_rpm = Emmx_Max_RPM;
    }

    uint8_t tmp_command[13] = {
        static_cast<uint8_t>(CAN_ID),
        ZDT_EMMX_POSITION_COMMAND,
        tmp_direction,
        static_cast<uint8_t>(tmp_rpm >> 8),
        static_cast<uint8_t>(tmp_rpm),
        Emmx_Acceleration,
        static_cast<uint8_t>(tmp_pulses >> 24),
        static_cast<uint8_t>(tmp_pulses >> 16),
        static_cast<uint8_t>(tmp_pulses >> 8),
        static_cast<uint8_t>(tmp_pulses),
        ZDT_MOTOR_ABSOLUTE_POSITION,
        ZDT_MOTOR_SYNC_DISABLE,
        ZDT_MOTOR_CHECKSUM,
    };

    uint8_t tmp_status = Send_Command(tmp_command, sizeof(tmp_command));
    if (tmp_status == static_cast<uint8_t>(HAL_OK))
    {
        Emmx_Last_Target_Angle = Target_Angle;
        Emmx_Position_Command_Valid = 1U;
    }

    return tmp_status;
}

uint8_t Class_ZDT_Motor::Send_Torque_Command(float Torque)
{
    float tmp_current = 0.0f;

    if (Torque_Constant > FLT_EPSILON)
    {
        tmp_current = Torque / Torque_Constant;
    }

    uint8_t tmp_direction = (tmp_current >= 0.0f) ? 0U : 1U;
    tmp_current = Math_Abs(tmp_current);
    Math_Constrain(&tmp_current, 0.0f, Max_Current);

    uint16_t tmp_ramp = ZDT_Motor_Current_Ramp_To_Raw(Target_Current_Ramp);
    uint16_t tmp_current_mA = ZDT_Motor_Current_A_To_mA(tmp_current);

    uint8_t tmp_command[9] = {
        static_cast<uint8_t>(CAN_ID),
        ZDT_MOTOR_TORQUE_COMMAND,
        tmp_direction,
        static_cast<uint8_t>(tmp_ramp >> 8),
        static_cast<uint8_t>(tmp_ramp),
        static_cast<uint8_t>(tmp_current_mA >> 8),
        static_cast<uint8_t>(tmp_current_mA),
        ZDT_MOTOR_SYNC_DISABLE,
        ZDT_MOTOR_CHECKSUM,
    };

    return Send_Command(tmp_command, sizeof(tmp_command));
}

uint8_t Class_ZDT_Motor::Send_Control_Command()
{
    if (ZDT_Motor_Control_Method == ZDT_Motor_Control_Method_EMMX_POSITION)
    {
        if (Pre_ZDT_Motor_Control_Status != ZDT_Motor_Control_Status)
        {
            uint8_t tmp_status = Send_Emmx_Enable_Command(
                ZDT_Motor_Control_Status == ZDT_Motor_Control_Status_ENABLE ? 1U : 0U);
            if (tmp_status == static_cast<uint8_t>(HAL_OK))
            {
                Pre_ZDT_Motor_Control_Status = ZDT_Motor_Control_Status;
                Emmx_Position_Command_Valid = 0U;
            }
            return tmp_status;
        }

        if (ZDT_Motor_Control_Status == ZDT_Motor_Control_Status_DISABLE)
        {
            return static_cast<uint8_t>(HAL_OK);
        }

        return Send_Emmx_Position_Command();
    }

    float tmp_output_torque = (ZDT_Motor_Control_Status == ZDT_Motor_Control_Status_ENABLE) ?
                              Output_Torque : 0.0f;
    return Send_Torque_Command(tmp_output_torque);
}

void Class_ZDT_Motor::Send_Query_Command(Enum_ZDT_Motor_Query_Type Query_Type)
{
    switch (Query_Type)
    {
        case (ZDT_Motor_Query_Type_CURRENT):
        case (ZDT_Motor_Query_Type_OMEGA):
        case (ZDT_Motor_Query_Type_POSITION):
        {
        }
        break;
        default:
        {
            return;
        }
    }

    uint8_t tmp_command[3] = {
        static_cast<uint8_t>(CAN_ID),
        static_cast<uint8_t>(Query_Type),
        ZDT_MOTOR_CHECKSUM,
    };

    (void)Send_Command(tmp_command, sizeof(tmp_command));
}

void Class_ZDT_Motor::Send_Timed_Query_Command(Enum_ZDT_Motor_Query_Type Query_Type, uint16_t Period_ms)
{
    switch (Query_Type)
    {
        case (ZDT_Motor_Query_Type_CURRENT):
        case (ZDT_Motor_Query_Type_OMEGA):
        case (ZDT_Motor_Query_Type_POSITION):
        {
        }
        break;
        default:
        {
            return;
        }
    }

    uint8_t tmp_command[7] = {
        static_cast<uint8_t>(CAN_ID),
        ZDT_MOTOR_TIMED_QUERY_COMMAND,
        ZDT_MOTOR_TIMED_QUERY_AUX_CODE,
        static_cast<uint8_t>(Query_Type),
        static_cast<uint8_t>(Period_ms >> 8),
        static_cast<uint8_t>(Period_ms),
        ZDT_MOTOR_CHECKSUM,
    };

    uint8_t tmp_status = Send_Command(tmp_command, sizeof(tmp_command));
    if ((tmp_status == (uint8_t)HAL_OK) && (Query_Type == ZDT_Motor_Query_Type_POSITION))
    {
        Position_Feedback_Period_ms = Period_ms;
    }
}

void Class_ZDT_Motor::Data_Process(const Struct_CAN_Rx_Buffer *CAN_RxMessage)
{
    if ((CAN_RxMessage == 0) ||
        (CAN_RxMessage->Header.IdType != FDCAN_EXTENDED_ID) ||
        ((CAN_RxMessage->Header.Identifier >> 8) != CAN_ID) ||
        ((CAN_RxMessage->Header.Identifier & 0xffU) != 0U))
    {
        return;
    }

    uint16_t tmp_length = ZDT_Motor_Get_Rx_Length(&CAN_RxMessage->Header);
    if (tmp_length == 0U)
    {
        return;
    }

    switch (CAN_RxMessage->Data[0])
    {
        case (ZDT_Motor_Query_Type_CURRENT):
        {
            if ((tmp_length < 4U) || (CAN_RxMessage->Data[3] != ZDT_MOTOR_CHECKSUM))
            {
                return;
            }

            uint16_t tmp_current_raw = (static_cast<uint16_t>(CAN_RxMessage->Data[1]) << 8) |
                                       static_cast<uint16_t>(CAN_RxMessage->Data[2]);
            Data.Now_Current = static_cast<float>(tmp_current_raw) / 1000.0f;
            Data.Now_Torque = Data.Now_Current * Torque_Constant;
            Flag++;
        }
        break;

        case (ZDT_Motor_Query_Type_OMEGA):
        {
            if ((tmp_length < 5U) || (CAN_RxMessage->Data[4] != ZDT_MOTOR_CHECKSUM))
            {
                return;
            }

            uint16_t tmp_omega_raw = (static_cast<uint16_t>(CAN_RxMessage->Data[2]) << 8) |
                                     static_cast<uint16_t>(CAN_RxMessage->Data[3]);
            float tmp_omega = static_cast<float>(tmp_omega_raw) * 0.1f * RPM_TO_RADPS;
            if (CAN_RxMessage->Data[1] != 0U)
            {
                tmp_omega = -tmp_omega;
            }

            Data.Now_Omega = tmp_omega;
            Omega_Valid_Flag = 1U;
            Flag++;
        }
        break;

        case (ZDT_Motor_Query_Type_POSITION):
        {
            if ((tmp_length < 7U) || (CAN_RxMessage->Data[6] != ZDT_MOTOR_CHECKSUM))
            {
                return;
            }

            uint32_t tmp_position_raw = (static_cast<uint32_t>(CAN_RxMessage->Data[2]) << 24) |
                                        (static_cast<uint32_t>(CAN_RxMessage->Data[3]) << 16) |
                                        (static_cast<uint32_t>(CAN_RxMessage->Data[4]) << 8) |
                                        static_cast<uint32_t>(CAN_RxMessage->Data[5]);
            float tmp_angle = (static_cast<float>(tmp_position_raw) / 10.0f) * DEG_TO_RAD;
            if (CAN_RxMessage->Data[1] != 0U)
            {
                tmp_angle = -tmp_angle;
            }

            if ((Angle_Valid_Flag != 0U) &&
                (Omega_Valid_Flag == 0U) &&
                (Position_Feedback_Period_ms != 0U))
            {
                Data.Now_Omega = (tmp_angle - Data.Now_Angle) /
                                 (static_cast<float>(Position_Feedback_Period_ms) * 0.001f);
            }

            Data.Now_Angle = tmp_angle;
            Angle_Valid_Flag = 1U;
            Flag++;
        }
        break;

        default:
        {
        }
        break;
    }
}

void Class_ZDT_Motor::CAN_RxCpltCallback(Struct_CAN_Rx_Buffer *CAN_RxMessage)
{
    Data_Process(CAN_RxMessage);
}

void Class_ZDT_Motor::TIM_Alive_PeriodElapsedCallback()
{
    if (Flag == Pre_Flag)
    {
        ZDT_Motor_Status = ZDT_Motor_Status_DISABLE;
    }
    else
    {
        ZDT_Motor_Status = ZDT_Motor_Status_ENABLE;
    }

    Pre_Flag = Flag;
}

void Class_ZDT_Motor::TIM_PID_PeriodElapsedCallback()
{
    if (Pre_ZDT_Motor_Control_Method != ZDT_Motor_Control_Method)
    {
        PID_Angle.Set_Integral_Error(0.0f);
        PID_Omega.Set_Integral_Error(0.0f);
        Output_Torque = 0.0f;
        Pre_ZDT_Motor_Control_Method = ZDT_Motor_Control_Method;
    }

    if (ZDT_Motor_Control_Status == ZDT_Motor_Control_Status_DISABLE)
    {
        PID_Angle.Set_Integral_Error(0.0f);
        PID_Omega.Set_Integral_Error(0.0f);
        Output_Torque = 0.0f;
        return;
    }

    switch (ZDT_Motor_Control_Method)
    {
        case (ZDT_Motor_Control_Method_TORQUE_MIT):
        {
            Output_Torque = MIT_K_P * (Target_Angle - Data.Now_Angle) +
                            MIT_K_D * (Target_Omega - Data.Now_Omega) +
                            Target_Torque;
        }
        break;

        case (ZDT_Motor_Control_Method_TORQUE_OMEGA):
        {
            PID_Omega.Set_Target(Target_Omega);
            PID_Omega.Set_Now(Data.Now_Omega);
            PID_Omega.TIM_Adjust_PeriodElapsedCallback();
            Output_Torque = PID_Omega.Get_Out();
        }
        break;

        case (ZDT_Motor_Control_Method_TORQUE_POSITION_OMEGA):
        {
            PID_Angle.Set_Target(Target_Angle);
            PID_Angle.Set_Now(Data.Now_Angle);
            PID_Angle.TIM_Adjust_PeriodElapsedCallback();

            float tmp_target_omega = PID_Angle.Get_Out();
            float tmp_omega_limit = Math_Abs(Target_Omega);
            if (tmp_omega_limit <= FLT_EPSILON)
            {
                tmp_omega_limit = Max_Omega;
            }
            Math_Constrain(&tmp_target_omega, -tmp_omega_limit, tmp_omega_limit);

            PID_Omega.Set_Target(tmp_target_omega);
            PID_Omega.Set_Now(Data.Now_Omega);
            PID_Omega.TIM_Adjust_PeriodElapsedCallback();
            Output_Torque = PID_Omega.Get_Out();
        }
        break;

        case (ZDT_Motor_Control_Method_EMMX_POSITION):
        {
            Output_Torque = 0.0f;
        }
        break;

        default:
        {
            Output_Torque = 0.0f;
        }
        break;
    }

    Math_Constrain(&Output_Torque, -Max_Torque, Max_Torque);
}

static uint16_t ZDT_Motor_Get_Rx_Length(const FDCAN_RxHeaderTypeDef *Header)
{
    if (Header->DataLength <= FDCAN_DLC_BYTES_8)
    {
        return (static_cast<uint16_t>(Header->DataLength));
    }

    switch (Header->DataLength)
    {
        case (FDCAN_DLC_BYTES_12):
        {
            return (12U);
        }
        case (FDCAN_DLC_BYTES_16):
        {
            return (16U);
        }
        case (FDCAN_DLC_BYTES_20):
        {
            return (20U);
        }
        case (FDCAN_DLC_BYTES_24):
        {
            return (24U);
        }
        case (FDCAN_DLC_BYTES_32):
        {
            return (32U);
        }
        case (FDCAN_DLC_BYTES_48):
        {
            return (48U);
        }
        case (FDCAN_DLC_BYTES_64):
        {
            return (64U);
        }
        default:
        {
            return (0U);
        }
    }
}

static uint16_t ZDT_Motor_Current_A_To_mA(float Current_A)
{
    float tmp_current = Math_Abs(Current_A) * 1000.0f;
    Math_Constrain(&tmp_current, 0.0f, 65535.0f);
    return (static_cast<uint16_t>(tmp_current + 0.5f));
}

static uint16_t ZDT_Motor_Omega_Radps_To_Raw(float Omega_Radps)
{
    float tmp_rpm = Math_Abs(Omega_Radps / RPM_TO_RADPS);
    float tmp_raw = tmp_rpm * 10.0f;
    Math_Constrain(&tmp_raw, 0.0f, 65535.0f);
    return (static_cast<uint16_t>(tmp_raw + 0.5f));
}

static uint16_t ZDT_Motor_Current_Ramp_To_Raw(float Current_Ramp)
{
    float tmp_mA_per_second = Math_Abs(Current_Ramp);
    Math_Constrain(&tmp_mA_per_second, 0.0f, 65535.0f);
    return (static_cast<uint16_t>(tmp_mA_per_second + 0.5f));
}

static uint32_t ZDT_Motor_Position_Rad_To_Raw(float Position_Rad)
{
    double tmp_position = static_cast<double>(Math_Abs(Position_Rad * RAD_TO_DEG)) * 10.0;

    if (tmp_position > static_cast<double>(UINT32_MAX))
    {
        tmp_position = static_cast<double>(UINT32_MAX);
    }

    return (static_cast<uint32_t>(tmp_position + 0.5));
}

static uint16_t ZDT_Motor_Omega_Radps_To_RPM(float Omega_Radps)
{
    float tmp_rpm = Math_Abs(Omega_Radps / RPM_TO_RADPS);
    Math_Constrain(&tmp_rpm, 0.0f, 65535.0f);
    return static_cast<uint16_t>(tmp_rpm + 0.5f);
}

static uint32_t ZDT_Motor_Position_Rad_To_Pulses(float Position_Rad, float Pulses_Per_Revolution)
{
    double tmp_pulses = static_cast<double>(Math_Abs(Position_Rad)) *
                        static_cast<double>(Pulses_Per_Revolution) / (2.0 * static_cast<double>(PI));

    if (tmp_pulses > static_cast<double>(UINT32_MAX))
    {
        tmp_pulses = static_cast<double>(UINT32_MAX);
    }

    return static_cast<uint32_t>(tmp_pulses + 0.5);
}

static uint16_t ZDT_Motor_Torque_To_Current_mA(float Torque, float Torque_Constant, float Max_Current)
{
    if ((Torque == 0.0f) || (Torque_Constant <= FLT_EPSILON))
    {
        return (0U);
    }

    float tmp_torque = Math_Abs(Torque);
    float tmp_current = tmp_torque / Torque_Constant;
    Math_Constrain(&tmp_current, 0.0f, Max_Current);

    uint16_t tmp_current_mA = ZDT_Motor_Current_A_To_mA(tmp_current);
    if (tmp_current_mA == 0U)
    {
        tmp_current_mA = 1U;
    }

    return (tmp_current_mA);
}
