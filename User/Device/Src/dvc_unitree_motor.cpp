/**
 * @file dvc_unitree_motor.cpp
 * @brief Unitree GO-M8010-6 motor configuration and operation
 */

/* Includes ------------------------------------------------------------------*/

#include "dvc_unitree_motor.h"

/* Private macros ------------------------------------------------------------*/

#define UNITREE_MOTOR_TX_FRAME_LENGTH (17U)
#define UNITREE_MOTOR_RX_FRAME_LENGTH (16U)
#define UNITREE_MOTOR_TX_DATA_LENGTH (15U)
#define UNITREE_MOTOR_RX_DATA_LENGTH (14U)

#define UNITREE_MOTOR_TX_HEAD_0 (0xFEU)
#define UNITREE_MOTOR_RX_HEAD_0 (0xFDU)
#define UNITREE_MOTOR_HEAD_1 (0xEEU)

#define UNITREE_MOTOR_ID_MAX (14U)

#define UNITREE_MOTOR_TORQUE_SCALE (256.0f)
#define UNITREE_MOTOR_OMEGA_SCALE (256.0f)
#define UNITREE_MOTOR_POSITION_SCALE (32768.0f)
#define UNITREE_MOTOR_GAIN_SCALE (1280.0f)
#define UNITREE_MOTOR_GAIN_MAX (25.599f)

#define UNITREE_MOTOR_CRC_INIT (0x0000U)
#define UNITREE_MOTOR_CRC_POLY (0x8408U)

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

static uint16_t Unitree_Motor_Calculate_CRC_CCITT(const uint8_t *Buffer, uint16_t Length);
static uint16_t Unitree_Motor_Read_Uint16(const uint8_t *Data);
static int16_t Unitree_Motor_Read_Int16(const uint8_t *Data);
static int32_t Unitree_Motor_Read_Int32(const uint8_t *Data);
static void Unitree_Motor_Write_Uint16(uint8_t *Data, uint16_t Value);
static void Unitree_Motor_Write_Int16(uint8_t *Data, int16_t Value);
static void Unitree_Motor_Write_Int32(uint8_t *Data, int32_t Value);
static int16_t Unitree_Motor_Float_To_Int16(float Value, float Scale);
static int32_t Unitree_Motor_Float_To_Int32(float Value, float Scale);
static uint16_t Unitree_Motor_Gain_To_Uint16(float Value);

/* Function prototypes -------------------------------------------------------*/

static uint16_t Unitree_Motor_Calculate_CRC_CCITT(const uint8_t *Buffer, uint16_t Length)
{
    uint16_t crc = UNITREE_MOTOR_CRC_INIT;

    while (Length--)
    {
        crc ^= *Buffer++;
        for (uint8_t i = 0; i < 8; i++)
        {
            crc = (crc & 0x0001U) ? static_cast<uint16_t>((crc >> 1) ^ UNITREE_MOTOR_CRC_POLY) : static_cast<uint16_t>(crc >> 1);
        }
    }

    return (crc);
}

static uint16_t Unitree_Motor_Read_Uint16(const uint8_t *Data)
{
    return (static_cast<uint16_t>(Data[0]) | static_cast<uint16_t>(Data[1] << 8));
}

static int16_t Unitree_Motor_Read_Int16(const uint8_t *Data)
{
    return (static_cast<int16_t>(Unitree_Motor_Read_Uint16(Data)));
}

static int32_t Unitree_Motor_Read_Int32(const uint8_t *Data)
{
    return (static_cast<int32_t>(static_cast<uint32_t>(Data[0]) |
                                 (static_cast<uint32_t>(Data[1]) << 8) |
                                 (static_cast<uint32_t>(Data[2]) << 16) |
                                 (static_cast<uint32_t>(Data[3]) << 24)));
}

static void Unitree_Motor_Write_Uint16(uint8_t *Data, uint16_t Value)
{
    Data[0] = static_cast<uint8_t>(Value);
    Data[1] = static_cast<uint8_t>(Value >> 8);
}

static void Unitree_Motor_Write_Int16(uint8_t *Data, int16_t Value)
{
    Unitree_Motor_Write_Uint16(Data, static_cast<uint16_t>(Value));
}

static void Unitree_Motor_Write_Int32(uint8_t *Data, int32_t Value)
{
    uint32_t tmp_value = static_cast<uint32_t>(Value);

    Data[0] = static_cast<uint8_t>(tmp_value);
    Data[1] = static_cast<uint8_t>(tmp_value >> 8);
    Data[2] = static_cast<uint8_t>(tmp_value >> 16);
    Data[3] = static_cast<uint8_t>(tmp_value >> 24);
}

static int16_t Unitree_Motor_Float_To_Int16(float Value, float Scale)
{
    float tmp_value = Value * Scale;

    Math_Constrain(&tmp_value, -32768.0f, 32767.0f);
    return (static_cast<int16_t>(tmp_value));
}

static int32_t Unitree_Motor_Float_To_Int32(float Value, float Scale)
{
    double tmp_value = static_cast<double>(Value) * static_cast<double>(Scale);

    if (tmp_value > 2147483647.0)
    {
        tmp_value = 2147483647.0;
    }
    else if (tmp_value < -2147483648.0)
    {
        tmp_value = -2147483648.0;
    }

    return (static_cast<int32_t>(tmp_value));
}

static uint16_t Unitree_Motor_Gain_To_Uint16(float Value)
{
    Math_Constrain(&Value, 0.0f, UNITREE_MOTOR_GAIN_MAX);
    return (static_cast<uint16_t>(Value * UNITREE_MOTOR_GAIN_SCALE));
}

void Class_Unitree_Motor::Init(Struct_UART_Manage_Object *__UART_Manage_Object, uint16_t __Node_ID,
                               float __Gear_Ratio, float __MIT_K_P, float __MIT_K_D)
{
    UART_Manage_Object = __UART_Manage_Object;
    Node_ID = (__Node_ID > UNITREE_MOTOR_ID_MAX) ? UNITREE_MOTOR_ID_MAX : __Node_ID;
    Gear_Ratio = (__Gear_Ratio > 0.0f) ? __Gear_Ratio : 1.0f;
    MIT_K_P = __MIT_K_P;
    MIT_K_D = __MIT_K_D;

    Unitree_Motor_Status = Unitree_Motor_Status_DISABLE;
    Unitree_Motor_Control_Status = Unitree_Motor_Control_Status_DISABLE;
    Flag = 0;
    Pre_Flag = 0;

    Data.Node_ID = Node_ID;
    Data.Working_Status = Unitree_Motor_Working_Status_LOCK;
    Data.Now_Angle = 0.0f;
    Data.Now_Omega = 0.0f;
    Data.Now_Torque = 0.0f;
    Data.Now_Rotor_Temperature = 0.0f;
    Data.Error_Status = Unitree_Motor_Error_Status_NONE;

    Target_Angle = 0.0f;
    Target_Omega = 0.0f;
    Target_Torque = 0.0f;
}

uint8_t Class_Unitree_Motor::Data_Process(uint8_t *Rx_Data)
{
    if ((Rx_Data[0] != UNITREE_MOTOR_RX_HEAD_0) || (Rx_Data[1] != UNITREE_MOTOR_HEAD_1))
    {
        return (0);
    }

    uint16_t tmp_crc = Unitree_Motor_Calculate_CRC_CCITT(Rx_Data, UNITREE_MOTOR_RX_DATA_LENGTH);
    uint16_t tmp_rx_crc = Unitree_Motor_Read_Uint16(&Rx_Data[UNITREE_MOTOR_RX_DATA_LENGTH]);
    if (tmp_crc != tmp_rx_crc)
    {
        return (0);
    }

    uint16_t tmp_node_id = Rx_Data[2] & 0x0fU;
    if (tmp_node_id != Node_ID)
    {
        return (0);
    }

    int16_t tmp_torque = Unitree_Motor_Read_Int16(&Rx_Data[3]);
    int16_t tmp_omega = Unitree_Motor_Read_Int16(&Rx_Data[5]);
    int32_t tmp_angle = Unitree_Motor_Read_Int32(&Rx_Data[7]);
    int8_t tmp_temperature = static_cast<int8_t>(Rx_Data[11]);
    uint16_t tmp_error_status = Unitree_Motor_Read_Uint16(&Rx_Data[12]);

    Data.Node_ID = tmp_node_id;
    Data.Working_Status = static_cast<Enum_Unitree_Motor_Working_Status>((Rx_Data[2] >> 4) & 0x07U);
    Data.Now_Angle = (static_cast<float>(tmp_angle) / UNITREE_MOTOR_POSITION_SCALE) * (2.0f * PI) / Gear_Ratio;
    Data.Now_Omega = (static_cast<float>(tmp_omega) / UNITREE_MOTOR_OMEGA_SCALE) * (2.0f * PI) / Gear_Ratio;
    Data.Now_Torque = (static_cast<float>(tmp_torque) / UNITREE_MOTOR_TORQUE_SCALE) * Gear_Ratio;
    Data.Now_Rotor_Temperature = static_cast<float>(tmp_temperature);
    Data.Error_Status = static_cast<Enum_Unitree_Motor_Error_Status>(tmp_error_status & 0x0007U);

    return (1);
}

void Class_Unitree_Motor::UART_RxCpltCallback(uint8_t *Rx_Data, uint16_t Length)
{
    if ((Rx_Data == 0) || (Length < UNITREE_MOTOR_RX_FRAME_LENGTH))
    {
        return;
    }

    for (uint16_t i = 0; i <= Length - UNITREE_MOTOR_RX_FRAME_LENGTH; i++)
    {
        if ((Rx_Data[i] == UNITREE_MOTOR_RX_HEAD_0) && (Rx_Data[i + 1] == UNITREE_MOTOR_HEAD_1))
        {
            if (Data_Process(&Rx_Data[i]) != 0)
            {
                Flag++;
                i += UNITREE_MOTOR_RX_FRAME_LENGTH - 1;
            }
        }
    }
}

void Class_Unitree_Motor::TIM_Alive_PeriodElapsedCallback()
{
    if (Flag == Pre_Flag)
    {
        Unitree_Motor_Status = Unitree_Motor_Status_DISABLE;
    }
    else
    {
        Unitree_Motor_Status = Unitree_Motor_Status_ENABLE;
    }

    Pre_Flag = Flag;
}

void Class_Unitree_Motor::TIM_Process_PeriodElapsedCallback()
{
    if ((UART_Manage_Object == 0) || (UART_Manage_Object->UART_Handler == 0))
    {
        return;
    }

    uint8_t *tmp_tx_data = UART_Manage_Object->Tx_Buffer;
    Enum_Unitree_Motor_Working_Status tmp_working_status = Unitree_Motor_Working_Status_LOCK;

    float tmp_target_torque = 0.0f;
    float tmp_target_omega = 0.0f;
    float tmp_target_angle = 0.0f;
    float tmp_target_k_p = 0.0f;
    float tmp_target_k_d = 0.0f;

    if (Unitree_Motor_Control_Status == Unitree_Motor_Control_Status_ENABLE)
    {
        tmp_working_status = Unitree_Motor_Working_Status_FOC;
        tmp_target_torque = Target_Torque;
        tmp_target_omega = Target_Omega;
        tmp_target_angle = Target_Angle;
        tmp_target_k_p = MIT_K_P;
        tmp_target_k_d = MIT_K_D;
    }

    tmp_tx_data[0] = UNITREE_MOTOR_TX_HEAD_0;
    tmp_tx_data[1] = UNITREE_MOTOR_HEAD_1;
    tmp_tx_data[2] = static_cast<uint8_t>((Node_ID & 0x000fU) | ((static_cast<uint8_t>(tmp_working_status) & 0x07U) << 4));

    int16_t tmp_torque = Unitree_Motor_Float_To_Int16(tmp_target_torque / Gear_Ratio, UNITREE_MOTOR_TORQUE_SCALE);
    int16_t tmp_omega = Unitree_Motor_Float_To_Int16(tmp_target_omega * Gear_Ratio / (2.0f * PI), UNITREE_MOTOR_OMEGA_SCALE);
    int32_t tmp_angle = Unitree_Motor_Float_To_Int32(tmp_target_angle * Gear_Ratio / (2.0f * PI), UNITREE_MOTOR_POSITION_SCALE);
    uint16_t tmp_k_p = Unitree_Motor_Gain_To_Uint16(tmp_target_k_p);
    uint16_t tmp_k_d = Unitree_Motor_Gain_To_Uint16(tmp_target_k_d);

    Unitree_Motor_Write_Int16(&tmp_tx_data[3], tmp_torque);
    Unitree_Motor_Write_Int16(&tmp_tx_data[5], tmp_omega);
    Unitree_Motor_Write_Int32(&tmp_tx_data[7], tmp_angle);
    Unitree_Motor_Write_Uint16(&tmp_tx_data[11], tmp_k_p);
    Unitree_Motor_Write_Uint16(&tmp_tx_data[13], tmp_k_d);

    uint16_t tmp_crc = Unitree_Motor_Calculate_CRC_CCITT(tmp_tx_data, UNITREE_MOTOR_TX_DATA_LENGTH);
    Unitree_Motor_Write_Uint16(&tmp_tx_data[15], tmp_crc);

    UART_Send_Data(UART_Manage_Object->UART_Handler, tmp_tx_data, UNITREE_MOTOR_TX_FRAME_LENGTH);
}
