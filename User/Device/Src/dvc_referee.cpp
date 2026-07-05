#include "dvc_referee.h"

void Class_Referee::Init(UART_HandleTypeDef *huart, uint8_t __Frame_Header)
{
    if (huart->Instance == USART1)
    {
        UART_Manage_Object = &UART1_Manage_Object;
    }
    else if (huart->Instance == UART5)
    {
        UART_Manage_Object = &UART5_Manage_Object;
    }
    else if (huart->Instance == UART7)
    {
        UART_Manage_Object = &UART7_Manage_Object;
    }
    else if (huart->Instance == UART8)
    {
        UART_Manage_Object = &UART8_Manage_Object;
    }
    else if (huart->Instance == UART9)
    {
        UART_Manage_Object = &UART9_Manage_Object;
    }
    else if (huart->Instance == USART10)
    {
        UART_Manage_Object = &UART10_Manage_Object;
    }

    Frame_Header = __Frame_Header;
}

uint16_t Class_Referee::Calculate_CRC16(const uint8_t *Buffer, uint16_t Length)
{
    uint16_t CRC = 0xFFFF;

    for (uint16_t i = 0; i < Length; i++)
    {
        CRC ^= Buffer[i];
        for (uint8_t bit = 0; bit < 8; bit++)
        {
            CRC = (CRC & 0x0001U) ? static_cast<uint16_t>((CRC >> 1) ^ 0xA001U) : static_cast<uint16_t>(CRC >> 1);
        }
    }

    return (CRC);
}

void Class_Referee::UART_RxCpltCallback(uint8_t *Rx_Data, uint16_t Length)
{
    Flag++;

    if (Length >= sizeof(Rx_Custom_Controller_Data.Data))
    {
        memcpy(Rx_Custom_Controller_Data.Data, Rx_Data, sizeof(Rx_Custom_Controller_Data.Data));
        Rx_Custom_Controller_Data.CRC_16 = Calculate_CRC16(Rx_Custom_Controller_Data.Data, sizeof(Rx_Custom_Controller_Data.Data));
    }
}

void Class_Referee::TIM1msMod50_Alive_PeriodElapsedCallback()
{
    Referee_Status = (Flag == Pre_Flag) ? Referee_Status_DISABLE : Referee_Status_ENABLE;
    Pre_Flag = Flag;
}

void Class_Referee::TIM_UART_Tx_PeriodElapsedCallback()
{
    if (UART_Manage_Object == 0 || UART_Manage_Object->UART_Handler == 0)
    {
        return;
    }

    memcpy(UART_Manage_Object->Tx_Buffer, Tx_Custom_Controller_Data.Data, sizeof(Tx_Custom_Controller_Data.Data));
    memcpy(UART_Manage_Object->Tx_Buffer + sizeof(Tx_Custom_Controller_Data.Data),
           &Tx_Custom_Controller_Data.CRC_16,
           sizeof(Tx_Custom_Controller_Data.CRC_16));

    UART_Send_Data(UART_Manage_Object->UART_Handler,
                   UART_Manage_Object->Tx_Buffer,
                   sizeof(Tx_Custom_Controller_Data));
}
