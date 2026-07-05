#ifndef DVC_REFEREE_H
#define DVC_REFEREE_H

#include "drv_uart.h"
#include <stdint.h>
#include <string.h>

enum Enum_Referee_Status
{
    Referee_Status_DISABLE = 0,
    Referee_Status_ENABLE,
};

struct Struct_Referee_Custom_Controller_Data
{
    uint8_t Data[30];
    uint16_t CRC_16;
} __attribute__((packed));

class Class_Referee
{
public:
    void Init(UART_HandleTypeDef *huart, uint8_t __Frame_Header = 0xA5);

    inline Enum_Referee_Status Get_Referee_Status();
    inline const Struct_Referee_Custom_Controller_Data *Get_Rx_Data();
    inline const Struct_Referee_Custom_Controller_Data *Get_Tx_Data();
    inline void Set_Tx_Data(const uint8_t *Data, uint16_t Length);

    void UART_RxCpltCallback(uint8_t *Rx_Data, uint16_t Length);
    void TIM1msMod50_Alive_PeriodElapsedCallback();
    void TIM_UART_Tx_PeriodElapsedCallback();

protected:
    Struct_UART_Manage_Object *UART_Manage_Object = 0;
    uint8_t Frame_Header = 0xA5;
    uint32_t Flag = 0;
    uint32_t Pre_Flag = 0;
    Enum_Referee_Status Referee_Status = Referee_Status_DISABLE;
    Struct_Referee_Custom_Controller_Data Rx_Custom_Controller_Data = {{0}, 0};
    Struct_Referee_Custom_Controller_Data Tx_Custom_Controller_Data = {{0}, 0};

    uint16_t Calculate_CRC16(const uint8_t *Buffer, uint16_t Length);
};

inline Enum_Referee_Status Class_Referee::Get_Referee_Status()
{
    return (Referee_Status);
}

inline const Struct_Referee_Custom_Controller_Data *Class_Referee::Get_Rx_Data()
{
    return (&Rx_Custom_Controller_Data);
}

inline const Struct_Referee_Custom_Controller_Data *Class_Referee::Get_Tx_Data()
{
    return (&Tx_Custom_Controller_Data);
}

inline void Class_Referee::Set_Tx_Data(const uint8_t *Data, uint16_t Length)
{
    uint16_t Copy_Length = Length > sizeof(Tx_Custom_Controller_Data.Data) ? sizeof(Tx_Custom_Controller_Data.Data) : Length;
    memset(Tx_Custom_Controller_Data.Data, 0, sizeof(Tx_Custom_Controller_Data.Data));
    memcpy(Tx_Custom_Controller_Data.Data, Data, Copy_Length);
    Tx_Custom_Controller_Data.CRC_16 = Calculate_CRC16(Tx_Custom_Controller_Data.Data, sizeof(Tx_Custom_Controller_Data.Data));
}

#endif
