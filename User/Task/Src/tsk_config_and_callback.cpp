#include "tsk_config_and_callback.h"

#include "ita_controller.h"
#include "drv_can.h"
#include "drv_spi.h"
#include "drv_tim.h"
#include "drv_uart.h"
#include "dvc_dwt.h"

Class_Controller controller;

void CAN1_Motor_Callback(Struct_CAN_Rx_Buffer *CAN_RxMessage)
{
    controller.CAN1_Motor_RxCpltCallback(CAN_RxMessage);
}

void CAN2_Motor_Callback(Struct_CAN_Rx_Buffer *CAN_RxMessage)
{
    controller.CAN2_Motor_RxCpltCallback(CAN_RxMessage);
}

void RS485_Motor_Bus_Callback(uint8_t *Buffer, uint16_t Length)
{
    controller.RS485_Motor_Bus_RxCpltCallback(Buffer, Length);
}

void Referee_UART_Callback(uint8_t *Buffer, uint16_t Length)
{
    controller.Referee_UART_RxCpltCallback(Buffer, Length);
}

void Peer_UART_Callback(uint8_t *Buffer, uint16_t Length)
{
    controller.Peer_UART_RxCpltCallback(Buffer, Length);
}

void LCD_SPI_Callback(uint8_t *Tx_Buffer, uint8_t *Rx_Buffer, uint16_t Length)
{
    controller.LCD_SPI_RxCpltCallback(Tx_Buffer, Rx_Buffer, Length);
}

void Task100us_TIM4_Callback()
{
    controller.TIM_Calculate_PeriodElapsedCallback();
}

void Task1ms_TIM5_Callback()
{
    static uint8_t mod50 = 0;

    mod50++;
    if (mod50 >= 50)
    {
        controller.TIM1msMod50_Alive_PeriodElapsedCallback();
        mod50 = 0;
    }
}

extern "C" void Task_Init()
{
    DWT_Init(480);

    CAN_Init(&hfdcan1, CAN1_Motor_Callback);
    CAN_Init(&hfdcan2, CAN2_Motor_Callback);

    UART_Init(&huart5, RS485_Motor_Bus_Callback, 64);
    UART_Init(&huart8, Peer_UART_Callback, 64);
    UART_Init(&huart10, Referee_UART_Callback, 64);

    SPI_Init(&hspi2, LCD_SPI_Callback);

    TIM_Init(&htim4, Task100us_TIM4_Callback);
    TIM_Init(&htim5, Task1ms_TIM5_Callback);

    controller.Init();

    HAL_TIM_Base_Start_IT(&htim4);
    HAL_TIM_Base_Start_IT(&htim5);
}

extern "C" void Task_Loop()
{
    controller.Task_Loop_Callback();
}
