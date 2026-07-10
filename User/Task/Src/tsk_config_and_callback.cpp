// SPDX-License-Identifier: AGPL-3.0-only
#include "tsk_config_and_callback.h"

#include "ita_controller.h"
#include "drv_can.h"
#include "drv_tim.h"
#include "drv_uart.h"
#include "dvc_dwt.h"

Class_Controller controller;
static uint8_t peer_rx_buffer[UART_BUFFER_SIZE];
static uint16_t peer_rx_length = 0;
static volatile uint32_t lcd_refresh_generation = 0;

static void Manipulator_CAN_Global_Filter_Init()
{
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
                                 FDCAN_REJECT,
                                 FDCAN_ACCEPT_IN_RX_FIFO0,
                                 FDCAN_REJECT_REMOTE,
                                 FDCAN_REJECT_REMOTE);

    HAL_FDCAN_ConfigGlobalFilter(&hfdcan2,
                                 FDCAN_REJECT,
                                 FDCAN_ACCEPT_IN_RX_FIFO1,
                                 FDCAN_REJECT_REMOTE,
                                 FDCAN_REJECT_REMOTE);
}

void CAN1_Motor_Callback(Struct_CAN_Rx_Buffer *CAN_RxMessage)
{
    controller.Left_Arm.CAN_RxCpltCallback(CAN_RxMessage);
}

void CAN2_Motor_Callback(Struct_CAN_Rx_Buffer *CAN_RxMessage)
{
    controller.Right_Arm.CAN_RxCpltCallback(CAN_RxMessage);
}

void RS485_USART2_Motor_Bus_Callback(uint8_t *Buffer, uint16_t Length)
{
    controller.Left_Arm.UART_RxCpltCallback(Buffer, Length);
}

void RS485_USART3_Motor_Bus_Callback(uint8_t *Buffer, uint16_t Length)
{
    controller.Right_Arm.UART_RxCpltCallback(Buffer, Length);
}

void Referee_UART_Callback(uint8_t *Buffer, uint16_t Length)
{
    controller.Referee.UART_RxCpltCallback(Buffer, Length);
}

void Peer_UART_Callback(uint8_t *Buffer, uint16_t Length)
{
    peer_rx_length = Length > UART_BUFFER_SIZE ? UART_BUFFER_SIZE : Length;
    memcpy(peer_rx_buffer, Buffer, peer_rx_length);
}

void Task100us_TIM4_Callback()
{
}

void Task1ms_TIM5_Callback()
{
    static uint8_t mod50 = 0;
    static uint8_t mod100 = 0;

    controller.Left_Arm.TIM_Calculate_PeriodElapsedCallback();
    controller.Right_Arm.TIM_Calculate_PeriodElapsedCallback();

    mod100++;
    if (mod100 >= 100)
    {
        lcd_refresh_generation++;
        mod100 = 0;
    }

    mod50++;
    if (mod50 >= 50)
    {
        controller.Left_Arm.TIM1msMod50_Alive_PeriodElapsedCallback();
        controller.Right_Arm.TIM1msMod50_Alive_PeriodElapsedCallback();
        controller.Referee.TIM1msMod50_Alive_PeriodElapsedCallback();
        controller.Referee.TIM_UART_Tx_PeriodElapsedCallback();
        mod50 = 0;
    }
}

extern "C" void Task_Init()
{
    DWT_Init(480);

    Manipulator_CAN_Global_Filter_Init();
    CAN_Init(&hfdcan1, CAN1_Motor_Callback);
    CAN_Init(&hfdcan2, CAN2_Motor_Callback);

    UART_Init(&huart2, RS485_USART2_Motor_Bus_Callback, 16);
    UART_Init(&huart3, RS485_USART3_Motor_Bus_Callback, 16);
    UART_Init(&huart7, Peer_UART_Callback, 64);
    UART_Init(&huart10, Referee_UART_Callback, 64);

    TIM_Init(&htim4, Task100us_TIM4_Callback);
    TIM_Init(&htim5, Task1ms_TIM5_Callback);

    controller.Init();

    HAL_TIM_Base_Start_IT(&htim4);
    HAL_TIM_Base_Start_IT(&htim5);
}

extern "C" void Task_Loop()
{
    static uint32_t last_lcd_refresh_generation = 0;

    if (last_lcd_refresh_generation != lcd_refresh_generation)
    {
        Struct_LCD_Status status = {};

        last_lcd_refresh_generation = lcd_refresh_generation;
        for (uint8_t joint = 0; joint < LCD_JOINT_COUNT; joint++)
        {
            status.Current_Joint_Angle[0][joint] = controller.Left_Arm.Get_Current_Joint_Angle(joint);
            status.Current_Joint_Angle[1][joint] = controller.Right_Arm.Get_Current_Joint_Angle(joint);
        }

        controller.LCD.Submit_Status(&status);
    }

    controller.LCD.Refresh();
}
