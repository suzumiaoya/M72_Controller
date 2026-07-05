#include "dvc_unitree_motor.h"

void Class_Unitree_Motor::Init(Struct_UART_Manage_Object *__UART_Manage_Object, uint16_t __Node_ID)
{
    UART_Manage_Object = __UART_Manage_Object;
    Node_ID = __Node_ID;
}

void Class_Unitree_Motor::UART_RxCpltCallback(uint8_t *Rx_Data, uint16_t Length)
{
    UNUSED(Rx_Data);
    UNUSED(Length);
    Flag++;
}

void Class_Unitree_Motor::TIM_Alive_PeriodElapsedCallback()
{
    Unitree_Motor_Status = (Flag == Pre_Flag) ? Unitree_Motor_Status_DISABLE : Unitree_Motor_Status_ENABLE;
    Pre_Flag = Flag;
}

void Class_Unitree_Motor::TIM_Process_PeriodElapsedCallback()
{
    if (Unitree_Motor_Control_Status == Unitree_Motor_Control_Status_DISABLE)
    {
        return;
    }
}
