#include "dvc_ak80_motor.h"

#include "fdcan.h"

void Class_AK80_Motor::Init(FDCAN_HandleTypeDef *hcan, uint16_t __CAN_ID)
{
    CAN_ID = __CAN_ID;
    if (hcan->Instance == FDCAN1)
    {
        CAN_Manage_Object = &CAN1_Manage_Object;
    }
    else if (hcan->Instance == FDCAN2)
    {
        CAN_Manage_Object = &CAN2_Manage_Object;
    }
}

void Class_AK80_Motor::CAN_RxCpltCallback(uint8_t *Rx_Data)
{
    UNUSED(Rx_Data);
    Flag++;
}

void Class_AK80_Motor::TIM_Alive_PeriodElapsedCallback()
{
    AK80_Motor_Status = (Flag == Pre_Flag) ? AK80_Motor_Status_DISABLE : AK80_Motor_Status_ENABLE;
    Pre_Flag = Flag;
}

void Class_AK80_Motor::TIM_Process_PeriodElapsedCallback()
{
    if (AK80_Motor_Control_Status == AK80_Motor_Control_Status_DISABLE || CAN_Manage_Object == 0)
    {
        return;
    }
}
