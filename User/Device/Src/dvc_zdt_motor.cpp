// SPDX-License-Identifier: AGPL-3.0-only
#include "dvc_zdt_motor.h"

#include "fdcan.h"

void Class_ZDT_Motor::Init(FDCAN_HandleTypeDef *hcan, uint16_t __CAN_ID)
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

void Class_ZDT_Motor::CAN_RxCpltCallback(uint8_t *Rx_Data)
{
    UNUSED(Rx_Data);
    Flag++;
}

void Class_ZDT_Motor::TIM_Alive_PeriodElapsedCallback()
{
    ZDT_Motor_Status = (Flag == Pre_Flag) ? ZDT_Motor_Status_DISABLE : ZDT_Motor_Status_ENABLE;
    Pre_Flag = Flag;
}

void Class_ZDT_Motor::TIM_Process_PeriodElapsedCallback()
{
    if (ZDT_Motor_Control_Status == ZDT_Motor_Control_Status_DISABLE || CAN_Manage_Object == 0)
    {
        return;
    }
}
