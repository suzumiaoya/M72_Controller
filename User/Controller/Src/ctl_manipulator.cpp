// SPDX-License-Identifier: AGPL-3.0-only
#include "ctl_manipulator.h"

#include "fdcan.h"
#include "usart.h"

static FDCAN_HandleTypeDef *Get_CAN_Handler(Enum_Bus_ID Bus_ID)
{
    switch (Bus_ID)
    {
        case (Bus_ID_CAN_1):
        {
            return (&hfdcan1);
        }
        case (Bus_ID_CAN_2):
        {
            return (&hfdcan2);
        }
        default:
        {
            return (0);
        }
    }
}

static Struct_UART_Manage_Object *Get_UART_Manage_Object(Enum_Bus_ID Bus_ID)
{
    switch (Bus_ID)
    {
        case (Bus_ID_RS485_USART2):
        {
            return (&UART2_Manage_Object);
        }
        case (Bus_ID_RS485_USART3):
        {
            return (&UART3_Manage_Object);
        }
        default:
        {
            return (0);
        }
    }
}

void Class_Manipulator::Init(Enum_Manipulator_ID __Manipulator_ID)
{
    Manipulator_ID = __Manipulator_ID;

    if (Manipulator_ID == Manipulator_ID_LEFT)
    {
        Joint_Limit = Left_Arm_Joint_Limit;
        Joint_Binding = Left_Arm_Joint_Binding;
    }
    else
    {
        Joint_Limit = Right_Arm_Joint_Limit;
        Joint_Binding = Right_Arm_Joint_Binding;
    }

    Motor_J0.Init(Get_UART_Manage_Object(Joint_Binding[Controller_Joint_ID_J0].Bus_ID),
                  Joint_Binding[Controller_Joint_ID_J0].Device_ID);
    Motor_J1.Init(Get_CAN_Handler(Joint_Binding[Controller_Joint_ID_J1].Bus_ID),
                  static_cast<Enum_AK_Motor_ID>(Joint_Binding[Controller_Joint_ID_J1].Device_ID));
    Motor_J2.Init(Get_CAN_Handler(Joint_Binding[Controller_Joint_ID_J2].Bus_ID),
                  static_cast<Enum_AK_Motor_ID>(Joint_Binding[Controller_Joint_ID_J2].Device_ID));
    Motor_J3.Init(Get_CAN_Handler(Joint_Binding[Controller_Joint_ID_J3].Bus_ID),
                  Joint_Binding[Controller_Joint_ID_J3].Device_ID);
    Motor_J4.Init(Get_CAN_Handler(Joint_Binding[Controller_Joint_ID_J4].Bus_ID),
                  Joint_Binding[Controller_Joint_ID_J4].Device_ID);
    Motor_J5.Init(Get_CAN_Handler(Joint_Binding[Controller_Joint_ID_J5].Bus_ID),
                  Joint_Binding[Controller_Joint_ID_J5].Device_ID);
    Update_Current_State();
}

void Class_Manipulator::Output()
{
    if (Manipulator_Control_Status == Manipulator_Control_Status_DISABLE)
    {
        Motor_J0.Set_Unitree_Motor_Control_Status(Unitree_Motor_Control_Status_DISABLE);
        Motor_J1.Set_AK_Control_Status(AK_Motor_Control_Status_DISABLE);
        Motor_J2.Set_AK_Control_Status(AK_Motor_Control_Status_DISABLE);
        Motor_J3.Set_ZDT_Motor_Control_Status(ZDT_Motor_Control_Status_DISABLE);
        Motor_J4.Set_ZDT_Motor_Control_Status(ZDT_Motor_Control_Status_DISABLE);
        Motor_J5.Set_ZDT_Motor_Control_Status(ZDT_Motor_Control_Status_DISABLE);
        return;
    }

    for (uint8_t i = 0; i < CONTROLLER_JOINT_NUM; i++)
    {
        Math_Constrain(&Target_Joint_Angle[i], Joint_Limit[i].Min_Angle, Joint_Limit[i].Max_Angle);
    }

    Motor_J0.Set_Unitree_Motor_Control_Status(Unitree_Motor_Control_Status_ENABLE);
    Motor_J1.Set_AK_Control_Status(AK_Motor_Control_Status_ENABLE);
    Motor_J2.Set_AK_Control_Status(AK_Motor_Control_Status_ENABLE);
    Motor_J3.Set_ZDT_Motor_Control_Status(ZDT_Motor_Control_Status_ENABLE);
    Motor_J4.Set_ZDT_Motor_Control_Status(ZDT_Motor_Control_Status_ENABLE);
    Motor_J5.Set_ZDT_Motor_Control_Status(ZDT_Motor_Control_Status_ENABLE);

    Motor_J0.Set_Target_Angle(Target_Joint_Angle[Controller_Joint_ID_J0]);
    Motor_J1.Set_Target_Angle(Target_Joint_Angle[Controller_Joint_ID_J1]);
    Motor_J2.Set_Target_Angle(Target_Joint_Angle[Controller_Joint_ID_J2]);
    Motor_J3.Set_Target_Angle(Target_Joint_Angle[Controller_Joint_ID_J3]);
    Motor_J4.Set_Target_Angle(Target_Joint_Angle[Controller_Joint_ID_J4]);
    Motor_J5.Set_Target_Angle(Target_Joint_Angle[Controller_Joint_ID_J5]);

    Motor_J0.Set_Target_Torque(Target_Joint_Torque[Controller_Joint_ID_J0]);
    Motor_J1.Set_Target_Torque(Target_Joint_Torque[Controller_Joint_ID_J1]);
    Motor_J2.Set_Target_Torque(Target_Joint_Torque[Controller_Joint_ID_J2]);
    Motor_J3.Set_Target_Torque(Target_Joint_Torque[Controller_Joint_ID_J3]);
    Motor_J4.Set_Target_Torque(Target_Joint_Torque[Controller_Joint_ID_J4]);
    Motor_J5.Set_Target_Torque(Target_Joint_Torque[Controller_Joint_ID_J5]);
}

void Class_Manipulator::Update_Current_State()
{
    Current_Joint_Angle[Controller_Joint_ID_J0] = Motor_J0.Get_Now_Angle();
    Current_Joint_Angle[Controller_Joint_ID_J1] = Motor_J1.Get_Now_Angle();
    Current_Joint_Angle[Controller_Joint_ID_J2] = Motor_J2.Get_Now_Angle();
    Current_Joint_Angle[Controller_Joint_ID_J3] = Motor_J3.Get_Now_Angle();
    Current_Joint_Angle[Controller_Joint_ID_J4] = Motor_J4.Get_Now_Angle();
    Current_Joint_Angle[Controller_Joint_ID_J5] = Motor_J5.Get_Now_Angle();

    Current_Joint_Torque[Controller_Joint_ID_J0] = Motor_J0.Get_Now_Torque();
    Current_Joint_Torque[Controller_Joint_ID_J1] = Motor_J1.Get_Now_Torque();
    Current_Joint_Torque[Controller_Joint_ID_J2] = Motor_J2.Get_Now_Torque();
    Current_Joint_Torque[Controller_Joint_ID_J3] = Motor_J3.Get_Now_Torque();
    Current_Joint_Torque[Controller_Joint_ID_J4] = Motor_J4.Get_Now_Torque();
    Current_Joint_Torque[Controller_Joint_ID_J5] = Motor_J5.Get_Now_Torque();
}

void Class_Manipulator::CAN_RxCpltCallback(Struct_CAN_Rx_Buffer *CAN_RxMessage)
{
    // AK电机返回帧头为0x00，判断第一位数据区分电机
    if (CAN_RxMessage->Header.Identifier == 0x00 &&
        CAN_RxMessage->Data[0] == Joint_Binding[Controller_Joint_ID_J1].Device_ID)
    {
        Motor_J1.CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    else if (CAN_RxMessage->Header.Identifier == 0x00 &&
        CAN_RxMessage->Data[0] == Joint_Binding[Controller_Joint_ID_J2].Device_ID)
    {
        Motor_J2.CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    // ZDT电机CAN接收回调
    else if (CAN_RxMessage->Header.IdType == FDCAN_EXTENDED_ID &&
        (CAN_RxMessage->Header.Identifier >> 8) == Joint_Binding[Controller_Joint_ID_J3].Device_ID)
    {
        Motor_J3.CAN_RxCpltCallback(CAN_RxMessage);
    }
    else if (CAN_RxMessage->Header.IdType == FDCAN_EXTENDED_ID &&
        (CAN_RxMessage->Header.Identifier >> 8) == Joint_Binding[Controller_Joint_ID_J4].Device_ID)
    {
        Motor_J4.CAN_RxCpltCallback(CAN_RxMessage);
    }
    else if (CAN_RxMessage->Header.IdType == FDCAN_EXTENDED_ID &&
        (CAN_RxMessage->Header.Identifier >> 8) == Joint_Binding[Controller_Joint_ID_J5].Device_ID)
    {
        Motor_J5.CAN_RxCpltCallback(CAN_RxMessage);
    }
}

void Class_Manipulator::UART_RxCpltCallback(uint8_t *Buffer, uint16_t Length)
{
    Motor_J0.UART_RxCpltCallback(Buffer, Length);
}

void Class_Manipulator::TIM_Calculate_PeriodElapsedCallback()
{
    static uint8_t mod3 = 0;
    mod3++;

    Output();

    Motor_J0.TIM_Process_PeriodElapsedCallback();

    // 分时发送CAN包
    switch(mod3)
    {
        case (1):
        {
            Motor_J1.Task_Process_PeriodElapsedCallback();
            Motor_J3.TIM_Process_PeriodElapsedCallback();
        }
        break;

        case (2):
        {
            Motor_J2.Task_Process_PeriodElapsedCallback();
            Motor_J4.TIM_Process_PeriodElapsedCallback();
        }
        break;

        case (3):
        {
            Motor_J5.TIM_Process_PeriodElapsedCallback();
            mod3 = 0;
        }
        break;
    }

    Update_Current_State();
}

void Class_Manipulator::TIM1msMod50_Alive_PeriodElapsedCallback()
{
    Motor_J0.TIM_Alive_PeriodElapsedCallback();
    Motor_J1.Task_Alive_PeriodElapsedCallback();
    Motor_J2.Task_Alive_PeriodElapsedCallback();
    Motor_J3.TIM_Alive_PeriodElapsedCallback();
    Motor_J4.TIM_Alive_PeriodElapsedCallback();
    Motor_J5.TIM_Alive_PeriodElapsedCallback();
}
