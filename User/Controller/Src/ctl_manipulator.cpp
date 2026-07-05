#include "ctl_manipulator.h"

#include "fdcan.h"
#include "usart.h"

void Class_Manipulator::Init(Enum_Manipulator_ID __Manipulator_ID)
{
    Manipulator_ID = __Manipulator_ID;

    if (Manipulator_ID == Manipulator_ID_LEFT)
    {
        Joint_Limit = Left_Arm_Joint_Limit;
        Joint_Binding = Left_Arm_Joint_Binding;

        Motor_J1.Init(&hfdcan1, Joint_Binding[Controller_Joint_ID_J1].Device_ID);
        Motor_J2.Init(&hfdcan1, Joint_Binding[Controller_Joint_ID_J2].Device_ID);
        Motor_J3.Init(&hfdcan1, Joint_Binding[Controller_Joint_ID_J3].Device_ID);
        Motor_J4.Init(&hfdcan1, Joint_Binding[Controller_Joint_ID_J4].Device_ID);
        Motor_J5.Init(&hfdcan1, Joint_Binding[Controller_Joint_ID_J5].Device_ID);
    }
    else
    {
        Joint_Limit = Right_Arm_Joint_Limit;
        Joint_Binding = Right_Arm_Joint_Binding;

        Motor_J1.Init(&hfdcan2, Joint_Binding[Controller_Joint_ID_J1].Device_ID);
        Motor_J2.Init(&hfdcan2, Joint_Binding[Controller_Joint_ID_J2].Device_ID);
        Motor_J3.Init(&hfdcan2, Joint_Binding[Controller_Joint_ID_J3].Device_ID);
        Motor_J4.Init(&hfdcan2, Joint_Binding[Controller_Joint_ID_J4].Device_ID);
        Motor_J5.Init(&hfdcan2, Joint_Binding[Controller_Joint_ID_J5].Device_ID);
    }

    Motor_J0.Init(&UART5_Manage_Object, Joint_Binding[Controller_Joint_ID_J0].Device_ID);
    Update_Current_State();
}

void Class_Manipulator::Output()
{
    if (Manipulator_Control_Status == Manipulator_Control_Status_DISABLE)
    {
        Motor_J0.Set_Unitree_Motor_Control_Status(Unitree_Motor_Control_Status_DISABLE);
        Motor_J1.Set_AK80_Motor_Control_Status(AK80_Motor_Control_Status_DISABLE);
        Motor_J2.Set_AK80_Motor_Control_Status(AK80_Motor_Control_Status_DISABLE);
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
    Motor_J1.Set_AK80_Motor_Control_Status(AK80_Motor_Control_Status_ENABLE);
    Motor_J2.Set_AK80_Motor_Control_Status(AK80_Motor_Control_Status_ENABLE);
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
    if (CAN_RxMessage->Header.Identifier == Joint_Binding[Controller_Joint_ID_J1].Device_ID)
    {
        Motor_J1.CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    else if (CAN_RxMessage->Header.Identifier == Joint_Binding[Controller_Joint_ID_J2].Device_ID)
    {
        Motor_J2.CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    else if (CAN_RxMessage->Header.Identifier == Joint_Binding[Controller_Joint_ID_J3].Device_ID)
    {
        Motor_J3.CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    else if (CAN_RxMessage->Header.Identifier == Joint_Binding[Controller_Joint_ID_J4].Device_ID)
    {
        Motor_J4.CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    else if (CAN_RxMessage->Header.Identifier == Joint_Binding[Controller_Joint_ID_J5].Device_ID)
    {
        Motor_J5.CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
}

void Class_Manipulator::UART_RxCpltCallback(uint8_t *Buffer, uint16_t Length)
{
    Motor_J0.UART_RxCpltCallback(Buffer, Length);
}

void Class_Manipulator::TIM_Calculate_PeriodElapsedCallback()
{
    Output();

    Motor_J0.TIM_Process_PeriodElapsedCallback();
    Motor_J1.TIM_Process_PeriodElapsedCallback();
    Motor_J2.TIM_Process_PeriodElapsedCallback();
    Motor_J3.TIM_Process_PeriodElapsedCallback();
    Motor_J4.TIM_Process_PeriodElapsedCallback();
    Motor_J5.TIM_Process_PeriodElapsedCallback();

    Update_Current_State();
}

void Class_Manipulator::TIM1msMod50_Alive_PeriodElapsedCallback()
{
    Motor_J0.TIM_Alive_PeriodElapsedCallback();
    Motor_J1.TIM_Alive_PeriodElapsedCallback();
    Motor_J2.TIM_Alive_PeriodElapsedCallback();
    Motor_J3.TIM_Alive_PeriodElapsedCallback();
    Motor_J4.TIM_Alive_PeriodElapsedCallback();
    Motor_J5.TIM_Alive_PeriodElapsedCallback();
}
