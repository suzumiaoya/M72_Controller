// SPDX-License-Identifier: AGPL-3.0-only
#include "ctl_manipulator.h"

#include "fdcan.h"
#include "usart.h"

#define MANIPULATOR_CAN_SCHEDULE_SLOT_COUNT          (8U)
#define MANIPULATOR_CAN_RESPONSE_TIMEOUT_TICKS       (3U)
#define MANIPULATOR_CAN_DISABLED_HEARTBEAT_DIVIDER   (10U)
#define MANIPULATOR_CAN_ANY_RESPONSE                 (0xffU)
#define MANIPULATOR_ZDT_POSITION_RESPONSE            (0x36U)
#define MANIPULATOR_ZDT_ENABLE_RESPONSE              (0xf3U)
#define MANIPULATOR_ZDT_STOP_RESPONSE                (0xfeU)

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

    CAN_Applied_Control_Status = Manipulator_Control_Status_DISABLE;
    CAN_Transition_Target = Manipulator_Control_Status_DISABLE;
    CAN_Transition_Active = 0U;
    CAN_Transition_Joint = Controller_Joint_ID_J1;
    CAN_Transition_Retry = 0U;
    CAN_Schedule_Slot = 0U;
    CAN_Superframe_Count = 0U;
    CAN_Transaction_Pending = 0U;
    CAN_Response_Timeout_Count = 0U;
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
        CAN_Complete_Pending_Transaction(Controller_Joint_ID_J1, CAN_RxMessage->Data[0]);
    }
    else if (CAN_RxMessage->Header.Identifier == 0x00 &&
        CAN_RxMessage->Data[0] == Joint_Binding[Controller_Joint_ID_J2].Device_ID)
    {
        Motor_J2.CAN_RxCpltCallback(CAN_RxMessage->Data);
        CAN_Complete_Pending_Transaction(Controller_Joint_ID_J2, CAN_RxMessage->Data[0]);
    }
    // ZDT电机CAN接收回调
    else if (CAN_RxMessage->Header.IdType == FDCAN_EXTENDED_ID &&
        (CAN_RxMessage->Header.Identifier >> 8) == Joint_Binding[Controller_Joint_ID_J3].Device_ID)
    {
        Motor_J3.CAN_RxCpltCallback(CAN_RxMessage);
        CAN_Complete_Pending_Transaction(Controller_Joint_ID_J3, CAN_RxMessage->Data[0]);
    }
    else if (CAN_RxMessage->Header.IdType == FDCAN_EXTENDED_ID &&
        (CAN_RxMessage->Header.Identifier >> 8) == Joint_Binding[Controller_Joint_ID_J4].Device_ID)
    {
        Motor_J4.CAN_RxCpltCallback(CAN_RxMessage);
        CAN_Complete_Pending_Transaction(Controller_Joint_ID_J4, CAN_RxMessage->Data[0]);
    }
    else if (CAN_RxMessage->Header.IdType == FDCAN_EXTENDED_ID &&
        (CAN_RxMessage->Header.Identifier >> 8) == Joint_Binding[Controller_Joint_ID_J5].Device_ID)
    {
        Motor_J5.CAN_RxCpltCallback(CAN_RxMessage);
        CAN_Complete_Pending_Transaction(Controller_Joint_ID_J5, CAN_RxMessage->Data[0]);
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

    Update_Current_State();
}

void Class_Manipulator::TIM_CAN_PeriodElapsedCallback()
{
    if (CAN_Transaction_Pending != 0U)
    {
        CAN_Response_Timeout_Count++;
        if (CAN_Response_Timeout_Count < MANIPULATOR_CAN_RESPONSE_TIMEOUT_TICKS)
        {
            return;
        }

        CAN_Transaction_Pending = 0U;
        CAN_Response_Timeout_Count = 0U;

        if (CAN_Pending_Is_Transition != 0U)
        {
            CAN_Transition_Retry++;
            if ((CAN_Transition_Target == Manipulator_Control_Status_ENABLE) &&
                (CAN_Transition_Retry >= 3U))
            {
                Manipulator_Control_Status = Manipulator_Control_Status_DISABLE;
                CAN_Transition_Target = Manipulator_Control_Status_DISABLE;
                CAN_Transition_Joint = Controller_Joint_ID_J1;
                CAN_Transition_Retry = 0U;
            }
            else if ((CAN_Transition_Target == Manipulator_Control_Status_DISABLE) &&
                     (CAN_Transition_Retry >= 2U))
            {
                CAN_Transition_Joint++;
                CAN_Transition_Retry = 0U;
            }
        }
        return;
    }

    if ((CAN_Transition_Active != 0U) && (CAN_Transition_Target != Manipulator_Control_Status))
    {
        CAN_Transition_Target = Manipulator_Control_Status;
        CAN_Transition_Joint = Controller_Joint_ID_J1;
        CAN_Transition_Retry = 0U;
    }
    else if ((CAN_Transition_Active == 0U) &&
             (CAN_Applied_Control_Status != Manipulator_Control_Status))
    {
        CAN_Transition_Active = 1U;
        CAN_Transition_Target = Manipulator_Control_Status;
        CAN_Transition_Joint = Controller_Joint_ID_J1;
        CAN_Transition_Retry = 0U;
    }

    if (CAN_Transition_Active != 0U)
    {
        if (CAN_Transition_Joint > Controller_Joint_ID_J5)
        {
            CAN_Applied_Control_Status = CAN_Transition_Target;
            CAN_Transition_Active = 0U;
            CAN_Schedule_Slot = 0U;
            CAN_Superframe_Count = (CAN_Applied_Control_Status == Manipulator_Control_Status_DISABLE) ? 1U : 0U;
        }
        else
        {
            (void)CAN_Send_Transition_Request();
            return;
        }
    }

    (void)CAN_Send_Scheduled_Request();
}

void Class_Manipulator::CAN_Set_Pending_Transaction(uint8_t Joint_ID, uint8_t Expected_Function, uint8_t Is_Transition)
{
    CAN_Pending_Joint = Joint_ID;
    CAN_Pending_Function = Expected_Function;
    CAN_Pending_Is_Transition = Is_Transition;
    CAN_Response_Timeout_Count = 0U;
    CAN_Transaction_Pending = 1U;
}

void Class_Manipulator::CAN_Complete_Pending_Transaction(uint8_t Joint_ID, uint8_t Response_Function)
{
    if ((CAN_Transaction_Pending == 0U) ||
        (CAN_Pending_Joint != Joint_ID) ||
        ((CAN_Pending_Function != MANIPULATOR_CAN_ANY_RESPONSE) &&
         (CAN_Pending_Function != Response_Function)))
    {
        return;
    }

    if ((CAN_Pending_Is_Transition != 0U) &&
        (CAN_Transition_Active != 0U) &&
        (CAN_Transition_Joint == Joint_ID))
    {
        CAN_Transition_Joint++;
        CAN_Transition_Retry = 0U;
    }

    CAN_Response_Timeout_Count = 0U;
    CAN_Transaction_Pending = 0U;
}

void Class_Manipulator::CAN_Advance_Schedule()
{
    CAN_Schedule_Slot++;
    if (CAN_Schedule_Slot >= MANIPULATOR_CAN_SCHEDULE_SLOT_COUNT)
    {
        CAN_Schedule_Slot = 0U;
        CAN_Superframe_Count++;
    }
}

uint8_t Class_Manipulator::CAN_Send_Transition_Request()
{
    uint8_t tmp_status = (uint8_t)HAL_ERROR;
    uint8_t tmp_expected_function = MANIPULATOR_CAN_ANY_RESPONSE;

    switch (CAN_Transition_Joint)
    {
        case (Controller_Joint_ID_J1):
        {
            tmp_status = Motor_J1.Send_Control_Status_Command(
                CAN_Transition_Target == Manipulator_Control_Status_ENABLE ?
                AK_Motor_Control_Status_ENABLE : AK_Motor_Control_Status_DISABLE);
        }
        break;
        case (Controller_Joint_ID_J2):
        {
            tmp_status = Motor_J2.Send_Control_Status_Command(
                CAN_Transition_Target == Manipulator_Control_Status_ENABLE ?
                AK_Motor_Control_Status_ENABLE : AK_Motor_Control_Status_DISABLE);
        }
        break;
        case (Controller_Joint_ID_J3):
        {
            tmp_expected_function = (CAN_Transition_Target == Manipulator_Control_Status_ENABLE) ?
                                    MANIPULATOR_ZDT_ENABLE_RESPONSE : MANIPULATOR_ZDT_STOP_RESPONSE;
            tmp_status = Motor_J3.Send_Control_Status_Command(
                CAN_Transition_Target == Manipulator_Control_Status_ENABLE ?
                ZDT_Motor_Control_Status_ENABLE : ZDT_Motor_Control_Status_DISABLE);
        }
        break;
        case (Controller_Joint_ID_J4):
        {
            tmp_expected_function = (CAN_Transition_Target == Manipulator_Control_Status_ENABLE) ?
                                    MANIPULATOR_ZDT_ENABLE_RESPONSE : MANIPULATOR_ZDT_STOP_RESPONSE;
            tmp_status = Motor_J4.Send_Control_Status_Command(
                CAN_Transition_Target == Manipulator_Control_Status_ENABLE ?
                ZDT_Motor_Control_Status_ENABLE : ZDT_Motor_Control_Status_DISABLE);
        }
        break;
        case (Controller_Joint_ID_J5):
        {
            tmp_expected_function = (CAN_Transition_Target == Manipulator_Control_Status_ENABLE) ?
                                    MANIPULATOR_ZDT_ENABLE_RESPONSE : MANIPULATOR_ZDT_STOP_RESPONSE;
            tmp_status = Motor_J5.Send_Control_Status_Command(
                CAN_Transition_Target == Manipulator_Control_Status_ENABLE ?
                ZDT_Motor_Control_Status_ENABLE : ZDT_Motor_Control_Status_DISABLE);
        }
        break;
        default:
        {
            return (uint8_t)HAL_ERROR;
        }
    }

    if (tmp_status == (uint8_t)HAL_OK)
    {
        CAN_Set_Pending_Transaction(CAN_Transition_Joint, tmp_expected_function, 1U);
    }
    return tmp_status;
}

uint8_t Class_Manipulator::CAN_Send_Scheduled_Request()
{
    uint8_t tmp_status = (uint8_t)HAL_OK;
    uint8_t tmp_joint = 0U;
    uint8_t tmp_expected_function = MANIPULATOR_CAN_ANY_RESPONSE;
    uint8_t tmp_send_request = 0U;
    uint8_t tmp_control_enabled = CAN_Applied_Control_Status == Manipulator_Control_Status_ENABLE ? 1U : 0U;
    uint8_t tmp_disabled_heartbeat = ((CAN_Superframe_Count % MANIPULATOR_CAN_DISABLED_HEARTBEAT_DIVIDER) == 0U) ? 1U : 0U;

    switch (CAN_Schedule_Slot)
    {
        case (0U):
        {
            tmp_joint = Controller_Joint_ID_J1;
            if (tmp_control_enabled != 0U)
            {
                tmp_status = Motor_J1.Send_Control_Command();
                tmp_send_request = 1U;
            }
            else if (tmp_disabled_heartbeat != 0U)
            {
                tmp_status = Motor_J1.Send_Control_Status_Command(AK_Motor_Control_Status_DISABLE);
                tmp_send_request = 1U;
            }
        }
        break;
        case (1U):
        {
            if (tmp_control_enabled != 0U)
            {
                tmp_joint = Controller_Joint_ID_J3;
                tmp_status = Motor_J3.Send_Control_Command();
                tmp_send_request = 1U;
            }
        }
        break;
        case (2U):
        {
            tmp_joint = Controller_Joint_ID_J2;
            if (tmp_control_enabled != 0U)
            {
                tmp_status = Motor_J2.Send_Control_Command();
                tmp_send_request = 1U;
            }
            else if (tmp_disabled_heartbeat != 0U)
            {
                tmp_status = Motor_J2.Send_Control_Status_Command(AK_Motor_Control_Status_DISABLE);
                tmp_send_request = 1U;
            }
        }
        break;
        case (3U):
        {
            if (tmp_control_enabled != 0U)
            {
                tmp_joint = Controller_Joint_ID_J4;
                tmp_status = Motor_J4.Send_Control_Command();
                tmp_send_request = 1U;
            }
        }
        break;
        case (4U):
        {
            if (tmp_control_enabled != 0U)
            {
                tmp_joint = Controller_Joint_ID_J5;
                tmp_status = Motor_J5.Send_Control_Command();
                tmp_send_request = 1U;
            }
        }
        break;
        case (5U):
        {
            tmp_joint = Controller_Joint_ID_J3;
            tmp_expected_function = MANIPULATOR_ZDT_POSITION_RESPONSE;
            tmp_status = Motor_J3.Send_Position_Query_Request();
            tmp_send_request = 1U;
        }
        break;
        case (6U):
        {
            tmp_joint = Controller_Joint_ID_J4;
            tmp_expected_function = MANIPULATOR_ZDT_POSITION_RESPONSE;
            tmp_status = Motor_J4.Send_Position_Query_Request();
            tmp_send_request = 1U;
        }
        break;
        case (7U):
        {
            tmp_joint = Controller_Joint_ID_J5;
            tmp_expected_function = MANIPULATOR_ZDT_POSITION_RESPONSE;
            tmp_status = Motor_J5.Send_Position_Query_Request();
            tmp_send_request = 1U;
        }
        break;
        default:
        {
            CAN_Schedule_Slot = 0U;
            return (uint8_t)HAL_ERROR;
        }
    }

    if (tmp_send_request == 0U)
    {
        CAN_Advance_Schedule();
        return (uint8_t)HAL_OK;
    }

    if (tmp_status == (uint8_t)HAL_OK)
    {
        CAN_Set_Pending_Transaction(tmp_joint, tmp_expected_function, 0U);
        CAN_Advance_Schedule();
    }
    else if (tmp_status != (uint8_t)HAL_BUSY)
    {
        CAN_Advance_Schedule();
    }

    return tmp_status;
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
