// SPDX-License-Identifier: AGPL-3.0-only
#include "ctl_manipulator.h"

#include "fdcan.h"
#include "usart.h"

#define MANIPULATOR_CAN_SCHEDULE_SLOT_COUNT       (16U)
#define MANIPULATOR_ZDT_POSITION_FEEDBACK_PERIOD  (4U)

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
        Joint_Angle_Alignment = Left_Arm_Joint_Alignment;
        Joint_Binding = Left_Arm_Joint_Binding;
    }
    else
    {
        Joint_Limit = Right_Arm_Joint_Limit;
        Joint_Angle_Alignment = Right_Arm_Joint_Alignment;
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

    Kinematics.Init();
    Dynamics.Init();

    CAN_Schedule_Slot = 0U;
    Update_Current_State();

    Motor_J3.Send_Timed_Query_Command(ZDT_Motor_Query_Type_POSITION,
                                      MANIPULATOR_ZDT_POSITION_FEEDBACK_PERIOD);
    Motor_J4.Send_Timed_Query_Command(ZDT_Motor_Query_Type_POSITION,
                                      MANIPULATOR_ZDT_POSITION_FEEDBACK_PERIOD);
    Motor_J5.Send_Timed_Query_Command(ZDT_Motor_Query_Type_POSITION,
                                      MANIPULATOR_ZDT_POSITION_FEEDBACK_PERIOD);
}

float Class_Manipulator::Motor_Angle_To_Joint_Angle(uint8_t Joint_ID, float Motor_Angle)
{
    if ((Joint_ID >= CONTROLLER_JOINT_NUM) || (Joint_Angle_Alignment == 0))
    {
        return (0.0f);
    }

    float tmp_direction = static_cast<float>(Joint_Angle_Alignment[Joint_ID].Direction);
    return (tmp_direction * (Motor_Angle - Joint_Angle_Alignment[Joint_ID].Motor_Angle_At_Reference));
}

float Class_Manipulator::Joint_Angle_To_Motor_Angle(uint8_t Joint_ID, float Joint_Angle)
{
    if ((Joint_ID >= CONTROLLER_JOINT_NUM) || (Joint_Angle_Alignment == 0))
    {
        return (0.0f);
    }

    float tmp_direction = static_cast<float>(Joint_Angle_Alignment[Joint_ID].Direction);
    return (Joint_Angle_Alignment[Joint_ID].Motor_Angle_At_Reference + tmp_direction * Joint_Angle);
}

float Class_Manipulator::Motor_Omega_To_Joint_Omega(uint8_t Joint_ID, float Motor_Omega)
{
    if ((Joint_ID >= CONTROLLER_JOINT_NUM) || (Joint_Angle_Alignment == 0))
    {
        return (0.0f);
    }

    float tmp_direction = static_cast<float>(Joint_Angle_Alignment[Joint_ID].Direction);
    return (tmp_direction * Motor_Omega);
}

float Class_Manipulator::Joint_Omega_To_Motor_Omega(uint8_t Joint_ID, float Joint_Omega)
{
    if ((Joint_ID >= CONTROLLER_JOINT_NUM) || (Joint_Angle_Alignment == 0))
    {
        return (0.0f);
    }

    float tmp_direction = static_cast<float>(Joint_Angle_Alignment[Joint_ID].Direction);
    return (tmp_direction * Joint_Omega);
}

float Class_Manipulator::Motor_Torque_To_Joint_Torque(uint8_t Joint_ID, float Motor_Torque)
{
    if ((Joint_ID >= CONTROLLER_JOINT_NUM) || (Joint_Angle_Alignment == 0))
    {
        return (0.0f);
    }

    float tmp_direction = static_cast<float>(Joint_Angle_Alignment[Joint_ID].Direction);
    return (tmp_direction * Motor_Torque);
}

float Class_Manipulator::Joint_Torque_To_Motor_Torque(uint8_t Joint_ID, float Joint_Torque)
{
    if ((Joint_ID >= CONTROLLER_JOINT_NUM) || (Joint_Angle_Alignment == 0))
    {
        return (0.0f);
    }

    float tmp_direction = static_cast<float>(Joint_Angle_Alignment[Joint_ID].Direction);
    return (tmp_direction * Joint_Torque);
}

/**
 * @brief 按当前关节角更新正运动学与重力补偿力矩
 *
 * 用Current_Joint_Angle而非Target_Joint_Angle: 重力矩取决于机械臂实际所处
 * 位姿, 用目标角度在跟踪误差较大时会补偿到错误的构型上
 */
void Class_Manipulator::Calculate_Model()
{
    Kinematics.Set_Joint_Angles(Current_Joint_Angle);
    Kinematics.Calculate();

    if (Manipulator_ID == Manipulator_ID_LEFT)
    {
        Dynamics.Set_Joint_Angles(Current_Joint_Angle);
        Dynamics.Calculate();

        for (uint8_t i = 0; i < CONTROLLER_JOINT_NUM; i++)
        {
            Gravity_Compensation_Torque[i] = Gravity_Compensation_Ratio[i] * Dynamics.Get_Gravity_Torque(i);
        }
    }
    else
    {
        for (uint8_t i = 0; i < CONTROLLER_JOINT_NUM; i++)
        {
            Gravity_Compensation_Torque[i] = 0.0f;
        }
    }
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

    Motor_J0.Set_Target_Angle(
        Joint_Angle_To_Motor_Angle(Controller_Joint_ID_J0, Target_Joint_Angle[Controller_Joint_ID_J0]));
    Motor_J1.Set_Target_Angle(
        Joint_Angle_To_Motor_Angle(Controller_Joint_ID_J1, Target_Joint_Angle[Controller_Joint_ID_J1]));
    Motor_J2.Set_Target_Angle(
        Joint_Angle_To_Motor_Angle(Controller_Joint_ID_J2, Target_Joint_Angle[Controller_Joint_ID_J2]));
    Motor_J3.Set_Target_Angle(
        Joint_Angle_To_Motor_Angle(Controller_Joint_ID_J3, Target_Joint_Angle[Controller_Joint_ID_J3]));
    Motor_J4.Set_Target_Angle(
        Joint_Angle_To_Motor_Angle(Controller_Joint_ID_J4, Target_Joint_Angle[Controller_Joint_ID_J4]));
    Motor_J5.Set_Target_Angle(
        Joint_Angle_To_Motor_Angle(Controller_Joint_ID_J5, Target_Joint_Angle[Controller_Joint_ID_J5]));

    Motor_J0.Set_Target_Omega(
        Joint_Omega_To_Motor_Omega(Controller_Joint_ID_J0, Target_Joint_Omega[Controller_Joint_ID_J0]));
    Motor_J1.Set_Target_Omega(
        Joint_Omega_To_Motor_Omega(Controller_Joint_ID_J1, Target_Joint_Omega[Controller_Joint_ID_J1]));
    Motor_J2.Set_Target_Omega(
        Joint_Omega_To_Motor_Omega(Controller_Joint_ID_J2, Target_Joint_Omega[Controller_Joint_ID_J2]));
    Motor_J3.Set_Target_Omega(
        Joint_Omega_To_Motor_Omega(Controller_Joint_ID_J3, Target_Joint_Omega[Controller_Joint_ID_J3]));
    Motor_J4.Set_Target_Omega(
        Joint_Omega_To_Motor_Omega(Controller_Joint_ID_J4, Target_Joint_Omega[Controller_Joint_ID_J4]));
    Motor_J5.Set_Target_Omega(
        Joint_Omega_To_Motor_Omega(Controller_Joint_ID_J5, Target_Joint_Omega[Controller_Joint_ID_J5]));

    // 重力补偿以前馈形式叠加, 不改动Target_Joint_Torque本身
    Motor_J0.Set_Target_Torque(
        Joint_Torque_To_Motor_Torque(Controller_Joint_ID_J0,
                                     Target_Joint_Torque[Controller_Joint_ID_J0]
                                         + Gravity_Compensation_Torque[Controller_Joint_ID_J0]));
    Motor_J1.Set_Target_Torque(
        Joint_Torque_To_Motor_Torque(Controller_Joint_ID_J1,
                                     Target_Joint_Torque[Controller_Joint_ID_J1]
                                         + Gravity_Compensation_Torque[Controller_Joint_ID_J1]));
    Motor_J2.Set_Target_Torque(
        Joint_Torque_To_Motor_Torque(Controller_Joint_ID_J2,
                                     Target_Joint_Torque[Controller_Joint_ID_J2]
                                         + Gravity_Compensation_Torque[Controller_Joint_ID_J2]));
    Motor_J3.Set_Target_Torque(
        Joint_Torque_To_Motor_Torque(Controller_Joint_ID_J3,
                                     Target_Joint_Torque[Controller_Joint_ID_J3]
                                         + Gravity_Compensation_Torque[Controller_Joint_ID_J3]));
    Motor_J4.Set_Target_Torque(
        Joint_Torque_To_Motor_Torque(Controller_Joint_ID_J4,
                                     Target_Joint_Torque[Controller_Joint_ID_J4]
                                         + Gravity_Compensation_Torque[Controller_Joint_ID_J4]));
    Motor_J5.Set_Target_Torque(
        Joint_Torque_To_Motor_Torque(Controller_Joint_ID_J5,
                                     Target_Joint_Torque[Controller_Joint_ID_J5]
                                         + Gravity_Compensation_Torque[Controller_Joint_ID_J5]));
}

void Class_Manipulator::Update_Current_State()
{
    Current_Joint_Angle[Controller_Joint_ID_J0] =
        Motor_Angle_To_Joint_Angle(Controller_Joint_ID_J0, Motor_J0.Get_Now_Angle());
    Current_Joint_Angle[Controller_Joint_ID_J1] =
        Motor_Angle_To_Joint_Angle(Controller_Joint_ID_J1, Motor_J1.Get_Now_Angle());
    Current_Joint_Angle[Controller_Joint_ID_J2] =
        Motor_Angle_To_Joint_Angle(Controller_Joint_ID_J2, Motor_J2.Get_Now_Angle());
    Current_Joint_Angle[Controller_Joint_ID_J3] =
        Motor_Angle_To_Joint_Angle(Controller_Joint_ID_J3, Motor_J3.Get_Now_Angle());
    Current_Joint_Angle[Controller_Joint_ID_J4] =
        Motor_Angle_To_Joint_Angle(Controller_Joint_ID_J4, Motor_J4.Get_Now_Angle());
    Current_Joint_Angle[Controller_Joint_ID_J5] =
        Motor_Angle_To_Joint_Angle(Controller_Joint_ID_J5, Motor_J5.Get_Now_Angle());

    Current_Joint_Omega[Controller_Joint_ID_J0] =
        Motor_Omega_To_Joint_Omega(Controller_Joint_ID_J0, Motor_J0.Get_Now_Omega());
    Current_Joint_Omega[Controller_Joint_ID_J1] =
        Motor_Omega_To_Joint_Omega(Controller_Joint_ID_J1, Motor_J1.Get_Now_Omega());
    Current_Joint_Omega[Controller_Joint_ID_J2] =
        Motor_Omega_To_Joint_Omega(Controller_Joint_ID_J2, Motor_J2.Get_Now_Omega());
    Current_Joint_Omega[Controller_Joint_ID_J3] =
        Motor_Omega_To_Joint_Omega(Controller_Joint_ID_J3, Motor_J3.Get_Now_Omega());
    Current_Joint_Omega[Controller_Joint_ID_J4] =
        Motor_Omega_To_Joint_Omega(Controller_Joint_ID_J4, Motor_J4.Get_Now_Omega());
    Current_Joint_Omega[Controller_Joint_ID_J5] =
        Motor_Omega_To_Joint_Omega(Controller_Joint_ID_J5, Motor_J5.Get_Now_Omega());

    Current_Joint_Torque[Controller_Joint_ID_J0] =
        Motor_Torque_To_Joint_Torque(Controller_Joint_ID_J0, Motor_J0.Get_Now_Torque());
    Current_Joint_Torque[Controller_Joint_ID_J1] =
        Motor_Torque_To_Joint_Torque(Controller_Joint_ID_J1, Motor_J1.Get_Now_Torque());
    Current_Joint_Torque[Controller_Joint_ID_J2] =
        Motor_Torque_To_Joint_Torque(Controller_Joint_ID_J2, Motor_J2.Get_Now_Torque());
    Current_Joint_Torque[Controller_Joint_ID_J3] =
        Motor_Torque_To_Joint_Torque(Controller_Joint_ID_J3, Motor_J3.Get_Now_Torque());
    Current_Joint_Torque[Controller_Joint_ID_J4] =
        Motor_Torque_To_Joint_Torque(Controller_Joint_ID_J4, Motor_J4.Get_Now_Torque());
    Current_Joint_Torque[Controller_Joint_ID_J5] =
        Motor_Torque_To_Joint_Torque(Controller_Joint_ID_J5, Motor_J5.Get_Now_Torque());
}

void Class_Manipulator::CAN_RxCpltCallback(Struct_CAN_Rx_Buffer *CAN_RxMessage)
{
    if (CAN_RxMessage == 0)
    {
        return;
    }

    if ((CAN_RxMessage->Header.Identifier == 0x00U) &&
        (CAN_RxMessage->Data[0] == Joint_Binding[Controller_Joint_ID_J1].Device_ID))
    {
        Motor_J1.CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    else if ((CAN_RxMessage->Header.Identifier == 0x00U) &&
             (CAN_RxMessage->Data[0] == Joint_Binding[Controller_Joint_ID_J2].Device_ID))
    {
        Motor_J2.CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    else if ((CAN_RxMessage->Header.IdType == FDCAN_EXTENDED_ID) &&
             ((CAN_RxMessage->Header.Identifier >> 8) == Joint_Binding[Controller_Joint_ID_J3].Device_ID))
    {
        Motor_J3.CAN_RxCpltCallback(CAN_RxMessage);
    }
    else if ((CAN_RxMessage->Header.IdType == FDCAN_EXTENDED_ID) &&
             ((CAN_RxMessage->Header.Identifier >> 8) == Joint_Binding[Controller_Joint_ID_J4].Device_ID))
    {
        Motor_J4.CAN_RxCpltCallback(CAN_RxMessage);
    }
    else if ((CAN_RxMessage->Header.IdType == FDCAN_EXTENDED_ID) &&
             ((CAN_RxMessage->Header.Identifier >> 8) == Joint_Binding[Controller_Joint_ID_J5].Device_ID))
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
    Calculate_Model();

    Output();

    Motor_J0.TIM_Process_PeriodElapsedCallback();
    Motor_J3.TIM_PID_PeriodElapsedCallback();
    Motor_J4.TIM_PID_PeriodElapsedCallback();
    Motor_J5.TIM_PID_PeriodElapsedCallback();

    Update_Current_State();
}

void Class_Manipulator::TIM_CAN_PeriodElapsedCallback()
{
    switch (CAN_Schedule_Slot)
    {
        case (0U):
        case (8U):
        {
            Motor_J1.Task_Process_PeriodElapsedCallback();
        }
        break;

        case (1U):
        case (9U):
        {
            Motor_J2.Task_Process_PeriodElapsedCallback();
        }
        break;

        case (2U):
        case (10U):
        {
            (void)Motor_J3.Send_Control_Command();
        }
        break;

        case (3U):
        case (11U):
        {
            (void)Motor_J4.Send_Control_Command();
        }
        break;

        case (4U):
        case (12U):
        {
            (void)Motor_J5.Send_Control_Command();
        }
        break;

        case (5U):
        {
            Motor_J3.Send_Query_Command(ZDT_Motor_Query_Type_CURRENT);
        }
        break;

        case (6U):
        {
            Motor_J4.Send_Query_Command(ZDT_Motor_Query_Type_CURRENT);
        }
        break;

        case (7U):
        {
            Motor_J5.Send_Query_Command(ZDT_Motor_Query_Type_CURRENT);
        }
        break;

        case (13U):
        {
            Motor_J3.Send_Query_Command(ZDT_Motor_Query_Type_OMEGA);
        }
        break;

        case (14U):
        {
            Motor_J4.Send_Query_Command(ZDT_Motor_Query_Type_OMEGA);
        }
        break;

        case (15U):
        {
            Motor_J5.Send_Query_Command(ZDT_Motor_Query_Type_OMEGA);
        }
        break;

        default:
        {
        }
        break;
    }

    CAN_Schedule_Slot++;
    if (CAN_Schedule_Slot >= MANIPULATOR_CAN_SCHEDULE_SLOT_COUNT)
    {
        CAN_Schedule_Slot = 0U;
    }
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
