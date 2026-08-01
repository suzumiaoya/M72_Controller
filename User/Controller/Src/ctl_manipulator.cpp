// SPDX-License-Identifier: AGPL-3.0-only
#include "ctl_manipulator.h"

#include "fdcan.h"
#include "usart.h"
#include <float.h>

#define MANIPULATOR_CAN_SCHEDULE_SLOT_COUNT       (20U)
#define MANIPULATOR_ZDT_POSITION_FEEDBACK_PERIOD  (4U)
#define STATIC_IDENTIFY_AK_SLOT_J1                (0U)
#define STATIC_IDENTIFY_AK_SLOT_J2                (5U)
#define STATIC_IDENTIFY_ZDT_SLOT_J3               (10U)
#define STATIC_IDENTIFY_ZDT_SLOT_J4               (11U)
#define STATIC_IDENTIFY_ZDT_SLOT_J5               (12U)
#define STATIC_IDENTIFY_SETTLE_TIME_MS            (1000U)
#define STATIC_IDENTIFY_SAMPLE_TIME_MS            (1000U)
#define STATIC_IDENTIFY_POSITION_TOLERANCE        (0.035f)
#define STATIC_IDENTIFY_OMEGA_TOLERANCE           (0.10f)
#define STATIC_IDENTIFY_REVERSE_APPROACH          (0.080f)
#define STATIC_IDENTIFY_FEEDBACK_TIMEOUT_MS       (2000U)
#define STATIC_IDENTIFY_MOVE_TIMEOUT_MS           (15000U)
#define STATIC_IDENTIFY_PROFILE_OMEGA_MIN         (0.02f)
#define STATIC_IDENTIFY_PROFILE_OMEGA_MAX         (0.50f)

enum Enum_Static_Identify_Fault_Code : uint8_t
{
    Static_Identify_Fault_NONE = 0U,
    Static_Identify_Fault_FEEDBACK_TIMEOUT = 1U,
    Static_Identify_Fault_MOVE_TIMEOUT = 2U,
    Static_Identify_Fault_ABORTED = 3U,
};

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

static float Static_Identify_Constrain_Workspace(float Target, const Struct_Joint_Limit *Limit)
{
    if ((Limit == 0) || (Limit->Min_Angle <= (-FLT_MAX * 0.5f)) ||
        (Limit->Max_Angle >= (FLT_MAX * 0.5f)))
    {
        return Target;
    }

    float tmp_center = 0.5f * (Limit->Min_Angle + Limit->Max_Angle);
    float tmp_half_range = 0.3f * (Limit->Max_Angle - Limit->Min_Angle);
    Math_Constrain(&Target, tmp_center - tmp_half_range, tmp_center + tmp_half_range);
    return Target;
}

void Class_Manipulator::Init(Enum_Manipulator_ID __Manipulator_ID)
{
    Manipulator_ID = __Manipulator_ID;
    Static_Identify_Enabled = Manipulator_ID == Static_Identify_Active_Arm ? 1U : 0U;

    const Struct_Dynamics_Link_Param *tmp_link_params = 0;

    if (Manipulator_ID == Manipulator_ID_LEFT)
    {
        Joint_Limit = Left_Arm_Joint_Limit;
        Joint_Angle_Alignment = Left_Arm_Joint_Alignment;
        Joint_Binding = Left_Arm_Joint_Binding;
        tmp_link_params = Left_Dynamics_Link_Param;
    }
    else
    {
        Joint_Limit = Right_Arm_Joint_Limit;
        Joint_Angle_Alignment = Right_Arm_Joint_Alignment;
        Joint_Binding = Right_Arm_Joint_Binding;
        tmp_link_params = Right_Dynamics_Link_Param;
    }

    Enum_ZDT_Motor_Control_Method tmp_zdt_method = Static_Identify_Enabled != 0U ?
        ZDT_Motor_Control_Method_EMMX_POSITION : ZDT_Motor_Control_Method_TORQUE_MIT;

    Motor_J0.Init(Get_UART_Manage_Object(Joint_Binding[Controller_Joint_ID_J0].Bus_ID),
                  Joint_Binding[Controller_Joint_ID_J0].Device_ID,
                  UNITREE_MOTOR_DEFAULT_GEAR_RATIO,
                  Static_Identify_Runtime.MIT_K_P[Controller_Joint_ID_J0],
                  Static_Identify_Runtime.MIT_K_D[Controller_Joint_ID_J0]);
    Motor_J1.Init(Get_CAN_Handler(Joint_Binding[Controller_Joint_ID_J1].Bus_ID),
                  static_cast<Enum_AK_Motor_ID>(Joint_Binding[Controller_Joint_ID_J1].Device_ID),
                  AK_CONTROL_METHOD_MIT,
                  Static_Identify_Runtime.MIT_K_P[Controller_Joint_ID_J1],
                  Static_Identify_Runtime.MIT_K_D[Controller_Joint_ID_J1]);
    Motor_J2.Init(Get_CAN_Handler(Joint_Binding[Controller_Joint_ID_J2].Bus_ID),
                  static_cast<Enum_AK_Motor_ID>(Joint_Binding[Controller_Joint_ID_J2].Device_ID),
                  AK_CONTROL_METHOD_MIT,
                  Static_Identify_Runtime.MIT_K_P[Controller_Joint_ID_J2],
                  Static_Identify_Runtime.MIT_K_D[Controller_Joint_ID_J2]);
    Motor_J3.Init(Get_CAN_Handler(Joint_Binding[Controller_Joint_ID_J3].Bus_ID),
                  Joint_Binding[Controller_Joint_ID_J3].Device_ID,
                  tmp_zdt_method,
                  ZDT_L60_MAX_TORQUE);
    Motor_J4.Init(Get_CAN_Handler(Joint_Binding[Controller_Joint_ID_J4].Bus_ID),
                  Joint_Binding[Controller_Joint_ID_J4].Device_ID,
                  tmp_zdt_method,
                  ZDT_L40_MAX_TORQUE);
    Motor_J5.Init(Get_CAN_Handler(Joint_Binding[Controller_Joint_ID_J5].Bus_ID),
                  Joint_Binding[Controller_Joint_ID_J5].Device_ID,
                  tmp_zdt_method,
                  ZDT_L40_MAX_TORQUE);

    if (Static_Identify_Enabled != 0U)
    {
        Motor_J3.Set_Emmx_Position_Config(ZDT_EMMX_PULSES_PER_REVOLUTION, ZDT_EMMX_MAX_RPM,
                                           ZDT_EMMX_DEFAULT_ACCELERATION, ZDT_EMMX_DEFAULT_RPM);
        Motor_J4.Set_Emmx_Position_Config(ZDT_EMMX_PULSES_PER_REVOLUTION, ZDT_EMMX_MAX_RPM,
                                           ZDT_EMMX_DEFAULT_ACCELERATION, ZDT_EMMX_DEFAULT_RPM);
        Motor_J5.Set_Emmx_Position_Config(ZDT_EMMX_PULSES_PER_REVOLUTION, ZDT_EMMX_MAX_RPM,
                                           ZDT_EMMX_DEFAULT_ACCELERATION, ZDT_EMMX_DEFAULT_RPM);
    }

    Kinematics.Init();
    Dynamics.Init();
    Dynamics.Set_Link_Params(tmp_link_params);

    CAN_Schedule_Slot = 0U;
    Static_Identify_State = Static_Identify_State_IDLE;
    Static_Identify_Pose_Index = 0U;
    Static_Identify_State_Tick = 0U;
    Static_Identify_Millisecond = 0U;
    Static_Identify_Fault_Code = 0U;
    Static_Identify_Start_Armed = 0U;
    CAN_Tx_Error_Count = 0U;
    CAN_Tx_Busy_Count = 0U;
    CAN_Tx_Min_Free_Level = 0xffU;
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
    // 运动学计算
    Kinematics.Set_Joint_Angles(Current_Joint_Angle);
    Kinematics.Fkine();

    // 动力学计算
    Dynamics.Set_Joint_Angles(Current_Joint_Angle);
    Dynamics.Calculate();

    for (uint8_t i = 0; i < CONTROLLER_JOINT_NUM; i++)
    {
        Gravity_Compensation_Torque[i] =
            Gravity_Compensation_Ratio[i] * Dynamics.Get_Gravity_Torque(i);
    }
}

void Class_Manipulator::Output()
{
    if (Static_Identify_Enabled != 0U)
    {
        uint8_t tmp_zdt_enable = (Static_Identify_State >= Static_Identify_State_WAIT_TUNING) &&
                                 (Static_Identify_State != Static_Identify_State_FAULT);
        uint8_t tmp_upstream_enable = (Static_Identify_State >= Static_Identify_State_MOVE_TO_POSE) &&
                                      (Static_Identify_State != Static_Identify_State_FAULT);

        Motor_J0.Set_Unitree_Motor_Control_Status(tmp_upstream_enable != 0U ?
            Unitree_Motor_Control_Status_ENABLE : Unitree_Motor_Control_Status_DISABLE);
        Motor_J1.Set_AK_Control_Status(tmp_upstream_enable != 0U ?
            AK_Motor_Control_Status_ENABLE : AK_Motor_Control_Status_DISABLE);
        Motor_J2.Set_AK_Control_Status(tmp_upstream_enable != 0U ?
            AK_Motor_Control_Status_ENABLE : AK_Motor_Control_Status_DISABLE);
        Motor_J3.Set_ZDT_Motor_Control_Status(tmp_zdt_enable != 0U ?
            ZDT_Motor_Control_Status_ENABLE : ZDT_Motor_Control_Status_DISABLE);
        Motor_J4.Set_ZDT_Motor_Control_Status(tmp_zdt_enable != 0U ?
            ZDT_Motor_Control_Status_ENABLE : ZDT_Motor_Control_Status_DISABLE);
        Motor_J5.Set_ZDT_Motor_Control_Status(tmp_zdt_enable != 0U ?
            ZDT_Motor_Control_Status_ENABLE : ZDT_Motor_Control_Status_DISABLE);

        Motor_J0.Set_Target_Angle(Joint_Angle_To_Motor_Angle(
            Controller_Joint_ID_J0, Target_Joint_Angle[Controller_Joint_ID_J0]));
        Motor_J1.Set_Target_Angle(Joint_Angle_To_Motor_Angle(
            Controller_Joint_ID_J1, Target_Joint_Angle[Controller_Joint_ID_J1]));
        Motor_J2.Set_Target_Angle(Joint_Angle_To_Motor_Angle(
            Controller_Joint_ID_J2, Target_Joint_Angle[Controller_Joint_ID_J2]));
        Motor_J3.Set_Target_Angle(Joint_Angle_To_Motor_Angle(
            Controller_Joint_ID_J3, Target_Joint_Angle[Controller_Joint_ID_J3]));
        Motor_J4.Set_Target_Angle(Joint_Angle_To_Motor_Angle(
            Controller_Joint_ID_J4, Target_Joint_Angle[Controller_Joint_ID_J4]));
        Motor_J5.Set_Target_Angle(Joint_Angle_To_Motor_Angle(
            Controller_Joint_ID_J5, Target_Joint_Angle[Controller_Joint_ID_J5]));

        Motor_J0.Set_Target_Omega(Joint_Omega_To_Motor_Omega(
            Controller_Joint_ID_J0, Target_Joint_Omega[Controller_Joint_ID_J0]));
        Motor_J1.Set_Target_Omega(Joint_Omega_To_Motor_Omega(
            Controller_Joint_ID_J1, Target_Joint_Omega[Controller_Joint_ID_J1]));
        Motor_J2.Set_Target_Omega(Joint_Omega_To_Motor_Omega(
            Controller_Joint_ID_J2, Target_Joint_Omega[Controller_Joint_ID_J2]));
        Motor_J3.Set_Target_Omega(Joint_Omega_To_Motor_Omega(
            Controller_Joint_ID_J3, Target_Joint_Omega[Controller_Joint_ID_J3]));
        Motor_J4.Set_Target_Omega(Joint_Omega_To_Motor_Omega(
            Controller_Joint_ID_J4, Target_Joint_Omega[Controller_Joint_ID_J4]));
        Motor_J5.Set_Target_Omega(Joint_Omega_To_Motor_Omega(
            Controller_Joint_ID_J5, Target_Joint_Omega[Controller_Joint_ID_J5]));

        Motor_J0.Set_Target_Torque(tmp_upstream_enable != 0U ?
            Joint_Torque_To_Motor_Torque(Controller_Joint_ID_J0,
                Target_Joint_Torque[Controller_Joint_ID_J0] +
                Gravity_Compensation_Torque[Controller_Joint_ID_J0]) : 0.0f);
        Motor_J1.Set_Target_Torque(tmp_upstream_enable != 0U ?
            Joint_Torque_To_Motor_Torque(Controller_Joint_ID_J1,
                Target_Joint_Torque[Controller_Joint_ID_J1] +
                Gravity_Compensation_Torque[Controller_Joint_ID_J1]) : 0.0f);
        Motor_J2.Set_Target_Torque(tmp_upstream_enable != 0U ?
            Joint_Torque_To_Motor_Torque(Controller_Joint_ID_J2,
                Target_Joint_Torque[Controller_Joint_ID_J2] +
                Gravity_Compensation_Torque[Controller_Joint_ID_J2]) : 0.0f);
        return;
    }

    // 失能模式
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

    // 非失能模式

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

    // 电机层设置目标角度，角速度与力矩
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


    // 角速度
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

void Class_Manipulator::Static_Identify_Set_State(Enum_Static_Identify_State State)
{
    Static_Identify_State = State;
    Static_Identify_State_Tick = 0U;
    Static_Identify_Start_Armed = 0U;
}

void Class_Manipulator::Static_Identify_Set_Target(const float *Joint_Angle)
{
    if (Joint_Angle == 0)
    {
        return;
    }

    for (uint8_t i = 0U; i < CONTROLLER_JOINT_NUM; i++)
    {
        Static_Identify_Target[i] = Static_Identify_Constrain_Workspace(Joint_Angle[i], &Joint_Limit[i]);
    }
}

void Class_Manipulator::Static_Identify_Update_J0_J2_Profile()
{
    float tmp_profile_omega = Static_Identify_Runtime.Joint_Profile_Omega;
    Math_Constrain(&tmp_profile_omega, STATIC_IDENTIFY_PROFILE_OMEGA_MIN,
                   STATIC_IDENTIFY_PROFILE_OMEGA_MAX);
    float tmp_step = tmp_profile_omega * 0.001f;

    float tmp_zdt_omega = Static_Identify_Runtime.ZDT_Target_Omega;
    Math_Constrain(&tmp_zdt_omega, 0.0f, ZDT_MOTOR_DEFAULT_MAX_OMEGA);

    for (uint8_t i = 0U; i < CONTROLLER_JOINT_NUM; i++)
    {
        float tmp_delta = Static_Identify_Target[i] - Target_Joint_Angle[i];
        if (Math_Abs(tmp_delta) <= tmp_step)
        {
            Target_Joint_Angle[i] = Static_Identify_Target[i];
            Target_Joint_Omega[i] = 0.0f;
        }
        else
        {
            Target_Joint_Angle[i] += tmp_delta > 0.0f ? tmp_step : -tmp_step;
            Target_Joint_Omega[i] = i <= Controller_Joint_ID_J2 ?
                (tmp_delta > 0.0f ? tmp_profile_omega : -tmp_profile_omega) : tmp_zdt_omega;
        }
    }
}

uint8_t Class_Manipulator::Static_Identify_At_Target()
{
    for (uint8_t i = 0U; i < CONTROLLER_JOINT_NUM; i++)
    {
        if ((Math_Abs(Static_Identify_Target[i] - Target_Joint_Angle[i]) > 0.0001f) ||
            (Math_Abs(Static_Identify_Target[i] - Current_Joint_Angle[i]) >
             STATIC_IDENTIFY_POSITION_TOLERANCE) ||
            (Math_Abs(Current_Joint_Omega[i]) > STATIC_IDENTIFY_OMEGA_TOLERANCE))
        {
            return 0U;
        }
    }

    return 1U;
}

uint8_t Class_Manipulator::Static_Identify_Feedback_Ready()
{
    return (Motor_J3.Get_Position_Feedback_Valid() != 0U) &&
           (Motor_J4.Get_Position_Feedback_Valid() != 0U) &&
           (Motor_J5.Get_Position_Feedback_Valid() != 0U);
}

void Class_Manipulator::Static_Identify_Update_Monitor()
{
    Static_Identify_Monitor.Millisecond = Static_Identify_Millisecond;
    Static_Identify_Monitor.Pose_Index = Static_Identify_Pose_Index;
    Static_Identify_Monitor.State = static_cast<uint8_t>(Static_Identify_State);
    Static_Identify_Monitor.Fault_Code = Static_Identify_Fault_Code;
    Static_Identify_Monitor.CAN_Tx_Error_Count = CAN_Tx_Error_Count;
    Static_Identify_Monitor.CAN_Tx_Busy_Count = CAN_Tx_Busy_Count;
    Static_Identify_Monitor.CAN_Tx_Min_Free_Level = CAN_Tx_Min_Free_Level;

    for (uint8_t i = 0U; i < CONTROLLER_JOINT_NUM; i++)
    {
        Static_Identify_Monitor.Target_Joint_Angle[i] = Target_Joint_Angle[i];
        Static_Identify_Monitor.Current_Joint_Angle[i] = Current_Joint_Angle[i];
        Static_Identify_Monitor.Current_Joint_Omega[i] = Current_Joint_Omega[i];
    }

    for (uint8_t i = 0U; i <= Controller_Joint_ID_J2; i++)
    {
        Static_Identify_Monitor.Current_Joint_Torque[i] = Current_Joint_Torque[i];
        Static_Identify_Monitor.Gravity_Compensation_Torque[i] = Gravity_Compensation_Torque[i];
    }
}

void Class_Manipulator::Static_Identify_Record_CAN_Status(uint8_t Status)
{
    FDCAN_HandleTypeDef *tmp_handler = Get_CAN_Handler(Joint_Binding[Controller_Joint_ID_J1].Bus_ID);
    if (tmp_handler != 0)
    {
        uint32_t tmp_free_level = HAL_FDCAN_GetTxFifoFreeLevel(tmp_handler);
        if (tmp_free_level < CAN_Tx_Min_Free_Level)
        {
            CAN_Tx_Min_Free_Level = static_cast<uint8_t>(tmp_free_level);
        }
    }

    if (Status == static_cast<uint8_t>(HAL_BUSY))
    {
        CAN_Tx_Busy_Count++;
    }
    else if (Status != static_cast<uint8_t>(HAL_OK))
    {
        CAN_Tx_Error_Count++;
    }
}

void Class_Manipulator::Static_Identify_PeriodElapsedCallback()
{
    if (Static_Identify_Enabled == 0U)
    {
        return;
    }

    Static_Identify_Millisecond++;
    Static_Identify_State_Tick++;

    float tmp_gravity_ratio = Static_Identify_Runtime.Gravity_Compensation_Ratio;
    Math_Constrain(&tmp_gravity_ratio, 0.0f, 2.0f);
    for (uint8_t i = 0U; i <= Controller_Joint_ID_J2; i++)
    {
        Gravity_Compensation_Ratio[i] = tmp_gravity_ratio;
    }
    for (uint8_t i = Controller_Joint_ID_J3; i < CONTROLLER_JOINT_NUM; i++)
    {
        Gravity_Compensation_Ratio[i] = 0.0f;
    }

    Motor_J0.Set_MIT_K_P(Static_Identify_Runtime.MIT_K_P[Controller_Joint_ID_J0]);
    Motor_J0.Set_MIT_K_D(Static_Identify_Runtime.MIT_K_D[Controller_Joint_ID_J0]);
    Motor_J1.Set_MIT_K_P(Static_Identify_Runtime.MIT_K_P[Controller_Joint_ID_J1]);
    Motor_J1.Set_MIT_K_D(Static_Identify_Runtime.MIT_K_D[Controller_Joint_ID_J1]);
    Motor_J2.Set_MIT_K_P(Static_Identify_Runtime.MIT_K_P[Controller_Joint_ID_J2]);
    Motor_J2.Set_MIT_K_D(Static_Identify_Runtime.MIT_K_D[Controller_Joint_ID_J2]);

    if (Static_Identify_Runtime.Abort_Request != 0U)
    {
        Static_Identify_Fault_Code = Static_Identify_Fault_ABORTED;
        Static_Identify_Set_State(Static_Identify_State_FAULT);
        return;
    }

    const Struct_Static_Identify_Pose *tmp_pose = Manipulator_ID == Manipulator_ID_LEFT ?
        Static_Identify_Left_Pose : Static_Identify_Right_Pose;

    switch (Static_Identify_State)
    {
        case (Static_Identify_State_IDLE):
        {
            if (Static_Identify_Runtime.Start_Request != 0U)
            {
                Static_Identify_Fault_Code = Static_Identify_Fault_NONE;
                Static_Identify_Set_State(Static_Identify_State_WAIT_FEEDBACK);
            }
        }
        break;

        case (Static_Identify_State_WAIT_FEEDBACK):
        {
            if (Static_Identify_Feedback_Ready() != 0U)
            {
                for (uint8_t i = 0U; i < CONTROLLER_JOINT_NUM; i++)
                {
                    Target_Joint_Angle[i] = Current_Joint_Angle[i];
                    Target_Joint_Omega[i] = 0.0f;
                    Target_Joint_Torque[i] = 0.0f;
                    Static_Identify_Target[i] = Current_Joint_Angle[i];
                }
                Static_Identify_Set_State(Static_Identify_State_WAIT_TUNING);
            }
            else if (Static_Identify_State_Tick >= STATIC_IDENTIFY_FEEDBACK_TIMEOUT_MS)
            {
                Static_Identify_Fault_Code = Static_Identify_Fault_FEEDBACK_TIMEOUT;
                Static_Identify_Set_State(Static_Identify_State_FAULT);
            }
        }
        break;

        case (Static_Identify_State_WAIT_TUNING):
        {
            if (Static_Identify_Runtime.Start_Request == 0U)
            {
                Static_Identify_Start_Armed = 1U;
            }
            else if (Static_Identify_Start_Armed != 0U)
            {
                Static_Identify_Pose_Index = 0U;
                Static_Identify_Set_Target(tmp_pose[Static_Identify_Pose_Index].Joint_Angle);
                Static_Identify_Set_State(Static_Identify_State_MOVE_TO_POSE);
            }
        }
        break;

        case (Static_Identify_State_MOVE_TO_POSE):
        {
            Static_Identify_Update_J0_J2_Profile();
            if (Static_Identify_At_Target() != 0U)
            {
                Static_Identify_Set_State(Static_Identify_State_SETTLE_FORWARD);
            }
            else if (Static_Identify_State_Tick >= STATIC_IDENTIFY_MOVE_TIMEOUT_MS)
            {
                Static_Identify_Fault_Code = Static_Identify_Fault_MOVE_TIMEOUT;
                Static_Identify_Set_State(Static_Identify_State_FAULT);
            }
        }
        break;

        case (Static_Identify_State_SETTLE_FORWARD):
        {
            Static_Identify_Update_J0_J2_Profile();
            if (Static_Identify_At_Target() == 0U)
            {
                Static_Identify_Set_State(Static_Identify_State_MOVE_TO_POSE);
            }
            else if (Static_Identify_State_Tick >= STATIC_IDENTIFY_SETTLE_TIME_MS)
            {
                Static_Identify_Set_State(Static_Identify_State_SAMPLE_FORWARD);
            }
        }
        break;

        case (Static_Identify_State_SAMPLE_FORWARD):
        {
            if (Static_Identify_At_Target() == 0U)
            {
                Static_Identify_Set_State(Static_Identify_State_MOVE_TO_POSE);
            }
            else if (Static_Identify_State_Tick >= STATIC_IDENTIFY_SAMPLE_TIME_MS)
            {
                float tmp_reverse_target[CONTROLLER_JOINT_NUM];
                for (uint8_t i = 0U; i < CONTROLLER_JOINT_NUM; i++)
                {
                    tmp_reverse_target[i] = tmp_pose[Static_Identify_Pose_Index].Joint_Angle[i] +
                                            STATIC_IDENTIFY_REVERSE_APPROACH;
                }
                Static_Identify_Set_Target(tmp_reverse_target);
                Static_Identify_Set_State(Static_Identify_State_MOVE_TO_REVERSE_APPROACH);
            }
        }
        break;

        case (Static_Identify_State_MOVE_TO_REVERSE_APPROACH):
        {
            Static_Identify_Update_J0_J2_Profile();
            if (Static_Identify_At_Target() != 0U)
            {
                Static_Identify_Set_Target(tmp_pose[Static_Identify_Pose_Index].Joint_Angle);
                Static_Identify_Set_State(Static_Identify_State_MOVE_TO_REVERSE_POSE);
            }
            else if (Static_Identify_State_Tick >= STATIC_IDENTIFY_MOVE_TIMEOUT_MS)
            {
                Static_Identify_Fault_Code = Static_Identify_Fault_MOVE_TIMEOUT;
                Static_Identify_Set_State(Static_Identify_State_FAULT);
            }
        }
        break;

        case (Static_Identify_State_MOVE_TO_REVERSE_POSE):
        {
            Static_Identify_Update_J0_J2_Profile();
            if (Static_Identify_At_Target() != 0U)
            {
                Static_Identify_Set_State(Static_Identify_State_SETTLE_REVERSE);
            }
            else if (Static_Identify_State_Tick >= STATIC_IDENTIFY_MOVE_TIMEOUT_MS)
            {
                Static_Identify_Fault_Code = Static_Identify_Fault_MOVE_TIMEOUT;
                Static_Identify_Set_State(Static_Identify_State_FAULT);
            }
        }
        break;

        case (Static_Identify_State_SETTLE_REVERSE):
        {
            Static_Identify_Update_J0_J2_Profile();
            if (Static_Identify_At_Target() == 0U)
            {
                Static_Identify_Set_State(Static_Identify_State_MOVE_TO_REVERSE_POSE);
            }
            else if (Static_Identify_State_Tick >= STATIC_IDENTIFY_SETTLE_TIME_MS)
            {
                Static_Identify_Set_State(Static_Identify_State_SAMPLE_REVERSE);
            }
        }
        break;

        case (Static_Identify_State_SAMPLE_REVERSE):
        {
            if (Static_Identify_At_Target() == 0U)
            {
                Static_Identify_Set_State(Static_Identify_State_MOVE_TO_REVERSE_POSE);
            }
            else if (Static_Identify_State_Tick >= STATIC_IDENTIFY_SAMPLE_TIME_MS)
            {
                Static_Identify_Pose_Index++;
                if (Static_Identify_Pose_Index >= STATIC_IDENTIFY_POSE_COUNT)
                {
                    Static_Identify_Set_State(Static_Identify_State_COMPLETE);
                }
                else
                {
                    Static_Identify_Set_Target(tmp_pose[Static_Identify_Pose_Index].Joint_Angle);
                    Static_Identify_Set_State(Static_Identify_State_MOVE_TO_POSE);
                }
            }
        }
        break;

        case (Static_Identify_State_COMPLETE):
        {
        }
        break;

        case (Static_Identify_State_FAULT):
        {
            if ((Static_Identify_Runtime.Start_Request == 0U) &&
                (Static_Identify_Runtime.Abort_Request == 0U))
            {
                Static_Identify_Set_State(Static_Identify_State_IDLE);
            }
        }
        break;

        default:
        {
        }
        break;
    }
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
    Update_Current_State();

    if (Static_Identify_Enabled != 0U)
    {
        Static_Identify_PeriodElapsedCallback();
    }

    Calculate_Model();

    Output();

    Motor_J0.TIM_Process_PeriodElapsedCallback();
    Motor_J3.TIM_PID_PeriodElapsedCallback();
    Motor_J4.TIM_PID_PeriodElapsedCallback();
    Motor_J5.TIM_PID_PeriodElapsedCallback();

    if (Static_Identify_Enabled != 0U)
    {
        Static_Identify_Update_Monitor();
    }
}

void Class_Manipulator::TIM_CAN_PeriodElapsedCallback()
{
    if (Static_Identify_Enabled != 0U)
    {
        uint8_t tmp_status = static_cast<uint8_t>(HAL_OK);

        switch (CAN_Schedule_Slot)
        {
            case (STATIC_IDENTIFY_AK_SLOT_J1):
            {
                tmp_status = Motor_J1.Task_Process_PeriodElapsedCallback();
            }
            break;

            case (STATIC_IDENTIFY_AK_SLOT_J2):
            {
                tmp_status = Motor_J2.Task_Process_PeriodElapsedCallback();
            }
            break;

            case (STATIC_IDENTIFY_ZDT_SLOT_J3):
            {
                tmp_status = Motor_J3.Send_Control_Command();
            }
            break;

            case (STATIC_IDENTIFY_ZDT_SLOT_J4):
            {
                tmp_status = Motor_J4.Send_Control_Command();
            }
            break;

            case (STATIC_IDENTIFY_ZDT_SLOT_J5):
            {
                tmp_status = Motor_J5.Send_Control_Command();
            }
            break;

            default:
            {
            }
            break;
        }

        if ((CAN_Schedule_Slot == STATIC_IDENTIFY_AK_SLOT_J1) ||
            (CAN_Schedule_Slot == STATIC_IDENTIFY_AK_SLOT_J2) ||
            (CAN_Schedule_Slot == STATIC_IDENTIFY_ZDT_SLOT_J3) ||
            (CAN_Schedule_Slot == STATIC_IDENTIFY_ZDT_SLOT_J4) ||
            (CAN_Schedule_Slot == STATIC_IDENTIFY_ZDT_SLOT_J5))
        {
            Static_Identify_Record_CAN_Status(tmp_status);
        }

        CAN_Schedule_Slot++;
        if (CAN_Schedule_Slot >= MANIPULATOR_CAN_SCHEDULE_SLOT_COUNT)
        {
            CAN_Schedule_Slot = 0U;
        }
        return;
    }

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
            Motor_J3.Send_Query_Command(ZDT_Motor_Query_Type_OMEGA);
        }
        break;

        case (6U):
        {
            Motor_J4.Send_Query_Command(ZDT_Motor_Query_Type_OMEGA);
        }
        break;

        case (7U):
        {
            Motor_J5.Send_Query_Command(ZDT_Motor_Query_Type_OMEGA);
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
    if (CAN_Schedule_Slot >= 16U)
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
