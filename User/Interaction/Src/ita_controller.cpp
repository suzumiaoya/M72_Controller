#include "ita_controller.h"

#include "drv_math.h"
#include "usart.h"
#include "spi.h"

void Class_Controller::Init()
{
    Left_Arm.Init(Manipulator_ID_LEFT);
    Right_Arm.Init(Manipulator_ID_RIGHT);

    Referee.Init(&huart10);
    LCD.Init(&hspi2);

    Left_Arm_DH_Model.Init();
    Right_Arm_DH_Model.Init();
    Left_Arm_Gravity_Comp.Init();
    Right_Arm_Gravity_Comp.Init();
}

void Class_Controller::Task_Loop_Callback()
{
    LCD.Refresh();
}

void Class_Controller::CAN1_Motor_RxCpltCallback(Struct_CAN_Rx_Buffer *CAN_RxMessage)
{
    Left_Arm.CAN_RxCpltCallback(CAN_RxMessage);
}

void Class_Controller::CAN2_Motor_RxCpltCallback(Struct_CAN_Rx_Buffer *CAN_RxMessage)
{
    Right_Arm.CAN_RxCpltCallback(CAN_RxMessage);
}

void Class_Controller::RS485_Motor_Bus_RxCpltCallback(uint8_t *Buffer, uint16_t Length)
{
    Left_Arm.UART_RxCpltCallback(Buffer, Length);
    Right_Arm.UART_RxCpltCallback(Buffer, Length);
}

void Class_Controller::Referee_UART_RxCpltCallback(uint8_t *Buffer, uint16_t Length)
{
    Referee.UART_RxCpltCallback(Buffer, Length);
}

void Class_Controller::Peer_UART_RxCpltCallback(uint8_t *Buffer, uint16_t Length)
{
    Peer_Rx_Length = Length > UART_BUFFER_SIZE ? UART_BUFFER_SIZE : Length;
    memcpy(Peer_Rx_Buffer, Buffer, Peer_Rx_Length);
}

void Class_Controller::LCD_SPI_RxCpltCallback(uint8_t *Tx_Buffer, uint8_t *Rx_Buffer, uint16_t Length)
{
    LCD.SPI_RxCpltCallback(Tx_Buffer, Rx_Buffer, Length);
}

void Class_Controller::Update_Manipulator_Model(Class_Manipulator &Manipulator,
                                                Class_DH_Model &DH_Model,
                                                Class_Gravity_Comp &Gravity_Comp)
{
    float Joint_Angle[CONTROLLER_JOINT_NUM] = {0.0f};

    for (uint8_t i = 0; i < CONTROLLER_JOINT_NUM; i++)
    {
        Joint_Angle[i] = Manipulator.Get_Current_Joint_Angle(i);
    }

    DH_Model.Set_Joint_Angles(Joint_Angle, CONTROLLER_JOINT_NUM);
    DH_Model.Calculate();

    Gravity_Comp.Set_Joint_Angles(Joint_Angle, CONTROLLER_JOINT_NUM);
    Gravity_Comp.Calculate();

    for (uint8_t i = 0; i < CONTROLLER_JOINT_NUM; i++)
    {
        Manipulator.Set_Target_Joint_Torque(i, Gravity_Comp.Get_Output_Torque(i));
    }
}

void Class_Controller::TIM_Calculate_PeriodElapsedCallback()
{
    Update_Manipulator_Model(Left_Arm, Left_Arm_DH_Model, Left_Arm_Gravity_Comp);
    Update_Manipulator_Model(Right_Arm, Right_Arm_DH_Model, Right_Arm_Gravity_Comp);

    Left_Arm.TIM_Calculate_PeriodElapsedCallback();
    Right_Arm.TIM_Calculate_PeriodElapsedCallback();
}

void Class_Controller::TIM1msMod50_Alive_PeriodElapsedCallback()
{
    Left_Arm.TIM1msMod50_Alive_PeriodElapsedCallback();
    Right_Arm.TIM1msMod50_Alive_PeriodElapsedCallback();
    Referee.TIM1msMod50_Alive_PeriodElapsedCallback();
    Referee.TIM_UART_Tx_PeriodElapsedCallback();
}
