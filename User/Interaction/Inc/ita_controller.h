#ifndef ITA_CONTROLLER_H
#define ITA_CONTROLLER_H

#include "ctl_manipulator.h"
#include "dvc_referee.h"
#include "dvc_lcd.h"
#include "alg_dh_model.h"
#include "alg_gravity_comp.h"

class Class_Controller
{
public:
    Class_Manipulator Left_Arm;
    Class_Manipulator Right_Arm;
    Class_Referee Referee;
    Class_LCD LCD;
    Class_DH_Model Left_Arm_DH_Model;
    Class_DH_Model Right_Arm_DH_Model;
    Class_Gravity_Comp Left_Arm_Gravity_Comp;
    Class_Gravity_Comp Right_Arm_Gravity_Comp;

    void Init();
    void Task_Loop_Callback();

    void CAN1_Motor_RxCpltCallback(Struct_CAN_Rx_Buffer *CAN_RxMessage);
    void CAN2_Motor_RxCpltCallback(Struct_CAN_Rx_Buffer *CAN_RxMessage);
    void RS485_Motor_Bus_RxCpltCallback(uint8_t *Buffer, uint16_t Length);
    void Referee_UART_RxCpltCallback(uint8_t *Buffer, uint16_t Length);
    void Peer_UART_RxCpltCallback(uint8_t *Buffer, uint16_t Length);
    void LCD_SPI_RxCpltCallback(uint8_t *Tx_Buffer, uint8_t *Rx_Buffer, uint16_t Length);

    void TIM_Calculate_PeriodElapsedCallback();
    void TIM1msMod50_Alive_PeriodElapsedCallback();

protected:
    uint8_t Peer_Rx_Buffer[UART_BUFFER_SIZE] = {0};
    uint16_t Peer_Rx_Length = 0;

    void Update_Manipulator_Model(Class_Manipulator &Manipulator,
                                  Class_DH_Model &DH_Model,
                                  Class_Gravity_Comp &Gravity_Comp);
};

#endif
