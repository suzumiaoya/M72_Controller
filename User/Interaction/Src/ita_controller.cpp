// SPDX-License-Identifier: AGPL-3.0-only
#include "ita_controller.h"

#include "usart.h"

void Class_Controller::Init()
{
    Left_Arm.Init(Manipulator_ID_LEFT);
    Right_Arm.Init(Manipulator_ID_RIGHT);

    Referee.Init(&huart10);
    LCD.Init(&hspi1);
}
