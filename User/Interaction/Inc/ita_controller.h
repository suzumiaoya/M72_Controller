// SPDX-License-Identifier: AGPL-3.0-only
#ifndef ITA_CONTROLLER_H
#define ITA_CONTROLLER_H

#include "ctl_manipulator.h"
#include "dvc_referee.h"
#include "dvc_lcd.h"
#include "ita_lcd_status.h"

class Class_Controller
{
public:
    Class_Manipulator Left_Arm;
    Class_Manipulator Right_Arm;
    Class_Referee Referee;
    Class_LCD LCD;
    Class_LCD_Status_Page LCD_Status_Page;

    void Init();
};

#endif
