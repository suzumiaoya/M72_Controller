// SPDX-License-Identifier: AGPL-3.0-only
#include "ita_controller.h"

#include "usart.h"

void Class_Controller::Init()
{
    Struct_LCD_Config LCD_Config;

    Left_Arm.Init(Manipulator_ID_LEFT);
    Right_Arm.Init(Manipulator_ID_RIGHT);

    Referee.Init(&huart10);
    LCD_Config.CS.Port = LCD_CS_GPIO_Port;
    LCD_Config.CS.Pin = LCD_CS_Pin;
    LCD_Config.DC.Port = LCD_DC_GPIO_Port;
    LCD_Config.DC.Pin = LCD_DC_Pin;
    LCD_Config.RES.Port = LCD_RES_GPIO_Port;
    LCD_Config.RES.Pin = LCD_RES_Pin;
    LCD_Config.BLK.Port = LCD_BLK_GPIO_Port;
    LCD_Config.BLK.Pin = LCD_BLK_Pin;
    LCD_Config.Backlight_Active_High = 1U;
    LCD_Config.Rotation = LCD_Rotation_0;
    LCD_Config.Inversion = 1U;

    LCD.Init(&hspi1, &LCD_Config);
    LCD_Status_Page.Init(&LCD);
}
