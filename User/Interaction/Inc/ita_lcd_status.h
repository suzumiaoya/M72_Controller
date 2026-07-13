// SPDX-License-Identifier: AGPL-3.0-only
#ifndef ITA_LCD_STATUS_H
#define ITA_LCD_STATUS_H

/**
 * @file ita_lcd_status.h
 * @brief M72 controller LCD status page
 */

/* Includes ------------------------------------------------------------------*/

#include "config.h"
#include "dvc_lcd.h"

/* Exported macros -----------------------------------------------------------*/

#define LCD_STATUS_ARM_COUNT       2U
#define LCD_STATUS_FIELD_CHAR_NUM 10U

/* Exported types ------------------------------------------------------------*/

struct Struct_LCD_Status_Page_Data
{
    float Current_Joint_Angle[LCD_STATUS_ARM_COUNT][CONTROLLER_JOINT_NUM];
};

class Class_LCD_Status_Page
{
public:
    void Init(Class_LCD *__LCD);
    void Submit_Status(const Struct_LCD_Status_Page_Data *__Status);
    void Refresh();
    void Invalidate();

protected:
    Class_LCD *LCD = 0;
    Enum_LCD_Status Last_LCD_Status = LCD_Status_DISABLE;
    Struct_LCD_Status_Page_Data Pending_Status = {};
    Struct_LCD_Status_Page_Data Active_Status = {};
    uint8_t Pending_Status_Valid = 0U;
    uint8_t Frame_Active = 0U;
    uint8_t Active_Field = 0U;
    uint8_t Init_Index = 0U;
    uint8_t Init_Complete = 0U;

    uint8_t Dirty_Field[LCD_STATUS_ARM_COUNT][CONTROLLER_JOINT_NUM] = {{0}};
    char Active_Text[LCD_STATUS_ARM_COUNT][CONTROLLER_JOINT_NUM][LCD_STATUS_FIELD_CHAR_NUM + 1U] = {{{0}}};
    char Displayed_Text[LCD_STATUS_ARM_COUNT][CONTROLLER_JOINT_NUM][LCD_STATUS_FIELD_CHAR_NUM + 1U] = {{{0}}};

    Enum_LCD_Request_Status Draw_Initial_Element();
    void Format_Status();
    void Format_Float(float __Value, char *__Text) const;
};

#endif
