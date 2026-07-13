// SPDX-License-Identifier: AGPL-3.0-only
/**
 * @file ita_lcd_status.cpp
 * @brief M72 controller LCD status page
 */

/* Includes ------------------------------------------------------------------*/

#include "ita_lcd_status.h"

#include <string.h>

/* Private macros ------------------------------------------------------------*/

namespace
{
constexpr uint8_t LCD_STATUS_STATIC_ELEMENT_COUNT =
    static_cast<uint8_t>(7U + CONTROLLER_JOINT_NUM * 4U);
}

/* Function prototypes -------------------------------------------------------*/

void Class_LCD_Status_Page::Init(Class_LCD *__LCD)
{
    LCD = __LCD;
    Last_LCD_Status = LCD_Status_DISABLE;
    Pending_Status_Valid = 0U;
    Invalidate();
}

void Class_LCD_Status_Page::Submit_Status(const Struct_LCD_Status_Page_Data *__Status)
{
    if (__Status == 0)
    {
        return;
    }

    Pending_Status = *__Status;
    Pending_Status_Valid = 1U;
}

void Class_LCD_Status_Page::Refresh()
{
    if (LCD == 0)
    {
        return;
    }

    Enum_LCD_Status Current_LCD_Status = LCD->Get_LCD_Status();
    if ((Last_LCD_Status == LCD_Status_ERROR) && (Current_LCD_Status == LCD_Status_ENABLE))
    {
        Invalidate();
    }
    Last_LCD_Status = Current_LCD_Status;

    if ((Current_LCD_Status == LCD_Status_DISABLE) || (Current_LCD_Status == LCD_Status_ERROR))
    {
        return;
    }

    if (Init_Complete == 0U)
    {
        while ((Init_Complete == 0U) && (LCD->Get_Free_Command_Count() != 0U))
        {
            Enum_LCD_Request_Status Result = Draw_Initial_Element();
            if (Result != LCD_Request_Status_OK)
            {
                return;
            }

            Init_Index++;
            if (Init_Index >= LCD_STATUS_STATIC_ELEMENT_COUNT)
            {
                char Initial_Text[LCD_STATUS_FIELD_CHAR_NUM + 1U] = {0};
                Format_Float(0.0f, Initial_Text);
                for (uint8_t Arm = 0U; Arm < LCD_STATUS_ARM_COUNT; Arm++)
                {
                    for (uint8_t Joint = 0U; Joint < CONTROLLER_JOINT_NUM; Joint++)
                    {
                        memcpy(Displayed_Text[Arm][Joint], Initial_Text, sizeof(Initial_Text));
                    }
                }
                Init_Complete = 1U;
            }
        }
        return;
    }

    if (Frame_Active == 0U)
    {
        if (Pending_Status_Valid == 0U)
        {
            return;
        }

        Active_Status = Pending_Status;
        Pending_Status_Valid = 0U;
        Format_Status();
        Active_Field = 0U;
        Frame_Active = 1U;
    }

    while (Active_Field < (LCD_STATUS_ARM_COUNT * CONTROLLER_JOINT_NUM))
    {
        uint8_t Arm = static_cast<uint8_t>(Active_Field / CONTROLLER_JOINT_NUM);
        uint8_t Joint = static_cast<uint8_t>(Active_Field % CONTROLLER_JOINT_NUM);

        if (Dirty_Field[Arm][Joint] != 0U)
        {
            if (LCD->Get_Free_Command_Count() == 0U)
            {
                return;
            }

            Struct_LCD_Text_Style Style;
            Style.Font = &LCD_Font_Default;
            Style.Foreground = LCD_Color_YELLOW;
            Style.Background = LCD_Color_BLACK;
            Style.Scale = 1U;

            uint16_t X = Arm == 0U ? 32U : 152U;
            uint16_t Y = static_cast<uint16_t>(52U + Joint * 34U);
            Enum_LCD_Request_Status Result = LCD->Draw_String(X, Y, Active_Text[Arm][Joint], &Style);
            if (Result != LCD_Request_Status_OK)
            {
                return;
            }

            memcpy(Displayed_Text[Arm][Joint],
                   Active_Text[Arm][Joint],
                   LCD_STATUS_FIELD_CHAR_NUM + 1U);
            Dirty_Field[Arm][Joint] = 0U;
        }

        Active_Field++;
    }

    Frame_Active = 0U;
}

void Class_LCD_Status_Page::Invalidate()
{
    Init_Index = 0U;
    Init_Complete = 0U;
    Frame_Active = 0U;
    Active_Field = 0U;
    memset(Dirty_Field, 0, sizeof(Dirty_Field));
    memset(Active_Text, 0, sizeof(Active_Text));
    memset(Displayed_Text, 0, sizeof(Displayed_Text));
}

Enum_LCD_Request_Status Class_LCD_Status_Page::Draw_Initial_Element()
{
    Struct_LCD_Text_Style Style;
    Style.Font = &LCD_Font_Default;
    Style.Foreground = LCD_Color_WHITE;
    Style.Background = LCD_Color_BLACK;
    Style.Scale = 1U;

    switch (Init_Index)
    {
    case 0U:
        return LCD->Clear(LCD_Color_BLACK);

    case 1U:
        return LCD->Draw_String(8U, 8U, "M72", &Style);

    case 2U:
        return LCD->Draw_String(32U, 8U, "CONTROLLER", &Style);

    case 3U:
        Style.Foreground = LCD_Color_GREEN;
        return LCD->Draw_String(8U, 28U, "LEFT", &Style);

    case 4U:
        Style.Foreground = LCD_Color_GREEN;
        return LCD->Draw_String(128U, 28U, "RIGHT", &Style);

    case 5U:
        Style.Foreground = LCD_Color_CYAN;
        return LCD->Draw_String(8U, 252U, "PAGE:", &Style);

    case 6U:
        Style.Foreground = LCD_Color_CYAN;
        return LCD->Draw_Integer(44U, 252U, 0, &Style);

    default:
    {
        uint8_t Element = static_cast<uint8_t>(Init_Index - 7U);
        uint8_t Joint = static_cast<uint8_t>(Element / 4U);
        uint8_t Part = static_cast<uint8_t>(Element % 4U);
        uint16_t Y = static_cast<uint16_t>(52U + Joint * 34U);
        char Label[4] = {'J', static_cast<char>('0' + Joint), ':', '\0'};

        if (Joint >= CONTROLLER_JOINT_NUM)
        {
            return LCD_Request_Status_INVALID;
        }

        if (Part == 0U)
        {
            Style.Foreground = LCD_Color_GREEN;
            return LCD->Draw_String(8U, Y, Label, &Style);
        }
        if (Part == 1U)
        {
            Style.Foreground = LCD_Color_GREEN;
            return LCD->Draw_String(128U, Y, Label, &Style);
        }

        char Initial_Text[LCD_STATUS_FIELD_CHAR_NUM + 1U] = {0};
        Format_Float(0.0f, Initial_Text);
        Style.Foreground = LCD_Color_YELLOW;
        return LCD->Draw_String(Part == 2U ? 32U : 152U, Y, Initial_Text, &Style);
    }
    }
}

void Class_LCD_Status_Page::Format_Status()
{
    for (uint8_t Arm = 0U; Arm < LCD_STATUS_ARM_COUNT; Arm++)
    {
        for (uint8_t Joint = 0U; Joint < CONTROLLER_JOINT_NUM; Joint++)
        {
            Format_Float(Active_Status.Current_Joint_Angle[Arm][Joint], Active_Text[Arm][Joint]);
            Dirty_Field[Arm][Joint] = memcmp(Active_Text[Arm][Joint],
                                              Displayed_Text[Arm][Joint],
                                              LCD_STATUS_FIELD_CHAR_NUM + 1U) != 0
                                               ? 1U
                                               : 0U;
        }
    }
}

void Class_LCD_Status_Page::Format_Float(float __Value, char *__Text) const
{
    char Value_Text[16] = {0};
    char Reverse_Integer[10];
    uint8_t Length = 0U;
    uint8_t Integer_Length = 0U;
    uint8_t Negative = __Value < 0.0f ? 1U : 0U;

    memset(__Text, ' ', LCD_STATUS_FIELD_CHAR_NUM);
    __Text[LCD_STATUS_FIELD_CHAR_NUM] = '\0';

    if (__Value != __Value)
    {
        memcpy(Value_Text, "NAN", 4U);
        Length = 3U;
    }
    else if (__Value > 21474836.0f)
    {
        memcpy(Value_Text, "INF", 4U);
        Length = 3U;
    }
    else if (__Value < -21474836.0f)
    {
        memcpy(Value_Text, "-INF", 5U);
        Length = 4U;
    }
    else
    {
        float Absolute_Value = Negative != 0U ? -__Value : __Value;
        uint32_t Scaled_Value = static_cast<uint32_t>(Absolute_Value * 100.0f + 0.5f);
        uint32_t Integer_Part = Scaled_Value / 100U;
        uint32_t Fractional_Part = Scaled_Value % 100U;

        if (Negative != 0U)
        {
            Value_Text[Length++] = '-';
        }
        do
        {
            Reverse_Integer[Integer_Length++] = static_cast<char>('0' + (Integer_Part % 10U));
            Integer_Part /= 10U;
        } while ((Integer_Part != 0U) && (Integer_Length < sizeof(Reverse_Integer)));

        while (Integer_Length != 0U)
        {
            Value_Text[Length++] = Reverse_Integer[--Integer_Length];
        }
        Value_Text[Length++] = '.';
        Value_Text[Length++] = static_cast<char>('0' + (Fractional_Part / 10U));
        Value_Text[Length++] = static_cast<char>('0' + (Fractional_Part % 10U));
        Value_Text[Length] = '\0';
    }

    if (Length > LCD_STATUS_FIELD_CHAR_NUM)
    {
        memset(__Text, '#', LCD_STATUS_FIELD_CHAR_NUM);
        return;
    }
    memcpy(&__Text[LCD_STATUS_FIELD_CHAR_NUM - Length], Value_Text, Length);
}
