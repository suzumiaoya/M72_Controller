// SPDX-License-Identifier: AGPL-3.0-only
/**
 * @file dvc_lcd.cpp
 * @author ZLLC
 * @brief ST7789 LCD initialization and asynchronous status refresh
 * @version 0.1
 * @date 2026-07-10
 *
 * @copyright ZLLC 2026
 */

/* Includes ------------------------------------------------------------------*/

#include "dvc_lcd.h"

#include "dvc_lcd_font.h"

#include <string.h>

/* Private macros and types --------------------------------------------------*/

namespace
{
constexpr uint8_t LCD_FONT_WIDTH = 6U;
constexpr uint8_t LCD_FONT_HEIGHT = 12U;
constexpr uint16_t LCD_FIELD_WIDTH = LCD_FIELD_CHARACTER_NUM * LCD_FONT_WIDTH;
constexpr uint16_t LCD_FIELD_HEIGHT = LCD_FONT_HEIGHT;
constexpr uint16_t LCD_FIELD_PIXEL_BYTES = LCD_FIELD_WIDTH * LCD_FIELD_HEIGHT * 2U;
constexpr uint16_t LCD_DMA_CHUNK_SIZE = 240U;
constexpr uint32_t LCD_SPI_TIMEOUT = 1000U;

struct Struct_LCD_Init_Command
{
    uint8_t Command;
    uint8_t Data_Length;
    uint16_t Delay_ms;
    uint8_t Data[14];
};

const Struct_LCD_Init_Command LCD_Init_Sequence[] = {
    {0x11U, 0U, 120U, {0U}},
    {0x36U, 1U, 0U, {0x00U}},
    {0x3AU, 1U, 0U, {0x05U}},
    {0xB2U, 5U, 0U, {0x0CU, 0x0CU, 0x00U, 0x33U, 0x33U}},
    {0xB7U, 1U, 0U, {0x35U}},
    {0xBBU, 1U, 0U, {0x32U}},
    {0xC2U, 1U, 0U, {0x01U}},
    {0xC3U, 1U, 0U, {0x15U}},
    {0xC4U, 1U, 0U, {0x20U}},
    {0xC6U, 1U, 0U, {0x0FU}},
    {0xD0U, 2U, 0U, {0xA4U, 0xA1U}},
    {0xE0U, 14U, 0U, {0xD0U, 0x08U, 0x0EU, 0x09U, 0x09U, 0x05U, 0x31U,
                        0x33U, 0x48U, 0x17U, 0x14U, 0x15U, 0x31U, 0x34U}},
    {0xE1U, 14U, 0U, {0xD0U, 0x08U, 0x0EU, 0x09U, 0x09U, 0x15U, 0x31U,
                        0x33U, 0x48U, 0x17U, 0x14U, 0x15U, 0x31U, 0x34U}},
    {0x21U, 0U, 0U, {0U}},
    {0x29U, 0U, 20U, {0U}},
};

Class_LCD *LCD_Active_Instance = 0;

uint8_t Get_Text_Length(const char *Text, uint8_t Maximum_Length)
{
    uint8_t Length = 0U;

    while ((Text != 0) && (Text[Length] != '\0') && (Length < Maximum_Length))
    {
        Length++;
    }

    return Length;
}
}

/* Function prototypes -------------------------------------------------------*/

void Class_LCD::Init(SPI_HandleTypeDef *__Driver_SPI)
{
    Driver_SPI = __Driver_SPI;
    LCD_Active_Instance = this;
    Pending_Status_Valid = 0U;
    Frame_Active = 0U;
    DMA_Busy = 0U;
    Transfer_State = LCD_Transfer_State_IDLE;
    memset(Displayed_Text, 0, sizeof(Displayed_Text));

    if (Driver_SPI == 0)
    {
        return;
    }

    Reset_Panel();
    Initialize_Panel();
    Draw_Startup_Page();
}

void Class_LCD::Set_Backlight(uint8_t Enable)
{
    HAL_GPIO_WritePin(LCD_BLK_GPIO_Port, LCD_BLK_Pin, Enable == 0U ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void Class_LCD::Submit_Status(const Struct_LCD_Status *Status)
{
    if (Status == 0)
    {
        return;
    }

    Pending_Status = *Status;
    Pending_Status_Valid = 1U;
}

void Class_LCD::Refresh()
{
    uint8_t Arm_Index;
    uint8_t Joint_Index;

    if ((Driver_SPI == 0) || (DMA_Busy != 0U) || (Current_Page != 0U))
    {
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
        Frame_Active = 1U;
        Active_Field = 0U;

        if (Select_Next_Dirty_Field() == 0U)
        {
            Frame_Active = 0U;
            return;
        }

        Transfer_State = LCD_Transfer_State_COLUMN_COMMAND;
    }

    switch (Transfer_State)
    {
    case LCD_Transfer_State_COLUMN_COMMAND:
        Control_Buffer[0] = 0x2AU;
        if (Start_DMA_Transfer(Control_Buffer, 1U, 0U) != 0U)
        {
            Transfer_State = LCD_Transfer_State_COLUMN_DATA;
        }
        break;

    case LCD_Transfer_State_COLUMN_DATA:
        Control_Buffer[0] = static_cast<uint8_t>(Field_X >> 8U);
        Control_Buffer[1] = static_cast<uint8_t>(Field_X);
        Control_Buffer[2] = static_cast<uint8_t>((Field_X + LCD_FIELD_WIDTH - 1U) >> 8U);
        Control_Buffer[3] = static_cast<uint8_t>(Field_X + LCD_FIELD_WIDTH - 1U);
        if (Start_DMA_Transfer(Control_Buffer, sizeof(Control_Buffer), 1U) != 0U)
        {
            Transfer_State = LCD_Transfer_State_ROW_COMMAND;
        }
        break;

    case LCD_Transfer_State_ROW_COMMAND:
        Control_Buffer[0] = 0x2BU;
        if (Start_DMA_Transfer(Control_Buffer, 1U, 0U) != 0U)
        {
            Transfer_State = LCD_Transfer_State_ROW_DATA;
        }
        break;

    case LCD_Transfer_State_ROW_DATA:
        Control_Buffer[0] = static_cast<uint8_t>((Field_Y + 20U) >> 8U);
        Control_Buffer[1] = static_cast<uint8_t>(Field_Y + 20U);
        Control_Buffer[2] = static_cast<uint8_t>((Field_Y + LCD_FIELD_HEIGHT - 1U + 20U) >> 8U);
        Control_Buffer[3] = static_cast<uint8_t>(Field_Y + LCD_FIELD_HEIGHT - 1U + 20U);
        if (Start_DMA_Transfer(Control_Buffer, sizeof(Control_Buffer), 1U) != 0U)
        {
            Transfer_State = LCD_Transfer_State_MEMORY_COMMAND;
        }
        break;

    case LCD_Transfer_State_MEMORY_COMMAND:
        Control_Buffer[0] = 0x2CU;
        if (Start_DMA_Transfer(Control_Buffer, 1U, 0U) != 0U)
        {
            Pixel_Offset = 0U;
            Transfer_State = LCD_Transfer_State_PIXEL_DATA;
        }
        break;

    case LCD_Transfer_State_PIXEL_DATA:
    {
        uint16_t Remaining = LCD_FIELD_PIXEL_BYTES - Pixel_Offset;
        uint16_t Length = Remaining > LCD_DMA_CHUNK_SIZE ? LCD_DMA_CHUNK_SIZE : Remaining;

        if (Start_DMA_Transfer(&Field_Buffer[Pixel_Offset], Length, 1U) != 0U)
        {
            Pixel_Offset += Length;
            if (Pixel_Offset >= LCD_FIELD_PIXEL_BYTES)
            {
                Transfer_State = LCD_Transfer_State_NEXT_FIELD;
            }
        }
        break;
    }

    case LCD_Transfer_State_NEXT_FIELD:
        Arm_Index = Active_Field / LCD_JOINT_COUNT;
        Joint_Index = Active_Field % LCD_JOINT_COUNT;
        memcpy(Displayed_Text[Arm_Index][Joint_Index],
               Active_Text[Arm_Index][Joint_Index],
               LCD_FIELD_CHARACTER_NUM + 1U);
        Dirty_Field[Arm_Index][Joint_Index] = 0U;
        Active_Field++;

        if (Select_Next_Dirty_Field() != 0U)
        {
            Transfer_State = LCD_Transfer_State_COLUMN_COMMAND;
        }
        else
        {
            Frame_Active = 0U;
            Transfer_State = LCD_Transfer_State_IDLE;
        }
        break;

    default:
        Frame_Active = 0U;
        Transfer_State = LCD_Transfer_State_IDLE;
        break;
    }
}

void Class_LCD::SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi != Driver_SPI)
    {
        return;
    }

    HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);
    DMA_Busy = 0U;
}

void Class_LCD::SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi != Driver_SPI)
    {
        return;
    }

    HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);
    DMA_Busy = 0U;
    Frame_Active = 0U;
    Transfer_State = LCD_Transfer_State_IDLE;
}

void Class_LCD::Reset_Panel()
{
    HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_RESET);
    Set_Backlight(0U);
    HAL_GPIO_WritePin(LCD_RES_GPIO_Port, LCD_RES_Pin, GPIO_PIN_RESET);
    HAL_Delay(100U);
    HAL_GPIO_WritePin(LCD_RES_GPIO_Port, LCD_RES_Pin, GPIO_PIN_SET);
    HAL_Delay(100U);
    Set_Backlight(1U);
    HAL_Delay(100U);
}

void Class_LCD::Initialize_Panel()
{
    for (uint16_t i = 0U; i < (sizeof(LCD_Init_Sequence) / sizeof(LCD_Init_Sequence[0])); i++)
    {
        Write_Command_Blocking(LCD_Init_Sequence[i].Command);
        if (LCD_Init_Sequence[i].Data_Length != 0U)
        {
            Write_Data_Blocking(LCD_Init_Sequence[i].Data, LCD_Init_Sequence[i].Data_Length);
        }
        if (LCD_Init_Sequence[i].Delay_ms != 0U)
        {
            HAL_Delay(LCD_Init_Sequence[i].Delay_ms);
        }
    }
}

void Class_LCD::Draw_Startup_Page()
{
    char Page_Text[LCD_FIELD_CHARACTER_NUM + 1] = {0};

    Clear_Blocking(LCD_Color_BLACK);
    Draw_Text_Blocking(8U, 8U, "M72", LCD_Color_WHITE, LCD_Color_BLACK);
    Draw_Text_Blocking(32U, 8U, "CONTROLLER", LCD_Color_WHITE, LCD_Color_BLACK);
    Draw_Text_Blocking(8U, 28U, "LEFT", LCD_Color_GREEN, LCD_Color_BLACK);
    Draw_Text_Blocking(128U, 28U, "RIGHT", LCD_Color_GREEN, LCD_Color_BLACK);
    Draw_Text_Blocking(8U, 252U, "PAGE:", LCD_Color_CYAN, LCD_Color_BLACK);
    Format_Integer(Current_Page, Page_Text);
    Build_Field(Page_Text, LCD_FIELD_CHARACTER_NUM, LCD_Color_CYAN, LCD_Color_BLACK);
    Draw_Field_Blocking(44U, 252U);

    for (uint8_t Joint = 0U; Joint < LCD_JOINT_COUNT; Joint++)
    {
        char Label[4] = {'J', static_cast<char>('0' + Joint), ':', '\0'};
        uint16_t Y = static_cast<uint16_t>(52U + Joint * 34U);

        Draw_Text_Blocking(8U, Y, Label, LCD_Color_GREEN, LCD_Color_BLACK);
        Draw_Text_Blocking(128U, Y, Label, LCD_Color_GREEN, LCD_Color_BLACK);

        Build_Field("0.00", LCD_FIELD_CHARACTER_NUM, LCD_Color_YELLOW, LCD_Color_BLACK);
        Draw_Field_Blocking(32U, Y);
        Draw_Field_Blocking(152U, Y);
    }
}

void Class_LCD::Draw_Text_Blocking(uint16_t X,
                                   uint16_t Y,
                                   const char *Text,
                                   uint16_t Foreground,
                                   uint16_t Background)
{
    uint8_t Length = Get_Text_Length(Text, LCD_FIELD_CHARACTER_NUM);

    if (Length == 0U)
    {
        return;
    }

    Build_Field(Text, Length, Foreground, Background);
    Set_Window_Blocking(X, Y, X + Length * LCD_FONT_WIDTH - 1U, Y + LCD_FONT_HEIGHT - 1U);
    Write_Data_Blocking(Field_Buffer, static_cast<uint16_t>(Length * LCD_FONT_WIDTH * LCD_FONT_HEIGHT * 2U));
}

void Class_LCD::Draw_Field_Blocking(uint16_t X, uint16_t Y)
{
    Set_Window_Blocking(X, Y, X + LCD_FIELD_WIDTH - 1U, Y + LCD_FIELD_HEIGHT - 1U);
    Write_Data_Blocking(Field_Buffer, LCD_FIELD_PIXEL_BYTES);
}

void Class_LCD::Clear_Blocking(uint16_t Color)
{
    static uint8_t Color_Buffer[LCD_DMA_CHUNK_SIZE];
    uint32_t Remaining = LCD_WIDTH * LCD_HEIGHT;

    for (uint16_t i = 0U; i < sizeof(Color_Buffer); i += 2U)
    {
        Color_Buffer[i] = static_cast<uint8_t>(Color >> 8U);
        Color_Buffer[i + 1U] = static_cast<uint8_t>(Color);
    }

    Set_Window_Blocking(0U, 0U, LCD_WIDTH - 1U, LCD_HEIGHT - 1U);
    while (Remaining != 0U)
    {
        uint16_t Pixel_Count = Remaining > (sizeof(Color_Buffer) / 2U)
                                   ? static_cast<uint16_t>(sizeof(Color_Buffer) / 2U)
                                   : static_cast<uint16_t>(Remaining);
        Write_Data_Blocking(Color_Buffer, static_cast<uint16_t>(Pixel_Count * 2U));
        Remaining -= Pixel_Count;
    }
}

void Class_LCD::Set_Window_Blocking(uint16_t X_Start, uint16_t Y_Start, uint16_t X_End, uint16_t Y_End)
{
    uint8_t Address[4];

    Write_Command_Blocking(0x2AU);
    Address[0] = static_cast<uint8_t>(X_Start >> 8U);
    Address[1] = static_cast<uint8_t>(X_Start);
    Address[2] = static_cast<uint8_t>(X_End >> 8U);
    Address[3] = static_cast<uint8_t>(X_End);
    Write_Data_Blocking(Address, sizeof(Address));

    Write_Command_Blocking(0x2BU);
    Address[0] = static_cast<uint8_t>((Y_Start + 20U) >> 8U);
    Address[1] = static_cast<uint8_t>(Y_Start + 20U);
    Address[2] = static_cast<uint8_t>((Y_End + 20U) >> 8U);
    Address[3] = static_cast<uint8_t>(Y_End + 20U);
    Write_Data_Blocking(Address, sizeof(Address));

    Write_Command_Blocking(0x2CU);
}

void Class_LCD::Write_Command_Blocking(uint8_t Command)
{
    HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(Driver_SPI, &Command, 1U, LCD_SPI_TIMEOUT);
    HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);
}

void Class_LCD::Write_Data_Blocking(const uint8_t *Data, uint16_t Length)
{
    HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_SET);
    HAL_SPI_Transmit(Driver_SPI, Data, Length, LCD_SPI_TIMEOUT);
    HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);
}

void Class_LCD::Format_Status()
{
    for (uint8_t Arm = 0U; Arm < LCD_ARM_COUNT; Arm++)
    {
        for (uint8_t Joint = 0U; Joint < LCD_JOINT_COUNT; Joint++)
        {
            Format_Float(Active_Status.Current_Joint_Angle[Arm][Joint], Active_Text[Arm][Joint]);
            Dirty_Field[Arm][Joint] = memcmp(Active_Text[Arm][Joint],
                                              Displayed_Text[Arm][Joint],
                                              LCD_FIELD_CHARACTER_NUM + 1U) != 0
                                      ? 1U
                                      : 0U;
        }
    }
}

void Class_LCD::Format_Integer(int32_t Value, char *Text) const
{
    char Reverse_Integer[10];
    uint8_t Length = 0U;
    uint8_t Integer_Length = 0U;
    uint8_t Negative = Value < 0 ? 1U : 0U;
    uint32_t Absolute_Value = Negative != 0U
                                  ? static_cast<uint32_t>(-(Value + 1)) + 1U
                                  : static_cast<uint32_t>(Value);

    memset(Text, ' ', LCD_FIELD_CHARACTER_NUM);
    Text[LCD_FIELD_CHARACTER_NUM] = '\0';

    do
    {
        Reverse_Integer[Integer_Length++] = static_cast<char>('0' + (Absolute_Value % 10U));
        Absolute_Value /= 10U;
    } while ((Absolute_Value != 0U) && (Integer_Length < sizeof(Reverse_Integer)));

    if (Negative != 0U)
    {
        Length++;
    }
    Length += Integer_Length;

    if (Length > LCD_FIELD_CHARACTER_NUM)
    {
        for (uint8_t i = 0U; i < LCD_FIELD_CHARACTER_NUM; i++)
        {
            Text[i] = '#';
        }
        return;
    }

    uint8_t Text_Index = LCD_FIELD_CHARACTER_NUM - Length;
    if (Negative != 0U)
    {
        Text[Text_Index++] = '-';
    }
    while (Integer_Length != 0U)
    {
        Text[Text_Index++] = Reverse_Integer[--Integer_Length];
    }
}

void Class_LCD::Format_Float(float Value, char *Text) const
{
    char Value_Text[16] = {0};
    uint8_t Length = 0U;
    uint8_t Negative = Value < 0.0f ? 1U : 0U;
    uint32_t Scale = 100U;
    uint32_t Scaled_Value;
    uint32_t Integer_Part;
    uint32_t Fractional_Part;
    char Reverse_Integer[10];
    uint8_t Integer_Length = 0U;

    memset(Text, ' ', LCD_FIELD_CHARACTER_NUM);
    Text[LCD_FIELD_CHARACTER_NUM] = '\0';

    if (Value != Value)
    {
        memcpy(Value_Text, "NAN", 4U);
        Length = 3U;
    }
    else if (Value > 21474836.0f)
    {
        memcpy(Value_Text, "INF", 4U);
        Length = 3U;
    }
    else if (Value < -21474836.0f)
    {
        memcpy(Value_Text, "-INF", 5U);
        Length = 4U;
    }
    else
    {
        float Absolute_Value = Negative != 0U ? -Value : Value;
        Scaled_Value = static_cast<uint32_t>(Absolute_Value * static_cast<float>(Scale) + 0.5f);
        Integer_Part = Scaled_Value / Scale;
        Fractional_Part = Scaled_Value % Scale;

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

    if (Length > LCD_FIELD_CHARACTER_NUM)
    {
        for (uint8_t i = 0U; i < LCD_FIELD_CHARACTER_NUM; i++)
        {
            Text[i] = '#';
        }
        return;
    }

    memcpy(&Text[LCD_FIELD_CHARACTER_NUM - Length], Value_Text, Length);
}

void Class_LCD::Build_Field(const char *Text,
                            uint8_t Character_Count,
                            uint16_t Foreground,
                            uint16_t Background)
{
    uint16_t Width = static_cast<uint16_t>(Character_Count) * LCD_FONT_WIDTH;
    uint8_t Text_Length = Get_Text_Length(Text, Character_Count);

    for (uint8_t Y = 0U; Y < LCD_FONT_HEIGHT; Y++)
    {
        uint8_t Font_Row = static_cast<uint8_t>((Y * 7U) / LCD_FONT_HEIGHT);

        for (uint16_t X = 0U; X < Width; X++)
        {
            uint8_t Character_Index = static_cast<uint8_t>(X / LCD_FONT_WIDTH);
            uint8_t Character_Column = static_cast<uint8_t>(X % LCD_FONT_WIDTH);
            char Character = Character_Index < Text_Length ? Text[Character_Index] : ' ';
            uint8_t Font_Row_Data = LCD_Font_Get_Row(Character, Font_Row);
            uint16_t Color = ((Character_Column < 5U) && ((Font_Row_Data & (1U << Character_Column)) != 0U))
                                 ? Foreground
                                 : Background;
            uint16_t Buffer_Offset = static_cast<uint16_t>((Y * Width + X) * 2U);

            Field_Buffer[Buffer_Offset] = static_cast<uint8_t>(Color >> 8U);
            Field_Buffer[Buffer_Offset + 1U] = static_cast<uint8_t>(Color);
        }
    }
}

uint8_t Class_LCD::Select_Next_Dirty_Field()
{
    while (Active_Field < (LCD_ARM_COUNT * LCD_JOINT_COUNT))
    {
        uint8_t Arm = Active_Field / LCD_JOINT_COUNT;
        uint8_t Joint = Active_Field % LCD_JOINT_COUNT;

        if (Dirty_Field[Arm][Joint] != 0U)
        {
            Field_X = Arm == 0U ? 32U : 152U;
            Field_Y = static_cast<uint16_t>(52U + Joint * 34U);
            Build_Field(Active_Text[Arm][Joint], LCD_FIELD_CHARACTER_NUM, LCD_Color_YELLOW, LCD_Color_BLACK);
            return 1U;
        }

        Active_Field++;
    }

    return 0U;
}

uint8_t Class_LCD::Start_DMA_Transfer(const uint8_t *Data, uint16_t Length, uint8_t Data_Mode)
{
    HAL_StatusTypeDef Status;

    HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, Data_Mode == 0U ? GPIO_PIN_RESET : GPIO_PIN_SET);
    DMA_Busy = 1U;
    Status = HAL_SPI_Transmit_DMA(Driver_SPI, Data, Length);

    if (Status != HAL_OK)
    {
        HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);
        DMA_Busy = 0U;
        Frame_Active = 0U;
        Transfer_State = LCD_Transfer_State_IDLE;
        return 0U;
    }

    return 1U;
}

extern "C" void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (LCD_Active_Instance != 0)
    {
        LCD_Active_Instance->SPI_TxCpltCallback(hspi);
    }
}

extern "C" void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (LCD_Active_Instance != 0)
    {
        LCD_Active_Instance->SPI_ErrorCallback(hspi);
    }
}
