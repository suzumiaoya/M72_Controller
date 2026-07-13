// SPDX-License-Identifier: AGPL-3.0-only
/**
 * @file dvc_lcd.cpp
 * @author ZLLC
 * @brief DM LCD ST7789 initialization and asynchronous drawing
 * @version 0.2
 * @date 2026-07-13
 *
 * @copyright ZLLC 2026
 */

/* Includes ------------------------------------------------------------------*/

#include "dvc_lcd.h"

#include "dvc_lcd_font.h"
#include "drv_spi.h"

#include <string.h>

/* Private macros and types --------------------------------------------------*/

namespace
{
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
    {0x29U, 0U, 20U, {0U}},
};

uint8_t Get_Text_Length(const char *Text, uint8_t Maximum_Length)
{
    uint8_t Length = 0U;

    while ((Text != 0) && (Text[Length] != '\0') && (Length < Maximum_Length))
    {
        Length++;
    }

    return Length;
}

uint8_t Get_Default_Font_Row(char Character, uint8_t Row)
{
    if (Row >= 12U)
    {
        return 0U;
    }
    return LCD_Font_Get_Row(Character, static_cast<uint8_t>((Row * 7U) / 12U));
}

Struct_LCD_Font Make_Default_Font()
{
    Struct_LCD_Font Font;

    Font.Width = 5U;
    Font.Height = 12U;
    Font.Bytes_Per_Row = 1U;
    Font.X_Advance = 6U;
    Font.Get_Row = Get_Default_Font_Row;
    return Font;
}

uint32_t Absolute_Difference(uint16_t A, uint16_t B)
{
    return A >= B ? static_cast<uint32_t>(A - B) : static_cast<uint32_t>(B - A);
}

uint16_t Integer_Sqrt(uint32_t Value)
{
    uint32_t Low = 0U;
    uint32_t High = 0xFFFFU;

    while (Low < High)
    {
        uint32_t Middle = (Low + High + 1U) / 2U;
        if (Middle <= (Value / Middle))
        {
            Low = Middle;
        }
        else
        {
            High = Middle - 1U;
        }
    }

    return static_cast<uint16_t>(Low);
}

uint8_t Get_Line_Clip_Code(int32_t X, int32_t Y, uint16_t Width, uint16_t Height)
{
    uint8_t Code = 0U;

    if (X < 0)
    {
        Code |= 0x01U;
    }
    else if (X >= Width)
    {
        Code |= 0x02U;
    }
    if (Y < 0)
    {
        Code |= 0x04U;
    }
    else if (Y >= Height)
    {
        Code |= 0x08U;
    }
    return Code;
}

uint8_t Clip_Line_To_Screen(uint16_t Width, uint16_t Height,
                            uint16_t *Start_X, uint16_t *Start_Y,
                            uint16_t *End_X, uint16_t *End_Y)
{
    int32_t X0 = *Start_X;
    int32_t Y0 = *Start_Y;
    int32_t X1 = *End_X;
    int32_t Y1 = *End_Y;

    for (;;)
    {
        uint8_t Code_0 = Get_Line_Clip_Code(X0, Y0, Width, Height);
        uint8_t Code_1 = Get_Line_Clip_Code(X1, Y1, Width, Height);

        if ((Code_0 | Code_1) == 0U)
        {
            *Start_X = static_cast<uint16_t>(X0);
            *Start_Y = static_cast<uint16_t>(Y0);
            *End_X = static_cast<uint16_t>(X1);
            *End_Y = static_cast<uint16_t>(Y1);
            return 1U;
        }
        if ((Code_0 & Code_1) != 0U)
        {
            return 0U;
        }

        uint8_t Outside = Code_0 != 0U ? Code_0 : Code_1;
        int32_t X;
        int32_t Y;

        if ((Outside & 0x08U) != 0U)
        {
            Y = Height - 1;
            X = X0 + static_cast<int32_t>((static_cast<int64_t>(X1 - X0) * (Y - Y0)) /
                                          (Y1 - Y0));
        }
        else if ((Outside & 0x04U) != 0U)
        {
            Y = 0;
            X = X0 + static_cast<int32_t>((static_cast<int64_t>(X1 - X0) * (Y - Y0)) /
                                          (Y1 - Y0));
        }
        else if ((Outside & 0x02U) != 0U)
        {
            X = Width - 1;
            Y = Y0 + static_cast<int32_t>((static_cast<int64_t>(Y1 - Y0) * (X - X0)) /
                                          (X1 - X0));
        }
        else
        {
            X = 0;
            Y = Y0 + static_cast<int32_t>((static_cast<int64_t>(Y1 - Y0) * (X - X0)) /
                                          (X1 - X0));
        }

        if (Outside == Code_0)
        {
            X0 = X;
            Y0 = Y;
        }
        else
        {
            X1 = X;
            Y1 = Y;
        }
    }
}

void Clean_DMA_Cache(const uint8_t *Data, uint16_t Length)
{
#if defined(SCB_CCR_DC_Msk)
    if ((Data != 0) && (Length != 0U) && ((SCB->CCR & SCB_CCR_DC_Msk) != 0U))
    {
        uintptr_t Start = reinterpret_cast<uintptr_t>(Data) & ~static_cast<uintptr_t>(31U);
        uintptr_t End = (reinterpret_cast<uintptr_t>(Data) + Length + 31U) & ~static_cast<uintptr_t>(31U);
        SCB_CleanDCache_by_Addr(reinterpret_cast<uint32_t *>(Start), static_cast<int32_t>(End - Start));
    }
#else
    (void)Data;
    (void)Length;
#endif
}

void LCD_SPI_Event_Callback(SPI_HandleTypeDef *SPI, Enum_SPI_Event Event, void *Context)
{
    Class_LCD *LCD = static_cast<Class_LCD *>(Context);

    if (LCD == 0)
    {
        return;
    }

    if (Event == SPI_Event_TX_COMPLETE)
    {
        LCD->SPI_TxCpltCallback(SPI);
    }
    else if (Event == SPI_Event_ERROR)
    {
        LCD->SPI_ErrorCallback(SPI);
    }
}
}

const Struct_LCD_Font LCD_Font_Default = Make_Default_Font();

static_assert(sizeof(Class_LCD) < 4096U, "Class_LCD must not contain a full-screen frame buffer");

/* Function prototypes -------------------------------------------------------*/

void Class_LCD::Init(SPI_HandleTypeDef *__Driver_SPI, const Struct_LCD_Config *__Config)
{
    Driver_SPI = __Driver_SPI;
    Config = Struct_LCD_Config();
    Queue_Head = 0U;
    Queue_Tail = 0U;
    Queue_Count = 0U;
    Active_Command_Valid = 0U;
    DMA_Busy = 0U;
    Transfer_State = LCD_Transfer_State_IDLE;
    Last_Result = LCD_Request_Status_OK;
    LCD_Status = LCD_Status_DISABLE;

    if ((Driver_SPI == 0) || (__Config == 0))
    {
        Last_Result = LCD_Request_Status_INVALID;
        LCD_Status = LCD_Status_ERROR;
        return;
    }

    Config = *__Config;
    if ((Config.CS.Port == 0) || (Config.DC.Port == 0) ||
        (Config.RES.Port == 0) || (Config.BLK.Port == 0))
    {
        Last_Result = LCD_Request_Status_INVALID;
        LCD_Status = LCD_Status_ERROR;
        return;
    }

    if (SPI_Register_Event_Callback(Driver_SPI, LCD_SPI_Event_Callback, this) == 0U)
    {
        Last_Result = LCD_Request_Status_ERROR;
        LCD_Status = LCD_Status_ERROR;
        return;
    }

    Reset_Panel();
    if ((Initialize_Panel() == 0U) || (Apply_Rotation(Config.Rotation) == 0U))
    {
        Last_Result = LCD_Request_Status_ERROR;
        LCD_Status = LCD_Status_ERROR;
        return;
    }

    if (Write_Command_Blocking(Config.Inversion == 0U ? 0x20U : 0x21U) == 0U ||
        Clear_Blocking(LCD_Color_BLACK) == 0U)
    {
        Last_Result = LCD_Request_Status_ERROR;
        LCD_Status = LCD_Status_ERROR;
        return;
    }

    Set_Backlight(1U);
    LCD_Status = LCD_Status_ENABLE;
}

Enum_LCD_Request_Status Class_LCD::Clear(uint16_t __Color)
{
    return Enqueue_Rectangle(LCD_Command_CLEAR, 0U, 0U, Logical_Width, Logical_Height, __Color);
}

Enum_LCD_Request_Status Class_LCD::Draw_Pixel(uint16_t __X, uint16_t __Y, uint16_t __Color)
{
    Struct_LCD_Command Command;

    if ((__X >= Logical_Width) || (__Y >= Logical_Height))
    {
        return LCD_Request_Status_OK;
    }

    Command.Type = LCD_Command_PIXEL;
    Command.X = __X;
    Command.Y = __Y;
    Command.Width = 1U;
    Command.Height = 1U;
    Command.Color = __Color;
    return Enqueue_Command(&Command);
}

Enum_LCD_Request_Status Class_LCD::Draw_Line(uint16_t __Start_X, uint16_t __Start_Y,
                                              uint16_t __End_X, uint16_t __End_Y,
                                              uint16_t __Color)
{
    Struct_LCD_Command Command;

    if (Clip_Line_To_Screen(Logical_Width, Logical_Height,
                            &__Start_X, &__Start_Y, &__End_X, &__End_Y) == 0U)
    {
        return LCD_Request_Status_OK;
    }
    if (__Start_X == __End_X)
    {
        uint16_t Y = __Start_Y < __End_Y ? __Start_Y : __End_Y;
        uint16_t Height = static_cast<uint16_t>(Absolute_Difference(__Start_Y, __End_Y) + 1U);
        return Enqueue_Rectangle(LCD_Command_FILL_RECTANGLE, __Start_X, Y, 1U, Height, __Color);
    }
    if (__Start_Y == __End_Y)
    {
        uint16_t X = __Start_X < __End_X ? __Start_X : __End_X;
        uint16_t Width = static_cast<uint16_t>(Absolute_Difference(__Start_X, __End_X) + 1U);
        return Enqueue_Rectangle(LCD_Command_FILL_RECTANGLE, X, __Start_Y, Width, 1U, __Color);
    }

    Command.Type = LCD_Command_LINE;
    Command.X = __Start_X;
    Command.Y = __Start_Y;
    Command.End_X = __End_X;
    Command.End_Y = __End_Y;
    Command.Color = __Color;
    return Enqueue_Command(&Command);
}

Enum_LCD_Request_Status Class_LCD::Draw_Rectangle(uint16_t __X, uint16_t __Y,
                                                   uint16_t __Width, uint16_t __Height,
                                                   uint16_t __Line_Width, uint16_t __Color)
{
    Struct_LCD_Command Command;

    if ((__Width == 0U) || (__Height == 0U) || (__Line_Width == 0U))
    {
        return LCD_Request_Status_INVALID;
    }
    if ((__X >= Logical_Width) || (__Y >= Logical_Height))
    {
        return LCD_Request_Status_OK;
    }

    if (__Width > static_cast<uint16_t>(Logical_Width - __X))
    {
        __Width = static_cast<uint16_t>(Logical_Width - __X);
    }
    if (__Height > static_cast<uint16_t>(Logical_Height - __Y))
    {
        __Height = static_cast<uint16_t>(Logical_Height - __Y);
    }

    Command.Type = LCD_Command_RECTANGLE;
    Command.X = __X;
    Command.Y = __Y;
    Command.Width = __Width;
    Command.Height = __Height;
    Command.Line_Width = __Line_Width;
    Command.Color = __Color;
    return Enqueue_Command(&Command);
}

Enum_LCD_Request_Status Class_LCD::Fill_Rectangle(uint16_t __X, uint16_t __Y,
                                                   uint16_t __Width, uint16_t __Height,
                                                   uint16_t __Color)
{
    return Enqueue_Rectangle(LCD_Command_FILL_RECTANGLE, __X, __Y, __Width, __Height, __Color);
}

Enum_LCD_Request_Status Class_LCD::Draw_Circle(uint16_t __Center_X, uint16_t __Center_Y,
                                                uint16_t __Radius, uint16_t __Line_Width,
                                                uint16_t __Color)
{
    Struct_LCD_Command Command;

    if ((__Radius == 0U) || (__Line_Width == 0U) ||
        (__Radius > LCD_LANDSCAPE_WIDTH))
    {
        return LCD_Request_Status_INVALID;
    }
    if ((static_cast<uint32_t>(__Center_X) > static_cast<uint32_t>(Logical_Width) + __Radius) ||
        (static_cast<uint32_t>(__Center_Y) > static_cast<uint32_t>(Logical_Height) + __Radius))
    {
        return LCD_Request_Status_OK;
    }

    Command.Type = LCD_Command_CIRCLE;
    Command.X = __Center_X;
    Command.Y = __Center_Y;
    Command.Radius = __Radius;
    Command.Line_Width = __Line_Width;
    Command.Color = __Color;
    return Enqueue_Command(&Command);
}

Enum_LCD_Request_Status Class_LCD::Fill_Circle(uint16_t __Center_X, uint16_t __Center_Y,
                                                uint16_t __Radius, uint16_t __Color)
{
    Struct_LCD_Command Command;

    if ((__Radius == 0U) || (__Radius > LCD_LANDSCAPE_WIDTH))
    {
        return LCD_Request_Status_INVALID;
    }
    if ((static_cast<uint32_t>(__Center_X) > static_cast<uint32_t>(Logical_Width) + __Radius) ||
        (static_cast<uint32_t>(__Center_Y) > static_cast<uint32_t>(Logical_Height) + __Radius))
    {
        return LCD_Request_Status_OK;
    }

    Command.Type = LCD_Command_FILL_CIRCLE;
    Command.X = __Center_X;
    Command.Y = __Center_Y;
    Command.Radius = __Radius;
    Command.Color = __Color;
    return Enqueue_Command(&Command);
}

Enum_LCD_Request_Status Class_LCD::Draw_String(uint16_t __X, uint16_t __Y,
                                                const char *__Text,
                                                const Struct_LCD_Text_Style *__Style)
{
    return Enqueue_Text(__X, __Y, __Text, __Style);
}

Enum_LCD_Request_Status Class_LCD::Draw_Integer(uint16_t __X, uint16_t __Y,
                                                 int32_t __Value,
                                                 const Struct_LCD_Text_Style *__Style)
{
    char Text[LCD_TEXT_MAX_LENGTH + 1U] = {0};

    Format_Integer(__Value, Text);
    return Enqueue_Text(__X, __Y, Text, __Style);
}

Enum_LCD_Request_Status Class_LCD::Draw_Float(uint16_t __X, uint16_t __Y,
                                               float __Value, uint8_t __Decimal_Digits,
                                               const Struct_LCD_Text_Style *__Style)
{
    char Text[LCD_TEXT_MAX_LENGTH + 1U] = {0};

    if (__Decimal_Digits > 6U)
    {
        return LCD_Request_Status_INVALID;
    }

    Format_Float(__Value, __Decimal_Digits, Text);
    return Enqueue_Text(__X, __Y, Text, __Style);
}

Enum_LCD_Request_Status Class_LCD::Draw_RGB565(uint16_t __X, uint16_t __Y,
                                                uint16_t __Width, uint16_t __Height,
                                                const uint16_t *__Pixels, uint16_t __Stride)
{
    Struct_LCD_Command Command;

    if ((__Pixels == 0) || (__Width == 0U) || (__Height == 0U) || (__Stride < __Width))
    {
        return LCD_Request_Status_INVALID;
    }

    if ((__X >= Logical_Width) || (__Y >= Logical_Height))
    {
        return LCD_Request_Status_OK;
    }

    Command.Type = LCD_Command_BITMAP;
    Command.X = __X;
    Command.Y = __Y;
    Command.Width = __Width;
    Command.Height = __Height;
    Command.Stride = __Stride;
    Command.Pixels = __Pixels;
    return Enqueue_Command(&Command);
}

void Class_LCD::Set_Backlight(uint8_t __Enable)
{
    GPIO_PinState State;

    if (Config.BLK.Port == 0)
    {
        return;
    }

    State = ((__Enable != 0U) == (Config.Backlight_Active_High != 0U))
                ? GPIO_PIN_SET
                : GPIO_PIN_RESET;
    HAL_GPIO_WritePin(Config.BLK.Port, Config.BLK.Pin, State);
}

Enum_LCD_Request_Status Class_LCD::Set_Rotation(Enum_LCD_Rotation __Rotation)
{
    if ((LCD_Status == LCD_Status_DISABLE) || (LCD_Status == LCD_Status_ERROR))
    {
        return LCD_Request_Status_ERROR;
    }
    if (__Rotation > LCD_Rotation_270)
    {
        return LCD_Request_Status_INVALID;
    }
    if ((DMA_Busy != 0U) || (Queue_Count != 0U) || (Active_Command_Valid != 0U))
    {
        return LCD_Request_Status_BUSY;
    }
    if (Apply_Rotation(__Rotation) == 0U)
    {
        Abort_Active_Command(LCD_Request_Status_ERROR);
        return LCD_Request_Status_ERROR;
    }

    Config.Rotation = __Rotation;
    return LCD_Request_Status_OK;
}

Enum_LCD_Request_Status Class_LCD::Set_Inversion(uint8_t __Enable)
{
    if ((LCD_Status == LCD_Status_DISABLE) || (LCD_Status == LCD_Status_ERROR))
    {
        return LCD_Request_Status_ERROR;
    }
    if ((DMA_Busy != 0U) || (Queue_Count != 0U) || (Active_Command_Valid != 0U))
    {
        return LCD_Request_Status_BUSY;
    }
    if (Write_Command_Blocking(__Enable == 0U ? 0x20U : 0x21U) == 0U)
    {
        Abort_Active_Command(LCD_Request_Status_ERROR);
        return LCD_Request_Status_ERROR;
    }

    Config.Inversion = __Enable == 0U ? 0U : 1U;
    return LCD_Request_Status_OK;
}

Enum_LCD_Request_Status Class_LCD::Set_Display(uint8_t __Enable)
{
    if ((LCD_Status == LCD_Status_DISABLE) || (LCD_Status == LCD_Status_ERROR))
    {
        return LCD_Request_Status_ERROR;
    }
    if ((DMA_Busy != 0U) || (Queue_Count != 0U) || (Active_Command_Valid != 0U))
    {
        return LCD_Request_Status_BUSY;
    }
    if (Write_Command_Blocking(__Enable == 0U ? 0x28U : 0x29U) == 0U)
    {
        Abort_Active_Command(LCD_Request_Status_ERROR);
        return LCD_Request_Status_ERROR;
    }
    return LCD_Request_Status_OK;
}

Enum_LCD_Request_Status Class_LCD::Set_Sleep(uint8_t __Enable)
{
    if ((LCD_Status == LCD_Status_DISABLE) || (LCD_Status == LCD_Status_ERROR))
    {
        return LCD_Request_Status_ERROR;
    }
    if ((DMA_Busy != 0U) || (Queue_Count != 0U) || (Active_Command_Valid != 0U))
    {
        return LCD_Request_Status_BUSY;
    }
    if (Write_Command_Blocking(__Enable == 0U ? 0x11U : 0x10U) == 0U)
    {
        Abort_Active_Command(LCD_Request_Status_ERROR);
        return LCD_Request_Status_ERROR;
    }

    HAL_Delay(120U);
    return LCD_Request_Status_OK;
}

void Class_LCD::Refresh()
{
    uint8_t Transfer_Result;

    if ((Driver_SPI == 0) || (LCD_Status == LCD_Status_DISABLE) ||
        (LCD_Status == LCD_Status_ERROR) || (DMA_Busy != 0U))
    {
        return;
    }

    if (Active_Command_Valid == 0U)
    {
        if (Queue_Count == 0U)
        {
            LCD_Status = LCD_Status_ENABLE;
            return;
        }

        Active_Command = Command_Queue[Queue_Head];
        Active_Command_Valid = 1U;
        Primitive_Index = 0U;
        Primitive_Total = 0U;
        Shape_Row = 0U;
        Shape_Part = 0U;
        Region_Valid = 0U;

        if (Active_Command.Type == LCD_Command_LINE)
        {
            uint32_t Delta_X = Absolute_Difference(Active_Command.X, Active_Command.End_X);
            uint32_t Delta_Y = Absolute_Difference(Active_Command.Y, Active_Command.End_Y);
            Primitive_Total = (Delta_X > Delta_Y ? Delta_X : Delta_Y) + 1U;
        }
        if (Prepare_Next_Region() == 0U)
        {
            Complete_Active_Command();
            return;
        }
        Transfer_State = LCD_Transfer_State_COLUMN_COMMAND;
        LCD_Status = LCD_Status_BUSY;
    }

    switch (Transfer_State)
    {
    case LCD_Transfer_State_COLUMN_COMMAND:
        Control_Buffer[0] = 0x2AU;
        Transfer_Result = Start_DMA_Transfer(Control_Buffer, 1U, 0U);
        if (Transfer_Result == 1U)
        {
            Transfer_State = LCD_Transfer_State_COLUMN_DATA;
        }
        break;

    case LCD_Transfer_State_COLUMN_DATA:
    {
        uint16_t X_End = static_cast<uint16_t>(Region_X + Region_Width - 1U + Gram_Offset_X);
        uint16_t X_Start = static_cast<uint16_t>(Region_X + Gram_Offset_X);
        Control_Buffer[0] = static_cast<uint8_t>(X_Start >> 8U);
        Control_Buffer[1] = static_cast<uint8_t>(X_Start);
        Control_Buffer[2] = static_cast<uint8_t>(X_End >> 8U);
        Control_Buffer[3] = static_cast<uint8_t>(X_End);
        Transfer_Result = Start_DMA_Transfer(Control_Buffer, sizeof(Control_Buffer), 1U);
        if (Transfer_Result == 1U)
        {
            Transfer_State = LCD_Transfer_State_ROW_COMMAND;
        }
        break;
    }

    case LCD_Transfer_State_ROW_COMMAND:
        Control_Buffer[0] = 0x2BU;
        Transfer_Result = Start_DMA_Transfer(Control_Buffer, 1U, 0U);
        if (Transfer_Result == 1U)
        {
            Transfer_State = LCD_Transfer_State_ROW_DATA;
        }
        break;

    case LCD_Transfer_State_ROW_DATA:
    {
        uint16_t Y_End = static_cast<uint16_t>(Region_Y + Region_Height - 1U + Gram_Offset_Y);
        uint16_t Y_Start = static_cast<uint16_t>(Region_Y + Gram_Offset_Y);
        Control_Buffer[0] = static_cast<uint8_t>(Y_Start >> 8U);
        Control_Buffer[1] = static_cast<uint8_t>(Y_Start);
        Control_Buffer[2] = static_cast<uint8_t>(Y_End >> 8U);
        Control_Buffer[3] = static_cast<uint8_t>(Y_End);
        Transfer_Result = Start_DMA_Transfer(Control_Buffer, sizeof(Control_Buffer), 1U);
        if (Transfer_Result == 1U)
        {
            Transfer_State = LCD_Transfer_State_MEMORY_COMMAND;
        }
        break;
    }

    case LCD_Transfer_State_MEMORY_COMMAND:
        Control_Buffer[0] = 0x2CU;
        Transfer_Result = Start_DMA_Transfer(Control_Buffer, 1U, 0U);
        if (Transfer_Result == 1U)
        {
            Region_Pixel_Offset = 0U;
            Transfer_State = LCD_Transfer_State_PIXEL_DATA;
        }
        break;

    case LCD_Transfer_State_PIXEL_DATA:
    {
        uint32_t Remaining = Region_Pixel_Count - Region_Pixel_Offset;
        uint16_t Pixel_Count = Remaining > (LCD_DMA_BUFFER_SIZE / 2U)
                                   ? static_cast<uint16_t>(LCD_DMA_BUFFER_SIZE / 2U)
                                   : static_cast<uint16_t>(Remaining);

        for (uint16_t i = 0U; i < Pixel_Count; i++)
        {
            uint32_t Pixel_Index = Region_Pixel_Offset + i;
            uint16_t Local_X = static_cast<uint16_t>(Pixel_Index % Region_Width);
            uint16_t Local_Y = static_cast<uint16_t>(Pixel_Index / Region_Width);
            uint16_t Color = Get_Region_Pixel(Local_X, Local_Y);
            DMA_Buffer[i * 2U] = static_cast<uint8_t>(Color >> 8U);
            DMA_Buffer[i * 2U + 1U] = static_cast<uint8_t>(Color);
        }

        Transfer_Result = Start_DMA_Transfer(DMA_Buffer, static_cast<uint16_t>(Pixel_Count * 2U), 1U);
        if (Transfer_Result == 1U)
        {
            Region_Pixel_Offset += Pixel_Count;
            if (Region_Pixel_Offset >= Region_Pixel_Count)
            {
                Transfer_State = LCD_Transfer_State_NEXT_REGION;
            }
        }
        break;
    }

    case LCD_Transfer_State_NEXT_REGION:
        if (Prepare_Next_Region() != 0U)
        {
            Transfer_State = LCD_Transfer_State_COLUMN_COMMAND;
        }
        else
        {
            Complete_Active_Command();
        }
        break;

    default:
        Abort_Active_Command(LCD_Request_Status_ERROR);
        break;
    }
}

void Class_LCD::SPI_TxCpltCallback(SPI_HandleTypeDef *__SPI)
{
    if (__SPI != Driver_SPI)
    {
        return;
    }

    HAL_GPIO_WritePin(Config.CS.Port, Config.CS.Pin, GPIO_PIN_SET);
    DMA_Busy = 0U;
}

void Class_LCD::SPI_ErrorCallback(SPI_HandleTypeDef *__SPI)
{
    if (__SPI != Driver_SPI)
    {
        return;
    }

    HAL_GPIO_WritePin(Config.CS.Port, Config.CS.Pin, GPIO_PIN_SET);
    DMA_Busy = 0U;
    Abort_Active_Command(LCD_Request_Status_ERROR);
}

Enum_LCD_Request_Status Class_LCD::Enqueue_Command(const Struct_LCD_Command *__Command)
{
    if (__Command == 0)
    {
        Last_Result = LCD_Request_Status_INVALID;
        return LCD_Request_Status_INVALID;
    }
    if ((LCD_Status == LCD_Status_DISABLE) || (LCD_Status == LCD_Status_ERROR))
    {
        Last_Result = LCD_Request_Status_ERROR;
        return LCD_Request_Status_ERROR;
    }
    if (Queue_Count >= LCD_COMMAND_QUEUE_DEPTH)
    {
        Last_Result = LCD_Request_Status_FULL;
        return LCD_Request_Status_FULL;
    }

    Command_Queue[Queue_Tail] = *__Command;
    Queue_Tail = static_cast<uint8_t>((Queue_Tail + 1U) % LCD_COMMAND_QUEUE_DEPTH);
    Queue_Count++;
    LCD_Status = LCD_Status_BUSY;
    Last_Result = LCD_Request_Status_OK;
    return LCD_Request_Status_OK;
}

Enum_LCD_Request_Status Class_LCD::Enqueue_Rectangle(Enum_LCD_Command_Type __Type,
                                                      uint16_t __X, uint16_t __Y,
                                                      uint16_t __Width, uint16_t __Height,
                                                      uint16_t __Color)
{
    Struct_LCD_Command Command;

    if ((__Width == 0U) || (__Height == 0U))
    {
        return LCD_Request_Status_INVALID;
    }
    if ((__X >= Logical_Width) || (__Y >= Logical_Height))
    {
        return LCD_Request_Status_OK;
    }

    Command.Type = __Type;
    Command.X = __X;
    Command.Y = __Y;
    Command.Width = __Width;
    Command.Height = __Height;
    Command.Color = __Color;
    return Enqueue_Command(&Command);
}

Enum_LCD_Request_Status Class_LCD::Enqueue_Text(uint16_t __X, uint16_t __Y,
                                                 const char *__Text,
                                                 const Struct_LCD_Text_Style *__Style)
{
    Struct_LCD_Command Command;
    Struct_LCD_Text_Style Style;
    uint32_t Text_Width;

    if (__Text == 0)
    {
        return LCD_Request_Status_INVALID;
    }

    Style.Font = &LCD_Font_Default;
    Style.Foreground = LCD_Color_WHITE;
    Style.Background = LCD_Color_BLACK;
    Style.Scale = 1U;
    if (__Style != 0)
    {
        Style = *__Style;
        if (Style.Font == 0)
        {
            Style.Font = &LCD_Font_Default;
        }
    }

    if ((Style.Scale == 0U) || (Style.Scale > 8U) ||
        (Style.Font->Width == 0U) || (Style.Font->Width > 8U) ||
        (Style.Font->Height == 0U) || (Style.Font->X_Advance == 0U))
    {
        return LCD_Request_Status_INVALID;
    }

    Command.Text_Length = Get_Text_Length(__Text, LCD_TEXT_MAX_LENGTH);
    if (Command.Text_Length == 0U)
    {
        return LCD_Request_Status_OK;
    }

    Text_Width = static_cast<uint32_t>(Command.Text_Length) * Style.Font->X_Advance * Style.Scale;
    if (Text_Width > 0xFFFFU)
    {
        return LCD_Request_Status_INVALID;
    }
    if ((__X >= Logical_Width) || (__Y >= Logical_Height))
    {
        return LCD_Request_Status_OK;
    }

    Command.Type = LCD_Command_TEXT;
    Command.X = __X;
    Command.Y = __Y;
    Command.Font = Style.Font;
    Command.Color = Style.Foreground;
    Command.Background = Style.Background;
    Command.Scale = Style.Scale;
    memcpy(Command.Text, __Text, Command.Text_Length);
    Command.Text[Command.Text_Length] = '\0';
    return Enqueue_Command(&Command);
}

void Class_LCD::Reset_Panel()
{
    HAL_GPIO_WritePin(Config.CS.Port, Config.CS.Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(Config.DC.Port, Config.DC.Pin, GPIO_PIN_RESET);
    Set_Backlight(0U);
    HAL_GPIO_WritePin(Config.RES.Port, Config.RES.Pin, GPIO_PIN_RESET);
    HAL_Delay(100U);
    HAL_GPIO_WritePin(Config.RES.Port, Config.RES.Pin, GPIO_PIN_SET);
    HAL_Delay(120U);
}

uint8_t Class_LCD::Initialize_Panel()
{
    for (uint16_t i = 0U; i < (sizeof(LCD_Init_Sequence) / sizeof(LCD_Init_Sequence[0])); i++)
    {
        if (Write_Command_Blocking(LCD_Init_Sequence[i].Command) == 0U)
        {
            return 0U;
        }
        if ((LCD_Init_Sequence[i].Data_Length != 0U) &&
            (Write_Data_Blocking(LCD_Init_Sequence[i].Data, LCD_Init_Sequence[i].Data_Length) == 0U))
        {
            return 0U;
        }
        if (LCD_Init_Sequence[i].Delay_ms != 0U)
        {
            HAL_Delay(LCD_Init_Sequence[i].Delay_ms);
        }
    }
    return 1U;
}

uint8_t Class_LCD::Apply_Rotation(Enum_LCD_Rotation __Rotation)
{
    uint8_t Memory_Access_Control;

    switch (__Rotation)
    {
    case LCD_Rotation_0:
        Memory_Access_Control = 0x00U;
        Logical_Width = LCD_WIDTH;
        Logical_Height = LCD_HEIGHT;
        Gram_Offset_X = LCD_GRAM_OFFSET_X;
        Gram_Offset_Y = LCD_GRAM_OFFSET_Y;
        break;

    case LCD_Rotation_90:
        Memory_Access_Control = 0x60U;
        Logical_Width = LCD_LANDSCAPE_WIDTH;
        Logical_Height = LCD_LANDSCAPE_HEIGHT;
        Gram_Offset_X = LCD_GRAM_OFFSET_Y;
        Gram_Offset_Y = LCD_GRAM_OFFSET_X;
        break;

    case LCD_Rotation_180:
        Memory_Access_Control = 0xC0U;
        Logical_Width = LCD_WIDTH;
        Logical_Height = LCD_HEIGHT;
        Gram_Offset_X = LCD_GRAM_OFFSET_X;
        Gram_Offset_Y = LCD_GRAM_OFFSET_Y;
        break;

    case LCD_Rotation_270:
        Memory_Access_Control = 0xA0U;
        Logical_Width = LCD_LANDSCAPE_WIDTH;
        Logical_Height = LCD_LANDSCAPE_HEIGHT;
        Gram_Offset_X = LCD_GRAM_OFFSET_Y;
        Gram_Offset_Y = LCD_GRAM_OFFSET_X;
        break;

    default:
        return 0U;
    }

    if ((Write_Command_Blocking(0x36U) == 0U) ||
        (Write_Data_Blocking(&Memory_Access_Control, 1U) == 0U))
    {
        return 0U;
    }

    Current_Rotation = __Rotation;
    return 1U;
}

uint8_t Class_LCD::Write_Command_Blocking(uint8_t __Command)
{
    HAL_StatusTypeDef Status;

    HAL_GPIO_WritePin(Config.CS.Port, Config.CS.Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Config.DC.Port, Config.DC.Pin, GPIO_PIN_RESET);
    Status = HAL_SPI_Transmit(Driver_SPI, &__Command, 1U, LCD_SPI_TIMEOUT);
    HAL_GPIO_WritePin(Config.CS.Port, Config.CS.Pin, GPIO_PIN_SET);
    return Status == HAL_OK ? 1U : 0U;
}

uint8_t Class_LCD::Write_Data_Blocking(const uint8_t *__Data, uint16_t __Length)
{
    HAL_StatusTypeDef Status;

    if ((__Data == 0) || (__Length == 0U))
    {
        return 0U;
    }

    HAL_GPIO_WritePin(Config.CS.Port, Config.CS.Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Config.DC.Port, Config.DC.Pin, GPIO_PIN_SET);
    Status = HAL_SPI_Transmit(Driver_SPI, const_cast<uint8_t *>(__Data), __Length, LCD_SPI_TIMEOUT);
    HAL_GPIO_WritePin(Config.CS.Port, Config.CS.Pin, GPIO_PIN_SET);
    return Status == HAL_OK ? 1U : 0U;
}

uint8_t Class_LCD::Set_Window_Blocking(uint16_t __X_Start, uint16_t __Y_Start,
                                       uint16_t __X_End, uint16_t __Y_End)
{
    uint8_t Address[4];

    if (Write_Command_Blocking(0x2AU) == 0U)
    {
        return 0U;
    }
    __X_Start = static_cast<uint16_t>(__X_Start + Gram_Offset_X);
    __X_End = static_cast<uint16_t>(__X_End + Gram_Offset_X);
    Address[0] = static_cast<uint8_t>(__X_Start >> 8U);
    Address[1] = static_cast<uint8_t>(__X_Start);
    Address[2] = static_cast<uint8_t>(__X_End >> 8U);
    Address[3] = static_cast<uint8_t>(__X_End);
    if (Write_Data_Blocking(Address, sizeof(Address)) == 0U)
    {
        return 0U;
    }

    if (Write_Command_Blocking(0x2BU) == 0U)
    {
        return 0U;
    }
    __Y_Start = static_cast<uint16_t>(__Y_Start + Gram_Offset_Y);
    __Y_End = static_cast<uint16_t>(__Y_End + Gram_Offset_Y);
    Address[0] = static_cast<uint8_t>(__Y_Start >> 8U);
    Address[1] = static_cast<uint8_t>(__Y_Start);
    Address[2] = static_cast<uint8_t>(__Y_End >> 8U);
    Address[3] = static_cast<uint8_t>(__Y_End);
    if (Write_Data_Blocking(Address, sizeof(Address)) == 0U)
    {
        return 0U;
    }

    return Write_Command_Blocking(0x2CU);
}

uint8_t Class_LCD::Clear_Blocking(uint16_t __Color)
{
    uint32_t Remaining = static_cast<uint32_t>(Logical_Width) * Logical_Height;

    for (uint16_t i = 0U; i < LCD_DMA_BUFFER_SIZE; i += 2U)
    {
        DMA_Buffer[i] = static_cast<uint8_t>(__Color >> 8U);
        DMA_Buffer[i + 1U] = static_cast<uint8_t>(__Color);
    }

    if (Set_Window_Blocking(0U, 0U,
                            static_cast<uint16_t>(Logical_Width - 1U),
                            static_cast<uint16_t>(Logical_Height - 1U)) == 0U)
    {
        return 0U;
    }

    while (Remaining != 0U)
    {
        uint16_t Pixel_Count = Remaining > (LCD_DMA_BUFFER_SIZE / 2U)
                                   ? static_cast<uint16_t>(LCD_DMA_BUFFER_SIZE / 2U)
                                   : static_cast<uint16_t>(Remaining);
        if (Write_Data_Blocking(DMA_Buffer, static_cast<uint16_t>(Pixel_Count * 2U)) == 0U)
        {
            return 0U;
        }
        Remaining -= Pixel_Count;
    }
    return 1U;
}

uint8_t Class_LCD::Start_DMA_Transfer(const uint8_t *__Data, uint16_t __Length, uint8_t __Data_Mode)
{
    HAL_StatusTypeDef Status;

    if ((__Data == 0) || (__Length == 0U))
    {
        Abort_Active_Command(LCD_Request_Status_INVALID);
        return 2U;
    }

    Clean_DMA_Cache(__Data, __Length);
    HAL_GPIO_WritePin(Config.CS.Port, Config.CS.Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Config.DC.Port, Config.DC.Pin,
                      __Data_Mode == 0U ? GPIO_PIN_RESET : GPIO_PIN_SET);
    Status = HAL_SPI_Transmit_DMA(Driver_SPI, const_cast<uint8_t *>(__Data), __Length);

    if (Status == HAL_OK)
    {
        DMA_Busy = 1U;
        return 1U;
    }

    HAL_GPIO_WritePin(Config.CS.Port, Config.CS.Pin, GPIO_PIN_SET);
    if (Status == HAL_BUSY)
    {
        return 0U;
    }

    Abort_Active_Command(LCD_Request_Status_ERROR);
    return 2U;
}

uint8_t Class_LCD::Prepare_Next_Region()
{
    uint16_t Point_X;
    uint16_t Point_Y;

    Region_Valid = 0U;

    switch (Active_Command.Type)
    {
    case LCD_Command_CLEAR:
    case LCD_Command_FILL_RECTANGLE:
        if (Primitive_Index != 0U)
        {
            return 0U;
        }
        Primitive_Index = 1U;
        if ((Active_Command.X >= Logical_Width) || (Active_Command.Y >= Logical_Height))
        {
            return 0U;
        }
        Region_X = Active_Command.X;
        Region_Y = Active_Command.Y;
        Region_Width = Active_Command.Width > static_cast<uint16_t>(Logical_Width - Region_X)
                           ? static_cast<uint16_t>(Logical_Width - Region_X)
                           : Active_Command.Width;
        Region_Height = Active_Command.Height > static_cast<uint16_t>(Logical_Height - Region_Y)
                            ? static_cast<uint16_t>(Logical_Height - Region_Y)
                            : Active_Command.Height;
        Region_Source_X = 0U;
        Region_Source_Y = 0U;
        break;

    case LCD_Command_TEXT:
        if (Primitive_Index != 0U)
        {
            return 0U;
        }
        Primitive_Index = 1U;
        if ((Active_Command.X >= Logical_Width) || (Active_Command.Y >= Logical_Height))
        {
            return 0U;
        }
        Region_X = Active_Command.X;
        Region_Y = Active_Command.Y;
        Region_Width = Get_Text_Width(&Active_Command);
        Region_Height = Get_Text_Height(&Active_Command);
        if (Region_Width > static_cast<uint16_t>(Logical_Width - Region_X))
        {
            Region_Width = static_cast<uint16_t>(Logical_Width - Region_X);
        }
        if (Region_Height > static_cast<uint16_t>(Logical_Height - Region_Y))
        {
            Region_Height = static_cast<uint16_t>(Logical_Height - Region_Y);
        }
        Region_Source_X = 0U;
        Region_Source_Y = 0U;
        break;

    case LCD_Command_BITMAP:
        if (Primitive_Index != 0U)
        {
            return 0U;
        }
        Primitive_Index = 1U;
        if ((Active_Command.X >= Logical_Width) || (Active_Command.Y >= Logical_Height))
        {
            return 0U;
        }
        Region_X = Active_Command.X;
        Region_Y = Active_Command.Y;
        Region_Width = Active_Command.Width > static_cast<uint16_t>(Logical_Width - Region_X)
                           ? static_cast<uint16_t>(Logical_Width - Region_X)
                           : Active_Command.Width;
        Region_Height = Active_Command.Height > static_cast<uint16_t>(Logical_Height - Region_Y)
                            ? static_cast<uint16_t>(Logical_Height - Region_Y)
                            : Active_Command.Height;
        Region_Source_X = Active_Command.Source_X;
        Region_Source_Y = Active_Command.Source_Y;
        break;

    case LCD_Command_PIXEL:
        if (Primitive_Index != 0U)
        {
            return 0U;
        }
        Primitive_Index = 1U;
        if ((Active_Command.X >= Logical_Width) || (Active_Command.Y >= Logical_Height))
        {
            return 0U;
        }
        Region_X = Active_Command.X;
        Region_Y = Active_Command.Y;
        Region_Width = 1U;
        Region_Height = 1U;
        Region_Source_X = 0U;
        Region_Source_Y = 0U;
        break;

    case LCD_Command_LINE:
        if (Find_Next_Primitive_Point(&Point_X, &Point_Y) == 0U)
        {
            return 0U;
        }
        Region_X = Point_X;
        Region_Y = Point_Y;
        Region_Width = 1U;
        Region_Height = 1U;
        Region_Source_X = 0U;
        Region_Source_Y = 0U;
        break;

    case LCD_Command_RECTANGLE:
    {
        uint16_t Line_Width = Active_Command.Line_Width;

        if (Line_Width > Active_Command.Width)
        {
            Line_Width = Active_Command.Width;
        }
        if (Line_Width > Active_Command.Height)
        {
            Line_Width = Active_Command.Height;
        }
        if (Shape_Row >= Active_Command.Height)
        {
            return 0U;
        }

        Region_Y = static_cast<uint16_t>(Active_Command.Y + Shape_Row);
        Region_Height = 1U;
        Region_Source_X = 0U;
        Region_Source_Y = 0U;

        if ((Shape_Row < Line_Width) ||
            (Shape_Row >= static_cast<uint16_t>(Active_Command.Height - Line_Width)))
        {
            Region_X = Active_Command.X;
            Region_Width = Active_Command.Width;
            Shape_Row++;
            Shape_Part = 0U;
        }
        else if (Shape_Part == 0U)
        {
            Region_X = Active_Command.X;
            Region_Width = Line_Width;
            Shape_Part = 1U;
        }
        else
        {
            Region_X = static_cast<uint16_t>(Active_Command.X + Active_Command.Width - Line_Width);
            Region_Width = Line_Width;
            Shape_Row++;
            Shape_Part = 0U;
        }
        break;
    }

    case LCD_Command_FILL_CIRCLE:
    {
        int32_t Delta_Y;
        uint32_t Radius_Square;
        uint16_t Half_Width;
        int32_t Start_X;
        int32_t End_X;

        for (;;)
        {
            if (Shape_Row > static_cast<uint16_t>(Active_Command.Radius * 2U))
            {
                return 0U;
            }

            Delta_Y = static_cast<int32_t>(Shape_Row) - Active_Command.Radius;
            Radius_Square = static_cast<uint32_t>(Active_Command.Radius) * Active_Command.Radius;
            Half_Width = Integer_Sqrt(Radius_Square - static_cast<uint32_t>(Delta_Y * Delta_Y));
            Start_X = static_cast<int32_t>(Active_Command.X) - Half_Width;
            End_X = static_cast<int32_t>(Active_Command.X) + Half_Width;
            Shape_Row++;

            if ((End_X < 0) || (Start_X >= Logical_Width) ||
                (static_cast<int32_t>(Active_Command.Y) + Delta_Y < 0) ||
                (static_cast<int32_t>(Active_Command.Y) + Delta_Y >= Logical_Height))
            {
                continue;
            }
            if (Start_X < 0)
            {
                Start_X = 0;
            }
            if (End_X >= Logical_Width)
            {
                End_X = Logical_Width - 1;
            }

            Region_X = static_cast<uint16_t>(Start_X);
            Region_Y = static_cast<uint16_t>(static_cast<int32_t>(Active_Command.Y) + Delta_Y);
            Region_Width = static_cast<uint16_t>(End_X - Start_X + 1);
            Region_Height = 1U;
            Region_Source_X = 0U;
            Region_Source_Y = 0U;
            break;
        }
        break;
    }

    case LCD_Command_CIRCLE:
    {
        int32_t Delta_Y;
        uint32_t Radius_Square;
        uint32_t Inner_Radius;
        uint16_t Outer_Width;
        uint16_t Inner_Width;
        uint8_t Inner_Row_Exists;
        int32_t Start_X;
        int32_t End_X;
        int32_t Row_Y;

        Inner_Radius = Active_Command.Radius > Active_Command.Line_Width
                           ? Active_Command.Radius - Active_Command.Line_Width
                           : 0U;

        for (;;)
        {
            if (Shape_Row > static_cast<uint16_t>(Active_Command.Radius * 2U))
            {
                return 0U;
            }

            Delta_Y = static_cast<int32_t>(Shape_Row) - Active_Command.Radius;
            Row_Y = static_cast<int32_t>(Active_Command.Y) + Delta_Y;
            Radius_Square = static_cast<uint32_t>(Active_Command.Radius) * Active_Command.Radius;
            Outer_Width = Integer_Sqrt(Radius_Square - static_cast<uint32_t>(Delta_Y * Delta_Y));
            {
                uint32_t Inner_Square = Inner_Radius * Inner_Radius;
                uint32_t Delta_Square = static_cast<uint32_t>(Delta_Y * Delta_Y);
                Inner_Row_Exists = Delta_Square <= Inner_Square ? 1U : 0U;
                Inner_Width = Inner_Row_Exists == 0U
                                  ? 0U
                                  : Integer_Sqrt(Inner_Square - Delta_Square);
            }

            if ((Row_Y < 0) || (Row_Y >= Logical_Height))
            {
                Shape_Row++;
                Shape_Part = 0U;
                continue;
            }

            if ((Inner_Radius == 0U) || (Inner_Row_Exists == 0U) ||
                (Outer_Width <= Inner_Width))
            {
                Start_X = static_cast<int32_t>(Active_Command.X) - Outer_Width;
                End_X = static_cast<int32_t>(Active_Command.X) + Outer_Width;
                Shape_Row++;
                Shape_Part = 0U;
            }
            else if (Shape_Part == 0U)
            {
                Start_X = static_cast<int32_t>(Active_Command.X) - Outer_Width;
                End_X = static_cast<int32_t>(Active_Command.X) - Inner_Width - 1;
                Shape_Part = 1U;
            }
            else
            {
                Start_X = static_cast<int32_t>(Active_Command.X) + Inner_Width + 1;
                End_X = static_cast<int32_t>(Active_Command.X) + Outer_Width;
                Shape_Row++;
                Shape_Part = 0U;
            }

            if ((End_X < 0) || (Start_X >= Logical_Width) || (Start_X > End_X))
            {
                continue;
            }
            if (Start_X < 0)
            {
                Start_X = 0;
            }
            if (End_X >= Logical_Width)
            {
                End_X = Logical_Width - 1;
            }

            Region_X = static_cast<uint16_t>(Start_X);
            Region_Y = static_cast<uint16_t>(Row_Y);
            Region_Width = static_cast<uint16_t>(End_X - Start_X + 1);
            Region_Height = 1U;
            Region_Source_X = 0U;
            Region_Source_Y = 0U;
            break;
        }
        break;
    }

    default:
        return 0U;
    }

    if ((Region_Width == 0U) || (Region_Height == 0U))
    {
        return 0U;
    }

    Region_Pixel_Offset = 0U;
    Region_Pixel_Count = static_cast<uint32_t>(Region_Width) * Region_Height;
    Region_Valid = 1U;
    return 1U;
}

uint8_t Class_LCD::Find_Next_Primitive_Point(uint16_t *__X, uint16_t *__Y)
{
    if ((__X == 0) || (__Y == 0))
    {
        return 0U;
    }

    while (Primitive_Index < Primitive_Total)
    {
        uint32_t Index = Primitive_Index++;
        if ((Get_Primitive_Point(Index, __X, __Y) != 0U) &&
            (*__X < Logical_Width) && (*__Y < Logical_Height))
        {
            return 1U;
        }
    }

    return 0U;
}

uint8_t Class_LCD::Get_Primitive_Point(uint32_t __Index, uint16_t *__X, uint16_t *__Y) const
{
    int32_t Point_X;
    int32_t Point_Y;

    if ((__X == 0) || (__Y == 0))
    {
        return 0U;
    }

    if (Active_Command.Type == LCD_Command_LINE)
    {
        uint32_t Steps = Primitive_Total > 0U ? Primitive_Total - 1U : 0U;
        int32_t Delta_X = static_cast<int32_t>(Active_Command.End_X) - Active_Command.X;
        int32_t Delta_Y = static_cast<int32_t>(Active_Command.End_Y) - Active_Command.Y;

        if (Steps == 0U)
        {
            Point_X = Active_Command.X;
            Point_Y = Active_Command.Y;
        }
        else
        {
            Point_X = static_cast<int32_t>(Active_Command.X) +
                      static_cast<int32_t>((static_cast<int64_t>(Delta_X) * __Index) /
                                           static_cast<int64_t>(Steps));
            Point_Y = static_cast<int32_t>(Active_Command.Y) +
                      static_cast<int32_t>((static_cast<int64_t>(Delta_Y) * __Index) /
                                           static_cast<int64_t>(Steps));
        }
    }
    else if (Active_Command.Type == LCD_Command_RECTANGLE)
    {
        uint16_t Local_X = static_cast<uint16_t>(__Index % Active_Command.Width);
        uint16_t Local_Y = static_cast<uint16_t>(__Index / Active_Command.Width);
        uint32_t Candidate_X = static_cast<uint32_t>(Active_Command.X) + Local_X;
        uint32_t Candidate_Y = static_cast<uint32_t>(Active_Command.Y) + Local_Y;

        if ((Candidate_X > 0xFFFFU) || (Candidate_Y > 0xFFFFU))
        {
            return 0U;
        }
        Point_X = static_cast<int32_t>(Candidate_X);
        Point_Y = static_cast<int32_t>(Candidate_Y);
    }
    else if ((Active_Command.Type == LCD_Command_CIRCLE) ||
             (Active_Command.Type == LCD_Command_FILL_CIRCLE))
    {
        uint32_t Side = static_cast<uint32_t>(Active_Command.Radius) * 2U + 1U;
        int32_t Local_X = static_cast<int32_t>(__Index % Side) - Active_Command.Radius;
        int32_t Local_Y = static_cast<int32_t>(__Index / Side) - Active_Command.Radius;
        Point_X = static_cast<int32_t>(Active_Command.X) + Local_X;
        Point_Y = static_cast<int32_t>(Active_Command.Y) + Local_Y;
    }
    else
    {
        return 0U;
    }

    if ((Point_X < 0) || (Point_Y < 0) || (Point_X > 0xFFFF) || (Point_Y > 0xFFFF))
    {
        return 0U;
    }

    *__X = static_cast<uint16_t>(Point_X);
    *__Y = static_cast<uint16_t>(Point_Y);
    return Is_Primitive_Pixel(*__X, *__Y);
}

uint8_t Class_LCD::Is_Primitive_Pixel(uint16_t __X, uint16_t __Y) const
{
    if (Active_Command.Type == LCD_Command_LINE)
    {
        return 1U;
    }

    if (Active_Command.Type == LCD_Command_RECTANGLE)
    {
        uint16_t Local_X = static_cast<uint16_t>(__X - Active_Command.X);
        uint16_t Local_Y = static_cast<uint16_t>(__Y - Active_Command.Y);
        uint16_t Line_Width = Active_Command.Line_Width;

        return ((Local_X < Line_Width) || (Local_Y < Line_Width) ||
                (Local_X >= (Active_Command.Width > Line_Width
                                 ? static_cast<uint16_t>(Active_Command.Width - Line_Width)
                                 : 0U)) ||
                (Local_Y >= (Active_Command.Height > Line_Width
                                 ? static_cast<uint16_t>(Active_Command.Height - Line_Width)
                                 : 0U)))
                   ? 1U
                   : 0U;
    }

    if ((Active_Command.Type == LCD_Command_CIRCLE) ||
        (Active_Command.Type == LCD_Command_FILL_CIRCLE))
    {
        int32_t Delta_X = static_cast<int32_t>(__X) - Active_Command.X;
        int32_t Delta_Y = static_cast<int32_t>(__Y) - Active_Command.Y;
        uint32_t Distance = static_cast<uint32_t>(Delta_X * Delta_X + Delta_Y * Delta_Y);
        uint32_t Outer_Radius = Active_Command.Radius;
        uint32_t Outer_Square = Outer_Radius * Outer_Radius;

        if (Active_Command.Type == LCD_Command_FILL_CIRCLE)
        {
            return Distance <= Outer_Square ? 1U : 0U;
        }

        uint32_t Inner_Radius = Outer_Radius > Active_Command.Line_Width
                                    ? Outer_Radius - Active_Command.Line_Width
                                    : 0U;
        uint32_t Inner_Square = Inner_Radius * Inner_Radius;
        if (Inner_Radius == 0U)
        {
            return Distance <= Outer_Square ? 1U : 0U;
        }
        return ((Distance <= Outer_Square) && (Distance > Inner_Square)) ? 1U : 0U;
    }

    return 0U;
}

uint16_t Class_LCD::Get_Text_Width(const Struct_LCD_Command *__Command) const
{
    if ((__Command == 0) || (__Command->Font == 0))
    {
        return 0U;
    }
    return static_cast<uint16_t>(static_cast<uint32_t>(__Command->Text_Length) *
                                 __Command->Font->X_Advance * __Command->Scale);
}

uint16_t Class_LCD::Get_Text_Height(const Struct_LCD_Command *__Command) const
{
    if ((__Command == 0) || (__Command->Font == 0))
    {
        return 0U;
    }
    return static_cast<uint16_t>(static_cast<uint16_t>(__Command->Font->Height) * __Command->Scale);
}

uint8_t Class_LCD::Get_Font_Row(const Struct_LCD_Font *__Font,
                                char __Character, uint8_t __Row) const
{
    if ((__Font == 0) || (__Row >= __Font->Height))
    {
        return 0U;
    }

    if (__Font->Get_Row != 0)
    {
        return __Font->Get_Row(__Character, __Row);
    }

    if ((__Font->Data == 0) || (__Font->Bytes_Per_Row == 0U))
    {
        return 0U;
    }

    uint8_t Character = static_cast<uint8_t>(__Character);
    if ((Character < __Font->First_Character) || (Character > __Font->Last_Character))
    {
        Character = static_cast<uint8_t>('?');
        if ((Character < __Font->First_Character) || (Character > __Font->Last_Character))
        {
            return 0U;
        }
    }

    uint32_t Glyph_Size = static_cast<uint32_t>(__Font->Height) * __Font->Bytes_Per_Row;
    uint32_t Offset = static_cast<uint32_t>(Character - __Font->First_Character) * Glyph_Size +
                      static_cast<uint32_t>(__Row) * __Font->Bytes_Per_Row;
    return __Font->Data[Offset];
}

uint16_t Class_LCD::Get_Rectangle_Pixel(const Struct_LCD_Command *__Command,
                                        uint16_t __X, uint16_t __Y) const
{
    (void)__X;
    (void)__Y;
    return __Command == 0 ? static_cast<uint16_t>(LCD_Color_BLACK) : __Command->Color;
}

uint16_t Class_LCD::Get_Text_Pixel(const Struct_LCD_Command *__Command,
                                   uint16_t __X, uint16_t __Y) const
{
    if ((__Command == 0) || (__Command->Font == 0) || (__Command->Scale == 0U))
    {
        return LCD_Color_BLACK;
    }

    uint16_t Character_Width = static_cast<uint16_t>(__Command->Font->X_Advance * __Command->Scale);
    uint16_t Character_Index = static_cast<uint16_t>(__X / Character_Width);
    if (Character_Index >= __Command->Text_Length)
    {
        return __Command->Background;
    }

    uint16_t Character_X = static_cast<uint16_t>((__X % Character_Width) / __Command->Scale);
    uint16_t Character_Y = static_cast<uint16_t>(__Y / __Command->Scale);
    if ((Character_X >= __Command->Font->Width) || (Character_Y >= __Command->Font->Height))
    {
        return __Command->Background;
    }

    uint8_t Row = Get_Font_Row(__Command->Font,
                               __Command->Text[Character_Index],
                               static_cast<uint8_t>(Character_Y));
    return (Row & (1U << Character_X)) != 0U ? __Command->Color : __Command->Background;
}

uint16_t Class_LCD::Get_Bitmap_Pixel(const Struct_LCD_Command *__Command,
                                     uint16_t __X, uint16_t __Y) const
{
    if ((__Command == 0) || (__Command->Pixels == 0))
    {
        return LCD_Color_BLACK;
    }
    return __Command->Pixels[static_cast<uint32_t>(__Y) * __Command->Stride + __X];
}

uint16_t Class_LCD::Get_Region_Pixel(uint16_t __X, uint16_t __Y) const
{
    uint16_t Source_X = static_cast<uint16_t>(Region_Source_X + __X);
    uint16_t Source_Y = static_cast<uint16_t>(Region_Source_Y + __Y);

    switch (Active_Command.Type)
    {
    case LCD_Command_TEXT:
        return Get_Text_Pixel(&Active_Command, Source_X, Source_Y);

    case LCD_Command_BITMAP:
        return Get_Bitmap_Pixel(&Active_Command, Source_X, Source_Y);

    case LCD_Command_CLEAR:
    case LCD_Command_FILL_RECTANGLE:
        return Get_Rectangle_Pixel(&Active_Command, Source_X, Source_Y);

    case LCD_Command_PIXEL:
    case LCD_Command_LINE:
    case LCD_Command_RECTANGLE:
    case LCD_Command_CIRCLE:
    case LCD_Command_FILL_CIRCLE:
        return Active_Command.Color;

    default:
        return LCD_Color_BLACK;
    }
}

uint16_t Class_LCD::Clip_Value(uint16_t __Value, uint16_t __Maximum) const
{
    return __Value > __Maximum ? __Maximum : __Value;
}

void Class_LCD::Complete_Active_Command()
{
    if ((Active_Command_Valid != 0U) && (Queue_Count != 0U))
    {
        Queue_Head = static_cast<uint8_t>((Queue_Head + 1U) % LCD_COMMAND_QUEUE_DEPTH);
        Queue_Count--;
    }

    Active_Command = Struct_LCD_Command();
    Active_Command_Valid = 0U;
    Region_Valid = 0U;
    Transfer_State = LCD_Transfer_State_IDLE;
    Last_Result = LCD_Request_Status_OK;
    LCD_Status = Queue_Count == 0U ? LCD_Status_ENABLE : LCD_Status_BUSY;
}

void Class_LCD::Abort_Active_Command(Enum_LCD_Request_Status __Result)
{
    HAL_GPIO_WritePin(Config.CS.Port, Config.CS.Pin, GPIO_PIN_SET);
    DMA_Busy = 0U;
    Queue_Head = 0U;
    Queue_Tail = 0U;
    Queue_Count = 0U;
    Active_Command = Struct_LCD_Command();
    Active_Command_Valid = 0U;
    Region_Valid = 0U;
    Transfer_State = LCD_Transfer_State_IDLE;
    Last_Result = __Result;
    LCD_Status = LCD_Status_ERROR;
}

void Class_LCD::Format_Integer(int32_t __Value, char *__Text) const
{
    char Reverse[11];
    uint8_t Length = 0U;
    uint8_t Negative = __Value < 0 ? 1U : 0U;
    uint32_t Absolute_Value = Negative != 0U
                                  ? static_cast<uint32_t>(-(__Value + 1)) + 1U
                                  : static_cast<uint32_t>(__Value);

    if (__Text == 0)
    {
        return;
    }

    do
    {
        Reverse[Length++] = static_cast<char>('0' + (Absolute_Value % 10U));
        Absolute_Value /= 10U;
    } while ((Absolute_Value != 0U) && (Length < sizeof(Reverse)));

    uint8_t Output = 0U;
    if (Negative != 0U)
    {
        __Text[Output++] = '-';
    }
    while (Length != 0U)
    {
        __Text[Output++] = Reverse[--Length];
    }
    __Text[Output] = '\0';
}

void Class_LCD::Format_Float(float __Value, uint8_t __Decimal_Digits, char *__Text) const
{
    char Reverse[11];
    uint8_t Reverse_Length = 0U;
    uint8_t Output = 0U;
    uint8_t Negative = __Value < 0.0f ? 1U : 0U;
    uint32_t Scale = 1U;

    if (__Text == 0)
    {
        return;
    }

    if (__Value != __Value)
    {
        memcpy(__Text, "NAN", 4U);
        return;
    }

    for (uint8_t i = 0U; i < __Decimal_Digits; i++)
    {
        Scale *= 10U;
    }

    float Absolute_Value = Negative != 0U ? -__Value : __Value;
    float Maximum_Value = static_cast<float>(0xFFFFFFFFU) / static_cast<float>(Scale);
    if (Absolute_Value >= Maximum_Value)
    {
        if (Negative != 0U)
        {
            memcpy(__Text, "-INF", 5U);
        }
        else
        {
            memcpy(__Text, "INF", 4U);
        }
        return;
    }

    uint32_t Scaled_Value = static_cast<uint32_t>(Absolute_Value * static_cast<float>(Scale) + 0.5f);
    uint32_t Integer_Part = Scaled_Value / Scale;
    uint32_t Fractional_Part = Scaled_Value % Scale;

    if (Negative != 0U)
    {
        __Text[Output++] = '-';
    }

    do
    {
        Reverse[Reverse_Length++] = static_cast<char>('0' + (Integer_Part % 10U));
        Integer_Part /= 10U;
    } while ((Integer_Part != 0U) && (Reverse_Length < sizeof(Reverse)));

    while (Reverse_Length != 0U)
    {
        __Text[Output++] = Reverse[--Reverse_Length];
    }

    if (__Decimal_Digits != 0U)
    {
        __Text[Output++] = '.';
        uint32_t Divisor = Scale / 10U;
        for (uint8_t i = 0U; i < __Decimal_Digits; i++)
        {
            __Text[Output++] = static_cast<char>('0' + ((Fractional_Part / Divisor) % 10U));
            Divisor /= 10U;
        }
    }

    __Text[Output] = '\0';
}
