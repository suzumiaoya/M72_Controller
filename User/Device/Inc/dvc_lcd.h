// SPDX-License-Identifier: AGPL-3.0-only
#ifndef DVC_LCD_H
#define DVC_LCD_H

/**
 * @file dvc_lcd.h
 * @author ZLLC
 * @brief DM LCD ST7789 configuration and drawing interface
 * @version 0.2
 * @date 2026-07-13
 *
 * @copyright ZLLC 2026
 */

/* Includes ------------------------------------------------------------------*/

#include "spi.h"

#include <stdint.h>

/* Exported macros -----------------------------------------------------------*/

#define LCD_WIDTH                    240U
#define LCD_HEIGHT                   280U
#define LCD_LANDSCAPE_WIDTH          LCD_HEIGHT
#define LCD_LANDSCAPE_HEIGHT         LCD_WIDTH
#define LCD_GRAM_OFFSET_X            0U
#define LCD_GRAM_OFFSET_Y            20U
#define LCD_COMMAND_QUEUE_DEPTH      16U
#define LCD_TEXT_MAX_LENGTH          40U
#define LCD_DMA_BUFFER_SIZE          1120U

/* Exported types ------------------------------------------------------------*/

enum Enum_LCD_Status
{
    LCD_Status_DISABLE = 0,
    LCD_Status_ENABLE,
    LCD_Status_BUSY,
    LCD_Status_ERROR,
};

enum Enum_LCD_Request_Status
{
    LCD_Request_Status_OK = 0,
    LCD_Request_Status_BUSY,
    LCD_Request_Status_FULL,
    LCD_Request_Status_INVALID,
    LCD_Request_Status_ERROR,
};

enum Enum_LCD_Rotation : uint8_t
{
    LCD_Rotation_0 = 0,
    LCD_Rotation_90,
    LCD_Rotation_180,
    LCD_Rotation_270,
};

enum Enum_LCD_Color
{
    LCD_Color_BLACK   = 0x0000U,
    LCD_Color_WHITE   = 0xFFFFU,
    LCD_Color_RED     = 0xF800U,
    LCD_Color_GREEN   = 0x07E0U,
    LCD_Color_BLUE    = 0x001FU,
    LCD_Color_YELLOW  = 0xFFE0U,
    LCD_Color_CYAN    = 0x07FFU,
    LCD_Color_MAGENTA = 0xF81FU,
    LCD_Color_ORANGE  = 0xFD20U,
};

struct Struct_LCD_Pin
{
    GPIO_TypeDef *Port = 0;
    uint16_t Pin = 0;
};

struct Struct_LCD_Config
{
    Struct_LCD_Pin CS;
    Struct_LCD_Pin DC;
    Struct_LCD_Pin RES;
    Struct_LCD_Pin BLK;
    uint8_t Backlight_Active_High = 1U;
    Enum_LCD_Rotation Rotation = LCD_Rotation_0;
    uint8_t Inversion = 1U;
};

typedef uint8_t (*LCD_Font_Get_Row_Call_Back)(char Character, uint8_t Row);

struct Struct_LCD_Font
{
    const uint8_t *Data = 0;
    uint8_t First_Character = 0U;
    uint8_t Last_Character = 0U;
    uint8_t Width = 0U;
    uint8_t Height = 0U;
    uint8_t Bytes_Per_Row = 0U;
    uint8_t X_Advance = 0U;
    LCD_Font_Get_Row_Call_Back Get_Row = 0;
};

struct Struct_LCD_Text_Style
{
    const Struct_LCD_Font *Font = 0;
    uint16_t Foreground = LCD_Color_WHITE;
    uint16_t Background = LCD_Color_BLACK;
    uint8_t Scale = 1U;
};

extern const Struct_LCD_Font LCD_Font_Default;

class Class_LCD
{
public:
    /**
     * @brief Bind the SPI/GPIO resources and initialize the fixed DM panel.
     */
    void Init(SPI_HandleTypeDef *__Driver_SPI, const Struct_LCD_Config *__Config);

    inline Enum_LCD_Status Get_LCD_Status();
    inline Enum_LCD_Request_Status Get_Last_Result();
    inline uint16_t Get_Width();
    inline uint16_t Get_Height();
    inline uint8_t Get_Free_Command_Count();

    Enum_LCD_Request_Status Clear(uint16_t __Color);
    Enum_LCD_Request_Status Draw_Pixel(uint16_t __X, uint16_t __Y, uint16_t __Color);
    Enum_LCD_Request_Status Draw_Line(uint16_t __Start_X, uint16_t __Start_Y,
                                      uint16_t __End_X, uint16_t __End_Y, uint16_t __Color);
    Enum_LCD_Request_Status Draw_Rectangle(uint16_t __X, uint16_t __Y,
                                           uint16_t __Width, uint16_t __Height,
                                           uint16_t __Line_Width, uint16_t __Color);
    Enum_LCD_Request_Status Fill_Rectangle(uint16_t __X, uint16_t __Y,
                                           uint16_t __Width, uint16_t __Height, uint16_t __Color);
    Enum_LCD_Request_Status Draw_Circle(uint16_t __Center_X, uint16_t __Center_Y,
                                        uint16_t __Radius, uint16_t __Line_Width, uint16_t __Color);
    Enum_LCD_Request_Status Fill_Circle(uint16_t __Center_X, uint16_t __Center_Y,
                                        uint16_t __Radius, uint16_t __Color);
    Enum_LCD_Request_Status Draw_String(uint16_t __X, uint16_t __Y,
                                        const char *__Text, const Struct_LCD_Text_Style *__Style);
    Enum_LCD_Request_Status Draw_Integer(uint16_t __X, uint16_t __Y,
                                         int32_t __Value, const Struct_LCD_Text_Style *__Style);
    Enum_LCD_Request_Status Draw_Float(uint16_t __X, uint16_t __Y,
                                       float __Value, uint8_t __Decimal_Digits,
                                       const Struct_LCD_Text_Style *__Style);
    Enum_LCD_Request_Status Draw_RGB565(uint16_t __X, uint16_t __Y,
                                        uint16_t __Width, uint16_t __Height,
                                        const uint16_t *__Pixels, uint16_t __Stride);

    void Set_Backlight(uint8_t __Enable);
    Enum_LCD_Request_Status Set_Rotation(Enum_LCD_Rotation __Rotation);
    Enum_LCD_Request_Status Set_Inversion(uint8_t __Enable);
    Enum_LCD_Request_Status Set_Display(uint8_t __Enable);
    Enum_LCD_Request_Status Set_Sleep(uint8_t __Enable);

    /**
     * @brief Advance at most one DMA segment from task context.
     */
    void Refresh();

    /**
     * @brief Forward SPI events from the shared SPI driver.
     */
    void SPI_TxCpltCallback(SPI_HandleTypeDef *__SPI);
    void SPI_ErrorCallback(SPI_HandleTypeDef *__SPI);

protected:
    enum Enum_LCD_Command_Type
    {
        LCD_Command_NONE = 0,
        LCD_Command_CLEAR,
        LCD_Command_PIXEL,
        LCD_Command_LINE,
        LCD_Command_RECTANGLE,
        LCD_Command_FILL_RECTANGLE,
        LCD_Command_CIRCLE,
        LCD_Command_FILL_CIRCLE,
        LCD_Command_TEXT,
        LCD_Command_BITMAP,
    };

    enum Enum_LCD_Transfer_State
    {
        LCD_Transfer_State_IDLE = 0,
        LCD_Transfer_State_COLUMN_COMMAND,
        LCD_Transfer_State_COLUMN_DATA,
        LCD_Transfer_State_ROW_COMMAND,
        LCD_Transfer_State_ROW_DATA,
        LCD_Transfer_State_MEMORY_COMMAND,
        LCD_Transfer_State_PIXEL_DATA,
        LCD_Transfer_State_NEXT_REGION,
    };

    struct Struct_LCD_Command
    {
        Enum_LCD_Command_Type Type = LCD_Command_NONE;
        uint16_t X = 0U;
        uint16_t Y = 0U;
        uint16_t Width = 0U;
        uint16_t Height = 0U;
        uint16_t End_X = 0U;
        uint16_t End_Y = 0U;
        uint16_t Radius = 0U;
        uint16_t Line_Width = 1U;
        uint16_t Color = LCD_Color_BLACK;
        uint16_t Source_X = 0U;
        uint16_t Source_Y = 0U;
        uint16_t Stride = 0U;
        uint8_t Decimal_Digits = 0U;
        uint8_t Text_Length = 0U;
        uint8_t Scale = 1U;
        const uint16_t *Pixels = 0;
        const Struct_LCD_Font *Font = 0;
        uint16_t Background = LCD_Color_BLACK;
        char Text[LCD_TEXT_MAX_LENGTH + 1U] = {0};
    };

    SPI_HandleTypeDef *Driver_SPI = 0;
    Struct_LCD_Config Config = {};
    Enum_LCD_Status LCD_Status = LCD_Status_DISABLE;
    Enum_LCD_Request_Status Last_Result = LCD_Request_Status_OK;
    Enum_LCD_Rotation Current_Rotation = LCD_Rotation_0;
    uint16_t Logical_Width = LCD_WIDTH;
    uint16_t Logical_Height = LCD_HEIGHT;
    uint16_t Gram_Offset_X = LCD_GRAM_OFFSET_X;
    uint16_t Gram_Offset_Y = LCD_GRAM_OFFSET_Y;

    Struct_LCD_Command Command_Queue[LCD_COMMAND_QUEUE_DEPTH] = {};
    uint8_t Queue_Head = 0U;
    uint8_t Queue_Tail = 0U;
    uint8_t Queue_Count = 0U;
    Struct_LCD_Command Active_Command = {};
    uint8_t Active_Command_Valid = 0U;

    volatile uint8_t DMA_Busy = 0U;
    Enum_LCD_Transfer_State Transfer_State = LCD_Transfer_State_IDLE;
    uint8_t Control_Buffer[4] = {0};
    uint8_t DMA_Buffer[LCD_DMA_BUFFER_SIZE] __attribute__((aligned(32))) = {0};

    uint16_t Region_X = 0U;
    uint16_t Region_Y = 0U;
    uint16_t Region_Width = 0U;
    uint16_t Region_Height = 0U;
    uint16_t Region_Source_X = 0U;
    uint16_t Region_Source_Y = 0U;
    uint32_t Region_Pixel_Offset = 0U;
    uint32_t Region_Pixel_Count = 0U;
    uint32_t Primitive_Index = 0U;
    uint32_t Primitive_Total = 0U;
    uint16_t Shape_Row = 0U;
    uint8_t Shape_Part = 0U;
    uint8_t Region_Valid = 0U;

    Enum_LCD_Request_Status Enqueue_Command(const Struct_LCD_Command *__Command);
    Enum_LCD_Request_Status Enqueue_Rectangle(Enum_LCD_Command_Type __Type,
                                              uint16_t __X, uint16_t __Y,
                                              uint16_t __Width, uint16_t __Height,
                                              uint16_t __Color);
    Enum_LCD_Request_Status Enqueue_Text(uint16_t __X, uint16_t __Y,
                                         const char *__Text, const Struct_LCD_Text_Style *__Style);

    void Reset_Panel();
    uint8_t Initialize_Panel();
    uint8_t Apply_Rotation(Enum_LCD_Rotation __Rotation);
    uint8_t Write_Command_Blocking(uint8_t __Command);
    uint8_t Write_Data_Blocking(const uint8_t *__Data, uint16_t __Length);
    uint8_t Set_Window_Blocking(uint16_t __X_Start, uint16_t __Y_Start,
                                uint16_t __X_End, uint16_t __Y_End);
    uint8_t Clear_Blocking(uint16_t __Color);
    uint8_t Start_DMA_Transfer(const uint8_t *__Data, uint16_t __Length, uint8_t __Data_Mode);

    uint8_t Prepare_Next_Region();
    uint8_t Find_Next_Primitive_Point(uint16_t *__X, uint16_t *__Y);
    uint8_t Get_Primitive_Point(uint32_t __Index, uint16_t *__X, uint16_t *__Y) const;
    uint8_t Is_Primitive_Pixel(uint16_t __X, uint16_t __Y) const;
    uint16_t Get_Text_Width(const Struct_LCD_Command *__Command) const;
    uint16_t Get_Text_Height(const Struct_LCD_Command *__Command) const;
    uint8_t Get_Font_Row(const Struct_LCD_Font *__Font, char __Character, uint8_t __Row) const;
    uint16_t Get_Rectangle_Pixel(const Struct_LCD_Command *__Command, uint16_t __X, uint16_t __Y) const;
    uint16_t Get_Text_Pixel(const Struct_LCD_Command *__Command, uint16_t __X, uint16_t __Y) const;
    uint16_t Get_Bitmap_Pixel(const Struct_LCD_Command *__Command, uint16_t __X, uint16_t __Y) const;
    uint16_t Get_Region_Pixel(uint16_t __X, uint16_t __Y) const;
    uint16_t Clip_Value(uint16_t __Value, uint16_t __Maximum) const;

    void Complete_Active_Command();
    void Abort_Active_Command(Enum_LCD_Request_Status __Result);
    void Format_Integer(int32_t __Value, char *__Text) const;
    void Format_Float(float __Value, uint8_t __Decimal_Digits, char *__Text) const;
};

inline Enum_LCD_Status Class_LCD::Get_LCD_Status()
{
    return (LCD_Status);
}

inline Enum_LCD_Request_Status Class_LCD::Get_Last_Result()
{
    return (Last_Result);
}

inline uint16_t Class_LCD::Get_Width()
{
    return (Logical_Width);
}

inline uint16_t Class_LCD::Get_Height()
{
    return (Logical_Height);
}

inline uint8_t Class_LCD::Get_Free_Command_Count()
{
    return static_cast<uint8_t>(LCD_COMMAND_QUEUE_DEPTH - Queue_Count);
}

#endif
