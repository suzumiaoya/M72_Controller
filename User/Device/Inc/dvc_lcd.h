// SPDX-License-Identifier: AGPL-3.0-only
#ifndef DVC_LCD_H
#define DVC_LCD_H

/**
 * @file dvc_lcd.h
 * @author ZLLC
 * @brief ST7789 LCD configuration and display interface
 * @version 0.1
 * @date 2026-07-10
 *
 * @copyright ZLLC 2026
 */

/* Includes ------------------------------------------------------------------*/

#include "spi.h"

#include <stdint.h>

/* Exported macros -----------------------------------------------------------*/

#define LCD_WIDTH               240U
#define LCD_HEIGHT              280U
#define LCD_JOINT_COUNT         6U
#define LCD_ARM_COUNT           2U
#define LCD_FIELD_CHARACTER_NUM 10U

/* Exported types ------------------------------------------------------------*/

enum Enum_LCD_Color
{
    LCD_Color_BLACK  = 0x0000U,
    LCD_Color_WHITE  = 0xFFFFU,
    LCD_Color_RED    = 0xF800U,
    LCD_Color_GREEN  = 0x07E0U,
    LCD_Color_BLUE   = 0x001FU,
    LCD_Color_YELLOW = 0xFFE0U,
    LCD_Color_CYAN   = 0x07FFU,
};

struct Struct_LCD_Status
{
    float Current_Joint_Angle[LCD_ARM_COUNT][LCD_JOINT_COUNT];
};

class Class_LCD
{
public:
    /**
     * @brief Bind SPI and draw the startup page before periodic timers start.
     */
    void Init(SPI_HandleTypeDef *__Driver_SPI);

    /**
     * @brief Enable or disable the LCD backlight.
     */
    void Set_Backlight(uint8_t Enable);

    /**
     * @brief Replace the pending status snapshot with the newest data.
     */
    void Submit_Status(const Struct_LCD_Status *Status);

    inline uint8_t Get_Page();
    inline void Set_Page(uint8_t __Current_Page);

    /**
     * @brief Advance at most one non-blocking DMA transfer from task context.
     */
    void Refresh();

    /**
     * @brief Complete a SPI TX DMA segment; called by the HAL callback.
     */
    void SPI_TxCpltCallback(SPI_HandleTypeDef *hspi);

    /**
     * @brief Abort the current LCD frame after a SPI error.
     */
    void SPI_ErrorCallback(SPI_HandleTypeDef *hspi);

protected:
    enum Enum_LCD_Transfer_State
    {
        LCD_Transfer_State_IDLE = 0,
        LCD_Transfer_State_COLUMN_COMMAND,
        LCD_Transfer_State_COLUMN_DATA,
        LCD_Transfer_State_ROW_COMMAND,
        LCD_Transfer_State_ROW_DATA,
        LCD_Transfer_State_MEMORY_COMMAND,
        LCD_Transfer_State_PIXEL_DATA,
        LCD_Transfer_State_NEXT_FIELD,
    };

    SPI_HandleTypeDef *Driver_SPI = 0;
    uint8_t Current_Page = 0;

    Struct_LCD_Status Pending_Status = {};
    Struct_LCD_Status Active_Status = {};
    uint8_t Pending_Status_Valid = 0;
    uint8_t Frame_Active = 0;
    volatile uint8_t DMA_Busy = 0;
    Enum_LCD_Transfer_State Transfer_State = LCD_Transfer_State_IDLE;

    uint8_t Active_Field = 0;
    uint16_t Field_X = 0;
    uint16_t Field_Y = 0;
    uint16_t Pixel_Offset = 0;
    uint8_t Dirty_Field[LCD_ARM_COUNT][LCD_JOINT_COUNT] = {{0}};
    char Active_Text[LCD_ARM_COUNT][LCD_JOINT_COUNT][LCD_FIELD_CHARACTER_NUM + 1] = {{{0}}};
    char Displayed_Text[LCD_ARM_COUNT][LCD_JOINT_COUNT][LCD_FIELD_CHARACTER_NUM + 1] = {{{0}}};

    uint8_t Control_Buffer[4] = {0};
    uint8_t Field_Buffer[LCD_FIELD_CHARACTER_NUM * 6U * 12U * 2U] = {0};

    void Reset_Panel();
    void Initialize_Panel();
    void Draw_Startup_Page();
    void Draw_Text_Blocking(uint16_t X, uint16_t Y, const char *Text, uint16_t Foreground, uint16_t Background);
    void Draw_Field_Blocking(uint16_t X, uint16_t Y);
    void Clear_Blocking(uint16_t Color);
    void Set_Window_Blocking(uint16_t X_Start, uint16_t Y_Start, uint16_t X_End, uint16_t Y_End);
    void Write_Command_Blocking(uint8_t Command);
    void Write_Data_Blocking(const uint8_t *Data, uint16_t Length);

    void Format_Status();
    void Format_Integer(int32_t Value, char *Text) const;
    void Format_Float(float Value, char *Text) const;
    void Build_Field(const char *Text, uint8_t Character_Count, uint16_t Foreground, uint16_t Background);
    uint8_t Select_Next_Dirty_Field();
    uint8_t Start_DMA_Transfer(const uint8_t *Data, uint16_t Length, uint8_t Data_Mode);
};

inline uint8_t Class_LCD::Get_Page()
{
    return (Current_Page);
}

inline void Class_LCD::Set_Page(uint8_t __Current_Page)
{
    Current_Page = __Current_Page;
}

#endif
