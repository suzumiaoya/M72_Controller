#ifndef DVC_LCD_H
#define DVC_LCD_H

#include "drv_spi.h"

class Class_LCD
{
public:
    void Init(SPI_HandleTypeDef *__Driver_SPI);

    inline uint8_t Get_Page();
    inline void Set_Page(uint8_t __Current_Page);

    void SPI_RxCpltCallback(uint8_t *Tx_Buffer, uint8_t *Rx_Buffer, uint16_t Length);
    void Refresh();

protected:
    SPI_HandleTypeDef *Driver_SPI = 0;
    uint8_t Current_Page = 0;
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
