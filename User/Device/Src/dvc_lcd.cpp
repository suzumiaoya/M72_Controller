#include "dvc_lcd.h"

void Class_LCD::Init(SPI_HandleTypeDef *__Driver_SPI)
{
    Driver_SPI = __Driver_SPI;
}

void Class_LCD::SPI_RxCpltCallback(uint8_t *Tx_Buffer, uint8_t *Rx_Buffer, uint16_t Length)
{
    UNUSED(Tx_Buffer);
    UNUSED(Rx_Buffer);
    UNUSED(Length);
}

void Class_LCD::Refresh()
{
    if (Driver_SPI == 0)
    {
        return;
    }
}
