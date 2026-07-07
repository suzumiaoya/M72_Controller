// SPDX-License-Identifier: AGPL-3.0-only
#ifndef TSK_CONFIG_AND_CALLBACK_H
#define TSK_CONFIG_AND_CALLBACK_H

#ifdef STM32H723xx
#include "stm32h7xx_hal.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

void Task_Init();
void Task_Loop();

#ifdef __cplusplus
}
#endif

#endif
