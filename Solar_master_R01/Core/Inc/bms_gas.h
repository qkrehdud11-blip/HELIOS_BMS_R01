/*
 * gas2.h
 *
 *  Created on: 2026. 3. 20.
 *      Author: kimsuyeon
 */

#ifndef INC_BMS_GAS_H_
#define INC_BMS_GAS_H_



#include "stm32f4xx_hal.h"
#include <stdint.h>

typedef enum
{
    GAS_SAFE = 0,
    GAS_WARNING,
    GAS_DANGER,
    GAS_FAULT
} GasLevel_t;

uint16_t Gas_ReadADC(void);
uint16_t Gas_ReadADC_Avg(void);

void Gas_BaselineInit(void);
void Gas_UpdateBaseline(void);
uint8_t Gas_IsBaselineReady(void);
uint16_t Gas_GetBaselineADC(void);

uint16_t Gas_ComputeDelta(uint16_t adc_now, uint16_t adc_base);

GasLevel_t Gas_GetLevelFromADC(uint16_t adc);
GasLevel_t Gas_TaskLevel(void);

uint16_t Gas_GetNowADC(void);
uint16_t Gas_GetNowDelta(void);
GasLevel_t Gas_GetNowState(void);


#endif /* INC_BMS_GAS_H_ */
