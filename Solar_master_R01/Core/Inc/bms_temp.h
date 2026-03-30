/*
 * temp.h
 *
 *  Created on: 2026. 3. 18.
 *      Author: kimsuyeon
 *
 *      [Main role]
 *
 *      ADC 읽기
 *      온도 계산
 *      SAFE / WARNING / DANGER 상태 판정
 *      현재 상태 반환
 */

#ifndef INC_BMS_TEMP_H_
#define INC_BMS_TEMP_H_

#include "stm32f4xx_hal.h"
#include <stdint.h>



/* =========================================================
 * 온도 상태
 * ========================================================= */
typedef enum
{
    TEMP_STATE_SAFE = 0,
    TEMP_STATE_WARNING,
    TEMP_STATE_DANGER,
    TEMP_STATE_FAULT
} TEMP_STATE;


/* =========================================================
 * Temp_Calculate
 * ========================================================= */
uint16_t Temp_ReadADC(void);
int16_t Temp_CalcCelsius(uint16_t adc);
TEMP_STATE Temp_GetState(int16_t temp_c);
TEMP_STATE Temp_Task(void);



/* =========================================================
 * 현재 값 조회
 * ========================================================= */
int16_t Temp_GetNowCelsius(void);
TEMP_STATE Temp_GetNowState(void);




#endif /* INC_BMS_TEMP_H_ */
