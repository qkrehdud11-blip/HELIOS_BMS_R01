/*
 * ina219_bms.h
 *
 *  Created on: 2026. 3. 18.
 *      Author: kimsuyeon
 *
 *      [Main role]
 *
 *      INA219 초기화
 *      전압 / 전류 읽기
 *      SAFE / WARNING / DANGER 상태 판정
 *      현재 상태 반환
 */

#ifndef INC_BMS_INA219_H_
#define INC_BMS_INA219_H_



#include "stm32f4xx_hal.h"
#include "i2c.h"
#include <stdint.h>

/* =========================================================
 * 현재 상태 enum
 * ========================================================= */
typedef enum
{
    CURRENT_STATE_SAFE = 0,
    CURRENT_STATE_WARNING,
    CURRENT_STATE_DANGER
} CURRENT_STATE;

typedef enum
{
    VOLTAGE_STATE_SAFE = 0,
    VOLTAGE_STATE_OVER_WARNING,
    VOLTAGE_STATE_OVER_DANGER,
    VOLTAGE_STATE_UNDER_WARNING,
    VOLTAGE_STATE_UNDER_DANGER
} VOLTAGE_STATE;

/* =========================================================
 * 센서 객체 정보
 * ========================================================= */
typedef struct
{
    I2C_HandleTypeDef *hi2c;
    uint8_t addr;
    int32_t rshunt_mohm;   /* shunt resistor in mΩ */
} INA219_BMS_t;

/* =========================================================
 * public API
 * ========================================================= */

/* 1회 초기화 (non-blocking, 상태머신 진입) */
void INA219_BMS_Init(void);

/* 주기적으로 호출하는 non-blocking task */
void INA219_BMS_Task(void);

/* 최신 측정값 반환 */
int32_t INA219_BMS_ReadVoltage_mV(void);
int32_t INA219_BMS_ReadCurrent_mA(void);

/* 데이터 유효 여부 */
uint8_t INA219_BMS_IsValid(void);

/* 상태 판정 */
CURRENT_STATE INA219_BMS_GetCurrentState(int32_t current_mA);
VOLTAGE_STATE INA219_BMS_GetVoltageState(int32_t voltage_mV);

/* 사용자 HAL callback 에서 호출할 라우터 함수 */
void INA219_BMS_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c);
void INA219_BMS_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c);
void INA219_BMS_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c);



#endif /* INC_BMS_INA219_H_ */
