/*
 * bms_sensor.h
 *
 *  Created on: 2026. 3. 21.
 *      Author: kimsuyeon
 *
 *      TEMP task, GAS task, INA219(BMS 전압/전류) task
 *			각 상태값, 측정값 저장
 *			getter 제공
 */

#ifndef INC_BMS_SENSOR_H_
#define INC_BMS_SENSOR_H_


#include "stm32f4xx_hal.h"
#include "bms_temp.h"
#include "bms_gas.h"
#include "bms_ina219.h"
#include <stdint.h>

void BMS_SENSOR_Init(void);
void BMS_SENSOR_Service(void);
void BMS_SENSOR_Task(void);

/* TEMP */
uint16_t GetTempADC(void);
int16_t GetTempC(void);
TEMP_STATE GetTempState(void);

/* GAS */
uint16_t GetGasADC(void);
uint16_t GetGasDelta(void);
uint8_t IsGasBaselineReady(void);
uint16_t GetGasBaseADC(void);
GasLevel_t GetGasState(void);

/* ELECTRICAL */
uint8_t IsBmsElectricalValid(void);
int32_t GetVoltagemV(void);
VOLTAGE_STATE GetVoltageState(void);
int32_t GetCurrentmA(void);
CURRENT_STATE GetCurrentState(void);



#endif /* INC_BMS_SENSOR_H_ */
