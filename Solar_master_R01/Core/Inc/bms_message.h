/*
 * bms_message.h
 *
 *  Created on: 2026. 3. 20.
 *      Author: kimsuyeon
 */

#ifndef INC_BMS_MESSAGE_H_
#define INC_BMS_MESSAGE_H_



#include "stm32f4xx_hal.h"
#include "bms_temp.h"
#include "bms_gas.h"
#include "bms_ina219.h"
#include "bms_sensor.h"
#include "bms_safety_manager.h"
#include "ultrasonic.h"

void SHOW_UART6_BMS(void);

/* 전역 HAL UART callback에서 라우팅용으로 호출 */
void BMS_MESSAGE_UartTxCpltCallback(UART_HandleTypeDef *huart);
void BMS_MESSAGE_UartErrorCallback(UART_HandleTypeDef *huart);



#endif /* INC_BMS_MESSAGE_H_ */
