/*
 * app_can_control.h
 *
 *  Created on: Mar 23, 2026
 *      Author: parkdoyoung
 */

#ifndef INC_APP_CAN_CONTROL_H_
#define INC_APP_CAN_CONTROL_H_



#include "stm32f4xx_hal.h"
#include <stdint.h>

uint8_t AppCan_SendTrace(uint8_t on);
uint8_t AppCan_SendSolar(uint8_t on);
uint8_t AppCan_SendForceStop(void);
uint8_t AppCan_SendForceReinit(void);
uint8_t AppCan_SendDrive(uint8_t on);




#endif /* INC_APP_CAN_CONTROL_H_ */
