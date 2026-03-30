/*
 * trace.h
 *
 *  Created on: 2026. 3. 16.
 *      Author: kimsuyeon
 */

#ifndef INC_TRACE_H_
#define INC_TRACE_H_

#include "stm32f4xx_hal.h"
#include "tim.h"
#include "usart.h"
#include "stdio.h"
#include "stdbool.h"

#define FILTER_SIZE 10
// 좌우, 상하 오차 관련 변수(800미만시 상단이 정지하지 않고 조금씩 움직임)
#define THRESHOLD 800

typedef enum {
  MODE_INIT,    // 태양 추적 초기화
  MODE_ACT      // 태양 추적 시작
} Mode;

extern Mode mode;

extern uint16_t adcValue[4];
extern uint16_t S1,S2,S3,S4;

extern int error_x;
extern int error_y;


//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void Trace_ForceInit(void);
void Trace_Mode(uint8_t mode);
void Trace_Init();

const char* Trace_GetStateString(void);



#endif /* INC_TRACE_H_ */
