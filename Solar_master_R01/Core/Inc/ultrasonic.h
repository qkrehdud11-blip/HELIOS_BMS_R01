/*
 * ultrasonic.h
 *
 *  Created on: Feb 28, 2026
 *      Author: lenovo
 */

#ifndef INC_ULTRASONIC_H_
#define INC_ULTRASONIC_H_




#include "stm32f4xx_hal.h"
#include <stdint.h>



typedef enum
{
  US_LEFT = 0,		// TIM3 CH4
  US_CENTER,		// TIM3 CH3
  US_RIGHT,			// TIM3 CH1
  US_COUNT
} ultrasonic_id_t;


typedef struct
{
  // TRIG GPIO
  GPIO_TypeDef *trig_port;
  uint16_t      trig_pin;

  // IC Timer/Channel
  TIM_HandleTypeDef *htim;
  uint32_t           tim_channel;   // e.g. TIM_CHANNEL_4
  uint32_t           tim_it_cc;      // e.g. TIM_IT_CC4

  // capture state
  volatile uint16_t ic_v1;
  volatile uint16_t ic_v2;
  volatile uint16_t echo_time_us;
  volatile uint8_t  distance_cm;
  volatile uint8_t  capture_flag;   // 0: waiting rising, 1: waiting falling
} ultrasonic_ch_t;



/* 초기화: 채널 설정 + IC Start(IT) */
void Ultrasonic_Init(void);

/* 3채널 TRIG 발생 + 각 채널 CC 인터럽트 enable */
void Ultrasonic_TriggerAll(void);

/* 특정 채널만 TRIG */
void Ultrasonic_TriggerOne(ultrasonic_id_t id);

/* 거리값 읽기 (cm) */
uint8_t Ultrasonic_GetDistanceCm(ultrasonic_id_t id);

/*
 * main.c의 HAL_TIM_IC_CaptureCallback() 안에서 이 함수 호출해주면 됨.
 * 예) void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){ Ultrasonic_IC_CaptureCallback(htim); }
 */
void Ultrasonic_IC_CaptureCallback(TIM_HandleTypeDef *htim);





#endif /* INC_ULTRASONIC_H_ */
