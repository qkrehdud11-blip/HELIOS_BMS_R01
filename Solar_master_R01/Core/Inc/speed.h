/*
 * speed.h
 *
 *  Created on: Feb 24, 2026
 *      Author: lenovo
 */

#ifndef INC_SPEED_H_
#define INC_SPEED_H_



#include "stm32f4xx_hal.h"
#include <stdint.h>
#include "tim.h"




typedef enum
{
  SPD_STOP = 0,   // 0%
  SPD_5,          // 5%
  SPD_10,   	  // 10%
  SPD_15,         // 15%
  SPD_20,	      // 20%
  SPD_25,         // 25%
  SPD_30,         // 30%
  SPD_35,         // 35%
  SPD_40,         // 40%
  SPD_45,         // 45%
  SPD_50,         // 50%
  SPD_55,         // 55%
  SPD_60,         // 60%
  SPD_65,         // 65%
  SPD_70,         // 70%
  SPD_75,         // 75%
  SPD_80,		  // 80%
  SPD_85,		  // 85%
  SPD_90,         // 90%
  SPD_95,		  // 95%
  SPD_100         // 100%
} speed_state_t;



typedef enum
{
  SPEED_LEFT = 0,
  SPEED_RIGHT
} speed_side_t;




/* 초기화: 좌/우 PWM 타이머 + 채널 지정 */
void Speed_Init(TIM_HandleTypeDef *htim_left, uint32_t ch_left,
                TIM_HandleTypeDef *htim_right, uint32_t ch_right);


/* PWM Start (Init 이후 호출) */
void Speed_Start(void);


/* raw CCR 값(0~ARR) 직접 설정 */
void Speed_SetRaw(speed_side_t side, uint16_t ccr);


/* 듀티 퍼센트(0~100) 직접 설정 */
void Speed_SetPercent(speed_side_t side, uint8_t percent);
void Speed_SetPercentBoth(uint8_t left_percent, uint8_t right_percent);


/* 상태 기반 설정(SPD_30/50/70/100) */
void Speed_SetState(speed_side_t side, speed_state_t st);
void Speed_SetStateBoth(speed_state_t left, speed_state_t right);


/* 정지 */
void Speed_StopAll(void);





#endif /* INC_SPEED_H_ */




