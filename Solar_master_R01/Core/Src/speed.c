/*
 * speed.c
 *
 *  Created on: Feb 24, 2026
 *      Author: lenovo
 */


#include "speed.h"




static TIM_HandleTypeDef *s_htimL = 0;
static TIM_HandleTypeDef *s_htimR = 0;
static uint32_t s_chL = TIM_CHANNEL_1;
static uint32_t s_chR = TIM_CHANNEL_2;


/* 상태 -> duty percent (0~100)
 * ---------------------------------------------------------
 * speed_state_t 는 5% 단위 enum (SPD_STOP=0, SPD_5=1, ..., SPD_100=20)
 * 값 * 5 = 퍼센트
 */
static uint8_t state_to_percent(speed_state_t st)
{
    uint8_t pct = (uint8_t)((uint8_t)st * 5U);
    return (pct > 100U) ? 100U : pct;
}

void Speed_Init(TIM_HandleTypeDef *htim_left, uint32_t ch_left,
                TIM_HandleTypeDef *htim_right, uint32_t ch_right)
{
  s_htimL = htim_left;
  s_htimR = htim_right;
  s_chL = ch_left;
  s_chR = ch_right;

  Speed_StopAll();
}


void Speed_Start(void)
{
	if (!s_htimL || !s_htimR) return;

	HAL_TIM_PWM_Start(s_htimL, s_chL);
	HAL_TIM_PWM_Start(s_htimR, s_chR);
}

void Speed_SetRaw(speed_side_t side, uint16_t ccr)
{
  if (!s_htimL || !s_htimR) return;

  TIM_HandleTypeDef *htim = (side == SPEED_LEFT) ? s_htimL : s_htimR;
  uint32_t ch = (side == SPEED_LEFT) ? s_chL : s_chR;

  uint32_t arr = __HAL_TIM_GET_AUTORELOAD(htim);
  if (ccr > arr) ccr = arr;

  __HAL_TIM_SET_COMPARE(htim, ch, ccr);
}


/* percent(0~100) -> CCR(0~ARR) */
void Speed_SetPercent(speed_side_t side, uint8_t percent)
{
  if (!s_htimL || !s_htimR) return;
  if (percent > 100) percent = 100;

  TIM_HandleTypeDef *htim = (side == SPEED_LEFT) ? s_htimL : s_htimR;
  uint32_t arr = __HAL_TIM_GET_AUTORELOAD(htim);

  /*
   * PWM mode 기준으로 보통 듀티 = CCR / (ARR+1)
   * 그래서 퍼센트 계산도 (ARR+1)을 기준으로 맞춰주면 100%가 정확해짐.
   */
  uint32_t period = arr + 1U;
  uint32_t ccr = (period * (uint32_t)percent) / 100U;

  /* CCR은 0~ARR 범위여야 하니까 100%일 때 arr로 클램프 */
  if (ccr >= period) ccr = arr;

  Speed_SetRaw(side, (uint16_t)ccr);
}

void Speed_SetPercentBoth(uint8_t left_percent, uint8_t right_percent)
{
  Speed_SetPercent(SPEED_LEFT, left_percent);
  Speed_SetPercent(SPEED_RIGHT, right_percent);
}

void Speed_SetState(speed_side_t side, speed_state_t st)
{
  Speed_SetPercent(side, state_to_percent(st));
}

void Speed_SetStateBoth(speed_state_t left, speed_state_t right)
{
  Speed_SetState(SPEED_LEFT, left);
  Speed_SetState(SPEED_RIGHT, right);
}

void Speed_StopAll(void)
{
  Speed_SetRaw(SPEED_LEFT, 0);
  Speed_SetRaw(SPEED_RIGHT, 0);
}
