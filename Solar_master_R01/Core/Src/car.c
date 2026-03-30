/*
 * car.c
 *
 *  Created on: Feb 24, 2026
 *      Author: lenovo
 */



#include "car.h"


void Car_Init(void)
{
  /* 안전하게 정지 상태로 시작 */
  Direction_Set(DIR_STOP);
  Speed_StopAll();

  /* PWM 출력 시작 (Speed_Init 이후 호출되어야 함) */
  Speed_Start();
}

void Car_Move(car_state_t state, speed_state_t speed)
{
  switch (state)
  {
    case CAR_STOP:
      Direction_Set(DIR_STOP);
      Speed_StopAll();
      break;

    case CAR_FRONT:
      Direction_Set(DIR_FRONT);
      Speed_SetStateBoth(speed, speed);
      break;

    case CAR_BACK:
      Direction_Set(DIR_BACK);
      Speed_SetStateBoth(speed, speed);
      break;

    /* 좌/우 회전: 피벗 (반대쪽 100% 후진) */
    case CAR_LEFT:
      Direction_Set(DIR_PIVOT_LEFT);
      Speed_SetStateBoth(SPD_100, speed);
      break;

    case CAR_RIGHT:
      Direction_Set(DIR_PIVOT_RIGHT);
      Speed_SetStateBoth(speed, SPD_100);
      break;

    /* 피벗 회전: 양쪽 반대 방향 (더 좁은 회전) */
    case CAR_PIVOT_LEFT:
      Direction_Set(DIR_PIVOT_LEFT);
      Speed_SetStateBoth(speed, speed);
      break;

    case CAR_PIVOT_RIGHT:
      Direction_Set(DIR_PIVOT_RIGHT);
      Speed_SetStateBoth(speed, speed);
      break;

    /* 대각 전진: 50/speed, speed/50 */
    case CAR_LEFTFRONT:
      Direction_Set(DIR_LEFTFRONT);
      Speed_SetStateBoth(SPD_30, speed);
      break;

    case CAR_RIGHTFRONT:
      Direction_Set(DIR_RIGHTFRONT);
      Speed_SetStateBoth(speed, SPD_30);
      break;

    /* 대각 후진 */
    case CAR_LEFTBACK:
      Direction_Set(DIR_BACK);
      Speed_SetStateBoth(SPD_50, speed);
      break;

    case CAR_RIGHTBACK:
      Direction_Set(DIR_BACK);
      Speed_SetStateBoth(speed, SPD_50);
      break;

    default:
      Direction_Set(DIR_STOP);
      Speed_StopAll();
      break;
  }
}

void Car_Stop(void)
{
  Direction_Set(DIR_STOP);
  Speed_StopAll();
}




