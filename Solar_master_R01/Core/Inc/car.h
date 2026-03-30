/*
 * car.h
 *
 *  Created on: Feb 24, 2026
 *      Author: lenovo
 */

#ifndef INC_CAR_H_
#define INC_CAR_H_


#include "stm32f4xx_hal.h"
#include <stdint.h>
#include "speed.h"
#include "direction.h"

typedef enum
{
  CAR_STOP = 0,
  CAR_FRONT,
  CAR_BACK,

  CAR_LEFT,        // 좌 0 / 우 speed (한쪽 정지)
  CAR_RIGHT,       // 좌 speed / 우 0 (한쪽 정지)

  CAR_PIVOT_LEFT,  // 좌 후진 speed / 우 전진 speed (피벗)
  CAR_PIVOT_RIGHT, // 좌 전진 speed / 우 후진 speed (피벗)

  CAR_LEFTFRONT,   // 50 / speed
  CAR_RIGHTFRONT,  // speed / 50
  CAR_LEFTBACK,
  CAR_RIGHTBACK
} car_state_t;


void Car_Init(void);


/* state + speed(기본 속도)로 주행 */
void Car_Move(car_state_t state, speed_state_t speed);


/* 즉시 정지 */
void Car_Stop(void);






#endif /* INC_CAR_H_ */
