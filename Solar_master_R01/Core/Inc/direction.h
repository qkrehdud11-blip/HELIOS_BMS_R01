/*
 * direction.h
 *
 *  Created on: Feb 24, 2026
 *      Author: lenovo
 */

#ifndef INC_DIRECTION_H_
#define INC_DIRECTION_H_



#include "stm32f4xx_hal.h"
#include <stdint.h>



typedef enum
{
  DIR_STOP = 0,

  DIR_FRONT,
  DIR_BACK,

  DIR_LEFT,          // 제자리 좌회전 (좌 정지, 우 전진)
  DIR_RIGHT,         // 제자리 우회전 (좌 전진, 우 정지)

  DIR_PIVOT_LEFT,    // 피벗 좌회전 (좌 후진, 우 전진)
  DIR_PIVOT_RIGHT,   // 피벗 우회전 (좌 전진, 우 후진)

  DIR_LEFTFRONT,
  DIR_RIGHTFRONT,

  DIR_LEFTBACK,
  DIR_RIGHTBACK

} dir_state_t;

void Direction_Init(void);
void Direction_Set(dir_state_t st);


#endif /* INC_DIRECTION_H_ */
