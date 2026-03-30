/*
 * statemachine.h
 *
 *  Created on: Mar 1, 2026
 *      Author: appletea
 */

#ifndef INC_STATEMACHINE_H_
#define INC_STATEMACHINE_H_


#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <stdbool.h>


//#include "car.h"
//#include "ultrasonic.h"

// KSY
#include "adc.h"
//#include "bms_safety_manager.h"

//LSH
#include "trace.h"



//#define INIT_SPEED 	80
//
//#define Block_Distance_Front 35
//#define Block_Distance_Side 20
//#define Crash_Distance 10
//#define Stuck_Timeout 300

// ================= 기준 값 =================
//
//#define BACK_DIST       10
//#define LRTH            15
//#define CRITICAL_DIST   40
//#define TURN_THRESHOLD  20
//
//typedef enum
//{
//	AUTO_STATE_SCAN,
//	AUTO_STATE_STOP,
//	AUTO_STATE_AVOID,
//	AUTO_STATE_BACK,
//}AUTO_STATE;

// 초기화: UART1 인터럽트 시작 등
void STMACHINE_Init(void);
//void ST_FLAG(uint8_t cmd);
void ST_MACHINE();



/* ================= DEBUG ================= */

//const char* GetModeString(void);

bool ST_GetSolarFlag(void);
bool ST_GetTraceFlag(void);
bool ST_GetForceLock(void);



//void DC_CONTROL_AUTO();
//void app_charger_mode(bool mode);



void SHOW_UART6_TRACE(void);


#endif /* INC_STATEMACHINE_H_ */
