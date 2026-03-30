


/*
 * statemachine.h
 *
 *  Created on: Mar 1, 2026
 *      Author: appletea
 *
 *  [Main role]
 *  ---------------------------------------------------------
 *  기존 주행 상태머신 모듈
 *
 *  역할
 *    - UART1 1바이트 수신 버퍼 관리
 *    - manual 주행 처리
 *    - auto 주행 처리
 *    - 초음파 기반 회피 로직 처리
 *
 *  구조 변경 포인트
 *  ---------------------------------------------------------
 *  - manual / auto 모드 판단은 app_bms가 담당
 *  - statemachine은 App_Bms_GetMode()를 보고 동작만 수행
 *  - UART RX complete callback은 전역 함수 대신
 *    라우터 함수 STMACHINE_UartRxCpltCallback() 제공
 */

#ifndef INC_STATEMACHINE_H_
#define INC_STATEMACHINE_H_



#include "stdio.h"
#include "stm32f4xx_hal.h"
#include "ultrasonic.h"
#include "stdbool.h"
#include "safe_drive_auto.h"

/* 초기화: UART1 인터럽트 시작 등 */
void STMACHINE_Init(void);


/* UART RX complete callback 라우터 */
void STMACHINE_UartRxCpltCallback(UART_HandleTypeDef *huart);

/* 외부에서 직접 명령 주입 (UART2/moserial 등) */
void STMACHINE_SubmitCmd(uint8_t cmd);

/* 기존 호환용 */
void ST_FLAG(uint8_t cmd);

/* 상위 task */
void ST_MACHINE(void);


#endif /* INC_STATEMACHINE_H_ */




//========= 동원이형 코드=================================================//
///*
// * statemachine.h
// *
// *  Created on: Mar 1, 2026
// *      Author: appletea
// */
//
//#ifndef INC_STATEMACHINE_H_
//#define INC_STATEMACHINE_H_
//
//#include "stdio.h"
//#include "stm32f4xx_hal.h"
//#include "speed.h"
//#include "direction.h"
//#include "car.h"
//#include "ultrasonic.h"
//#include "stdbool.h"
//
//
//#define Block_Distance_Front 35
//#define Block_Distance_Side 20
//#define Crash_Distance 10
//#define Stuck_Timeout 300
//
//
//
//// 초기화: UART1 인터럽트 시작 등
//void STMACHINE_Init(void);
//
//void SHOW_UART2();
//
//typedef enum
//{
//	AUTO_STATE_SCAN,
//	AUTO_STATE_STOP,
//	AUTO_STATE_AVOID,
//	AUTO_STATE_BACK,
//}AUTO_STATE;
//
//
//
//void ST_FLAG(uint8_t cmd);
//void ST_MACHINE();
//void DC_CONTROL_AUTO();
//
//
//
//
//
//#endif /* INC_STATEMACHINE_H_ */
