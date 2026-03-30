




/*
 * safe_drive_auto.h
 *
 *  Created on: 2026. 3. 23.
 *
 *  [Main role]
 *  ---------------------------------------------------------
 *  초음파 기반 auto 주행을 SafeDrive 계층으로 연결하는 모듈
 *
 *  역할
 *    - auto 상태 관리
 *    - 초음파 센서값 기반 회피 판단
 *    - SafeDrive_Move() / SafeDrive_Stop() 호출
 */

#ifndef INC_SAFE_DRIVE_AUTO_H_
#define INC_SAFE_DRIVE_AUTO_H_

#include "stm32f4xx_hal.h"
#include <stdint.h>

/* =========================================================
 * auto 상태
 * ========================================================= */
typedef enum
{
    AUTO_STATE_SCAN = 0,
    AUTO_STATE_STOP,
    AUTO_STATE_PIVOT,        /* 단위 피벗 (재측정 후 반복 가능) */
    AUTO_STATE_FORWARD_HOLD, /* 피벗 후 직진 유지 (centering 무시, 충돌만 체크) */
    AUTO_STATE_BACK,
} AUTO_STATE;

/* =========================================================
 * 초기화 / task
 * ========================================================= */
void SafeDriveAuto_Init(void);
void SafeDriveAuto_Task(void);


#endif /* INC_SAFE_DRIVE_AUTO_H_ */
