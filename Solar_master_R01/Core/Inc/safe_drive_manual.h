




/*
 * safe_drive_manual.h
 *
 *  Created on: 2026. 3. 23.
 *
 *  [Main role]
 *  ---------------------------------------------------------
 *  수동 주행 명령을 SafeDrive 계층으로 연결하는 모듈
 *
 *  역할
 *    - 리모컨 수동 주행 문자 해석
 *    - SafeDrive_Move() / SafeDrive_Stop() 호출
 *    - A / P / D / K 같은 토글 명령은 여기서 처리하지 않음
 */

#ifndef INC_SAFE_DRIVE_MANUAL_H_
#define INC_SAFE_DRIVE_MANUAL_H_

#include "stm32f4xx_hal.h"
#include <stdint.h>

/* =========================================================
 * 수동 주행 명령 처리
 * ---------------------------------------------------------
 * return
 *   1 : 주행 명령으로 처리함
 *   0 : 수동 주행 명령 아님
 * ========================================================= */
uint8_t SafeDriveManual_HandleCmd(uint8_t cmd);


#endif /* INC_SAFE_DRIVE_MANUAL_H_ */






