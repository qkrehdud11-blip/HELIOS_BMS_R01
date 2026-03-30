/*
 * safe_drive.h
 *
 *  Created on: 2026. 3. 23.
 *
 *  [Main role]
 *  ---------------------------------------------------------
 *  기존 주행 코드의 실제 모터 구동 앞단에서
 *  BMS 안전 제한 속도를 적용하는 wrapper
 *
 *  역할
 *    - statemachine / auto / manual 이 요청한 속도를 받는다.
 *    - App_Bms_ApplySpeedLimit() 로 현재 안전 제한을 적용한다.
 *    - 최종 속도를 기존 Car_Move() 형식에 맞게 전달한다.
 *
 *  주의
 *  ---------------------------------------------------------
 *  - danger latch가 떠도 바로 정지시키지 않는다.
 *  - bms_safety의 applied_limit_pct가 ramp로 천천히 떨어지므로,
 *    그 결과를 그대로 따라가야 "팍 떨어지지 않는" 감속이 된다.
 *  - 최종 적용 속도가 0%가 되었을 때만 Car_Stop()을 호출한다.
 */

#ifndef INC_SAFE_DRIVE_H_
#define INC_SAFE_DRIVE_H_

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include "car.h"

/* =========================================================
 * public API
 * ========================================================= */

/* 즉시 정지
 * ---------------------------------------------------------
 * 사용 예:
 *   - 사용자가 강제 정지(P) 명령을 내렸을 때
 *   - 시스템 초기화 시 정지 상태를 강제로 만들고 싶을 때
 */
void SafeDrive_Stop(void);

/* 기존 speed_state_t 기반 주행 요청
 * ---------------------------------------------------------
 * 사용 예:
 *   SafeDrive_Move(CAR_FRONT, SPD_80);
 *   SafeDrive_Move(CAR_LEFT,  SPD_40);
 */
void SafeDrive_Move(uint8_t dir, speed_state_t req_speed);

/* % 기반 주행 요청
 * ---------------------------------------------------------
 * 내부적으로 5% 단위 speed_state_t 로 변환한다.
 * 사용 예:
 *   SafeDrive_MovePct(CAR_FRONT, 73); -> 70%로 내림 변환
 */
void SafeDrive_MovePct(uint8_t dir, uint8_t req_pct);

/* BMS 제한 재적용
 * ---------------------------------------------------------
 * App_Bms_Task()에서 BMS_SAFETY_Task() 직후 매 루프 호출.
 * applied_limit_pct가 바뀌었을 때 이미 주행 중인 모터 속도에
 * 즉시 새 제한을 적용한다.
 * 정지 상태(last_dir == CAR_STOP)이면 아무것도 하지 않는다.
 */
void SafeDrive_ReapplyLimit(void);


#endif /* INC_SAFE_DRIVE_H_ */
