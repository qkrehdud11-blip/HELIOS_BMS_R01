/*
 * app_bms.h
 *
 *  Created on: 2026. 3. 23.
 *
 *  [Main role]
 *  ---------------------------------------------------------
 *  BMS 상위 통합 app
 *
 *  역할
 *    - sensor / safety / statemachine / log 총괄
 *    - 리모컨 토글 명령 상태 관리
 *    - force stop / re-init 토글 관리
 *    - tracking / charging 토글 관리
 *    - CAN slave 제어 hook 제공
 *
 *  버튼 의미
 *  ---------------------------------------------------------
 *    A : Manual / Auto 토글
 *    P : 강제정지 / 초기화 토글
 *    D : 태양 추적 ON / OFF 토글
 *    K : 태양 충전 ON / OFF 토글
 *
 *  force stop 정책
 *  ---------------------------------------------------------
 *    - safety sensing 만 계속 동작
 *    - 주행 / 추적 / 충전 / 일반 동작 정지
 *    - A / D / K 입력 무시
 *    - 다시 P 누르면 재초기화 후 동작 복귀
 */

#ifndef INC_APP_BMS_H_
#define INC_APP_BMS_H_

#include "stm32f4xx_hal.h"
#include <stdint.h>

typedef enum
{
    APP_BMS_MODE_MANUAL = 0,
    APP_BMS_MODE_AUTO
} AppBmsMode_t;

/* =========================================================
 * CAN hook 함수 포인터
 * ---------------------------------------------------------
 * return
 *   0 : 송신 요청 성공
 *   1 : 송신 요청 실패
 * ========================================================= */
typedef uint8_t (*AppBmsCanTrackHook_t)(uint8_t on);
typedef uint8_t (*AppBmsCanChargeHook_t)(uint8_t on);

void App_Bms_Init(void);
void App_Bms_Task(void);

/* UART1 등에서 리모컨 문자 1개를 확정 수신했을 때 호출 */
void App_Bms_NotifyRemoteCmd(uint8_t cmd);

/* =========================================================
 * CAN hook 등록
 * ========================================================= */
void App_Bms_RegisterCanHooks(AppBmsCanTrackHook_t track_hook,
                              AppBmsCanChargeHook_t charge_hook);

/* danger latch만 해제 */
void App_Bms_ResetLatch(void);

/* safe_drive 계층에서 사용할 속도 제한 API */
uint8_t App_Bms_ApplySpeedLimit(uint8_t req_pct);

/* getter */
uint8_t App_Bms_IsInitDone(void);

AppBmsMode_t App_Bms_GetMode(void);
uint8_t App_Bms_IsManualMode(void);
uint8_t App_Bms_IsAutoMode(void);

uint8_t App_Bms_IsForceStopLocked(void);

uint8_t App_Bms_IsTrackOn(void);
uint8_t App_Bms_IsChargeOn(void);

uint8_t App_Bms_GetLastRemoteCmd(void);

uint8_t App_Bms_IsStopLatched(void);
uint8_t App_Bms_GetWarningCount(void);
uint8_t App_Bms_IsDangerNow(void);

uint8_t App_Bms_GetTargetLimitPct(void);
uint8_t App_Bms_GetAppliedLimitPct(void);


#endif /* INC_APP_BMS_H_ */
