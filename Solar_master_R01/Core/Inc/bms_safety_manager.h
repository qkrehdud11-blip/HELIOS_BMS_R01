




#ifndef INC_BMS_SAFETY_H_
#define INC_BMS_SAFETY_H_




#include "stm32f4xx_hal.h"
#include <stdint.h>

/* =========================================================
 * 초기화 / 주기 task
 * ========================================================= */
void BMS_SAFETY_Init(void);
void BMS_SAFETY_Task(void);

/* =========================================================
 * danger latch 제어
 * ---------------------------------------------------------
 * latch만 지운다.
 * 현재 danger 상태가 남아 있으면 다음 task에서 다시 latch될 수 있다.
 * ========================================================= */
void BMS_SAFETY_ResetLatch(void);

/* =========================================================
 * 속도 제한 적용
 * ---------------------------------------------------------
 * req_pct :
 *   상위 주행 코드가 요청한 속도(%)
 *
 * 반환 :
 *   현재 safety 제한이 적용된 최종 허용 속도(%)
 * ========================================================= */
uint8_t BMS_SAFETY_ApplyLimit(uint8_t req_pct);

/* =========================================================
 * 현재 상태 조회
 * ========================================================= */
uint8_t BMS_SAFETY_GetWarningCount(void);
uint8_t BMS_SAFETY_IsDangerNow(void);
uint8_t BMS_SAFETY_IsDangerLatched(void);

uint8_t BMS_SAFETY_GetElectricalWarning(void);
uint8_t BMS_SAFETY_GetElectricalDanger(void);

uint8_t BMS_SAFETY_GetTargetLimitPct(void);
uint8_t BMS_SAFETY_GetAppliedLimitPct(void);

/* =========================================================
 * 원인 / 배너 조회
 * ---------------------------------------------------------
 * GetBanner():
 *   기존 코드 호환용 요약 문자열
 *
 * GetNowReason():
 *   현재 active 상태의 대표 원인
 *
 * GetLatchedReason():
 *   danger latch를 만든 최초 원인
 * ========================================================= */
const char *BMS_SAFETY_GetBanner(void);
const char *BMS_SAFETY_GetNowReason(void);
const char *BMS_SAFETY_GetLatchedReason(void);





#endif /* INC_BMS_SAFETY_H_ */
