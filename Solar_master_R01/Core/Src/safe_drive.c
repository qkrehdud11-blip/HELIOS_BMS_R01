/*
 * safe_drive.c
 *
 *  Created on: 2026. 3. 23.
 *
 *  [Main role]
 *  ---------------------------------------------------------
 *  BMS 안전 정책을 실제 주행 속도에 반영하는 wrapper
 *
 *  핵심 동작
 *  ---------------------------------------------------------
 *  1) statemachine / auto / manual 이 요청한 속도를 받는다.
 *  2) App_Bms_ApplySpeedLimit() 로 현재 안전 제한을 적용한다.
 *  3) 최종 속도를 5% 단위 speed_state_t 로 변환한다.
 *  4) 최종 속도가 0%면 Car_Stop()
 *  5) 아니면 Car_Move(dir, speed_state_t)
 *
 *  중요
 *  ---------------------------------------------------------
 *  - danger latch가 떠도 바로 Car_Stop() 하지 않는다.
 *  - bms_safety.c 에서 applied_limit_pct가
 *      warning : 200ms마다 5%
 *      danger  : 100ms마다 5%
 *      recover : 300ms마다 5%
 *    로 천천히 변하므로,
 *    여기서는 그 결과를 그대로 주행에 반영해야 한다.
 */

#include "safe_drive.h"

#include "app_bms.h"
#include "car.h"

/* =========================================================
 * 내부 디버그용 마지막 값
 * ========================================================= */
static uint8_t s_last_req_pct = 0U;
static uint8_t s_last_dir = (uint8_t)CAR_STOP;

/* =========================================================
 * 내부 helper
 * ========================================================= */

/* speed_state_t -> % 변환
 * ---------------------------------------------------------
 * SPD_STOP = 0%
 * SPD_5    = 5%
 * ...
 * SPD_100  = 100%
 */
static uint8_t SafeDrive_SpeedToPct(speed_state_t speed)
{
    if ((uint8_t)speed > (uint8_t)SPD_100)
    {
        return 100U;
    }

    return ((uint8_t)speed * 5U);
}

/* % -> 5% 단위로 내림 정규화
 * ---------------------------------------------------------
 * 예:
 *   73 -> 70
 *   61 -> 60
 *   4  -> 0
 */
static uint8_t SafeDrive_NormalizePct(uint8_t pct)
{
    if (pct > 100U)
    {
        pct = 100U;
    }

    return (uint8_t)((pct / 5U) * 5U);
}

/* % -> speed_state_t 변환
 * ---------------------------------------------------------
 * 5% 단위 enum 이므로 정규화 후 나누면 된다.
 */
static speed_state_t SafeDrive_PctToSpeed(uint8_t pct)
{
    pct = SafeDrive_NormalizePct(pct);
    return (speed_state_t)(pct / 5U);
}

/* =========================================================
 * SafeDrive_Stop
 * ---------------------------------------------------------
 * 사용자 강제 정지나 모드 전환 시 즉시 정지용
 * ========================================================= */
void SafeDrive_Stop(void)
{
    s_last_req_pct = 0U;
    s_last_dir = (uint8_t)CAR_STOP;

    Car_Stop();
}

/* =========================================================
 * SafeDrive_Move
 * ---------------------------------------------------------
 * 기존 speed_state_t 기반 요청을 받아
 * BMS 정책 적용 후 실제 Car_Move() 호출
 * ========================================================= */
void SafeDrive_Move(uint8_t dir, speed_state_t req_speed)
{
    uint8_t req_pct;
    uint8_t final_pct;
    speed_state_t final_speed;

    /* 요청 speed enum -> % 변환 */
    req_pct = SafeDrive_SpeedToPct(req_speed);
    s_last_req_pct = req_pct;
    s_last_dir = dir;

    /* 현재 BMS 정책 적용
     * -----------------------------------------------------
     * 여기서는 danger latch 여부를 보고 즉시 stop하지 않는다.
     * App_Bms_ApplySpeedLimit()가 현재 applied_limit_pct를 반환하고,
     * 그 값은 bms_safety에서 ramp로 천천히 감소한다.
     */
    final_pct = App_Bms_ApplySpeedLimit(req_pct);

    /* speed enum 구조에 맞게 5% 단위 정규화 */
    final_pct = SafeDrive_NormalizePct(final_pct);
    final_speed = SafeDrive_PctToSpeed(final_pct);

    /* 최종 적용 속도가 0이면 정지 */
    if (final_speed == SPD_STOP)
    {
        Car_Stop();
        return;
    }

    /* 실제 주행 */
    Car_Move(dir, final_speed);
}

/* =========================================================
 * SafeDrive_MovePct
 * ---------------------------------------------------------
 * % 기반 요청을 받아 SafeDrive_Move()와 동일 동작 수행
 * ========================================================= */
void SafeDrive_MovePct(uint8_t dir, uint8_t req_pct)
{
    speed_state_t req_speed;

    req_pct = SafeDrive_NormalizePct(req_pct);
    req_speed = SafeDrive_PctToSpeed(req_pct);

    SafeDrive_Move(dir, req_speed);
}

/* =========================================================
 * SafeDrive_ReapplyLimit
 * ---------------------------------------------------------
 * BMS applied_limit_pct가 바뀌었을 때 이미 주행 중인 모터에
 * 즉시 새 제한 속도를 적용한다.
 *
 * App_Bms_Task()에서 BMS_SAFETY_Task() 직후 매 루프 호출.
 *
 * 정지 상태(CAR_STOP)이면 아무것도 하지 않는다.
 * ========================================================= */
void SafeDrive_ReapplyLimit(void)
{
    if (s_last_dir == (uint8_t)CAR_STOP)
    {
        return;
    }

    SafeDrive_MovePct(s_last_dir, s_last_req_pct);
}

