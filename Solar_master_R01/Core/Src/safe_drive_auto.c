




/*
 * safe_drive_auto.c
 *
 *  Created on: 2026. 3. 23.
 *
 *  [Main role]
 *  ---------------------------------------------------------
 *  초음파 기반 auto 주행 로직
 *
 *  핵심 동작
 *  ---------------------------------------------------------
 *  1) 초음파 3개 거리 측정
 *  2) 정면 충돌 위험이면 후진
 *  3) 회피/보정 필요 시 즉시 단위 피벗 반복 (Incremental Pivot)
 *  4) 피벗 후 재측정 → 충분히 보정됐으면 FORWARD_HOLD 진입
 *  5) FORWARD_HOLD: centering 무시, 충돌 위험만 체크 후 SCAN 복귀
 *
 *  회전 판단 우선순위 (decide_pivot)
 *  ---------------------------------------------------------
 *  1) 한쪽 벽만 위험  → 반대 방향 피벗
 *  2) 양쪽 다 위험    → 더 가까운 쪽 기준으로 반대 방향
 *  3) 그 외           → diff 기반 중앙 보정
 *
 *  튜닝 파라미터
 *  ---------------------------------------------------------
 *  BLOCK_DISTANCE_FRONT : 정면 회피 개시 거리 (cm)
 *  BLOCK_DISTANCE_SIDE  : 측면 위험 거리 (cm) - 45도 센서 보정 고려
 *  PIVOT_TIME           : 단위 피벗 지속 시간 (ms)
 *  FORWARD_HOLD_TIME    : 피벗 후 직진 유지 시간 (ms)
 *  CENTER_DIFF          : 중앙 보정 발동 좌우 차이 (cm)
 *
 *  주의
 *  ---------------------------------------------------------
 *  - 실제 주행 출력은 SafeDrive_Move() / SafeDrive_Stop()로 수행한다.
 *  - 따라서 safety 감속 / 정지 정책이 자동주행에도 동일하게 반영된다.
 */

#include "safe_drive_auto.h"

#include "safe_drive.h"
#include "ultrasonic.h"
#include "direction.h"
#include "speed.h"
#include "car.h"

/* =========================================================
 * 기준값 (거리: cm, 시간: ms)
 * ========================================================= */
#define BLOCK_DISTANCE_FRONT    30U   /* 정면 회피 개시 거리 */
#define BLOCK_DISTANCE_SIDE     10U   /* 측면 위험 거리 (45도 센서 → 28U 도 고려) */
#define CRASH_DISTANCE          10U   /* 긴급 후진 거리 */
#define CENTER_DIFF             20U    /* 중앙 보정 발동 좌우 차이 (cm) */

#define PIVOT_TIME            150U   /* 단위 피벗 지속 시간 (ms) */
#define FORWARD_HOLD_TIME      100U   /* 피벗 후 직진 유지 시간 (ms) */

/* =========================================================
 * 내부 상태
 * ========================================================= */
static AUTO_STATE  auto_st   = AUTO_STATE_STOP;
static uint32_t    auto_tick = 0U;
static car_state_t car_dir = CAR_PIVOT_LEFT;

/* =========================================================
 * 피벗 방향 결정
 *
 * 우선순위:
 *   1) 한쪽만 위험  → 반대 방향 피벗
 *   2) 양쪽 다 위험 → 더 가까운 쪽 기준
 *   3) 일반         → diff 기반
 * ========================================================= */
static car_state_t DECIDE_TURN(uint8_t l, uint8_t r)
{
    uint8_t l_danger = (l < BLOCK_DISTANCE_SIDE);
    uint8_t r_danger = (r < BLOCK_DISTANCE_SIDE);

    if (l_danger && !r_danger)
    {
        return CAR_PIVOT_RIGHT;
    }
    if (r_danger && !l_danger)
    {
        return CAR_PIVOT_LEFT;
    }
    if (l_danger && r_danger)
    {
        /* 양쪽 다 좁음: 더 가까운 쪽 기준 */
        return (l <= r) ? CAR_PIVOT_RIGHT : CAR_PIVOT_LEFT;
    }

    /* 여유 있는 쪽(더 먼 쪽)으로 회전 */
    return (r >= l) ? CAR_PIVOT_RIGHT : CAR_PIVOT_LEFT;
}

/* =========================================================
 * 보정/회피 필요 여부 판단
 * ========================================================= */
static uint8_t needs_correction(uint8_t l, uint8_t r, uint8_t c)
{
    /* 위험 상황만 보정 트리거 (중앙 정렬 보정 제거)
     * CENTER_DIFF 조건 제거: 좌우 비대칭만으로 피벗하면 오버슈팅 반복됨 */
    if (c < BLOCK_DISTANCE_FRONT)  return 1U;
    if (l < BLOCK_DISTANCE_SIDE)   return 1U;
    if (r < BLOCK_DISTANCE_SIDE)   return 1U;

    return 0U;
}

/* =========================================================
 * 초기화
 * ========================================================= */
void SafeDriveAuto_Init(void)
{
    auto_st   = AUTO_STATE_SCAN;
    auto_tick = HAL_GetTick();
}

/* =========================================================
 * auto task
 * ========================================================= */
void SafeDriveAuto_Task(void)
{
    uint32_t now;
    uint8_t l_raw;
    uint8_t c_raw;
    uint8_t r_raw;
    uint8_t l;
    uint8_t c;
    uint8_t r;

    Ultrasonic_TriggerAll();
    now = HAL_GetTick();

    l_raw = Ultrasonic_GetDistanceCm(US_LEFT);
    c_raw = Ultrasonic_GetDistanceCm(US_CENTER);
    r_raw = Ultrasonic_GetDistanceCm(US_RIGHT);

    /* 0 또는 100 초과 값은 100으로 정규화 */
    l = (l_raw == 0U || l_raw > 100U) ? 100U : l_raw;
    c = (c_raw == 0U || c_raw > 100U) ? 100U : c_raw;
    r = (r_raw == 0U || r_raw > 100U) ? 100U : r_raw;

    switch (auto_st)
    {
        /* ------------------------------------------------- */
        case AUTO_STATE_SCAN:
        /* ------------------------------------------------- */
            if (c < CRASH_DISTANCE)
            {
                /* 긴급 후진 */
                SafeDrive_Move(CAR_BACK, SPD_70);
                auto_tick = now;
                auto_st   = AUTO_STATE_BACK;
            }
            else if (needs_correction(l, r, c))
            {
                /* 즉시 피벗 (무거운 차 관성으로 자연 감속) */
                car_dir = DECIDE_TURN(l, r);
                SafeDrive_Move(car_dir, SPD_100);
                auto_tick = now;
                auto_st   = AUTO_STATE_PIVOT;
            }
            else
            {
                SafeDrive_Move(CAR_FRONT, SPD_65);
            }
            break;

        /* ------------------------------------------------- */
        case AUTO_STATE_PIVOT:
        /* ------------------------------------------------- */
            /* 단위 피벗 완료 후 재측정 */
            if ((now - auto_tick) >= PIVOT_TIME)
            {
                if (needs_correction(l, r, c))
                {
                    /* 방향 재판단 후 피벗 반복 */
                    car_dir = DECIDE_TURN(l, r);
                    SafeDrive_Move(car_dir, SPD_100);
                    auto_tick = now;
                }
                else
                {
                    /* 보정 완료 → 직진 유지 구간 진입 */
                    SafeDrive_Move(CAR_FRONT, SPD_65);
                    auto_tick = now;
                    auto_st   = AUTO_STATE_FORWARD_HOLD;
                }
            }
            break;

        /* ------------------------------------------------- */
        case AUTO_STATE_FORWARD_HOLD:
        /* ------------------------------------------------- */
            /* centering 무시, 충돌 위험만 체크 */
            if (c < CRASH_DISTANCE)
            {
                SafeDrive_Move(CAR_BACK, SPD_50);
                auto_tick = now;
                auto_st   = AUTO_STATE_BACK;
            }
            else if ((c < BLOCK_DISTANCE_FRONT) ||
                     (l < BLOCK_DISTANCE_SIDE) ||
                     (r < BLOCK_DISTANCE_SIDE))
            {
                car_dir = DECIDE_TURN(l, r);
                SafeDrive_Move(car_dir, SPD_100);
                auto_tick = now;
                auto_st   = AUTO_STATE_PIVOT;
            }
            else if ((now - auto_tick) >= FORWARD_HOLD_TIME)
            {
                auto_st = AUTO_STATE_SCAN;
            }
            break;

        /* ------------------------------------------------- */
        case AUTO_STATE_BACK:
        /* ------------------------------------------------- */
            if ((now - auto_tick) >= 250)
            {
                SafeDrive_Stop();
                auto_st = AUTO_STATE_SCAN;
            }
            break;

        default:
            auto_st = AUTO_STATE_SCAN;
            break;
    }
}

