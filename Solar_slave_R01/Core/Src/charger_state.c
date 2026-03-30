/*
 * charger_state.c
 *
 *  Created on: 2026. 3. 20.
 *
 *  설명:
 *  ---------------------------------------------------------
 *  태양전지 충전기 상위 상태머신 구현
 *
 *  이번 버전의 핵심 안정화 포인트
 *  ---------------------------------------------------------
 *  1) update_done 직접 접근 제거
 *     -> SolarSensing_FetchUpdateDone() 사용
 *
 *  2) snapshot은 새 샘플이 들어왔을 때만 갱신
 *     -> HAL_BUSY 때문에 false COMM fault 나는 문제 방지
 *
 *  3) COMM fault는 sample age 기반으로 판정
 *     -> "아직 DMA 진행 중" 상태를 fault로 보지 않음
 *
 *  4) range / comm soft fault debounce 유지
 *     -> false BATT_RANGE / false FAULT 완화
 *
 *  5) INIT prime timeout 추가
 *     -> 센서 초기 준비가 지나치게 오래 걸릴 때 FAULT로 전이
 */

#include "charger_state.h"
#include <string.h>
#include <math.h>

/* =========================================================
 * 내부 유틸 함수
 * ========================================================= */

/* float clamp */
static float ChargerState_Clamp(float x, float x_min, float x_max)
{
    if (x < x_min)
    {
        return x_min;
    }

    if (x > x_max)
    {
        return x_max;
    }

    return x;
}

/* uint32_t saturating add */
static uint32_t ChargerState_AddSaturatedU32(uint32_t value, uint32_t add)
{
    if (value > (0xFFFFFFFFUL - add))
    {
        return 0xFFFFFFFFUL;
    }

    return value + add;
}

/* SOC 계산
 * ---------------------------------------------------------
 * CONTROL_SOC_MIN_V ~ CONTROL_SOC_MAX_V 사이를
 * 0~100%로 선형 매핑한다.
 */
static float ChargerState_CalcSocPercent(float batt_v)
{
    float soc;

    soc = (batt_v - CONTROL_SOC_MIN_V) /
          (CONTROL_SOC_MAX_V - CONTROL_SOC_MIN_V);

    soc = ChargerState_Clamp(soc, 0.0f, 1.0f);
    return soc * 100.0f;
}

/* 효율 계산
 * ---------------------------------------------------------
 * 입력 전력이 너무 작을 때는 0으로 처리해
 * 불필요한 튐을 줄인다.
 */
static float ChargerState_CalcEfficiency(float pin_w, float pout_w)
{
    float eff;

    if (pin_w <= 0.05f)
    {
        return 0.0f;
    }

    eff = (pout_w / pin_w) * 100.0f;
    eff = ChargerState_Clamp(eff, 0.0f, 100.0f);

    return eff;
}

/* 상태 전이 */
static void ChargerState_Change(ChargerState_t *sm, ChargerStateId_t next_state)
{
    if (sm == NULL)
    {
        return;
    }

    if (sm->state != next_state)
    {
        sm->prev_state = sm->state;
        sm->state = next_state;
        sm->state_timer_ms = 0U;
    }
}

/* 제어기 강제 정지
 * ---------------------------------------------------------
 * CHARGING 상태가 아닐 때는 제어기를 확실히 STOP 상태로 두고,
 * 로그용 last 값도 STOP 의미로 정리한다.
 */
static void ChargerState_ForceStopControl(ChargerState_t *sm)
{
    if ((sm == NULL) || (sm->ctrl == NULL))
    {
        return;
    }

    SolarPiControl_Reset(sm->ctrl);

    sm->control_mode_last = SOLAR_CTRL_MODE_STOP;
    sm->duty_last = 0.0f;
    sm->i_ref_last = 0.0f;
    sm->i_cc_last = CONTROL_I_CC_MAX;
    sm->i_cv_last = CONTROL_I_CC_MAX;
    sm->i_mppt_last = CONTROL_MPPT_I_INIT;
}

/* =========================================================
 * 완료된 DMA 샘플 회수
 * ---------------------------------------------------------
 * 중요:
 * - sensor->dma.update_done 직접 접근하지 않는다.
 * - SolarSensing_FetchUpdateDone()로 안전하게 회수한다.
 * - 새 샘플이 없으면 last 값은 유지하고 age만 증가시킨다.
 * ========================================================= */
static void ChargerState_HarvestCompletedSamples(ChargerState_t *sm)
{
    uint8_t solar_done;
    uint8_t batt_done;

    if ((sm == NULL) || (sm->solar_sensor == NULL) || (sm->batt_sensor == NULL))
    {
        return;
    }

    /* 이번 주기 fresh flag 초기화 */
    sm->solar_sample_fresh = 0U;
    sm->batt_sample_fresh = 0U;

    solar_done = SolarSensing_FetchUpdateDone(sm->solar_sensor);
    batt_done  = SolarSensing_FetchUpdateDone(sm->batt_sensor);

    if (solar_done != 0U)
    {
        sm->solar_sample_ready = 1U;
        sm->solar_sample_fresh = 1U;
        sm->solar_sample_age_ms = 0U;
    }
    else
    {
        sm->solar_sample_age_ms =
            ChargerState_AddSaturatedU32(sm->solar_sample_age_ms,
                                         CHARGER_STATE_RUN_PERIOD_MS);
    }

    if (batt_done != 0U)
    {
        /* 배터리 센서는 완료 직후 필터를 적용해 out 값을 안정화한다. */
        SolarSensing_BatteryFilterUpdate(sm->batt_sensor);

        sm->batt_sample_ready = 1U;
        sm->batt_sample_fresh = 1U;
        sm->batt_sample_age_ms = 0U;
    }
    else
    {
        sm->batt_sample_age_ms =
            ChargerState_AddSaturatedU32(sm->batt_sample_age_ms,
                                         CHARGER_STATE_RUN_PERIOD_MS);
    }

    if ((sm->solar_sample_ready != 0U) &&
        (sm->batt_sample_ready != 0U))
    {
        sm->sensors_primed = 1U;
    }
}

/* =========================================================
 * 새 샘플이 들어온 센서만 snapshot 갱신
 * ---------------------------------------------------------
 * 중요:
 * - 매 주기 무조건 GetSnapshot()으로 last 값을 덮어쓰지 않는다.
 * - fresh sample이 있을 때만 last 값을 갱신한다.
 * - 이렇게 해야 HAL_BUSY 시점 때문에 false COMM 판단이 나지 않는다.
 * ========================================================= */
static void ChargerState_UpdateSnapshotsFromFreshSamples(ChargerState_t *sm)
{
    SolarSensingSnapshot_t snap;

    if (sm == NULL)
    {
        return;
    }

    /* -----------------------------------------------------
     * solar fresh sample 반영
     * ----------------------------------------------------- */
    if ((sm->solar_sample_fresh != 0U) && (sm->solar_sensor != NULL))
    {
        SolarSensing_GetSnapshot(sm->solar_sensor, &snap);

        sm->solar_range_ok   = snap.range_valid;
        sm->solar_bus_v_last = snap.bus_v;
        sm->solar_src_v_last = snap.source_v;
        sm->solar_i_last     = snap.current_ma * 0.001f;

        /* 태양전지 입력 전류는 음수 노이즈를 0으로 clamp */
        if (sm->solar_i_last < 0.0f)
        {
            sm->solar_i_last = 0.0f;
        }
    }

    /* -----------------------------------------------------
     * battery fresh sample 반영
     * ----------------------------------------------------- */
    if ((sm->batt_sample_fresh != 0U) && (sm->batt_sensor != NULL))
    {
        SolarSensing_GetSnapshot(sm->batt_sensor, &snap);

        sm->batt_range_ok   = snap.range_valid;
        sm->batt_bus_v_last = snap.bus_v;
        sm->batt_v_last     = snap.bus_v + CONTROL_BATT_V_OFFSET;
        sm->batt_i_last     = snap.current_ma * 0.001f;

        /* 충전 전류는 음수 노이즈를 0으로 clamp */
        if (sm->batt_i_last < 0.0f)
        {
            sm->batt_i_last = 0.0f;
        }
    }

    /* -----------------------------------------------------
     * 파생값은 최신 last 값을 기준으로 항상 다시 계산
     * ----------------------------------------------------- */
    sm->solar_p_last = sm->solar_src_v_last * sm->solar_i_last;
    sm->batt_p_last  = sm->batt_v_last * sm->batt_i_last;
    sm->eff_last     = ChargerState_CalcEfficiency(sm->solar_p_last, sm->batt_p_last);
    sm->soc_last     = ChargerState_CalcSocPercent(sm->batt_v_last);
}

/* =========================================================
 * COMM / RANGE 상태 갱신
 * ---------------------------------------------------------
 * COMM fault는 last_status가 아니라 sample age로 본다.
 *
 * 이유:
 *   SolarSensing_StartUpdateDMA() 시작 시점에 last_status가 HAL_BUSY가 되므로
 *   이를 바로 통신 실패로 보면 false COMM fault가 난다.
 * ========================================================= */
static void ChargerState_UpdateSensorHealth(ChargerState_t *sm)
{
    if (sm == NULL)
    {
        return;
    }

    /* 새 샘플을 최소 1회 이상 받았고,
     * 마지막 수신 샘플이 너무 오래되지 않았으면 COMM OK
     */
    sm->solar_comm_ok =
        (uint8_t)((sm->solar_sample_ready != 0U) &&
                  (sm->solar_sample_age_ms <= CHARGER_COMM_STALE_TIMEOUT_MS));

    sm->batt_comm_ok =
        (uint8_t)((sm->batt_sample_ready != 0U) &&
                  (sm->batt_sample_age_ms <= CHARGER_COMM_STALE_TIMEOUT_MS));

    /* range_ok는 latest valid sample 기준 상태를 유지한다.
     * fresh sample이 없을 때는 이전 상태를 유지한다.
     */
}

/* =========================================================
 * 단일 fault signal debounce
 * ========================================================= */
static void ChargerState_UpdateFaultLatch(uint8_t signal_ok,
                                          uint8_t assert_count,
                                          uint8_t clear_count,
                                          uint8_t *fault_active,
                                          uint8_t *bad_count,
                                          uint8_t *good_count)
{
    if ((fault_active == NULL) || (bad_count == NULL) || (good_count == NULL))
    {
        return;
    }

    if (signal_ok == 0U)
    {
        if (*bad_count < 255U)
        {
            (*bad_count)++;
        }

        *good_count = 0U;

        if (*bad_count >= assert_count)
        {
            *fault_active = 1U;
        }
    }
    else
    {
        if (*good_count < 255U)
        {
            (*good_count)++;
        }

        *bad_count = 0U;

        if (*good_count >= clear_count)
        {
            *fault_active = 0U;
        }
    }
}

/* =========================================================
 * soft fault debounce 업데이트
 * ========================================================= */
static void ChargerState_UpdateDebouncedSensorFaults(ChargerState_t *sm)
{
    if (sm == NULL)
    {
        return;
    }

    ChargerState_UpdateFaultLatch(sm->solar_comm_ok,
                                  CHARGER_COMM_FAULT_ASSERT_COUNT,
                                  CHARGER_COMM_FAULT_CLEAR_COUNT,
                                  &sm->solar_comm_fault_active,
                                  &sm->solar_comm_bad_count,
                                  &sm->solar_comm_good_count);

    ChargerState_UpdateFaultLatch(sm->batt_comm_ok,
                                  CHARGER_COMM_FAULT_ASSERT_COUNT,
                                  CHARGER_COMM_FAULT_CLEAR_COUNT,
                                  &sm->batt_comm_fault_active,
                                  &sm->batt_comm_bad_count,
                                  &sm->batt_comm_good_count);

    ChargerState_UpdateFaultLatch(sm->solar_range_ok,
                                  CHARGER_RANGE_FAULT_ASSERT_COUNT,
                                  CHARGER_RANGE_FAULT_CLEAR_COUNT,
                                  &sm->solar_range_fault_active,
                                  &sm->solar_range_bad_count,
                                  &sm->solar_range_good_count);

    ChargerState_UpdateFaultLatch(sm->batt_range_ok,
                                  CHARGER_RANGE_FAULT_ASSERT_COUNT,
                                  CHARGER_RANGE_FAULT_CLEAR_COUNT,
                                  &sm->batt_range_fault_active,
                                  &sm->batt_range_bad_count,
                                  &sm->batt_range_good_count);
}

/* =========================================================
 * 입력 상태 평가
 * ---------------------------------------------------------
 * 여기서는 raw 순간값이 아니라
 * debounce된 active fault + 마지막 유효 샘플 값을 이용한다.
 * ========================================================= */
static void ChargerState_EvalInputs(ChargerState_t *sm)
{
    if (sm == NULL)
    {
        return;
    }

    sm->solar_ok =
        (sm->solar_comm_fault_active == 0U) &&
        (sm->solar_range_fault_active == 0U) &&
        (sm->solar_src_v_last >= CHARGER_MIN_SOLAR_SRC_V);

    sm->batt_ok =
        (sm->batt_comm_fault_active == 0U) &&
        (sm->batt_range_fault_active == 0U) &&
        (sm->batt_v_last >= CHARGER_BATT_VALID_MIN_V) &&
        (sm->batt_v_last <= CHARGER_BATT_VALID_MAX_V);

    sm->source_ready = 0U;

    if ((sm->solar_ok != 0U) &&
        (sm->batt_ok != 0U) &&
        (sm->solar_bus_v_last > (sm->batt_v_last + CONTROL_SOLAR_MIN_BUCK_MARGIN_V)))
    {
        sm->source_ready = 1U;
    }
}

/* =========================================================
 * fault 평가
 * ---------------------------------------------------------
 * soft fault:
 *   debounce된 active 상태 사용
 *
 * hard fault:
 *   stale한 마지막 값으로 오판하지 않도록
 *   해당 센서가 comm/range 모두 정상일 때만 검사
 * ========================================================= */
static uint32_t ChargerState_EvalFaults(const ChargerState_t *sm)
{
    uint32_t fault = CHARGER_FAULT_NONE;
    uint8_t batt_sensor_valid_for_hard_fault;
    uint8_t solar_sensor_valid_for_hard_fault;

    if (sm == NULL)
    {
        return (CHARGER_FAULT_SOLAR_COMM | CHARGER_FAULT_BATT_COMM);
    }

    if (sm->solar_comm_fault_active != 0U)
    {
        fault |= CHARGER_FAULT_SOLAR_COMM;
    }

    if (sm->batt_comm_fault_active != 0U)
    {
        fault |= CHARGER_FAULT_BATT_COMM;
    }

    if (sm->solar_range_fault_active != 0U)
    {
        fault |= CHARGER_FAULT_SOLAR_RANGE;
    }

    if (sm->batt_range_fault_active != 0U)
    {
        fault |= CHARGER_FAULT_BATT_RANGE;
    }

    batt_sensor_valid_for_hard_fault =
        (uint8_t)((sm->batt_comm_fault_active == 0U) &&
                  (sm->batt_range_fault_active == 0U));

    solar_sensor_valid_for_hard_fault =
        (uint8_t)((sm->solar_comm_fault_active == 0U) &&
                  (sm->solar_range_fault_active == 0U));

    if (batt_sensor_valid_for_hard_fault != 0U)
    {
        if (sm->batt_v_last > CHARGER_FAULT_BATT_OVERVOLT_V)
        {
            fault |= CHARGER_FAULT_BATT_OVERVOLT;
        }

        if (sm->batt_v_last < CHARGER_FAULT_BATT_UNDERVOLT_V)
        {
            fault |= CHARGER_FAULT_BATT_UNDERVOLT;
        }

        if (fabsf(sm->batt_i_last) > CHARGER_FAULT_BATT_ABS_CURRENT_A)
        {
            fault |= CHARGER_FAULT_BATT_OVERCURRENT;
        }
    }

    if (solar_sensor_valid_for_hard_fault != 0U)
    {
        if (fabsf(sm->solar_i_last) > CHARGER_FAULT_SOLAR_ABS_CURRENT_A)
        {
            fault |= CHARGER_FAULT_SOLAR_OVERCURRENT;
        }
    }

    return fault;
}

/* =========================================================
 * DONE 안정 판정
 * ========================================================= */
static uint8_t ChargerState_UpdateDoneDetect(ChargerState_t *sm, uint8_t done_condition)
{
    if (sm == NULL)
    {
        return 0U;
    }

    if (done_condition != 0U)
    {
        if (sm->done_detect_count < 255U)
        {
            sm->done_detect_count++;
        }
    }
    else
    {
        sm->done_detect_count = 0U;
    }

    return (uint8_t)(sm->done_detect_count >= CHARGER_DONE_DETECT_COUNT);
}

/* =========================================================
 * 다음 주기에 사용할 새 DMA 센싱 시작
 * ---------------------------------------------------------
 * 논블로킹 구조 유지:
 *   - busy면 HAL_BUSY가 나와도 그냥 다음 주기에 다시 시도
 *   - timeout/stuck 감시는 Service()에서 수행
 * ========================================================= */
static void ChargerState_StartNextSenseCycle(ChargerState_t *sm)
{
    if ((sm == NULL) || (sm->solar_sensor == NULL) || (sm->batt_sensor == NULL))
    {
        return;
    }

    SolarSensing_Service(sm->solar_sensor);
    SolarSensing_Service(sm->batt_sensor);

    (void)SolarSensing_StartUpdateDMA(sm->solar_sensor);
    (void)SolarSensing_StartUpdateDMA(sm->batt_sensor);
}

/* =========================================================
 * 문자열 함수
 * ========================================================= */
const char* ChargerState_StateString(ChargerStateId_t state)
{
    switch (state)
    {
    case CHARGER_STATE_INIT:        return "INIT";
    case CHARGER_STATE_WAIT_SOURCE: return "WAIT_SOURCE";
    case CHARGER_STATE_CHARGING:    return "CHARGING";
    case CHARGER_STATE_DONE:        return "DONE";
    case CHARGER_STATE_FAULT:       return "FAULT";
    default:                        return "UNKNOWN";
    }
}

const char* ChargerState_PrimaryFaultString(uint32_t fault_flags)
{
    if (fault_flags & CHARGER_FAULT_INIT_PRIME_TIMEOUT) return "INIT_TIMEOUT";
    if (fault_flags & CHARGER_FAULT_SOLAR_COMM)         return "SOLAR_COMM";
    if (fault_flags & CHARGER_FAULT_BATT_COMM)          return "BATT_COMM";
    if (fault_flags & CHARGER_FAULT_SOLAR_RANGE)        return "SOLAR_RANGE";
    if (fault_flags & CHARGER_FAULT_BATT_RANGE)         return "BATT_RANGE";
    if (fault_flags & CHARGER_FAULT_BATT_OVERVOLT)      return "BATT_OVERVOLT";
    if (fault_flags & CHARGER_FAULT_BATT_UNDERVOLT)     return "BATT_UNDERVOLT";
    if (fault_flags & CHARGER_FAULT_BATT_OVERCURRENT)   return "BATT_OVERCURRENT";
    if (fault_flags & CHARGER_FAULT_SOLAR_OVERCURRENT)  return "SOLAR_OVERCURRENT";
    return "NONE";
}

/* =========================================================
 * 초기화 / 리셋
 * ========================================================= */
void ChargerState_Reset(ChargerState_t *sm)
{
    if (sm == NULL)
    {
        return;
    }

    sm->state = CHARGER_STATE_INIT;
    sm->prev_state = CHARGER_STATE_INIT;

    sm->fault_flags = CHARGER_FAULT_NONE;

    sm->state_timer_ms = 0U;
    sm->fault_clear_timer_ms = 0U;

    sm->solar_sample_ready = 0U;
    sm->batt_sample_ready = 0U;
    sm->solar_sample_fresh = 0U;
    sm->batt_sample_fresh = 0U;
    sm->sensors_primed = 0U;

    sm->solar_sample_age_ms = 0U;
    sm->batt_sample_age_ms = 0U;

    sm->solar_comm_ok = 0U;
    sm->solar_range_ok = 0U;
    sm->batt_comm_ok = 0U;
    sm->batt_range_ok = 0U;

    sm->solar_comm_fault_active = 0U;
    sm->batt_comm_fault_active = 0U;
    sm->solar_range_fault_active = 0U;
    sm->batt_range_fault_active = 0U;

    sm->solar_comm_bad_count = 0U;
    sm->solar_comm_good_count = 0U;
    sm->batt_comm_bad_count = 0U;
    sm->batt_comm_good_count = 0U;

    sm->solar_range_bad_count = 0U;
    sm->solar_range_good_count = 0U;
    sm->batt_range_bad_count = 0U;
    sm->batt_range_good_count = 0U;

    sm->done_detect_count = 0U;

    sm->solar_ok = 0U;
    sm->batt_ok = 0U;
    sm->source_ready = 0U;

    sm->solar_src_v_last = 0.0f;
    sm->solar_bus_v_last = 0.0f;
    sm->solar_i_last = 0.0f;
    sm->solar_p_last = 0.0f;

    sm->batt_bus_v_last = 0.0f;
    sm->batt_v_last = 0.0f;
    sm->batt_i_last = 0.0f;
    sm->batt_p_last = 0.0f;

    sm->eff_last = 0.0f;
    sm->soc_last = 0.0f;

    sm->control_mode_last = SOLAR_CTRL_MODE_STOP;

    sm->duty_last = 0.0f;
    sm->i_ref_last = 0.0f;
    sm->i_cc_last = CONTROL_I_CC_MAX;
    sm->i_cv_last = CONTROL_I_CC_MAX;
    sm->i_mppt_last = CONTROL_MPPT_I_INIT;

    ChargerState_ForceStopControl(sm);
}

void ChargerState_Init(ChargerState_t *sm,
                       SolarSensing_t *solar_sensor,
                       SolarSensing_t *batt_sensor,
                       SolarPiControl_t *ctrl)
{
    if (sm == NULL)
    {
        return;
    }

    memset(sm, 0, sizeof(ChargerState_t));

    sm->solar_sensor = solar_sensor;
    sm->batt_sensor  = batt_sensor;
    sm->ctrl         = ctrl;

    ChargerState_Reset(sm);
}

/* =========================================================
 * 메인 상태머신
 * ========================================================= */
void ChargerState_Run(ChargerState_t *sm)
{
    uint32_t fault_now = CHARGER_FAULT_NONE;

    if ((sm == NULL) ||
        (sm->solar_sensor == NULL) ||
        (sm->batt_sensor == NULL) ||
        (sm->ctrl == NULL))
    {
        return;
    }

    /* 1) 이번 주기 완료된 샘플 회수 */
    ChargerState_HarvestCompletedSamples(sm);

    /* 2) 새 샘플이 들어온 센서만 last snapshot 갱신 */
    ChargerState_UpdateSnapshotsFromFreshSamples(sm);

    /* 3) sensors primed 이후에만 health / debounce / fault 평가 */
    if (sm->sensors_primed != 0U)
    {
        ChargerState_UpdateSensorHealth(sm);
        ChargerState_UpdateDebouncedSensorFaults(sm);
        ChargerState_EvalInputs(sm);

        fault_now = ChargerState_EvalFaults(sm);
        sm->fault_flags = fault_now;
    }
    else
    {
        /* -------------------------------------------------
         * 아직 prime 전이면 일반 fault 평가는 하지 않는다.
         *
         * 단, INIT timeout으로 이미 FAULT에 들어간 상태라면
         * 로그에서 원인이 보이도록 fault_flags를 유지한다.
         * ------------------------------------------------- */
        fault_now = CHARGER_FAULT_NONE;

        if (!((sm->state == CHARGER_STATE_FAULT) &&
              (sm->fault_flags == CHARGER_FAULT_INIT_PRIME_TIMEOUT)))
        {
            sm->fault_flags = CHARGER_FAULT_NONE;
        }
    }

    sm->state_timer_ms =
        ChargerState_AddSaturatedU32(sm->state_timer_ms,
                                     CHARGER_STATE_RUN_PERIOD_MS);

    switch (sm->state)
    {
    case CHARGER_STATE_INIT:
        ChargerState_ForceStopControl(sm);
        sm->done_detect_count = 0U;
        sm->fault_clear_timer_ms = 0U;

        if (sm->sensors_primed != 0U)
        {
            sm->fault_flags = CHARGER_FAULT_NONE;
            ChargerState_Change(sm, CHARGER_STATE_WAIT_SOURCE);
        }
        else if (sm->state_timer_ms >= CHARGER_INIT_PRIME_TIMEOUT_MS)
        {
            sm->fault_flags = CHARGER_FAULT_INIT_PRIME_TIMEOUT;
            ChargerState_Change(sm, CHARGER_STATE_FAULT);
        }
        break;

    case CHARGER_STATE_WAIT_SOURCE:
        ChargerState_ForceStopControl(sm);
        sm->fault_clear_timer_ms = 0U;

        if (sm->sensors_primed == 0U)
        {
            sm->done_detect_count = 0U;
            break;
        }

        if (fault_now != CHARGER_FAULT_NONE)
        {
            sm->done_detect_count = 0U;
            ChargerState_Change(sm, CHARGER_STATE_FAULT);
            break;
        }

        if ((sm->solar_ok != 0U) &&
            (sm->batt_ok != 0U) &&
            (sm->source_ready != 0U))
        {
            /* 이미 배터리 전압이 목표 이상이면
             * 바로 CHARGING으로 가지 않고 DONE 안정 판정을 먼저 본다.
             */
            if (sm->batt_v_last >= CONTROL_TARGET_V_BAT)
            {
                if (ChargerState_UpdateDoneDetect(sm, 1U) != 0U)
                {
                    ChargerState_Change(sm, CHARGER_STATE_DONE);
                }
            }
            else
            {
                sm->done_detect_count = 0U;
                SolarPiControl_Reset(sm->ctrl);
                ChargerState_Change(sm, CHARGER_STATE_CHARGING);
            }
        }
        else
        {
            sm->done_detect_count = 0U;
        }
        break;

    case CHARGER_STATE_CHARGING:
        sm->fault_clear_timer_ms = 0U;

        if (sm->sensors_primed == 0U)
        {
            ChargerState_ForceStopControl(sm);
            sm->done_detect_count = 0U;
            ChargerState_Change(sm, CHARGER_STATE_INIT);
            break;
        }

        if (fault_now != CHARGER_FAULT_NONE)
        {
            ChargerState_ForceStopControl(sm);
            sm->done_detect_count = 0U;
            ChargerState_Change(sm, CHARGER_STATE_FAULT);
            break;
        }

        if ((sm->solar_ok == 0U) || (sm->batt_ok == 0U) || (sm->source_ready == 0U))
        {
            ChargerState_ForceStopControl(sm);
            sm->done_detect_count = 0U;
            ChargerState_Change(sm, CHARGER_STATE_WAIT_SOURCE);
            break;
        }

        /* 실제 제어기 업데이트 */
        SolarPiControl_Update(sm->ctrl,
                              sm->solar_sensor,
                              sm->batt_sensor);

        sm->control_mode_last = sm->ctrl->mode;
        sm->duty_last = sm->ctrl->duty;
        sm->i_ref_last = sm->ctrl->i_ref_a;
        sm->i_cc_last = sm->ctrl->i_cc_a;
        sm->i_cv_last = sm->ctrl->i_cv_a;
        sm->i_mppt_last = sm->ctrl->i_mppt_a;

        /* stop_latched + 목표 전압 도달 상태가
         * 연속해서 유지될 때 DONE으로 전이
         */
        if (ChargerState_UpdateDoneDetect(sm,
                                          (uint8_t)((sm->ctrl->stop_latched != 0U) &&
                                                    (sm->batt_v_last >= CONTROL_TARGET_V_BAT))) != 0U)
        {
            ChargerState_ForceStopControl(sm);
            ChargerState_Change(sm, CHARGER_STATE_DONE);
            break;
        }
        break;

    case CHARGER_STATE_DONE:
        ChargerState_ForceStopControl(sm);
        sm->fault_clear_timer_ms = 0U;

        if (sm->sensors_primed == 0U)
        {
            sm->done_detect_count = 0U;
            ChargerState_Change(sm, CHARGER_STATE_INIT);
            break;
        }

        if (fault_now != CHARGER_FAULT_NONE)
        {
            sm->done_detect_count = 0U;
            ChargerState_Change(sm, CHARGER_STATE_FAULT);
            break;
        }

        /* 배터리 전압이 재시작 임계 아래로 내려가면 다시 대기 상태 */
        if (sm->batt_v_last <= CONTROL_CV_EXIT_V)
        {
            sm->done_detect_count = 0U;
            ChargerState_Change(sm, CHARGER_STATE_WAIT_SOURCE);
        }
        break;

    case CHARGER_STATE_FAULT:
        ChargerState_ForceStopControl(sm);
        sm->done_detect_count = 0U;

        /* INIT timeout으로 들어온 경우,
         * 아직 prime이 안 되었으면 recovery timer를 돌리지 않는다.
         */
        if (sm->sensors_primed == 0U)
        {
            sm->fault_clear_timer_ms = 0U;
            break;
        }

        if (fault_now == CHARGER_FAULT_NONE)
        {
            sm->fault_clear_timer_ms =
                ChargerState_AddSaturatedU32(sm->fault_clear_timer_ms,
                                             CHARGER_STATE_RUN_PERIOD_MS);

            if (sm->fault_clear_timer_ms >= CHARGER_FAULT_RECOVERY_MS)
            {
                sm->fault_clear_timer_ms = 0U;
                sm->fault_flags = CHARGER_FAULT_NONE;
                ChargerState_Change(sm, CHARGER_STATE_WAIT_SOURCE);
            }
        }
        else
        {
            sm->fault_clear_timer_ms = 0U;
        }
        break;

    default:
        ChargerState_ForceStopControl(sm);
        sm->done_detect_count = 0U;
        sm->fault_clear_timer_ms = 0U;
        sm->fault_flags = CHARGER_FAULT_NONE;
        ChargerState_Change(sm, CHARGER_STATE_INIT);
        break;
    }

    /* CHARGING 상태가 아니면 로그용 제어 출력은 STOP 의미로 정리 */
    if (sm->state != CHARGER_STATE_CHARGING)
    {
        sm->control_mode_last = SOLAR_CTRL_MODE_STOP;
        sm->duty_last = 0.0f;
        sm->i_ref_last = 0.0f;
        sm->i_cc_last = CONTROL_I_CC_MAX;
        sm->i_cv_last = CONTROL_I_CC_MAX;
        sm->i_mppt_last = CONTROL_MPPT_I_INIT;
    }

    /* 마지막에 다음 주기용 DMA 센싱 시작 */
    ChargerState_StartNextSenseCycle(sm);
}
