/*
 * bms_safety_manager.c
 *
 *  Created on: 2026. 3. 23.
 *
 *  설명
 *  ---------------------------------------------------------
 *  TEMP / GAS / ELECTRICAL 상태를 종합해서
 *  warning 개수, danger now/latch, 속도 제한을 관리한다.
 *
 *  정책
 *  ---------------------------------------------------------
 *  - 기본 제한 : 80%
 *  - warning 1개 : 60%
 *  - warning 2개 : 40%
 *  - warning 3개 이상 : 0%
 *  - danger 1개 이상 : 0% + latch
 *
 *  electrical 그룹 규칙
 *  ---------------------------------------------------------
 *  - voltage/current는 electrical 하나의 그룹으로 계산
 *  - voltage/current 중 하나라도 warning이면 electrical warning 1개
 *  - voltage/current 중 하나라도 danger이거나 sensor invalid면 electrical danger 1개
 *
 *  핵심 변경점
 *  ---------------------------------------------------------
 *  - 현재 원인과 latch 원인을 따로 보관한다.
 *  - 그래서 로그에서
 *      NOW:SAFE | LATCH:VOLT_UNDER_DANGER
 *    같은 표현이 가능하다.
 */

#include "bms_safety_manager.h"
#include "bms_sensor.h"

/* =========================================================
 * 내부 주기 / 제한 정책
 * ========================================================= */
#define BMS_SAFETY_TASK_PERIOD_MS         400U

#define BMS_SAFETY_LIMIT_BASE_PCT         80U
#define BMS_SAFETY_LIMIT_WARN1_PCT        60U
#define BMS_SAFETY_LIMIT_WARN2_PCT        40U
#define BMS_SAFETY_LIMIT_WARN3_PCT         0U
#define BMS_SAFETY_LIMIT_DANGER_PCT        0U

/* applied_limit_pct가 target_limit_pct를 따라가는 속도
 * ---------------------------------------------------------
 * - warning 감속 : RAMP_DOWN_STEP_PCT
 * - danger  감속 : RAMP_DOWN_DANGER_STEP_PCT (별도 관리)
 * - 복구         : RAMP_UP_STEP_PCT
 */
#define BMS_SAFETY_RAMP_DOWN_STEP_PCT         2U
#define BMS_SAFETY_RAMP_DOWN_DANGER_STEP_PCT  3U
#define BMS_SAFETY_RAMP_UP_STEP_PCT           3U

/* danger debounce
 * ---------------------------------------------------------
 * 순간적인 spike를 무시하기 위해
 * danger로 판정하려면 연속으로 이 횟수만큼 danger이어야 한다.
 * BMS_SAFETY_TASK_PERIOD_MS * BMS_SAFETY_DANGER_DEBOUNCE_CNT 만큼의 지연 발생
 * 예: 500ms * 2 = 1000ms 연속 danger이어야 실제 danger 선언
 */
#define BMS_SAFETY_DANGER_DEBOUNCE_CNT        2U

/* =========================================================
 * 내부 상태
 * ========================================================= */
static uint8_t  s_warn_cnt = 0U;
static uint8_t  s_danger_now = 0U;
static uint8_t  s_danger_latched = 0U;
static uint8_t  s_danger_debounce_cnt = 0U;  /* 연속 danger 횟수 */

static uint8_t  s_elec_warn = 0U;
static uint8_t  s_elec_danger = 0U;

static uint8_t  s_target_limit_pct = BMS_SAFETY_LIMIT_BASE_PCT;
static uint8_t  s_applied_limit_pct = BMS_SAFETY_LIMIT_BASE_PCT;

static const char *s_now_reason = "SAFE";
static const char *s_latched_reason = "NONE";

static uint32_t s_prev_tick = 0U;

/* =========================================================
 * 내부 유틸
 * ========================================================= */
static uint8_t BMS_SAFETY_IsTempWarn(TEMP_STATE st)
{
    return (st == TEMP_STATE_WARNING) ? 1U : 0U;
}

static uint8_t BMS_SAFETY_IsTempDanger(TEMP_STATE st)
{
    return ((st == TEMP_STATE_DANGER) || (st == TEMP_STATE_FAULT)) ? 1U : 0U;
}

static uint8_t BMS_SAFETY_IsGasWarn(GasLevel_t st)
{
    return (st == GAS_WARNING) ? 1U : 0U;
}

static uint8_t BMS_SAFETY_IsGasDanger(GasLevel_t st)
{
    return ((st == GAS_DANGER) || (st == GAS_FAULT)) ? 1U : 0U;
}

static uint8_t BMS_SAFETY_IsVoltWarn(VOLTAGE_STATE st)
{
    return ((st == VOLTAGE_STATE_OVER_WARNING) ||
            (st == VOLTAGE_STATE_UNDER_WARNING)) ? 1U : 0U;
}

static uint8_t BMS_SAFETY_IsVoltDanger(VOLTAGE_STATE st)
{
    return ((st == VOLTAGE_STATE_OVER_DANGER) ||
            (st == VOLTAGE_STATE_UNDER_DANGER)) ? 1U : 0U;
}

static uint8_t BMS_SAFETY_IsCurrWarn(CURRENT_STATE st)
{
    return (st == CURRENT_STATE_WARNING) ? 1U : 0U;
}

static uint8_t BMS_SAFETY_IsCurrDanger(CURRENT_STATE st)
{
    return (st == CURRENT_STATE_DANGER) ? 1U : 0U;
}

/* =========================================================
 * 대표 원인 문자열 계산
 * ---------------------------------------------------------
 * 우선순위 :
 *  danger > warning > safe
 *
 * 여러 개가 동시에 걸리면 우선순위가 높은 것 하나를 대표 원인으로 쓴다.
 * ========================================================= */
static const char *BMS_SAFETY_CalcNowReason(void)
{
    TEMP_STATE    t_st;
    GasLevel_t    g_st;
    VOLTAGE_STATE v_st;
    CURRENT_STATE c_st;
    uint8_t       elec_valid;

    t_st = GetTempState();
    g_st = GetGasState();
    v_st = GetVoltageState();
    c_st = GetCurrentState();
    elec_valid = IsBmsElectricalValid();

    /* ---------------- danger 우선 ---------------- */
    if (t_st == TEMP_STATE_FAULT)
    {
        return "TEMP_FAULT";
    }

    if (t_st == TEMP_STATE_DANGER)
    {
        return "TEMP_DANGER";
    }

    if (g_st == GAS_FAULT)
    {
        return "GAS_FAULT";
    }

    if (g_st == GAS_DANGER)
    {
        return "GAS_DANGER";
    }

    if (elec_valid == 0U)
    {
        return "ELEC_INVALID";
    }

    if (v_st == VOLTAGE_STATE_OVER_DANGER)
    {
        return "VOLT_OVER_DANGER";
    }

    if (v_st == VOLTAGE_STATE_UNDER_DANGER)
    {
        return "VOLT_UNDER_DANGER";
    }

    if (c_st == CURRENT_STATE_DANGER)
    {
        return "CURR_OVER_DANGER";
    }

    /* ---------------- warning ---------------- */
    if (t_st == TEMP_STATE_WARNING)
    {
        return "TEMP_WARNING";
    }

    if (g_st == GAS_WARNING)
    {
        return "GAS_WARNING";
    }

    if (v_st == VOLTAGE_STATE_OVER_WARNING)
    {
        return "VOLT_OVER_WARNING";
    }

    if (v_st == VOLTAGE_STATE_UNDER_WARNING)
    {
        return "VOLT_UNDER_WARNING";
    }

    if (c_st == CURRENT_STATE_WARNING)
    {
        return "CURR_OVER_WARNING";
    }

    return "SAFE";
}

/* =========================================================
 * electrical warning / danger 계산
 * ========================================================= */
static void BMS_SAFETY_UpdateElectricalFlags(void)
{
    uint8_t valid;
    VOLTAGE_STATE v_st;
    CURRENT_STATE c_st;

    valid = IsBmsElectricalValid();
    v_st = GetVoltageState();
    c_st = GetCurrentState();

    s_elec_warn = 0U;
    s_elec_danger = 0U;

    if (valid == 0U)
    {
        s_elec_danger = 1U;
        return;
    }

    if ((BMS_SAFETY_IsVoltDanger(v_st) != 0U) ||
        (BMS_SAFETY_IsCurrDanger(c_st) != 0U))
    {
        s_elec_danger = 1U;
        return;
    }

    if ((BMS_SAFETY_IsVoltWarn(v_st) != 0U) ||
        (BMS_SAFETY_IsCurrWarn(c_st) != 0U))
    {
        s_elec_warn = 1U;
    }
}

/* =========================================================
 * warning 개수 / danger 상태 계산
 * ========================================================= */
static void BMS_SAFETY_UpdateGroupState(void)
{
    TEMP_STATE t_st;
    GasLevel_t g_st;
    uint8_t raw_danger;

    t_st = GetTempState();
    g_st = GetGasState();

    s_warn_cnt = 0U;

    if (BMS_SAFETY_IsTempWarn(t_st) != 0U)
    {
        s_warn_cnt++;
    }

    if (BMS_SAFETY_IsGasWarn(g_st) != 0U)
    {
        s_warn_cnt++;
    }

    if (s_elec_warn != 0U)
    {
        s_warn_cnt++;
    }

    /* raw danger 감지 */
    raw_danger = 0U;

    if ((BMS_SAFETY_IsTempDanger(t_st) != 0U) ||
        (BMS_SAFETY_IsGasDanger(g_st) != 0U) ||
        (s_elec_danger != 0U))
    {
        raw_danger = 1U;
    }

    /* debounce 적용
     * ---------------------------------------------------------
     * - raw danger가 연속으로 DEBOUNCE_CNT 이상이어야 s_danger_now = 1
     * - 단, temp/elec danger는 즉시 반응 (가스 oscillation만 필터)
     * - raw danger가 없어지면 즉시 0으로 클리어
     */
    if (raw_danger != 0U)
    {
        if (s_danger_debounce_cnt < BMS_SAFETY_DANGER_DEBOUNCE_CNT)
        {
            s_danger_debounce_cnt++;
        }

        /* 온도/전기 danger는 debounce 없이 즉시 → 안전 우선 */
        if ((BMS_SAFETY_IsTempDanger(t_st) != 0U) ||
            (s_elec_danger != 0U))
        {
            s_danger_now = 1U;
        }
        else if (s_danger_debounce_cnt >= BMS_SAFETY_DANGER_DEBOUNCE_CNT)
        {
            s_danger_now = 1U;
        }
        else
        {
            /* 아직 debounce 미충족 - gas spike는 warning으로 격하 */
            s_danger_now = 0U;
            if (s_warn_cnt < 3U)
            {
                /* gas warning이 없어도 경고는 표시 */
                s_warn_cnt = (s_warn_cnt > 0U) ? s_warn_cnt : 1U;
            }
        }
    }
    else
    {
        s_danger_debounce_cnt = 0U;
        s_danger_now = 0U;
    }
}

/* =========================================================
 * 목표 제한 속도 계산
 * ========================================================= */
static void BMS_SAFETY_UpdateTargetLimit(void)
{
    if (s_danger_now != 0U)
    {
        s_target_limit_pct = BMS_SAFETY_LIMIT_DANGER_PCT;
        return;
    }

    switch (s_warn_cnt)
    {
        case 0U:
            s_target_limit_pct = BMS_SAFETY_LIMIT_BASE_PCT;
            break;

        case 1U:
            s_target_limit_pct = BMS_SAFETY_LIMIT_WARN1_PCT;
            break;

        case 2U:
            s_target_limit_pct = BMS_SAFETY_LIMIT_WARN2_PCT;
            break;

        default:
            s_target_limit_pct = BMS_SAFETY_LIMIT_WARN3_PCT;
            break;
    }
}

/* =========================================================
 * 실제 적용 제한 속도 ramp
 * ========================================================= */
static void BMS_SAFETY_UpdateAppliedLimit(void)
{
    uint8_t step;

    if (s_applied_limit_pct > s_target_limit_pct)
    {
        step = (s_danger_now != 0U) ? BMS_SAFETY_RAMP_DOWN_DANGER_STEP_PCT
                                    : BMS_SAFETY_RAMP_DOWN_STEP_PCT;

        if ((s_applied_limit_pct - s_target_limit_pct) > step)
        {
            s_applied_limit_pct -= step;
        }
        else
        {
            s_applied_limit_pct = s_target_limit_pct;
        }
    }
    else if (s_applied_limit_pct < s_target_limit_pct)
    {
        if ((s_target_limit_pct - s_applied_limit_pct) > BMS_SAFETY_RAMP_UP_STEP_PCT)
        {
            s_applied_limit_pct += BMS_SAFETY_RAMP_UP_STEP_PCT;
        }
        else
        {
            s_applied_limit_pct = s_target_limit_pct;
        }
    }
    else
    {
        /* 동일하면 유지 */
    }
}

/* =========================================================
 * 공개 함수
 * ========================================================= */
void BMS_SAFETY_Init(void)
{
    s_warn_cnt = 0U;
    s_danger_now = 0U;
    s_danger_latched = 0U;
    s_danger_debounce_cnt = 0U;

    s_elec_warn = 0U;
    s_elec_danger = 0U;

    s_target_limit_pct = BMS_SAFETY_LIMIT_BASE_PCT;
    s_applied_limit_pct = BMS_SAFETY_LIMIT_BASE_PCT;

    s_now_reason = "SAFE";
    s_latched_reason = "NONE";

    s_prev_tick = HAL_GetTick();
}

void BMS_SAFETY_Task(void)
{
    uint32_t now;

    now = HAL_GetTick();

    if ((now - s_prev_tick) < BMS_SAFETY_TASK_PERIOD_MS)
    {
        return;
    }

    s_prev_tick = now;

    BMS_SAFETY_UpdateElectricalFlags();
    BMS_SAFETY_UpdateGroupState();

    s_now_reason = BMS_SAFETY_CalcNowReason();

    /* danger가 새로 발생한 순간의 원인을 latch에 저장 */
    if ((s_danger_now != 0U) && (s_danger_latched == 0U))
    {
        s_danger_latched = 1U;
        s_latched_reason = s_now_reason;
    }

    BMS_SAFETY_UpdateTargetLimit();
    BMS_SAFETY_UpdateAppliedLimit();
}

void BMS_SAFETY_ResetLatch(void)
{
    s_danger_latched = 0U;
    s_latched_reason = "NONE";
}

uint8_t BMS_SAFETY_ApplyLimit(uint8_t req_pct)
{
    if (req_pct > s_applied_limit_pct)
    {
        return s_applied_limit_pct;
    }

    return req_pct;
}

uint8_t BMS_SAFETY_GetWarningCount(void)
{
    return s_warn_cnt;
}

uint8_t BMS_SAFETY_IsDangerNow(void)
{
    return s_danger_now;
}

uint8_t BMS_SAFETY_IsDangerLatched(void)
{
    return s_danger_latched;
}

uint8_t BMS_SAFETY_GetElectricalWarning(void)
{
    return s_elec_warn;
}

uint8_t BMS_SAFETY_GetElectricalDanger(void)
{
    return s_elec_danger;
}

uint8_t BMS_SAFETY_GetTargetLimitPct(void)
{
    return s_target_limit_pct;
}

uint8_t BMS_SAFETY_GetAppliedLimitPct(void)
{
    return s_applied_limit_pct;
}

const char *BMS_SAFETY_GetBanner(void)
{
    /* 기존 코드 호환용 요약 배너
     * 우선순위:
     *   현재 danger -> 현재 warning -> latched danger -> SAFE
     */
    if (s_danger_now != 0U)
    {
        return s_now_reason;
    }

    if (s_warn_cnt > 0U)
    {
        return s_now_reason;
    }

    if (s_danger_latched != 0U)
    {
        return s_latched_reason;
    }

    return "SAFE";
}

const char *BMS_SAFETY_GetNowReason(void)
{
    return s_now_reason;
}

const char *BMS_SAFETY_GetLatchedReason(void)
{
    return s_latched_reason;
}
