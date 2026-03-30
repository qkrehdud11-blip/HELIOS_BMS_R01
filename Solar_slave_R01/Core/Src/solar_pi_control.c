/*
 * solar_pi_control.c
 *
 *  Created on: 2026. 3. 20.
 *
 *  설명:
 *  ---------------------------------------------------------
 *  태양전지 기반 1셀 리튬 배터리 충전용 PI 제어 모듈
 *
 *  핵심 구조:
 *    Icc   = CC 최대 전류 제한
 *    Icv   = CV 전압 루프 기반 current limit
 *    Imppt = 태양전지 상태 기반 current limit
 *
 *    i_ref = min(Icc, Icv, Imppt)
 *
 *    duty  = current PI(i_ref - batt_i)
 *
 *  이번 버전의 핵심 개선점
 *  ---------------------------------------------------------
 *  1) last_status == HAL_OK 의존 제거
 *     -> snapshot range_valid + 수치 sanity check 사용
 *
 *  2) MPPT current limit 정규화
 *     -> Imppt는 항상
 *          0A 또는 [MPPT_I_MIN ~ MPPT_I_MAX]
 *        범위만 가지도록 정리
 *
 *  3) source-limit assist 유지
 *     -> duty 포화 + 실제 전류 미추종 시
 *        MPPT 제한을 실제 가능한 영역으로 빠르게 낮춤
 */

#include "solar_pi_control.h"
#include <math.h>
#include <string.h>

/* =========================================================
 * 내부용 clamp
 * ========================================================= */
static float SolarPiControl_Clamp(float x, float x_min, float x_max)
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

/* =========================================================
 * 내부용 min3
 * ========================================================= */
static float SolarPiControl_Min3(float a, float b, float c)
{
    float m = a;

    if (b < m)
    {
        m = b;
    }

    if (c < m)
    {
        m = c;
    }

    return m;
}

/* =========================================================
 * 내부용 float 유효성 검사
 * ========================================================= */
static uint8_t SolarPiControl_IsFiniteFloat(float x)
{
    return (uint8_t)(isfinite(x) ? 1U : 0U);
}

/* =========================================================
 * 내부용 SOC 계산
 * ---------------------------------------------------------
 * 단순 선형 근사
 * ========================================================= */
static float SolarPiControl_CalcSocPercent(float batt_v_ctrl)
{
    float soc;

    soc = (batt_v_ctrl - CONTROL_SOC_MIN_V) /
          (CONTROL_SOC_MAX_V - CONTROL_SOC_MIN_V);

    soc = SolarPiControl_Clamp(soc, 0.0f, 1.0f);

    return soc * 100.0f;
}

/* =========================================================
 * 내부용 효율 계산
 * ---------------------------------------------------------
 * 입력 전력이 너무 작으면 0% 처리
 * ========================================================= */
static float SolarPiControl_CalcEfficiency(float pin_w, float pout_w)
{
    float eff;

    if (pin_w <= 0.05f)
    {
        return 0.0f;
    }

    eff = (pout_w / pin_w) * 100.0f;
    eff = SolarPiControl_Clamp(eff, 0.0f, 100.0f);

    return eff;
}

/* =========================================================
 * 내부용: snapshot sanity check
 * ---------------------------------------------------------
 * last_status는 charger_state가 별도로 다루므로
 * 여기서는
 *   1) range_valid
 *   2) finite 여부
 *   3) 아주 명백한 수치 이상 여부
 * 만 검사한다.
 * ========================================================= */
static uint8_t SolarPiControl_IsSnapshotUsable(const SolarSensingSnapshot_t *snap,
                                               float max_bus_v,
                                               float max_source_v)
{
    float current_a;

    if (snap == NULL)
    {
        return 0U;
    }

    if (snap->range_valid == 0U)
    {
        return 0U;
    }

    if ((SolarPiControl_IsFiniteFloat(snap->bus_v) == 0U) ||
        (SolarPiControl_IsFiniteFloat(snap->source_v) == 0U) ||
        (SolarPiControl_IsFiniteFloat(snap->current_ma) == 0U))
    {
        return 0U;
    }

    if ((snap->bus_v < 0.0f) || (snap->bus_v > max_bus_v))
    {
        return 0U;
    }

    if ((snap->source_v < 0.0f) || (snap->source_v > max_source_v))
    {
        return 0U;
    }

    current_a = snap->current_ma * 0.001f;
    if (fabsf(current_a) > CONTROL_SANITY_MAX_CURRENT_A)
    {
        return 0U;
    }

    return 1U;
}

/* =========================================================
 * 내부용: MPPT current limit 정규화
 * ---------------------------------------------------------
 * MPPT limit은 항상 다음 두 형태만 허용한다.
 *   1) 0A
 *   2) [MPPT_I_MIN ~ MPPT_I_MAX]
 *
 * 이유:
 *   기존 코드에서는 감소 과정에서 i_mppt_a가 음수로 남을 수 있었다.
 *   그러면 min(Icc, Icv, Imppt) 이후 i_ref=0으로 굳어질 수 있다.
 * ========================================================= */
static float SolarPiControl_NormalizeMpptCurrent(float i_mppt_a)
{
    if (i_mppt_a <= 0.0f)
    {
        return 0.0f;
    }

    if (i_mppt_a < CONTROL_MPPT_I_MIN)
    {
        return CONTROL_MPPT_I_MIN;
    }

    if (i_mppt_a > CONTROL_MPPT_I_MAX)
    {
        return CONTROL_MPPT_I_MAX;
    }

    return i_mppt_a;
}

/* =========================================================
 * mode 문자열
 * ========================================================= */
const char* SolarPiControl_ModeString(SolarCtrlMode_t mode)
{
    switch (mode)
    {
    case SOLAR_CTRL_MODE_STOP: return "STOP";
    case SOLAR_CTRL_MODE_MPPT: return "MPPT";
    case SOLAR_CTRL_MODE_CC:   return "CC";
    case SOLAR_CTRL_MODE_CV:   return "CV";
    default:                   return "UNKNOWN";
    }
}

/* =========================================================
 * 내부용: current PI
 * ---------------------------------------------------------
 * 입력:
 *   err_a = i_ref - i_meas
 *
 * 출력:
 *   duty
 *
 * anti-windup:
 *   출력이 포화되어 있고, 적분이 더 포화 방향으로 가는 경우
 *   적분을 멈춘다.
 * ========================================================= */
static float SolarPiControl_CurrentPiStep(SolarPiControl_t *ctrl, float err_a)
{
    float p_term;
    float u_unsat;
    float u_sat;

    p_term = CONTROL_KP_I * err_a;
    u_unsat = p_term + ctrl->i_int_duty;

    if (!((u_unsat >= CONTROL_DUTY_MAX && err_a > 0.0f) ||
          (u_unsat <= CONTROL_DUTY_MIN && err_a < 0.0f)))
    {
        ctrl->i_int_duty += CONTROL_KI_I * CONTROL_DT_SEC * err_a;
    }

    ctrl->i_int_duty = SolarPiControl_Clamp(ctrl->i_int_duty,
                                            CONTROL_DUTY_MIN,
                                            CONTROL_DUTY_MAX);

    u_unsat = p_term + ctrl->i_int_duty;
    u_sat = SolarPiControl_Clamp(u_unsat, CONTROL_DUTY_MIN, CONTROL_DUTY_MAX);

    return u_sat;
}

/* =========================================================
 * 내부용: voltage PI
 * ---------------------------------------------------------
 * 입력:
 *   err_v = target_v - batt_v_ctrl
 *
 * 출력:
 *   current limit [A]
 * ========================================================= */
static float SolarPiControl_VoltagePiStep(SolarPiControl_t *ctrl, float err_v)
{
    float p_term;
    float u_unsat;
    float u_sat;

    p_term = CONTROL_KP_V * err_v;
    u_unsat = p_term + ctrl->v_int_a;

    if (!((u_unsat >= CONTROL_I_CC_MAX && err_v > 0.0f) ||
          (u_unsat <= 0.0f && err_v < 0.0f)))
    {
        ctrl->v_int_a += CONTROL_KI_V * CONTROL_DT_SEC * err_v;
    }

    ctrl->v_int_a = SolarPiControl_Clamp(ctrl->v_int_a,
                                         0.0f,
                                         CONTROL_I_CC_MAX);

    u_unsat = p_term + ctrl->v_int_a;
    u_sat = SolarPiControl_Clamp(u_unsat, 0.0f, CONTROL_I_CC_MAX);

    return u_sat;
}

/* =========================================================
 * 내부용: snapshot -> 제어기 상태 반영
 * ---------------------------------------------------------
 * 목적:
 *   snapshot에서 제어에 필요한 측정값을 읽어와
 *   ctrl 내부 상태를 일관성 있게 갱신한다.
 * ========================================================= */
static void SolarPiControl_LoadMeasurements(SolarPiControl_t *ctrl,
                                            const SolarSensingSnapshot_t *solar_snap,
                                            const SolarSensingSnapshot_t *batt_snap)
{
    ctrl->solar_v   = solar_snap->source_v;
    ctrl->solar_i_a = solar_snap->current_ma * 0.001f;
    ctrl->solar_p_w = solar_snap->power_source_w;

    ctrl->batt_bus_v  = batt_snap->bus_v;
    ctrl->batt_v_ctrl = batt_snap->bus_v + CONTROL_BATT_V_OFFSET;
    ctrl->batt_i_a    = batt_snap->current_ma * 0.001f;

    /* 음수 방향 노이즈는 0으로 정리 */
    if (ctrl->solar_i_a < 0.0f)
    {
        ctrl->solar_i_a = 0.0f;
    }

    if (ctrl->batt_i_a < 0.0f)
    {
        ctrl->batt_i_a = 0.0f;
    }

    if (ctrl->solar_p_w < 0.0f)
    {
        ctrl->solar_p_w = 0.0f;
    }

    ctrl->headroom_v   = ctrl->solar_v - ctrl->batt_v_ctrl;
    ctrl->batt_p_w     = ctrl->batt_v_ctrl * ctrl->batt_i_a;
    ctrl->eff_percent  = SolarPiControl_CalcEfficiency(ctrl->solar_p_w, ctrl->batt_p_w);
    ctrl->soc_percent  = SolarPiControl_CalcSocPercent(ctrl->batt_v_ctrl);
}

/* =========================================================
 * 내부용: stop 상태 출력 정리
 * ---------------------------------------------------------
 * 충전 정지 시 duty와 current PI 적분기를 정리한다.
 * voltage PI / MPPT 내부 상태는 유지한다.
 * ========================================================= */
static void SolarPiControl_EnterStop(SolarPiControl_t *ctrl)
{
    ctrl->mode = SOLAR_CTRL_MODE_STOP;
    ctrl->duty = 0.0f;
    ctrl->i_ref_a = 0.0f;
    ctrl->i_int_duty = 0.0f;
    ctrl->source_limited = 0U;
}

/* =========================================================
 * 내부용: MPPT 기본 current limit 갱신
 * ---------------------------------------------------------
 * 기본 철학:
 *   1) solar 전압 headroom이 부족하면 빠르게 감소
 *   2) headroom이 충분하면 증가
 *   3) 중간 구간은 단순 P&O 미세 조정
 *
 * headroom_v = Vsolar - Vbatt_ctrl
 * ========================================================= */
static void SolarPiControl_UpdateMpptLimit(SolarPiControl_t *ctrl)
{
    float d_power_w;

    ctrl->mppt_timer_s += CONTROL_DT_SEC;

    if (ctrl->mppt_timer_s < CONTROL_MPPT_UPDATE_T_SEC)
    {
        return;
    }

    ctrl->mppt_timer_s = 0.0f;

    /* buck 입력 여유가 너무 부족하면 충전 정지 수준으로 제한 */
    if (ctrl->headroom_v <= CONTROL_SOLAR_MIN_BUCK_MARGIN_V)
    {
        ctrl->i_mppt_a = 0.0f;
        ctrl->mppt_prev_power_w = ctrl->solar_p_w;
        return;
    }

    /* headroom이 부족하면 더 빠르게 감소 */
    if (ctrl->headroom_v < CONTROL_SOLAR_HEADROOM_DROP_V)
    {
        ctrl->i_mppt_a -= CONTROL_MPPT_DROP_STEP_A;
    }
    /* headroom이 충분하면 증가 */
    else if (ctrl->headroom_v > CONTROL_SOLAR_HEADROOM_RISE_V)
    {
        ctrl->i_mppt_a += CONTROL_MPPT_RISE_STEP_A;
    }
    /* 중간 구간은 P&O 스타일 미세 조정 */
    else
    {
        d_power_w = ctrl->solar_p_w - ctrl->mppt_prev_power_w;

        if (fabsf(d_power_w) > CONTROL_MPPT_POWER_EPS_W)
        {
            if (d_power_w < 0.0f)
            {
                ctrl->mppt_dir = (int8_t)(-ctrl->mppt_dir);
            }
        }

        ctrl->i_mppt_a += ((float)ctrl->mppt_dir) * CONTROL_MPPT_STEP_A;
    }

    ctrl->i_mppt_a = SolarPiControl_NormalizeMpptCurrent(ctrl->i_mppt_a);
    ctrl->mppt_prev_power_w = ctrl->solar_p_w;
}

/* =========================================================
 * 내부용: source-limit assist
 * ---------------------------------------------------------
 * 목적:
 *   duty는 거의 최대인데 실제 충전전류가 못 따라오면
 *   지금은 CC를 유지할 상황이 아니라
 *   태양전지가 낼 수 있는 수준으로 Imppt를 빨리 낮춘다.
 *
 * 판단 기준:
 *   - 직전 주기 duty가 거의 포화
 *   - 실제 batt_i가 직전 i_ref를 충분히 못 따라옴
 *
 * 주의:
 *   이 함수는 "이번 주기 i_ref 계산 전"에 호출되므로
 *   ctrl->duty, ctrl->i_ref_a는 직전 주기 값이다.
 * ========================================================= */
static void SolarPiControl_ApplySourceLimitAssist(SolarPiControl_t *ctrl)
{
    float limited_ref;

    ctrl->source_limited = 0U;

    /* 입력 headroom 자체가 거의 없으면 0A */
    if (ctrl->headroom_v <= CONTROL_SOLAR_MIN_BUCK_MARGIN_V)
    {
        ctrl->i_mppt_a = 0.0f;
        ctrl->source_limited = 1U;
        return;
    }

    /* duty 포화 + 실제 전류 미추종이면
     * 현재 source limit 상태로 판단한다.
     */
    if ((ctrl->duty >= CONTROL_SOURCE_LIMIT_DETECT_DUTY) &&
        (ctrl->batt_i_a + CONTROL_SOURCE_LIMIT_CURRENT_MARGIN_A < ctrl->i_ref_a))
    {
        limited_ref = ctrl->batt_i_a + CONTROL_SOURCE_LIMIT_RECOVER_MARGIN_A;

        if (limited_ref <= 0.0f)
        {
            ctrl->i_mppt_a = 0.0f;
        }
        else
        {
            ctrl->i_mppt_a = SolarPiControl_NormalizeMpptCurrent(limited_ref);
        }

        ctrl->source_limited = 1U;
        return;
    }

    /* duty가 꼭 포화가 아니더라도
     * headroom이 빠듯하고 Imppt가 실제 전류보다 과도하게 높게 남아 있으면
     * 조금 더 현실값 쪽으로 눌러준다.
     */
    if ((ctrl->headroom_v < CONTROL_SOLAR_HEADROOM_DROP_V) &&
        (ctrl->i_mppt_a > (ctrl->batt_i_a + CONTROL_SOURCE_LIMIT_RECOVER_MARGIN_A)))
    {
        limited_ref = ctrl->batt_i_a + CONTROL_SOURCE_LIMIT_RECOVER_MARGIN_A;

        if (limited_ref <= 0.0f)
        {
            ctrl->i_mppt_a = 0.0f;
        }
        else
        {
            ctrl->i_mppt_a = SolarPiControl_NormalizeMpptCurrent(limited_ref);
        }

        ctrl->source_limited = 1U;
    }
}

/* =========================================================
 * SolarPiControl_Init
 * ========================================================= */
void SolarPiControl_Init(SolarPiControl_t *ctrl)
{
    if (ctrl == NULL)
    {
        return;
    }

    memset(ctrl, 0, sizeof(SolarPiControl_t));

    ctrl->mode = SOLAR_CTRL_MODE_STOP;
    ctrl->i_cc_a = CONTROL_I_CC_MAX;
    ctrl->i_cv_a = CONTROL_I_CC_MAX;
    ctrl->i_mppt_a = CONTROL_MPPT_I_INIT;
    ctrl->mppt_dir = 1;
}

/* =========================================================
 * SolarPiControl_Reset
 * ---------------------------------------------------------
 * PI 및 MPPT 내부 상태를 초기화한다.
 * ========================================================= */
void SolarPiControl_Reset(SolarPiControl_t *ctrl)
{
    if (ctrl == NULL)
    {
        return;
    }

    ctrl->mode = SOLAR_CTRL_MODE_STOP;
    ctrl->stop_latched = 0U;
    ctrl->cv_active = 0U;
    ctrl->source_limited = 0U;

    ctrl->solar_v = 0.0f;
    ctrl->solar_i_a = 0.0f;
    ctrl->solar_p_w = 0.0f;

    ctrl->batt_bus_v = 0.0f;
    ctrl->batt_v_ctrl = 0.0f;
    ctrl->batt_i_a = 0.0f;
    ctrl->batt_p_w = 0.0f;

    ctrl->headroom_v = 0.0f;
    ctrl->eff_percent = 0.0f;
    ctrl->soc_percent = 0.0f;

    ctrl->i_cc_a = CONTROL_I_CC_MAX;
    ctrl->i_cv_a = CONTROL_I_CC_MAX;
    ctrl->i_mppt_a = CONTROL_MPPT_I_INIT;
    ctrl->i_ref_a = 0.0f;

    ctrl->v_int_a = 0.0f;
    ctrl->i_int_duty = 0.0f;

    ctrl->mppt_prev_power_w = 0.0f;
    ctrl->mppt_timer_s = 0.0f;
    ctrl->mppt_dir = 1;

    ctrl->duty = 0.0f;
}

/* =========================================================
 * SolarPiControl_Update
 *
 * 입력:
 *   solar_sensor : 태양전지측 센서값
 *   batt_sensor  : 배터리측 센서값
 *
 * 제어 구조:
 *   Icc   = 최대 충전 전류 제한
 *   Icv   = 전압 루프 기반 current limit
 *   Imppt = 태양전지 상태 기반 current limit
 *
 *   i_ref = min(Icc, Icv, Imppt)
 *
 *   duty  = current PI(i_ref - batt_i)
 * ========================================================= */
void SolarPiControl_Update(SolarPiControl_t *ctrl,
                           const SolarSensing_t *solar_sensor,
                           const SolarSensing_t *batt_sensor)
{
    SolarSensingSnapshot_t solar_snap;
    SolarSensingSnapshot_t batt_snap;

    float err_v;
    float min_ref;
    uint8_t solar_valid;
    uint8_t batt_valid;

    if ((ctrl == NULL) || (solar_sensor == NULL) || (batt_sensor == NULL))
    {
        return;
    }

    /* -----------------------------------------------------
     * 0) 안전한 snapshot 확보
     * ----------------------------------------------------- */
    SolarSensing_GetSnapshot(solar_sensor, &solar_snap);
    SolarSensing_GetSnapshot(batt_sensor, &batt_snap);

    /* -----------------------------------------------------
     * 1) snapshot sanity 검사
     *
     * 주의:
     *   여기서는 last_status를 보지 않는다.
     *   통신 freshness / debounce는 charger_state가 담당하고,
     *   제어기는 "현재 갖고 있는 snapshot이 제어에 넣을 수 있는가"
     *   만 판단한다.
     * ----------------------------------------------------- */
    solar_valid = SolarPiControl_IsSnapshotUsable(&solar_snap,
                                                  CONTROL_SANITY_MAX_SOLAR_V,
                                                  CONTROL_SANITY_MAX_SOLAR_V);

    batt_valid = SolarPiControl_IsSnapshotUsable(&batt_snap,
                                                 CONTROL_SANITY_MAX_BATT_BUS_V,
                                                 CONTROL_SANITY_MAX_BATT_BUS_V);

    if ((solar_valid == 0U) || (batt_valid == 0U))
    {
        SolarPiControl_EnterStop(ctrl);
        return;
    }

    /* -----------------------------------------------------
     * 2) 측정값 반영
     * ----------------------------------------------------- */
    SolarPiControl_LoadMeasurements(ctrl, &solar_snap, &batt_snap);

    /* -----------------------------------------------------
     * 3) STOP latch 해제 조건
     * ----------------------------------------------------- */
    if (ctrl->stop_latched != 0U)
    {
        if (ctrl->batt_v_ctrl <= CONTROL_CV_EXIT_V)
        {
            ctrl->stop_latched = 0U;
        }
        else
        {
            SolarPiControl_EnterStop(ctrl);
            return;
        }
    }

    /* -----------------------------------------------------
     * 4) CV 활성 구간 판단
     * ----------------------------------------------------- */
    if (ctrl->batt_v_ctrl >= CONTROL_CV_ENTRY_V)
    {
        ctrl->cv_active = 1U;
    }
    else if (ctrl->batt_v_ctrl <= CONTROL_CV_EXIT_V)
    {
        ctrl->cv_active = 0U;
    }

    /* -----------------------------------------------------
     * 5) Icc
     * ----------------------------------------------------- */
    ctrl->i_cc_a = CONTROL_I_CC_MAX;

    /* -----------------------------------------------------
     * 6) Icv
     * ----------------------------------------------------- */
    err_v = CONTROL_TARGET_V_BAT - ctrl->batt_v_ctrl;

    if (ctrl->cv_active != 0U)
    {
        ctrl->i_cv_a = SolarPiControl_VoltagePiStep(ctrl, err_v);
    }
    else
    {
        /* CV 영역 밖에서는 전압 루프가 충전을 방해하지 않도록
         * Icv를 CC max로 유지한다.
         *
         * 다만 CV 진입 시 급격한 점프를 줄이기 위해
         * integrator를 현재 오차 기준으로 미리 맞춰 둔다.
         */
        ctrl->v_int_a = SolarPiControl_Clamp(CONTROL_I_CC_MAX - (CONTROL_KP_V * err_v),
                                             0.0f,
                                             CONTROL_I_CC_MAX);

        ctrl->i_cv_a = CONTROL_I_CC_MAX;
    }

    /* -----------------------------------------------------
     * 7) Imppt 기본 갱신
     * ----------------------------------------------------- */
    SolarPiControl_UpdateMpptLimit(ctrl);

    /* -----------------------------------------------------
     * 8) source-limit assist
     * ----------------------------------------------------- */
    SolarPiControl_ApplySourceLimitAssist(ctrl);

    /* -----------------------------------------------------
     * 9) 충전 종료 조건
     * ----------------------------------------------------- */
    if ((ctrl->batt_v_ctrl >= CONTROL_TARGET_V_BAT) &&
        (ctrl->batt_i_a <= CONTROL_TOP_OFF_STOP_I_A) &&
        (ctrl->cv_active != 0U))
    {
        ctrl->stop_latched = 1U;
        SolarPiControl_EnterStop(ctrl);
        return;
    }

    /* -----------------------------------------------------
     * 10) 최종 current reference
     * ----------------------------------------------------- */
    min_ref = SolarPiControl_Min3(ctrl->i_cc_a,
                                  ctrl->i_cv_a,
                                  ctrl->i_mppt_a);

    ctrl->i_ref_a = SolarPiControl_Clamp(min_ref, 0.0f, CONTROL_I_CC_MAX);

    /* -----------------------------------------------------
     * 11) i_ref가 0이면 STOP
     * ----------------------------------------------------- */
    if (ctrl->i_ref_a <= CONTROL_EPS)
    {
        SolarPiControl_EnterStop(ctrl);
        return;
    }

    /* -----------------------------------------------------
     * 12) mode 결정
     *
     * 최종 current ref를 누가 가장 강하게 제한했는지 기준으로 판단
     * ----------------------------------------------------- */
    if ((ctrl->cv_active != 0U) &&
        (ctrl->i_cv_a <= (ctrl->i_mppt_a + CONTROL_MODE_COMPARE_EPS_A)) &&
        (ctrl->i_cv_a <= (ctrl->i_cc_a   + CONTROL_MODE_COMPARE_EPS_A)))
    {
        ctrl->mode = SOLAR_CTRL_MODE_CV;
    }
    else if ((ctrl->i_mppt_a <= (ctrl->i_cv_a + CONTROL_MODE_COMPARE_EPS_A)) &&
             (ctrl->i_mppt_a <  (ctrl->i_cc_a - CONTROL_CC_APPROACH_MARGIN_A)))
    {
        ctrl->mode = SOLAR_CTRL_MODE_MPPT;
    }
    else
    {
        ctrl->mode = SOLAR_CTRL_MODE_CC;
    }

    /* -----------------------------------------------------
     * 13) current loop PI -> duty
     * ----------------------------------------------------- */
    ctrl->duty = SolarPiControl_CurrentPiStep(ctrl,
                                              (ctrl->i_ref_a - ctrl->batt_i_a));
}
