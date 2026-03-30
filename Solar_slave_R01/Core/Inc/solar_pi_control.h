/*
 * solar_pi_control.h
 *
 *  Created on: 2026. 3. 20.
 *
 *  설명:
 *  ---------------------------------------------------------
 *  태양전지 기반 1셀 리튬 배터리 충전용 PI 제어 모듈
 *
 *  이 모듈은 solar_sensing 모듈이 제공하는 최신 snapshot을 입력으로 받아,
 *  아래 제어를 수행한다.
 *
 *    1) MPPT 기반 입력 전류 제한
 *    2) CC 최대 충전 전류 제한
 *    3) CV 전압 루프 기반 전류 제한
 *    4) 최종 전류 지령 생성
 *       i_ref = min(Icc, Icv, Imppt)
 *    5) 배터리 전류 기반 current PI로 duty 계산
 *
 *  mode 의미:
 *    STOP : 충전 정지
 *    MPPT : 태양전지 입력 조건에 의해 제한
 *    CC   : 최대 충전 전류 제한
 *    CV   : 배터리 전압 루프 제한
 *
 *  이번 리팩토링 핵심
 *  ---------------------------------------------------------
 *  1) last_status == HAL_OK 의존 제거
 *     - charger_state가 이미 sample age / range / debounce를 관리하므로
 *       제어기에서는 snapshot range_valid + 수치 sanity check를 사용한다.
 *
 *  2) Imppt 음수 잔류 방지
 *     - MPPT current limit은 항상
 *         0A 또는 [CONTROL_MPPT_I_MIN ~ CONTROL_MPPT_I_MAX]
 *       범위만 가지도록 정리했다.
 *
 *  3) source-limit assist 유지
 *     - duty 포화 + 실제 전류 미추종 시
 *       Imppt를 실제 전류 근처로 빠르게 낮춰
 *       저광량에서 duty=0.95 고착을 줄인다.
 */

#ifndef INC_SOLAR_PI_CONTROL_H_
#define INC_SOLAR_PI_CONTROL_H_

#include "solar_sensing.h"
#include <stdint.h>

/* =========================================================
 * control timing
 * ---------------------------------------------------------
 * app / charger_state 에서 10ms 주기로 제어한다고 가정
 * ========================================================= */
#define CONTROL_DT_SEC                           0.010f

/* =========================================================
 * battery / charge setup
 * ---------------------------------------------------------
 * SOC 표시 기준:
 *   4.20V = 100%
 *
 * 실제 충전 상한:
 *   약 90% 운용점인 4.10V 근처
 * ========================================================= */
#define CONTROL_TARGET_V_BAT                     4.10f
#define CONTROL_CV_ENTRY_V                       4.06f
#define CONTROL_CV_EXIT_V                        4.00f

#define CONTROL_SOC_MIN_V                        3.30f
#define CONTROL_SOC_MAX_V                        4.20f

/* ---------------------------------------------------------
 * battery 제어 전압 보정
 *
 * 현재는 우선 0.000f 로 두고,
 * INA219 bus_v 와 멀티미터 실측값 차이를 다시 확인한 뒤
 * 필요하면 소량(+/- 수십 mV 수준)만 재보정하는 것을 권장한다.
 * --------------------------------------------------------- */
#define CONTROL_BATT_V_OFFSET                    0.000f

/* =========================================================
 * current limit
 * ========================================================= */
#define CONTROL_I_CC_MAX                         0.80f

/* =========================================================
 * MPPT current limit
 * ---------------------------------------------------------
 * 저광량에서는 입력 제한 역할
 * 광량이 충분하면 빠르게 CC 한계까지 접근
 * ========================================================= */
#define CONTROL_MPPT_I_INIT                      0.50f
#define CONTROL_MPPT_I_MIN                       0.05f
#define CONTROL_MPPT_I_MAX                       0.90f

#define CONTROL_MPPT_UPDATE_T_SEC                0.05f
#define CONTROL_MPPT_STEP_A                      0.01f
#define CONTROL_MPPT_RISE_STEP_A                 0.03f
#define CONTROL_MPPT_DROP_STEP_A                 0.06f
#define CONTROL_MPPT_POWER_EPS_W                 0.02f

/* =========================================================
 * solar headroom 기준
 * ---------------------------------------------------------
 * buck 동작을 위해 입력 전압이 배터리보다 충분히 높아야 한다.
 *
 * headroom_v = Vsolar - Vbatt_ctrl
 * ========================================================= */
#define CONTROL_SOLAR_MIN_BUCK_MARGIN_V          0.08f
#define CONTROL_SOLAR_HEADROOM_DROP_V            0.40f
#define CONTROL_SOLAR_HEADROOM_RISE_V            0.80f

/* =========================================================
 * mode 판정 / 종료 전류
 * ========================================================= */
#define CONTROL_CC_APPROACH_MARGIN_A             0.02f
#define CONTROL_TOP_OFF_STOP_I_A                 0.03f

/* mode 비교 시 사용할 작은 epsilon */
#define CONTROL_MODE_COMPARE_EPS_A               0.005f

/* =========================================================
 * source-limit assist
 * ---------------------------------------------------------
 * 목적:
 *   duty는 거의 최대인데 실제 충전전류가 못 따라오는 경우
 *   태양전지가 현재 요구 전류를 못 공급하고 있다고 판단해서
 *   Imppt를 현실값 쪽으로 빠르게 낮춘다.
 * ========================================================= */
#define CONTROL_SOURCE_LIMIT_DETECT_DUTY         0.92f
#define CONTROL_SOURCE_LIMIT_CURRENT_MARGIN_A    0.05f
#define CONTROL_SOURCE_LIMIT_RECOVER_MARGIN_A    0.03f

/* =========================================================
 * snapshot sanity check
 * ---------------------------------------------------------
 * charger_state가 상위 건강 상태를 관리하더라도,
 * 제어기 내부에서는 명백히 비정상인 수치가 들어왔을 때
 * 보수적으로 STOP으로 정리할 수 있어야 한다.
 *
 * 아래 범위는 "정상 동작 범위"가 아니라
 * "제어기에 넣어도 되는 최소 sanity 범위"이다.
 * ========================================================= */
#define CONTROL_SANITY_MAX_SOLAR_V               26.50f
#define CONTROL_SANITY_MAX_BATT_BUS_V            26.00f
#define CONTROL_SANITY_MAX_CURRENT_A             3.20f

/* =========================================================
 * PI gains
 * ---------------------------------------------------------
 * current loop 출력: duty
 * voltage loop 출력: current [A]
 * ========================================================= */
#define CONTROL_KP_I                             0.90f
#define CONTROL_KI_I                             8.00f

#define CONTROL_KP_V                             6.00f
#define CONTROL_KI_V                             3.00f

/* =========================================================
 * duty limit
 * ========================================================= */
#define CONTROL_DUTY_MIN                         0.00f
#define CONTROL_DUTY_MAX                         0.95f

/* =========================================================
 * small epsilon
 * ========================================================= */
#define CONTROL_EPS                              1.0e-6f

/* =========================================================
 * charge mode
 * ========================================================= */
typedef enum
{
    SOLAR_CTRL_MODE_STOP = 0,
    SOLAR_CTRL_MODE_MPPT,
    SOLAR_CTRL_MODE_CC,
    SOLAR_CTRL_MODE_CV
} SolarCtrlMode_t;

/* =========================================================
 * control state
 * ========================================================= */
typedef struct
{
    /* 현재 mode */
    SolarCtrlMode_t mode;

    /* STOP / CV 상태 */
    uint8_t stop_latched;
    uint8_t cv_active;
    uint8_t source_limited;

    /* 최근 센서 기반 상태값 */
    float solar_v;
    float solar_i_a;
    float solar_p_w;

    float batt_bus_v;
    float batt_v_ctrl;
    float batt_i_a;
    float batt_p_w;

    float headroom_v;
    float eff_percent;
    float soc_percent;

    /* 제한값 */
    float i_cc_a;
    float i_cv_a;
    float i_mppt_a;
    float i_ref_a;

    /* PI 내부 상태 */
    float v_int_a;
    float i_int_duty;

    /* MPPT 내부 상태 */
    float mppt_prev_power_w;
    float mppt_timer_s;
    int8_t mppt_dir;

    /* 최종 출력 */
    float duty;

} SolarPiControl_t;

/* =========================================================
 * public API
 * ========================================================= */
void SolarPiControl_Init(SolarPiControl_t *ctrl);
void SolarPiControl_Reset(SolarPiControl_t *ctrl);

void SolarPiControl_Update(SolarPiControl_t *ctrl,
                           const SolarSensing_t *solar_sensor,
                           const SolarSensing_t *batt_sensor);

const char* SolarPiControl_ModeString(SolarCtrlMode_t mode);

#endif /* INC_SOLAR_PI_CONTROL_H_ */
