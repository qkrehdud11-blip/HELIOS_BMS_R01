/*
 * charger_state.h
 *
 *  Created on: 2026. 3. 20.
 *
 *  설명:
 *  ---------------------------------------------------------
 *  태양전지 충전기 상위 상태머신
 *
 *  역할:
 *    - solar / battery 센서 상태 수집
 *    - 센서 통신 / 범위 fault debounce
 *    - WAIT_SOURCE / CHARGING / DONE / FAULT 상태 전이
 *    - SolarPiControl 호출 타이밍 관리
 *    - 로그용 마지막 상태값 보관
 *
 *  이번 버전의 핵심 포인트
 *  ---------------------------------------------------------
 *  1) sensor->update_done 직접 접근 제거
 *     -> SolarSensing_FetchUpdateDone() 사용
 *
 *  2) HAL_BUSY를 통신 fault로 간주하지 않음
 *     -> 새 샘플 수신 시점과 sample age를 기준으로 COMM fault 판정
 *
 *  3) snapshot은 "새 샘플이 들어왔을 때만" 갱신
 *     -> 마지막 유효 샘플을 안정적으로 유지
 *
 *  4) false BATT_RANGE / false COMM / false FAULT / DONE 튐 완화
 *
 *  5) INIT prime timeout 추가
 *     -> 부팅 후 일정 시간 안에 solar / batt 샘플이 모두 준비되지 않으면
 *        INIT에 무한정 머무르지 않고 FAULT로 전이
 */

#ifndef __CHARGER_STATE_H
#define __CHARGER_STATE_H

#include "solar_pi_control.h"
#include <stdint.h>

/* =========================================================
 * charger_state 실행 주기
 * ---------------------------------------------------------
 * app_charger에서 10ms 주기로 호출하는 것을 기준으로 한다.
 * ========================================================= */
#define CHARGER_STATE_RUN_PERIOD_MS              10U

/* =========================================================
 * INIT prime timeout
 * ---------------------------------------------------------
 * 부팅 후 이 시간 안에 solar / batt 센서가 모두 최소 1회 이상
 * 유효 샘플을 확보하지 못하면 INIT timeout fault로 본다.
 *
 * 목적:
 *   - 센서 미연결
 *   - I2C 주소 오류
 *   - DMA/IRQ 설정 문제
 * 같은 초기 문제를 빨리 드러내기 위함
 * ========================================================= */
#define CHARGER_INIT_PRIME_TIMEOUT_MS          1500U

/* =========================================================
 * fault recovery timing
 * ---------------------------------------------------------
 * FAULT 상태에서 fault가 사라진 뒤
 * 일정 시간 연속 안정 상태가 유지되면 WAIT_SOURCE로 복귀
 * ========================================================= */
#define CHARGER_FAULT_RECOVERY_MS               500U

/* =========================================================
 * sample stale timeout
 * ---------------------------------------------------------
 * 새 샘플이 이 시간 이상 들어오지 않으면
 * 통신 이상 후보로 본다.
 *
 * 주의:
 * - 10ms task, INA219 DMA 주기, 내부 conversion time을 고려해
 *   너무 짧지 않게 설정한다.
 * - 1~2회 지연으로 false COMM fault 나지 않도록 여유를 둔다.
 * ========================================================= */
#define CHARGER_COMM_STALE_TIMEOUT_MS            80U

/* =========================================================
 * soft fault debounce
 * ---------------------------------------------------------
 * soft fault:
 *   - SOLAR/BATT COMM
 *   - SOLAR/BATT RANGE
 *
 * 이유:
 *   센서 샘플 1~2회 튐으로 즉시 FAULT 들어가면
 *   충전기 전체가 너무 쉽게 STOP 된다.
 * ========================================================= */
#define CHARGER_COMM_FAULT_ASSERT_COUNT           2U
#define CHARGER_COMM_FAULT_CLEAR_COUNT            3U

#define CHARGER_RANGE_FAULT_ASSERT_COUNT          3U
#define CHARGER_RANGE_FAULT_CLEAR_COUNT           5U

/* =========================================================
 * DONE 안정 판정
 * ---------------------------------------------------------
 * 충전 완료 / 재시작 튐 방지를 위해
 * DONE 조건을 연속 샘플로 확인한다.
 * ========================================================= */
#define CHARGER_DONE_DETECT_COUNT                 3U

/* =========================================================
 * 입력 / 배터리 유효 범위
 * ---------------------------------------------------------
 * 상위 상태머신에서 사용하는 논리 범위
 * ========================================================= */
#define CHARGER_MIN_SOLAR_SRC_V                   4.00f
#define CHARGER_BATT_VALID_MIN_V                  2.50f
#define CHARGER_BATT_VALID_MAX_V                  4.35f

/* =========================================================
 * 보호용 hard fault threshold
 * ---------------------------------------------------------
 * 실제 보호 조건
 *
 * 주의:
 * - hard fault는 stale한 마지막 값으로 오판하지 않도록
 *   소스가 유효할 때만 검사한다.
 * ========================================================= */
#define CHARGER_FAULT_BATT_OVERVOLT_V             4.25f
#define CHARGER_FAULT_BATT_UNDERVOLT_V            2.50f

#define CHARGER_FAULT_BATT_ABS_CURRENT_A          1.00f
#define CHARGER_FAULT_SOLAR_ABS_CURRENT_A         1.00f

/* =========================================================
 * charger high-level state
 * ========================================================= */
typedef enum
{
    CHARGER_STATE_INIT = 0,
    CHARGER_STATE_WAIT_SOURCE,
    CHARGER_STATE_CHARGING,
    CHARGER_STATE_DONE,
    CHARGER_STATE_FAULT
} ChargerStateId_t;

/* =========================================================
 * fault bit flags
 * ========================================================= */
#define CHARGER_FAULT_NONE                  0x00000000UL
#define CHARGER_FAULT_SOLAR_COMM            0x00000001UL
#define CHARGER_FAULT_BATT_COMM             0x00000002UL
#define CHARGER_FAULT_SOLAR_RANGE           0x00000004UL
#define CHARGER_FAULT_BATT_RANGE            0x00000008UL
#define CHARGER_FAULT_BATT_OVERVOLT         0x00000010UL
#define CHARGER_FAULT_BATT_UNDERVOLT        0x00000020UL
#define CHARGER_FAULT_BATT_OVERCURRENT      0x00000040UL
#define CHARGER_FAULT_SOLAR_OVERCURRENT     0x00000080UL
#define CHARGER_FAULT_INIT_PRIME_TIMEOUT    0x00000100UL

/* =========================================================
 * ChargerState_t
 * ---------------------------------------------------------
 * last 값들은 "마지막 유효 샘플" 기준으로 유지한다.
 *
 * 즉,
 *   - 새 샘플이 안 들어온 주기에는 덮어쓰지 않음
 *   - 통신 이상 시에도 직전 유효값을 로그용으로 유지 가능
 *
 * COMM fault는 last_status가 아니라
 *   sample_age_ms + debounce
 * 로 판단한다.
 * ========================================================= */
typedef struct
{
    SolarSensing_t   *solar_sensor;
    SolarSensing_t   *batt_sensor;
    SolarPiControl_t *ctrl;

    ChargerStateId_t state;
    ChargerStateId_t prev_state;

    uint32_t fault_flags;

    uint32_t state_timer_ms;
    uint32_t fault_clear_timer_ms;

    /* -----------------------------------------------------
     * 샘플 상태
     * -----------------------------------------------------
     * sample_ready :
     *   지금까지 최소 1회 이상 샘플을 정상 회수한 적 있음
     *
     * sample_fresh :
     *   이번 주기에 새 샘플이 막 들어왔음
     *
     * sensors_primed :
     *   solar / batt 둘 다 최소 1회 정상 샘플 확보 완료
     * ----------------------------------------------------- */
    uint8_t solar_sample_ready;
    uint8_t batt_sample_ready;
    uint8_t solar_sample_fresh;
    uint8_t batt_sample_fresh;
    uint8_t sensors_primed;

    /* -----------------------------------------------------
     * sample age
     * -----------------------------------------------------
     * 마지막 수신 샘플 이후 경과 시간
     * COMM fault는 이 값을 기준으로 판정한다.
     * ----------------------------------------------------- */
    uint32_t solar_sample_age_ms;
    uint32_t batt_sample_age_ms;

    /* -----------------------------------------------------
     * 최신 health 상태
     * -----------------------------------------------------
     * comm_ok :
     *   sample 수신 이력 + sample age로 판정
     *
     * range_ok :
     *   sensor가 확정한 latest range_valid
     * ----------------------------------------------------- */
    uint8_t solar_comm_ok;
    uint8_t solar_range_ok;
    uint8_t batt_comm_ok;
    uint8_t batt_range_ok;

    /* debounce 후 active fault 상태 */
    uint8_t solar_comm_fault_active;
    uint8_t batt_comm_fault_active;
    uint8_t solar_range_fault_active;
    uint8_t batt_range_fault_active;

    /* fault debounce counters */
    uint8_t solar_comm_bad_count;
    uint8_t solar_comm_good_count;
    uint8_t batt_comm_bad_count;
    uint8_t batt_comm_good_count;

    uint8_t solar_range_bad_count;
    uint8_t solar_range_good_count;
    uint8_t batt_range_bad_count;
    uint8_t batt_range_good_count;

    /* DONE stable detect */
    uint8_t done_detect_count;

    /* 상위 입력 판정 */
    uint8_t solar_ok;
    uint8_t batt_ok;
    uint8_t source_ready;

    /* -----------------------------------------------------
     * 마지막 유효 샘플 기반 로그용 값
     * ----------------------------------------------------- */
    float solar_src_v_last;
    float solar_bus_v_last;
    float solar_i_last;
    float solar_p_last;

    float batt_bus_v_last;
    float batt_v_last;
    float batt_i_last;
    float batt_p_last;

    float eff_last;
    float soc_last;

    SolarCtrlMode_t control_mode_last;

    float duty_last;
    float i_ref_last;
    float i_cc_last;
    float i_cv_last;
    float i_mppt_last;

} ChargerState_t;

/* =========================================================
 * public API
 * ========================================================= */
void ChargerState_Init(ChargerState_t *sm,
                       SolarSensing_t *solar_sensor,
                       SolarSensing_t *batt_sensor,
                       SolarPiControl_t *ctrl);

void ChargerState_Reset(ChargerState_t *sm);

void ChargerState_Run(ChargerState_t *sm);

const char* ChargerState_StateString(ChargerStateId_t state);
const char* ChargerState_PrimaryFaultString(uint32_t fault_flags);

#endif /* __CHARGER_STATE_H */
