/*
 * app_bms.c
 *
 *  Created on: 2026. 3. 23.
 *
 *  [Main role]
 *  ---------------------------------------------------------
 *  BMS 상위 통합 app 구현
 *
 *  현재 구조
 *  ---------------------------------------------------------
 *  1) main 에서는 App_Bms_Init(), App_Bms_Task()만 호출
 *  2) sensor non-blocking service 진행
 *  3) force stop 아니면 ST_MACHINE() 실행
 *  4) sensor / safety / log 계속 갱신
 *  5) track / charge / force CAN pending 처리
 *
 *  버튼 정책
 *  ---------------------------------------------------------
 *    A : Manual / Auto 토글
 *    P : 강제정지 / 초기화 토글
 *    D : 태양 추적 ON / OFF 토글
 *    K : 태양 충전 ON / OFF 토글
 *
 *  force stop 정책
 *  ---------------------------------------------------------
 *  - 안전 센싱만 계속 동작
 *  - 주행 정지
 *  - tracking / charging OFF 강제
 *  - A / D / K 입력 무시
 *  - 다시 P 누르면 재초기화 후 복귀
 */

#include "app_bms.h"
#include "can.h"
#include "bms_sensor.h"
#include "bms_safety_manager.h"
#include "bms_message.h"
#include "statemachine.h"
#include "car.h"
#include "safe_drive.h"

#include "app_can_control.h"
#include "app_proto.h"

#include <stdio.h>

/* =========================================================
 * 로그 설정
 * ---------------------------------------------------------
 * UART2는 moserial 입력 용도로도 쓸 수 있으므로 기본 OFF 권장
 * UART6는 상태 로그 확인용으로 사용
 * ========================================================= */

/* =========================================================
 * 입력 debounce
 * ---------------------------------------------------------
 * 동일 문자가 너무 빠르게 연속 들어오면 무시
 * 리모컨 bounce / moserial 중복 입력 방지용
 * ========================================================= */
#define APP_BMS_CMD_DEBOUNCE_MS            150U

/* =========================================================
 * 내부 상태
 * ========================================================= */
typedef struct
{
    uint8_t init_done;

    /* 리모컨 관련 상태 */
    AppBmsMode_t mode;
    uint8_t last_remote_cmd;

    /* 강제 정지 잠금
     * 0 : 정상 동작 가능
     * 1 : force stop 상태
     */
    uint8_t force_stop_lock;

    /* 센서 danger에 의한 자동 강제정지 여부
     * 0 : 수동 P 버튼에 의한 force stop (또는 정상 동작)
     * 1 : 가스/전류/전압/온도 Danger 감지에 의한 자동 force stop
     */
    uint8_t danger_auto_stopped;

    /* 기능 토글 상태 */
    uint8_t track_on;
    uint8_t charge_on;

    /* CAN pending */
    uint8_t track_tx_pending;
    uint8_t charge_tx_pending;
    uint8_t drive_on;
    uint8_t drive_tx_pending;

    uint8_t force_tx_pending;
    uint8_t force_tx_value;

    /* CAN hook */
    AppBmsCanTrackHook_t track_hook;
    AppBmsCanChargeHook_t charge_hook;

    /* debounce */
    uint8_t last_cmd_for_debounce;
    uint32_t last_cmd_tick;
} AppBms_t;

static AppBms_t g_app_bms;

/* =========================================================
 * CAN hook helper
 * ---------------------------------------------------------
 * 성공/실패를 상위 pending 처리에서 알 수 있도록 반환값 유지
 * ========================================================= */
static uint8_t App_Bms_CanTrackHook(uint8_t on)
{
    return AppCan_SendTrace(on);
}

static uint8_t App_Bms_CanChargeHook(uint8_t on)
{
    return AppCan_SendSolar(on);
}

/* =========================================================
 * 내부 helper
 * ========================================================= */
static uint8_t App_Bms_IsReady(void)
{
    return g_app_bms.init_done;
}

static uint8_t App_Bms_NormalizeCmd(uint8_t cmd)
{
    if ((cmd >= 'a') && (cmd <= 'z'))
    {
        return (uint8_t)(cmd - ('a' - 'A'));
    }

    return cmd;
}

static uint8_t App_Bms_IsDebounceDrop(uint8_t cmd)
{
    uint32_t now;

    now = HAL_GetTick();

    if ((cmd == g_app_bms.last_cmd_for_debounce) &&
        ((now - g_app_bms.last_cmd_tick) < APP_BMS_CMD_DEBOUNCE_MS))
    {
        return 1U;
    }

    g_app_bms.last_cmd_for_debounce = cmd;
    g_app_bms.last_cmd_tick = now;

    return 0U;
}

static void App_Bms_ResetLocalState(void)
{
    g_app_bms.init_done = 0U;

    g_app_bms.mode = APP_BMS_MODE_MANUAL;
    g_app_bms.last_remote_cmd = 0U;

    g_app_bms.force_stop_lock = 0U;
    g_app_bms.danger_auto_stopped = 0U;

    g_app_bms.track_on = 0U;
    g_app_bms.charge_on = 0U;

    g_app_bms.track_tx_pending = 0U;
    g_app_bms.charge_tx_pending = 0U;
    g_app_bms.drive_on = 0U;
    g_app_bms.drive_tx_pending = 0U;

    g_app_bms.force_tx_pending = 0U;
    g_app_bms.force_tx_value = 0U;

    g_app_bms.track_hook = (AppBmsCanTrackHook_t)0;
    g_app_bms.charge_hook = (AppBmsCanChargeHook_t)0;

    g_app_bms.last_cmd_for_debounce = 0U;
    g_app_bms.last_cmd_tick = 0U;
}

static void App_Bms_ToggleMode(void)
{
    if (g_app_bms.mode == APP_BMS_MODE_MANUAL)
    {
        g_app_bms.mode = APP_BMS_MODE_AUTO;
        g_app_bms.drive_on = 1U;
    }
    else
    {
        g_app_bms.mode = APP_BMS_MODE_MANUAL;
        g_app_bms.drive_on = 0U;
    }

    g_app_bms.drive_tx_pending = 1U;
}

static void App_Bms_SetTrack(uint8_t on)
{
    g_app_bms.track_on = (on != 0U) ? 1U : 0U;
    g_app_bms.track_tx_pending = 1U;
}

static void App_Bms_SetCharge(uint8_t on)
{
    g_app_bms.charge_on = (on != 0U) ? 1U : 0U;
    g_app_bms.charge_tx_pending = 1U;
}

static void App_Bms_ToggleTrack(void)
{
    App_Bms_SetTrack((g_app_bms.track_on == 0U) ? 1U : 0U);
}

static void App_Bms_ToggleCharge(void)
{
    App_Bms_SetCharge((g_app_bms.charge_on == 0U) ? 1U : 0U);
}

static void App_Bms_QueueForce(uint8_t force_value)
{
    g_app_bms.force_tx_value = force_value;
    g_app_bms.force_tx_pending = 1U;
}

/* =========================================================
 * force stop 진입
 * ---------------------------------------------------------
 * 안전 센싱 제외 모든 동작 정지
 * - 주행 정지
 * - 추적 OFF
 * - 충전 OFF
 * - 이후 ST_MACHINE() 실행 금지
 * ========================================================= */
static void App_Bms_EnterForceStop(void)
{
    g_app_bms.force_stop_lock = 1U;

    /* 즉시 주행 정지 */
    Car_Stop();

    /* 추적 / 충전 / 자율주행 강제 OFF */
    g_app_bms.track_on = 0U;
    g_app_bms.charge_on = 0U;
    g_app_bms.drive_on = 0U;

    g_app_bms.track_tx_pending = 1U;
    g_app_bms.charge_tx_pending = 1U;
    g_app_bms.drive_tx_pending = 1U;

    /* 상태머신 재초기화 (TraceInit)
     * manual / auto 구분 없이 주행 관련 상태를 초기값으로 리셋한다.
     */
    STMACHINE_Init();

    /* force stop도 재시도 가능하도록 pending 등록 */
    App_Bms_QueueForce(APP_FORCE_STOP);

    /* 첫 시도는 즉시 한 번 날려도 된다.
     * 실패하면 Task에서 재시도한다.
     */
    (void)AppCan_SendForceStop();
}

/* =========================================================
 * force stop 해제 + 런타임 재초기화
 * ---------------------------------------------------------
 * 다시 동작 가능한 상태로 복귀
 *
 * 기본값 정책
 * - mode    : MANUAL
 * - track   : OFF
 * - charge  : OFF
 * ========================================================= */
static void App_Bms_ReinitRuntime(void)
{
    /* 기본 상태 먼저 정리 */
    g_app_bms.mode = APP_BMS_MODE_MANUAL;
    g_app_bms.last_remote_cmd = 0U;

    g_app_bms.force_stop_lock = 0U;
    g_app_bms.danger_auto_stopped = 0U;

    g_app_bms.track_on = 0U;
    g_app_bms.charge_on = 0U;
    g_app_bms.drive_on = 0U;

    /* slave에도 OFF 재전달 가능하도록 pending 세트 */
    g_app_bms.track_tx_pending = 1U;
    g_app_bms.charge_tx_pending = 1U;
    g_app_bms.drive_tx_pending = 1U;

    /* 센서 / safety / state machine 재초기화 */
    BMS_SENSOR_Init();
    BMS_SAFETY_Init();
    STMACHINE_Init();

    /* force reinit도 재시도 가능하도록 pending 등록 */
    App_Bms_QueueForce(APP_FORCE_REINIT);

    /* 첫 시도는 즉시 한 번 날려도 된다.
     * 실패하면 Task에서 재시도한다.
     */
    (void)AppCan_SendForceReinit();
}

/* =========================================================
 * 센서 Danger 자동 강제정지
 * ---------------------------------------------------------
 * 가스 / 전류 / 전압 / 온도 중 하나라도 Danger 상태가 되면
 * 수동으로 "P"를 누른 것과 동일하게 강제정지 처리한다.
 *
 * - force_stop_lock == 0 일 때만 진입
 * - force_stop_lock == 1 이면 이미 정지 중이므로 재진입 없음
 * - 해제는 반드시 "P" 버튼으로만 가능 (App_Bms_ReinitRuntime)
 * ========================================================= */
static void App_Bms_CheckDangerAutoStop(void)
{
    if (g_app_bms.force_stop_lock != 0U)
    {
        /* 이미 force stop 상태 → 추가 처리 불필요 */
        return;
    }

    if (BMS_SAFETY_IsDangerNow() == 0U)
    {
        /* 현재 danger 없음 → 정상 동작 */
        return;
    }

    /* Danger 감지 → 자동 강제정지 진입 */
    g_app_bms.danger_auto_stopped = 1U;

    printf("[APP] DANGER AUTO STOP reason=%s → force stop engaged\r\n",
           BMS_SAFETY_GetNowReason());

    App_Bms_EnterForceStop();
}

static void App_Bms_HandleForceToggle(void)
{
    if (g_app_bms.force_stop_lock == 0U)
    {
        App_Bms_EnterForceStop();
    }
    else
    {
        App_Bms_ReinitRuntime();
    }
}

/* =========================================================
 * CAN pending 처리
 * ---------------------------------------------------------
 * 성공했을 때만 pending clear
 * 실패하면 다음 Task에서 재시도
 * ========================================================= */
static void App_Bms_ProcessCanPending(void)
{
    uint8_t ret;

    /* 1) force */
    if (g_app_bms.force_tx_pending != 0U)
    {
        if (g_app_bms.force_tx_value == APP_FORCE_STOP)
        {
            ret = AppCan_SendForceStop();
        }
        else if (g_app_bms.force_tx_value == APP_FORCE_REINIT)
        {
            ret = AppCan_SendForceReinit();
        }
        else
        {
            ret = 1U;
        }

        if (ret == 0U)
        {
            g_app_bms.force_tx_pending = 0U;
        }
    }

    /* 2) track */
    if ((g_app_bms.track_tx_pending != 0U) &&
        (g_app_bms.track_hook != (AppBmsCanTrackHook_t)0))
    {
        ret = g_app_bms.track_hook(g_app_bms.track_on);
        if (ret == 0U)
        {
            g_app_bms.track_tx_pending = 0U;
        }
    }

    /* 3) charge */
    if ((g_app_bms.charge_tx_pending != 0U) &&
        (g_app_bms.charge_hook != (AppBmsCanChargeHook_t)0))
    {
        ret = g_app_bms.charge_hook(g_app_bms.charge_on);
        if (ret == 0U)
        {
            g_app_bms.charge_tx_pending = 0U;
        }
    }

    /* 4) drive (자율주행 ON/OFF) */
    if (g_app_bms.drive_tx_pending != 0U)
    {
        ret = AppCan_SendDrive(g_app_bms.drive_on);
        if (ret == 0U)
        {
            g_app_bms.drive_tx_pending = 0U;
        }
    }
}

void App_Bms_RegisterCanHooks(AppBmsCanTrackHook_t track_hook,
                              AppBmsCanChargeHook_t charge_hook)
{
    g_app_bms.track_hook = track_hook;
    g_app_bms.charge_hook = charge_hook;
}

/* =========================================================
 * public API
 * ========================================================= */
void App_Bms_Init(void)
{
    App_Bms_ResetLocalState();

    /* 센서 초기화 */
    BMS_SENSOR_Init();

    /* safety 초기화
     * 정상 주행 기준 80%는 safety manager 내부 상수 사용
     */
    BMS_SAFETY_Init();

    /* 기존 주행 상태머신 초기화 */
    STMACHINE_Init();

    /* D/K CAN hook 연결 */
    App_Bms_RegisterCanHooks(App_Bms_CanTrackHook, App_Bms_CanChargeHook);

    g_app_bms.init_done = 1U;
}

void App_Bms_Task(void)
{
    /* 0 BMS 시스템 작동 확인 */
    if (App_Bms_IsReady() == 0U)
    {
        return;
    }

    /* 1 INA219 센서값 수신 */
    BMS_SENSOR_Service();

    /* 2 force stop 아닐 때만 주행 상태머신 동작 */
    if (g_app_bms.force_stop_lock != 0U)
    {
        Car_Stop();
    }
    else
    {
        ST_MACHINE();
    }

    /* 3 모든 센서 측정 값 읽기  */
    BMS_SENSOR_Task();

    /* 4 safety 정책 갱신 */
    BMS_SAFETY_Task();

    /* 4-1 Danger 자동 강제정지 체크
     * safety 정책 갱신 직후 danger 여부를 확인하여
     * Danger 상태 진입 시 자동으로 force stop 처리한다.
     * (수동 P 버튼과 동일한 효과, P 버튼으로만 해제 가능)
     */
    App_Bms_CheckDangerAutoStop();

    /* 4-2 BMS 제한 재적용
     * applied_limit_pct가 바뀌었을 때 이미 주행 중인 모터에
     * 즉시 새 속도 제한을 반영한다.
     * 정지 상태이면 내부에서 아무것도 하지 않는다.
     */
    SafeDrive_ReapplyLimit();

    /* 5 CAN pending 처리
     * force stop 중에도 OFF / force 명령은 나가야 하므로 계속 처리
     */
    App_Bms_ProcessCanPending();

    /* 6 Slave로 CAN통신 송출 */
	Can_Task();

    /* 7 BMS 로그 송출 */
    SHOW_UART6_BMS();
}

void App_Bms_NotifyRemoteCmd(uint8_t cmd)
{
    cmd = App_Bms_NormalizeCmd(cmd);

    if (App_Bms_IsDebounceDrop(cmd) != 0U)
    {
        return;
    }

    g_app_bms.last_remote_cmd = cmd;

    /* P는 언제나 처리 가능 */
    if (cmd == 'P')
    {
        App_Bms_HandleForceToggle();
        printf("[APP] P mode=%u lock=%u\r\n",
               (unsigned)g_app_bms.mode,
               (unsigned)g_app_bms.force_stop_lock);
        return;
    }

    /* force stop 상태에서는 P 외 모든 입력 무시 */
    if (g_app_bms.force_stop_lock != 0U)
    {
        printf("[APP] IGNORE %c lock=%u\r\n",
               (char)cmd,
               (unsigned)g_app_bms.force_stop_lock);
        return;
    }

    switch (cmd)
    {
        case 'A':
            App_Bms_ToggleMode();
            printf("[APP] A mode=%u lock=%u\r\n",
                   (unsigned)g_app_bms.mode,
                   (unsigned)g_app_bms.force_stop_lock);
            break;

        case 'D':
            App_Bms_ToggleTrack();
            printf("[APP] D track=%u\r\n",
                   (unsigned)g_app_bms.track_on);
            break;

        case 'K':
            App_Bms_ToggleCharge();
            printf("[APP] K charge=%u\r\n",
                   (unsigned)g_app_bms.charge_on);
            break;

        default:
            /* F/B/L/R/S 등 주행 문자는 기존 statemachine이 처리 */
            break;
    }
}

void App_Bms_ResetLatch(void)
{
    if (App_Bms_IsReady() == 0U)
    {
        return;
    }

    BMS_SAFETY_ResetLatch();
}

uint8_t App_Bms_ApplySpeedLimit(uint8_t req_pct)
{
    if (App_Bms_IsReady() == 0U)
    {
        return 0U;
    }

    if (g_app_bms.force_stop_lock != 0U)
    {
        return 0U;
    }

    return BMS_SAFETY_ApplyLimit(req_pct);
}

/* =========================================================
 * getter
 * ========================================================= */
uint8_t App_Bms_IsInitDone(void)
{
    return g_app_bms.init_done;
}

AppBmsMode_t App_Bms_GetMode(void)
{
    return g_app_bms.mode;
}

uint8_t App_Bms_IsManualMode(void)
{
    return (g_app_bms.mode == APP_BMS_MODE_MANUAL) ? 1U : 0U;
}

uint8_t App_Bms_IsAutoMode(void)
{
    return (g_app_bms.mode == APP_BMS_MODE_AUTO) ? 1U : 0U;
}

uint8_t App_Bms_IsForceStopLocked(void)
{
    return g_app_bms.force_stop_lock;
}

uint8_t App_Bms_IsTrackOn(void)
{
    return g_app_bms.track_on;
}

uint8_t App_Bms_IsChargeOn(void)
{
    return g_app_bms.charge_on;
}

uint8_t App_Bms_GetLastRemoteCmd(void)
{
    return g_app_bms.last_remote_cmd;
}

uint8_t App_Bms_IsStopLatched(void)
{
    if (App_Bms_IsReady() == 0U)
    {
        return 0U;
    }

    if (g_app_bms.force_stop_lock != 0U)
    {
        return 1U;
    }

    return BMS_SAFETY_IsDangerLatched();
}

uint8_t App_Bms_GetWarningCount(void)
{
    if (App_Bms_IsReady() == 0U)
    {
        return 0U;
    }

    return BMS_SAFETY_GetWarningCount();
}

uint8_t App_Bms_IsDangerNow(void)
{
    if (App_Bms_IsReady() == 0U)
    {
        return 0U;
    }

    return BMS_SAFETY_IsDangerNow();
}

uint8_t App_Bms_GetTargetLimitPct(void)
{
    if (App_Bms_IsReady() == 0U)
    {
        return 0U;
    }

    if (g_app_bms.force_stop_lock != 0U)
    {
        return 0U;
    }

    return BMS_SAFETY_GetTargetLimitPct();
}

uint8_t App_Bms_GetAppliedLimitPct(void)
{
    if (App_Bms_IsReady() == 0U)
    {
        return 0U;
    }

    if (g_app_bms.force_stop_lock != 0U)
    {
        return 0U;
    }

    return BMS_SAFETY_GetAppliedLimitPct();
}
