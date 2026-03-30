
/*
 * statemachine.c
 *
 *  Created on: Mar 1, 2026
 *      Author: appletea
 *
 *  [Main role]
 *  ---------------------------------------------------------
 *  슬레이브 상위 상태머신
 *
 *  역할
 *    - UART6 로컬 테스트 명령 처리
 *    - CAN 제어 프레임 처리
 *    - trace / solar / force lock 상태 관리
 *    - force stop 시 trace / solar 강제 OFF
 *    - reinit 시 다시 동작 가능한 상태로 복귀
 */

#include "statemachine.h"

#include "adc.h"
#include "tim.h"
#include "usart.h"

#include "app_proto.h"
#include "can.h"
#include "trace.h"
#include "app_charger.h"

#include <stdio.h>
#include <string.h>

/* =========================================================
 * 외부 핸들
 * ========================================================= */
extern UART_HandleTypeDef huart6;

/* =========================================================
 * 내부 상태
 * ========================================================= */
static bool trace_flag    = 0;
static bool solar_flag    = 0;
static bool force_lock    = 0;
static bool auto_drive    = 0;  /* 자율주행 모드 중 1
                                 * → trace 수동 활성화 차단
                                 * → 자율주행 종료 후에야 trace ON 허용 */

/* UART6 1바이트 수신 */
static volatile uint8_t rxData[1];
static volatile bool bt6Flag = 0;
static uint8_t rxCmd = 0;

/* ADC value */
uint16_t adcValue[4];

/* =========================================================
 * 내부 helper
 * ========================================================= */

/* ---------------------------------------------------------
 * Trace ON / OFF 설정
 * ---------------------------------------------------------
 * 차단 조건:
 *   1) force_lock 중 ON 요청 무시
 *   2) auto_drive 중 ON 요청 무시 (자율주행 종료 후에야 허용)
 * OFF는 언제나 허용
 * --------------------------------------------------------- */
static void ST_SetTrace(bool on)
{
    if (on)
    {
        if (force_lock || auto_drive)
        {
            return;
        }

        trace_flag = 1;
    }
    else
    {
        trace_flag = 0;
        Trace_ForceInit();
    }
}

/* ---------------------------------------------------------
 * 자율주행 모드 ON / OFF
 * ---------------------------------------------------------
 * ON : Trace_ForceInit() 후 auto_drive = 1
 *      → trace 수동 제어 차단
 * OFF: auto_drive = 0
 *      → 이후 D키 또는 CAN TRACE_CTRL 로 trace 활성화 가능
 * --------------------------------------------------------- */
static void ST_SetAutoDrive(bool on)
{
    if (on)
    {
        if (force_lock)
        {
            return;
        }

        /* trace 트래커를 초기 위치로 복귀 후 자율주행 시작 */
        trace_flag = 0;
        Trace_ForceInit();
        auto_drive = 1;
    }
    else
    {
        auto_drive = 0;
        /* trace 활성화는 이후 수동으로만 가능 (자동으로 켜지지 않음) */
    }
}

/* ---------------------------------------------------------
 * Solar Charger ON / OFF 설정
 * ---------------------------------------------------------
 * force stop 중에는 ON 요청 무시
 * OFF는 언제나 허용
 *
 * 주의:
 *   ON으로 바뀌는 순간에만 Init 1회 수행
 *   OFF는 Stop 함수로 출력 확실히 차단
 * --------------------------------------------------------- */
static void ST_SetSolar(bool on)
{
    if (on)
    {
        if (force_lock)
        {
            return;
        }

        if (solar_flag == 0)
        {
            App_Charger_Init();
        }

        solar_flag = 1;
    }
    else
    {
        solar_flag = 0;
        App_Charger_Stop();
    }
}

/* ---------------------------------------------------------
 * Force Stop 진입
 * ---------------------------------------------------------
 * 정책
 *   - 추적 OFF
 *   - 충전 OFF
 *   - 일반 입력 무시
 *   - force reinit 전까지 lock 유지
 * --------------------------------------------------------- */
static void ST_EnterForceStop(void)
{
    /* 먼저 기능 OFF */
    ST_SetAutoDrive(false);
    ST_SetTrace(false);
    ST_SetSolar(false);

    /* 이후 lock */
    force_lock = 1;
}

/* ---------------------------------------------------------
 * Reinit
 * ---------------------------------------------------------
 * 정책
 *   - 잠금 해제
 *   - trace / solar 는 기본 OFF 유지
 *   - 추적/충전 모듈은 안전한 초기 상태로 복귀
 * --------------------------------------------------------- */
static void ST_ReinitAll(void)
{
    /* 내부 상태 기본값 */
    trace_flag = 0;
    solar_flag = 0;
    force_lock = 0;
    auto_drive = 0;

    /* 추적 모듈 초기 위치로 복귀 */
    Trace_ForceInit();

    /* 충전 모듈 재초기화 후 출력은 꺼둠
     * -----------------------------------
     * HwInit : I2C / 센서 / 타이머 완전 재초기화
     * Init   : PI 제어 / 상태머신 소프트 리셋
     * 실제 충전 시작은 solar_flag=1일 때만 Task가 돎
     */
    App_Charger_HwInit();
    App_Charger_Init();
    App_Charger_Stop();
}

/* ---------------------------------------------------------
 * CAN 명령 처리
 * --------------------------------------------------------- */
static void ST_HandleCanControl(uint8_t cmd, uint8_t value)
{
    switch (cmd)
    {
        case APP_CMD_TRACE_CTRL:
            ST_SetTrace(value == APP_CTRL_ON);
            break;

        case APP_CMD_SOLAR_CTRL:
            ST_SetSolar(value == APP_CTRL_ON);
            break;

        case APP_CMD_DRIVE_CTRL:
            ST_SetAutoDrive(value == APP_CTRL_ON);
            break;

        case APP_CMD_FORCE_CTRL:
            if (value == APP_FORCE_STOP)
            {
                ST_EnterForceStop();
            }
            else if (value == APP_FORCE_REINIT)
            {
                ST_ReinitAll();
            }
            break;

        default:
            break;
    }
}


/* =========================================================
 * INIT
 * ========================================================= */
void STMACHINE_Init(void)
{
    /* UART6 수신 인터럽트 시작 */
    HAL_UART_Receive_IT(&huart6, (uint8_t *)rxData, 1);

    /* ADC DMA 시작 */
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adcValue, 4);



    /* Trace servo PWM 시작 */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);


    /* 내부 상태 기본값 */
    trace_flag = 0;
    solar_flag = 0;
    force_lock = 0;
    auto_drive = 0;

    Trace_ForceInit();

    /* 하드웨어 초기화 (1회): I2C / INA219 / TIM4 / TIM10 */
    App_Charger_HwInit();
    /* 소프트 리셋: PI 제어 / 상태머신 초기화 후 출력 꺼둠 */
    App_Charger_Init();
    App_Charger_Stop();
}

/* =========================================================
 * UART RX CALLBACK
 * ---------------------------------------------------------
 * 주의:
 *   프로젝트 전체에 HAL_UART_RxCpltCallback가
 *   이미 다른 파일에도 있으면 중복 정의 에러가 납니다.
 *   그 경우 이 함수는 지우고 main.c에서 라우팅하십시오.
 * ========================================================= */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == NULL)
    {
        return;
    }

    if (huart->Instance == USART6)
    {
        rxCmd = rxData[0];
        bt6Flag = 1;

        HAL_UART_Receive_IT(&huart6, (uint8_t *)rxData, 1);
    }
}

void SHOW_UART6_TRACE(void)
{
    static uint32_t trace_uart6_prevTick = 0;
    static char msgBuf[160];
    uint32_t now = HAL_GetTick();
    int len;

    if ((now - trace_uart6_prevTick) < 1000U)
    {
        return;
    }

    if (huart6.gState != HAL_UART_STATE_READY)
    {
        return;
    }

    trace_uart6_prevTick = now;

    len = sprintf(msgBuf,
                  "[BT_TRACE] ADC:%d|%d|%d|%d X:%d|Y:%d TRACE:%s SOLAR:%s FORCE:%s AUTO:%s\r\n",
                  S1, S2, S3, S4,
                  error_x, error_y,
                  trace_flag ? "ON" : "OFF",
                  solar_flag ? "ON" : "OFF",
                  force_lock ? "ON" : "OFF",
                  auto_drive ? "ON" : "OFF");

    HAL_UART_Transmit_DMA(&huart6, (uint8_t *)msgBuf, (uint16_t)len);
}

/* =========================================================
 * getter
 * ========================================================= */
bool ST_GetSolarFlag(void)
{
    return solar_flag;
}

bool ST_GetTraceFlag(void)
{
    return trace_flag;
}

bool ST_GetForceLock(void)
{
    return force_lock;
}

/* =========================================================
 * ST MACHINE
 * ========================================================= */
void ST_MACHINE(void)
{
    /* 0 CAN 수신 확인 */
        Can_Task();
        CanFrame_t rx;

	/* 1 CAN 수신 변환 및 Auto, Force 확인 */

    if (Can_ReadFrame(&rx) != 0U)
    {
        if ((rx.id == APP_CAN_ID_CTRL) && (rx.dlc >= 2U))
        {
            ST_HandleCanControl(rx.data[0], rx.data[1]);
        }
    }

	/* 2 Solar_Charge ON/OFF */
    if (solar_flag)
    {
        App_Charger_Task();
    }

	/* 3 Solar_Trace ON/OFF */
    Trace_Mode(trace_flag ? MODE_ACT : MODE_INIT);

    /* 4 충전 로그 출력 */
    SHOW_UART6_APP_CHARGER();
}

