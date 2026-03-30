



/*
 * statemachine.c
 *
 *  Created on: Mar 1, 2026
 *      Author: appletea
 *
 *  [Main role]
 *  ---------------------------------------------------------
 *  리모컨 UART1 수신과 주행 분배만 담당
 *
 *  역할
 *    - UART1 1바이트 수신
 *    - app_bms에 토글 명령 전달
 *    - manual 모드면 safe_drive_manual 호출
 *    - auto 모드면 safe_drive_auto 호출
 */

#include "statemachine.h"

#include "app_bms.h"
#include "app_proto.h"
#include "safe_drive.h"
#include "safe_drive_manual.h"
#include "tim.h"
#include "usart.h"
#include "speed.h"
#include "direction.h"
#include "car.h"

extern UART_HandleTypeDef huart1;

/* =========================================================
 * UART1 수신 상태
 * ========================================================= */
static volatile uint8_t rx_data[1];
static volatile uint8_t rx_flag = 0U;
static uint8_t rx_cmd = 0U;


/* =========================================================
 * 초기화
 * ========================================================= */
void STMACHINE_Init(void)
{
    /* UART1 1바이트 수신 시작 */
    HAL_UART_Receive_IT(&huart1, (uint8_t *)rx_data, 1);

    /* 초음파 초기화 */
    Ultrasonic_Init();

    /* speed 모듈에 TIM2 채널 연결 */
    Speed_Init(&htim2, TIM_CHANNEL_1, &htim2, TIM_CHANNEL_2);

    /* PWM 시작 + 초기 STOP */
    Car_Init();

    /* auto 모드 상태 초기화 */
    SafeDriveAuto_Init();
}

/* =========================================================
 * UART1 수신 콜백
 * ---------------------------------------------------------
 * 리모컨 문자 수신 시
 * 1) 내부 버퍼 저장
 * 2) app_bms에 알림
 * 3) 다음 수신 재시작
 * ========================================================= */
void STMACHINE_UartRxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == NULL)
    {
        return;
    }

    if (huart->Instance == USART1)
    {
        rx_cmd = rx_data[0];
        rx_flag = 1U;

        /* app_bms에 토글 명령 전달 */
        App_Bms_NotifyRemoteCmd(rx_cmd);

        /* 다음 수신 재시작 */
        HAL_UART_Receive_IT(&huart1, (uint8_t *)rx_data, 1);
    }
}


/* =========================================================
 * 외부 명령 주입
 * ---------------------------------------------------------
 * UART2(moserial) 등 UART1 이외 경로에서 명령을 넣을 때 사용
 * ========================================================= */
void STMACHINE_SubmitCmd(uint8_t cmd)
{
    rx_cmd = cmd;
    rx_flag = 1U;
    App_Bms_NotifyRemoteCmd(cmd);
}

/* =========================================================
 * 기존 호환용
 * ---------------------------------------------------------
 * A로 auto 진입 시 auto 상태를 scan부터 시작하도록 맞춘다.
 * manual 복귀 시 즉시 정지시켜 잔류 명령을 막는다.
 * ========================================================= */
void ST_FLAG(uint8_t cmd)
{
    if (cmd == 'A')
    {
        if (App_Bms_IsAutoMode() != 0U)
        {
            SafeDriveAuto_Init();
        }
        else
        {
            SafeDrive_Stop();
        }
    }
}

/* =========================================================
 * 상위 task
 * ---------------------------------------------------------
 * 1) UART1 새 문자 처리
 * 2) manual 모드면 manual safe drive 처리
 * 3) auto 모드면 auto safe drive 처리
 * ========================================================= */
void ST_MACHINE(void)
{
    if (rx_flag != 0U)
    {
        /* 필요 시 echo */
        HAL_UART_Transmit(&huart1, (uint8_t *)&rx_cmd, 1, 10);

        ST_FLAG(rx_cmd);

        if (App_Bms_IsManualMode() != 0U)
        {
            (void)SafeDriveManual_HandleCmd(rx_cmd);
        }

        rx_flag = 0U;
    }

    if (App_Bms_IsAutoMode() != 0U)
    {
        SafeDriveAuto_Task();
    }
}
