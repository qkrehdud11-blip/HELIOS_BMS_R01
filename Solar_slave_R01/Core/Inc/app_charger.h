/*
 * app_charger.h
 *
 *  Created on: 2026. 3. 20.
 *
 *  설명:
 *  ---------------------------------------------------------
 *  태양전지 충전 시스템 전용 상위 app 모듈
 *
 *  내부 포함 모듈:
 *    - solar_sensing
 *    - solar_pi_control
 *    - charger_state
 *
 *  역할:
 *    1) 충전 시스템 초기화
 *    2) 충전 시스템 주기 task 실행
 *    3) 1ms tick 누적
 *    4) UART2 DMA 기반 논블로킹 로그 출력
 *    5) HAL I2C / UART callback 라우팅
 *
 *  사용 방식:
 *    - main.c 에서 App_Charger_Init() 1회 호출
 *    - while(1) 에서 App_Charger_Task() 반복 호출
 *    - TIM10 1ms interrupt 에서 App_Charger_Tick1ms() 호출
 *    - HAL_I2C_MemRxCpltCallback() 에서
 *      App_Charger_I2CMemRxCpltCallback() 호출
 *    - HAL_I2C_ErrorCallback() 에서
 *      App_Charger_I2CErrorCallback() 호출
 *    - HAL_UART_TxCpltCallback() 에서
 *      App_Charger_UartTxCpltCallback() 호출
 *    - HAL_UART_ErrorCallback() 에서
 *      App_Charger_UartErrorCallback() 호출
 */

#ifndef __APP_CHARGER_H
#define __APP_CHARGER_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

/* 충전 시스템 하드웨어 초기화 (보드 전원 인가 후 1회만 호출)
 * ---------------------------------------------------------
 * I2C 리셋, INA219 센서 초기화, TIM4 PWM 시작, TIM10 타이머 시작
 * K 토글 시에는 호출하지 않는다.
 */
void App_Charger_HwInit(void);

/* 충전 시스템 소프트 리셋 (K ON 토글마다 호출)
 * ---------------------------------------------------------
 * PI 제어 / 충전 상태머신만 초기화한다.
 * 센서·타이머·I2C는 건드리지 않는다.
 * duty를 0으로 강제한 뒤 제어 루프가 다시 돌아가도록 준비한다.
 */
void App_Charger_Init(void);

void App_Charger_Stop(void);

/* 충전 시스템 메인 task */
void App_Charger_Task(void);

/* 1ms tick 누적 */
void App_Charger_Tick1ms(void);

/* UART2 DMA 논블로킹 문자열 로그
 * ---------------------------------------------------------
 * - 내부 큐에 문자열을 복사해 넣고
 * - UART2 DMA 송신이 비어 있으면 즉시 시작한다.
 * - 큐가 가득 차면 새 로그는 버려질 수 있다.
 *
 * 주의:
 * - ISR 안에서 과도하게 자주 호출하는 것은 권장하지 않는다.
 * - 일반적인 init / task 문맥에서 사용하는 것을 권장한다.
 */
void App_Charger_LogString(const char *str);

/* HAL I2C callback 라우터 */
void App_Charger_I2CMemRxCpltCallback(I2C_HandleTypeDef *hi2c);
void App_Charger_I2CErrorCallback(I2C_HandleTypeDef *hi2c);

/* HAL UART callback 라우터 */
void App_Charger_UartTxCpltCallback(UART_HandleTypeDef *huart);
void App_Charger_UartErrorCallback(UART_HandleTypeDef *huart);

/* UART6 전체 상태 출력, 1초 주기 (non-blocking DMA)
 * ---------------------------------------------------------
 * 출력 형식:
 *   [HH:MM:SS] [STATE:...][MODE:...][FAULT:...]
 *   Vpv=V Ipv=mA Ppv=W | Vbat=V Ichg=mA Pchg=W |
 *   Eff=% SOC=% Duty=
 */
void SHOW_UART6_APP_CHARGER(void);

#endif /* __APP_CHARGER_H */
