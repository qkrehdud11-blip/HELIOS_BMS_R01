/*
 * app_can_control.c
 *
 *  Created on: Mar 23, 2026
 *      Author: parkdoyoung
 */


#include "app_can_control.h"

#include "app_proto.h"
#include "can.h"

/* =========================================================
 * 내부 helper
 * ---------------------------------------------------------
 * return
 *   0 : 송신 요청 성공
 *   1 : 실패
 * ========================================================= */
static uint8_t AppCan_SendCtrl(uint8_t cmd, uint8_t value)
{
    uint8_t data[2];

    data[0] = cmd;
    data[1] = value;

    /* -----------------------------------------------------
     * 기존 CAN 상위 함수 사용
     * -----------------------------------------------------
     * 프로젝트의 실제 함수명이 다르면 이 한 줄만 바꾸면 된다.
     * 예:
     *   if (Can_SendFrame(APP_CAN_ID_CTRL, data, 2U) != 0U)
     * ----------------------------------------------------- */

    return Can_SendStd(APP_CAN_ID_CTRL, 2U, data);
}

/* =========================================================
 * TRACE ON / OFF
 * ========================================================= */
uint8_t AppCan_SendTrace(uint8_t on)
{
    return AppCan_SendCtrl(APP_CMD_TRACE_CTRL,
                           (on != 0U) ? APP_CTRL_ON : APP_CTRL_OFF);
}

/* =========================================================
 * SOLAR ON / OFF
 * ========================================================= */
uint8_t AppCan_SendSolar(uint8_t on)
{
    return AppCan_SendCtrl(APP_CMD_SOLAR_CTRL,
                           (on != 0U) ? APP_CTRL_ON : APP_CTRL_OFF);
}

/* =========================================================
 * FORCE STOP
 * ========================================================= */
uint8_t AppCan_SendForceStop(void)
{
    return AppCan_SendCtrl(APP_CMD_FORCE_CTRL, APP_FORCE_STOP);
}

/* =========================================================
 * FORCE REINIT
 * ========================================================= */
uint8_t AppCan_SendForceReinit(void)
{
    return AppCan_SendCtrl(APP_CMD_FORCE_CTRL, APP_FORCE_REINIT);
}

/* =========================================================
 * DRIVE ON / OFF (자율주행 모드)
 * ========================================================= */
uint8_t AppCan_SendDrive(uint8_t on)
{
    return AppCan_SendCtrl(APP_CMD_DRIVE_CTRL,
                           (on != 0U) ? APP_CTRL_ON : APP_CTRL_OFF);
}
