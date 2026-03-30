/*
 * app_proto.h
 *
 *  Created on: Mar 22, 2026
 *      Author: parkdoyoung
 */




#ifndef INC_APP_PROTO_H_
#define INC_APP_PROTO_H_

/* =========================================================
 * 응용 계층 CAN 프로토콜 정의
 *
 * frame format
 *   id      : APP_CAN_ID_CTRL
 *   data[0] : cmd
 *   data[1] : value
 * ========================================================= */
#define APP_CAN_ID_CTRL         0x123U

//#define APP_CMD_LED             0x01U
//
//#define APP_LED_OFF             0x00U
//#define APP_LED_ON              0x01U


/* cmd */
#define APP_CMD_TRACE_CTRL          0x10U
#define APP_CMD_SOLAR_CTRL          0x11U
#define APP_CMD_FORCE_CTRL          0x12U
#define APP_CMD_DRIVE_CTRL          0x13U   /* 자율주행 ON/OFF */

/* 일반 ON / OFF value */
#define APP_CTRL_OFF                0x00U
#define APP_CTRL_ON                 0x01U

/* force control value */
#define APP_FORCE_STOP              0x01U
#define APP_FORCE_REINIT            0x02U

#endif /* INC_APP_PROTO_H_ */
