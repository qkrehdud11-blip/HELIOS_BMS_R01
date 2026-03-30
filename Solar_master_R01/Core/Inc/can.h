/*
 * can.h
 *
 *  Created on: Mar 22, 2026
 *      Author: parkdoyoung
 */



#ifndef INC_CAN_H_
#define INC_CAN_H_

#include "main.h"

/* =========================================================
 * 표준 CAN frame
 * ========================================================= */
typedef struct
{
    uint16_t id;
    uint8_t  dlc;
    uint8_t  data[8];
} CanFrame_t;

/* =========================================================
 * 공개 함수
 * ========================================================= */
void Can_Init(void);
void Can_Task(void);
uint8_t Can_IsReady(void);

/* =========================================================
 * 표준 ID frame 송신 요청
 * ---------------------------------------------------------
 * 1 반환: 송신 요청 등록 성공
 * 0 반환: 아직 ready 아님 / 이전 송신 대기중 / 인자 오류
 * ========================================================= */
uint8_t Can_SendStd(uint16_t id, uint8_t dlc, const uint8_t *data);

/* =========================================================
 * 수신 frame 읽기
 * ---------------------------------------------------------
 * 새 frame이 있으면 out으로 복사 후 1 반환
 * 없으면 0 반환
 * ========================================================= */
uint8_t Can_ReadFrame(CanFrame_t *out);

/* =========================================================
 * HAL callback forwarding
 * ========================================================= */
void Can_SpiTxRxCpltCallback(SPI_HandleTypeDef *hspi);
void Can_SpiErrorCallback(SPI_HandleTypeDef *hspi);
void Can_ExtiCallback(uint16_t GPIO_Pin);

#endif /* INC_CAN_H_ */
