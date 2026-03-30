/*
 * mcp2515.h
 *
 *  Created on: Mar 22, 2026
 *      Author: parkdoyoung
 */



#ifndef INC_MCP2515_H_
#define INC_MCP2515_H_

#include "main.h"

/* =========================================================
 * MCP2515 SPI 명령
 * ========================================================= */
#define MCP2515_CMD_RESET             0xC0U
#define MCP2515_CMD_READ              0x03U
#define MCP2515_CMD_WRITE             0x02U
#define MCP2515_CMD_BIT_MODIFY        0x05U
#define MCP2515_CMD_READ_STATUS       0xA0U
#define MCP2515_CMD_LOAD_TXB0         0x40U
#define MCP2515_CMD_RTS_TXB0          0x81U
#define MCP2515_CMD_READ_RXB0         0x90U

/* =========================================================
 * MCP2515 레지스터
 * ========================================================= */
#define MCP2515_REG_CANSTAT           0x0EU
#define MCP2515_REG_CANCTRL           0x0FU
#define MCP2515_REG_CNF3              0x28U
#define MCP2515_REG_CNF2              0x29U
#define MCP2515_REG_CNF1              0x2AU
#define MCP2515_REG_CANINTE           0x2BU
#define MCP2515_REG_CANINTF           0x2CU
#define MCP2515_REG_RXB0CTRL          0x60U
#define MCP2515_REG_RXB1CTRL          0x70U

/* =========================================================
 * mode / bit mask
 * ========================================================= */
#define MCP2515_REQOP_MASK            0xE0U
#define MCP2515_MODE_NORMAL           0x00U
#define MCP2515_MODE_SLEEP            0x20U
#define MCP2515_MODE_LOOPBACK         0x40U
#define MCP2515_MODE_LISTEN_ONLY      0x60U
#define MCP2515_MODE_CONFIG           0x80U

#define MCP2515_CANINTF_RX0IF         0x01U
#define MCP2515_CANINTE_RX0IE         0x01U

/* =========================================================
 * 내부 operation type
 * ========================================================= */
typedef enum
{
    MCP2515_OP_NONE = 0,
    MCP2515_OP_RESET,
    MCP2515_OP_READ_REG,
    MCP2515_OP_WRITE_REG,
    MCP2515_OP_BIT_MODIFY,
    MCP2515_OP_LOAD_TXB0,
    MCP2515_OP_RTS_TXB0,
    MCP2515_OP_READ_RXB0
} Mcp2515Op_t;

/* =========================================================
 * MCP2515 컨텍스트
 * ---------------------------------------------------------
 * SPI callback과 main loop에서 함께 접근하므로
 * busy / done / err는 volatile로 둔다.
 * ========================================================= */
typedef struct
{
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;

    volatile uint8_t busy;
    volatile uint8_t done;
    volatile uint8_t err;

    uint8_t tx_buf[16];
    uint8_t rx_buf[16];
    uint16_t len;

    Mcp2515Op_t op;
} Mcp2515_t;

/* =========================================================
 * 공개 함수
 * ========================================================= */
void Mcp2515_Init(Mcp2515_t *m,
                  SPI_HandleTypeDef *hspi,
                  GPIO_TypeDef *cs_port,
                  uint16_t cs_pin);

HAL_StatusTypeDef Mcp2515_StartReset(Mcp2515_t *m);
HAL_StatusTypeDef Mcp2515_StartReadReg(Mcp2515_t *m, uint8_t reg);
HAL_StatusTypeDef Mcp2515_StartWriteReg(Mcp2515_t *m, uint8_t reg, uint8_t value);
HAL_StatusTypeDef Mcp2515_StartBitModify(Mcp2515_t *m, uint8_t reg, uint8_t mask, uint8_t value);
HAL_StatusTypeDef Mcp2515_StartLoadTxb0Std(Mcp2515_t *m, uint16_t id, uint8_t dlc, const uint8_t *data);
HAL_StatusTypeDef Mcp2515_StartRtsTxb0(Mcp2515_t *m);
HAL_StatusTypeDef Mcp2515_StartReadRxb0(Mcp2515_t *m);

uint8_t Mcp2515_IsBusy(const Mcp2515_t *m);
uint8_t Mcp2515_HasError(const Mcp2515_t *m);
uint8_t Mcp2515_ConsumeDone(Mcp2515_t *m);
void Mcp2515_ClearError(Mcp2515_t *m);

const uint8_t *Mcp2515_GetRxBuf(const Mcp2515_t *m);

void Mcp2515_SpiTxRxCpltCallback(Mcp2515_t *m, SPI_HandleTypeDef *hspi);
void Mcp2515_SpiErrorCallback(Mcp2515_t *m, SPI_HandleTypeDef *hspi);

#endif /* INC_MCP2515_H_ */
