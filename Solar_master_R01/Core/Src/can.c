/*
 * can.c
 *
 *  Created on: Mar 22, 2026
 *      Author: parkdoyoung
 */


#include "can.h"
#include "mcp2515.h"
#include "spi.h"
#include "gpio.h"

#include <stdio.h>
#include <string.h>

/* =========================================================
 * 핀 기본값
 * ========================================================= */
#ifndef CAN_CS_GPIO_Port
#define CAN_CS_GPIO_Port      GPIOB
#endif

#ifndef CAN_CS_Pin
#define CAN_CS_Pin            GPIO_PIN_12
#endif

#ifndef CAN_INT_GPIO_Port
#define CAN_INT_GPIO_Port     GPIOC
#endif

#ifndef CAN_INT_Pin
#define CAN_INT_Pin           GPIO_PIN_7
#endif

/* =========================================================
 * bit timing
 * ---------------------------------------------------------
 * 현재 확정값:
 *   MCP2515 crystal : 8MHz
 *   bitrate         : 125kbps
 * ========================================================= */
#define CAN_CNF1_VALUE                0x01U
#define CAN_CNF2_VALUE                0xBAU
#define CAN_CNF3_VALUE                0x03U

typedef enum
{
    CAN_ST_RESET_REQ = 0,
    CAN_ST_RESET_WAIT,
    CAN_ST_RESET_HOLD,

    CAN_ST_READ_CFG_REQ,
    CAN_ST_READ_CFG_WAIT,

    CAN_ST_WRITE_CNF1_REQ,
    CAN_ST_WRITE_CNF1_WAIT,
    CAN_ST_WRITE_CNF2_REQ,
    CAN_ST_WRITE_CNF2_WAIT,
    CAN_ST_WRITE_CNF3_REQ,
    CAN_ST_WRITE_CNF3_WAIT,

    CAN_ST_WRITE_RXB0_REQ,
    CAN_ST_WRITE_RXB0_WAIT,
    CAN_ST_WRITE_RXB1_REQ,
    CAN_ST_WRITE_RXB1_WAIT,

    CAN_ST_WRITE_CANINTF_REQ,
    CAN_ST_WRITE_CANINTF_WAIT,
    CAN_ST_WRITE_CANINTE_REQ,
    CAN_ST_WRITE_CANINTE_WAIT,

    CAN_ST_REQ_NORMAL_REQ,
    CAN_ST_REQ_NORMAL_WAIT,
    CAN_ST_CHECK_NORMAL_REQ,
    CAN_ST_CHECK_NORMAL_WAIT,

    CAN_ST_READY,

    CAN_ST_RX_READ_REQ,
    CAN_ST_RX_READ_WAIT,
    CAN_ST_RX_CLR_REQ,
    CAN_ST_RX_CLR_WAIT,

    CAN_ST_TX_LOAD_REQ,
    CAN_ST_TX_LOAD_WAIT,
    CAN_ST_TX_RTS_REQ,
    CAN_ST_TX_RTS_WAIT,

    CAN_ST_FAIL
} CanState_t;

typedef struct
{
    Mcp2515_t mcp;

    volatile uint8_t int_pending;

    uint8_t ready;
    uint8_t ready_logged;
    uint8_t fail_logged;

    uint32_t wait_tick;
    uint32_t retry_tick;

    CanState_t st;

    CanFrame_t tx_frame;
    uint8_t tx_pending;

    CanFrame_t rx_frame;
    uint8_t rx_new;
} CanCtx_t;

static CanCtx_t g_can;

/* =========================================================
 * 내부 helper
 * ========================================================= */
static void Can_ParseRxb0Frame(CanFrame_t *f)
{
    const uint8_t *rx;
    uint8_t dlc;

    if (f == NULL)
    {
        return;
    }

    rx = Mcp2515_GetRxBuf(&g_can.mcp);

    f->id = (uint16_t)(((uint16_t)rx[1] << 3) |
                       ((uint16_t)rx[2] >> 5));

    dlc = (uint8_t)(rx[5] & 0x0FU);
    if (dlc > 8U)
    {
        dlc = 8U;
    }

    f->dlc = dlc;
    memset(f->data, 0, sizeof(f->data));
    memcpy(f->data, &rx[6], dlc);
}

static void Can_SetFail(const char *msg)
{
    g_can.ready = 0U;

    if (g_can.fail_logged == 0U)
    {
        printf("[CAN][FAIL] %s\r\n", msg);
        g_can.fail_logged = 1U;
    }

    g_can.retry_tick = HAL_GetTick();
    g_can.st = CAN_ST_FAIL;
}

static uint8_t Can_WaitDoneOrFail(const char *msg)
{
    if (Mcp2515_HasError(&g_can.mcp) != 0U)
    {
        Can_SetFail(msg);
        return 0U;
    }

    if (Mcp2515_ConsumeDone(&g_can.mcp) == 0U)
    {
        return 0U;
    }

    return 1U;
}

/* =========================================================
 * 공개 함수
 * ========================================================= */
void Can_Init(void)
{
    memset(&g_can, 0, sizeof(g_can));

    Mcp2515_Init(&g_can.mcp, &hspi1, CAN_CS_GPIO_Port, CAN_CS_Pin);

    g_can.st = CAN_ST_RESET_REQ;
}

uint8_t Can_IsReady(void)
{
    return g_can.ready;
}

uint8_t Can_SendStd(uint16_t id, uint8_t dlc, const uint8_t *data)
{
    if ((data == NULL) || (dlc > 8U))
    {
        return 0U;
    }

    if (g_can.ready == 0U)
    {
        return 0U;
    }

    if (g_can.tx_pending != 0U)
    {
        return 0U;
    }

    g_can.tx_frame.id  = id;
    g_can.tx_frame.dlc = dlc;
    memset(g_can.tx_frame.data, 0, sizeof(g_can.tx_frame.data));
    memcpy(g_can.tx_frame.data, data, dlc);

    g_can.tx_pending = 1U;
    return 1U;
}

uint8_t Can_ReadFrame(CanFrame_t *out)
{
    if ((out == NULL) || (g_can.rx_new == 0U))
    {
        return 0U;
    }

    *out = g_can.rx_frame;
    g_can.rx_new = 0U;

    return 1U;
}

void Can_Task(void)
{
    const uint8_t *rx;

    switch (g_can.st)
    {
        case CAN_ST_RESET_REQ:
            if (Mcp2515_StartReset(&g_can.mcp) == HAL_OK)
            {
                g_can.st = CAN_ST_RESET_WAIT;
            }
            break;

        case CAN_ST_RESET_WAIT:
            if (Can_WaitDoneOrFail("reset spi error") != 0U)
            {
                g_can.wait_tick = HAL_GetTick();
                g_can.st = CAN_ST_RESET_HOLD;
            }
            break;

        case CAN_ST_RESET_HOLD:
            if ((HAL_GetTick() - g_can.wait_tick) >= 2U)
            {
                g_can.st = CAN_ST_READ_CFG_REQ;
            }
            break;

        case CAN_ST_READ_CFG_REQ:
            if (Mcp2515_StartReadReg(&g_can.mcp, MCP2515_REG_CANSTAT) == HAL_OK)
            {
                g_can.st = CAN_ST_READ_CFG_WAIT;
            }
            break;

        case CAN_ST_READ_CFG_WAIT:
            if (Can_WaitDoneOrFail("read canstat error") != 0U)
            {
                rx = Mcp2515_GetRxBuf(&g_can.mcp);

                if ((rx[2] & MCP2515_REQOP_MASK) != MCP2515_MODE_CONFIG)
                {
                    Can_SetFail("config mode check fail");
                }
                else
                {
                    g_can.st = CAN_ST_WRITE_CNF1_REQ;
                }
            }
            break;

        case CAN_ST_WRITE_CNF1_REQ:
            if (Mcp2515_StartWriteReg(&g_can.mcp, MCP2515_REG_CNF1, CAN_CNF1_VALUE) == HAL_OK)
            {
                g_can.st = CAN_ST_WRITE_CNF1_WAIT;
            }
            break;

        case CAN_ST_WRITE_CNF1_WAIT:
            if (Can_WaitDoneOrFail("write cnf1 error") != 0U)
            {
                g_can.st = CAN_ST_WRITE_CNF2_REQ;
            }
            break;

        case CAN_ST_WRITE_CNF2_REQ:
            if (Mcp2515_StartWriteReg(&g_can.mcp, MCP2515_REG_CNF2, CAN_CNF2_VALUE) == HAL_OK)
            {
                g_can.st = CAN_ST_WRITE_CNF2_WAIT;
            }
            break;

        case CAN_ST_WRITE_CNF2_WAIT:
            if (Can_WaitDoneOrFail("write cnf2 error") != 0U)
            {
                g_can.st = CAN_ST_WRITE_CNF3_REQ;
            }
            break;

        case CAN_ST_WRITE_CNF3_REQ:
            if (Mcp2515_StartWriteReg(&g_can.mcp, MCP2515_REG_CNF3, CAN_CNF3_VALUE) == HAL_OK)
            {
                g_can.st = CAN_ST_WRITE_CNF3_WAIT;
            }
            break;

        case CAN_ST_WRITE_CNF3_WAIT:
            if (Can_WaitDoneOrFail("write cnf3 error") != 0U)
            {
                g_can.st = CAN_ST_WRITE_RXB0_REQ;
            }
            break;

        case CAN_ST_WRITE_RXB0_REQ:
            /* RXM=11 : filter/mask 무시, 모든 frame 수신 허용 */
            if (Mcp2515_StartWriteReg(&g_can.mcp, MCP2515_REG_RXB0CTRL, 0x60U) == HAL_OK)
            {
                g_can.st = CAN_ST_WRITE_RXB0_WAIT;
            }
            break;

        case CAN_ST_WRITE_RXB0_WAIT:
            if (Can_WaitDoneOrFail("write rxb0ctrl error") != 0U)
            {
                g_can.st = CAN_ST_WRITE_RXB1_REQ;
            }
            break;

        case CAN_ST_WRITE_RXB1_REQ:
            if (Mcp2515_StartWriteReg(&g_can.mcp, MCP2515_REG_RXB1CTRL, 0x60U) == HAL_OK)
            {
                g_can.st = CAN_ST_WRITE_RXB1_WAIT;
            }
            break;

        case CAN_ST_WRITE_RXB1_WAIT:
            if (Can_WaitDoneOrFail("write rxb1ctrl error") != 0U)
            {
                g_can.st = CAN_ST_WRITE_CANINTF_REQ;
            }
            break;

        case CAN_ST_WRITE_CANINTF_REQ:
            if (Mcp2515_StartWriteReg(&g_can.mcp, MCP2515_REG_CANINTF, 0x00U) == HAL_OK)
            {
                g_can.st = CAN_ST_WRITE_CANINTF_WAIT;
            }
            break;

        case CAN_ST_WRITE_CANINTF_WAIT:
            if (Can_WaitDoneOrFail("clear canintf error") != 0U)
            {
                g_can.st = CAN_ST_WRITE_CANINTE_REQ;
            }
            break;

        case CAN_ST_WRITE_CANINTE_REQ:
            if (Mcp2515_StartWriteReg(&g_can.mcp,
                                      MCP2515_REG_CANINTE,
                                      MCP2515_CANINTE_RX0IE) == HAL_OK)
            {
                g_can.st = CAN_ST_WRITE_CANINTE_WAIT;
            }
            break;

        case CAN_ST_WRITE_CANINTE_WAIT:
            if (Can_WaitDoneOrFail("write caninte error") != 0U)
            {
                g_can.st = CAN_ST_REQ_NORMAL_REQ;
            }
            break;

        case CAN_ST_REQ_NORMAL_REQ:
            if (Mcp2515_StartBitModify(&g_can.mcp,
                                       MCP2515_REG_CANCTRL,
                                       MCP2515_REQOP_MASK,
                                       MCP2515_MODE_NORMAL) == HAL_OK)
            {
                g_can.st = CAN_ST_REQ_NORMAL_WAIT;
            }
            break;

        case CAN_ST_REQ_NORMAL_WAIT:
            if (Can_WaitDoneOrFail("req normal error") != 0U)
            {
                g_can.st = CAN_ST_CHECK_NORMAL_REQ;
            }
            break;

        case CAN_ST_CHECK_NORMAL_REQ:
            if (Mcp2515_StartReadReg(&g_can.mcp, MCP2515_REG_CANSTAT) == HAL_OK)
            {
                g_can.st = CAN_ST_CHECK_NORMAL_WAIT;
            }
            break;

        case CAN_ST_CHECK_NORMAL_WAIT:
            if (Can_WaitDoneOrFail("check normal error") != 0U)
            {
                rx = Mcp2515_GetRxBuf(&g_can.mcp);

                if ((rx[2] & MCP2515_REQOP_MASK) != MCP2515_MODE_NORMAL)
                {
                    Can_SetFail("normal mode check fail");
                }
                else
                {
                    g_can.ready = 1U;
                    g_can.ready_logged = 0U;
                    g_can.fail_logged = 0U;
                    g_can.st = CAN_ST_READY;
                }
            }
            break;

        case CAN_ST_READY:
            if (g_can.ready_logged == 0U)
            {
                printf("[CAN] READY canstat=0x00\r\n");
                g_can.ready_logged = 1U;
            }

            if ((g_can.int_pending != 0U) ||
                (HAL_GPIO_ReadPin(CAN_INT_GPIO_Port, CAN_INT_Pin) == GPIO_PIN_RESET))
            {
                g_can.int_pending = 0U;
                g_can.st = CAN_ST_RX_READ_REQ;
            }
            else if (g_can.tx_pending != 0U)
            {
                g_can.st = CAN_ST_TX_LOAD_REQ;
            }
            break;

        case CAN_ST_RX_READ_REQ:
            if (Mcp2515_StartReadRxb0(&g_can.mcp) == HAL_OK)
            {
                g_can.st = CAN_ST_RX_READ_WAIT;
            }
            break;

        case CAN_ST_RX_READ_WAIT:
            if (Can_WaitDoneOrFail("read rxb0 error") != 0U)
            {
                Can_ParseRxb0Frame(&g_can.rx_frame);
                g_can.rx_new = 1U;
                g_can.st = CAN_ST_RX_CLR_REQ;
            }
            break;

        case CAN_ST_RX_CLR_REQ:
            if (Mcp2515_StartBitModify(&g_can.mcp,
                                       MCP2515_REG_CANINTF,
                                       MCP2515_CANINTF_RX0IF,
                                       0x00U) == HAL_OK)
            {
                g_can.st = CAN_ST_RX_CLR_WAIT;
            }
            break;

        case CAN_ST_RX_CLR_WAIT:
            if (Can_WaitDoneOrFail("clear rx0if error") != 0U)
            {
                g_can.st = CAN_ST_READY;
            }
            break;

        case CAN_ST_TX_LOAD_REQ:
            if (Mcp2515_StartLoadTxb0Std(&g_can.mcp,
                                         g_can.tx_frame.id,
                                         g_can.tx_frame.dlc,
                                         g_can.tx_frame.data) == HAL_OK)
            {
                g_can.st = CAN_ST_TX_LOAD_WAIT;
            }
            break;

        case CAN_ST_TX_LOAD_WAIT:
            if (Can_WaitDoneOrFail("load txb0 error") != 0U)
            {
                g_can.st = CAN_ST_TX_RTS_REQ;
            }
            break;

        case CAN_ST_TX_RTS_REQ:
            if (Mcp2515_StartRtsTxb0(&g_can.mcp) == HAL_OK)
            {
                g_can.st = CAN_ST_TX_RTS_WAIT;
            }
            break;

        case CAN_ST_TX_RTS_WAIT:
            if (Can_WaitDoneOrFail("rts txb0 error") != 0U)
            {
                g_can.tx_pending = 0U;
                g_can.st = CAN_ST_READY;
            }
            break;

        case CAN_ST_FAIL:
            if ((HAL_GetTick() - g_can.retry_tick) >= 1000U)
            {
                Mcp2515_ClearError(&g_can.mcp);

                g_can.int_pending  = 0U;
                g_can.ready        = 0U;
                g_can.ready_logged = 0U;
                g_can.fail_logged  = 0U;
                g_can.tx_pending   = 0U;
                g_can.rx_new       = 0U;

                g_can.st = CAN_ST_RESET_REQ;
            }
            break;

        default:
            Can_SetFail("invalid state");
            break;
    }
}

void Can_SpiTxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    Mcp2515_SpiTxRxCpltCallback(&g_can.mcp, hspi);
}

void Can_SpiErrorCallback(SPI_HandleTypeDef *hspi)
{
    Mcp2515_SpiErrorCallback(&g_can.mcp, hspi);
}

void Can_ExtiCallback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == CAN_INT_Pin)
    {
        g_can.int_pending = 1U;
    }
}
