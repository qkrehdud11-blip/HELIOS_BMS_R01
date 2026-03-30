


#include "mcp2515.h"

#include <string.h>

/* =========================================================
 * 내부 helper
 * ========================================================= */
static void Mcp2515_CsLow(Mcp2515_t *m)
{
    HAL_GPIO_WritePin(m->cs_port, m->cs_pin, GPIO_PIN_RESET);
}

static void Mcp2515_CsHigh(Mcp2515_t *m)
{
    HAL_GPIO_WritePin(m->cs_port, m->cs_pin, GPIO_PIN_SET);
}

static HAL_StatusTypeDef Mcp2515_StartSpi(Mcp2515_t *m,
                                          Mcp2515Op_t op,
                                          const uint8_t *src,
                                          uint16_t len)
{
    if ((m == NULL) || (m->hspi == NULL) || (src == NULL) || (len == 0U) || (len > sizeof(m->tx_buf)))
    {
        return HAL_ERROR;
    }

    if (m->busy != 0U)
    {
        return HAL_BUSY;
    }

    memcpy(m->tx_buf, src, len);
    memset(m->rx_buf, 0, len);

    m->done = 0U;
    m->err  = 0U;
    m->busy = 1U;
    m->len  = len;
    m->op   = op;

    Mcp2515_CsLow(m);

    if (HAL_SPI_TransmitReceive_IT(m->hspi, m->tx_buf, m->rx_buf, len) != HAL_OK)
    {
        Mcp2515_CsHigh(m);
        m->busy = 0U;
        m->err  = 1U;
        m->op   = MCP2515_OP_NONE;
        return HAL_ERROR;
    }

    return HAL_OK;
}

/* =========================================================
 * 공개 함수
 * ========================================================= */
void Mcp2515_Init(Mcp2515_t *m,
                  SPI_HandleTypeDef *hspi,
                  GPIO_TypeDef *cs_port,
                  uint16_t cs_pin)
{
    if (m == NULL)
    {
        return;
    }

    memset(m, 0, sizeof(*m));

    m->hspi    = hspi;
    m->cs_port = cs_port;
    m->cs_pin  = cs_pin;
    m->op      = MCP2515_OP_NONE;

    if ((m->cs_port != NULL) && (m->cs_pin != 0U))
    {
        Mcp2515_CsHigh(m);
    }
}

HAL_StatusTypeDef Mcp2515_StartReset(Mcp2515_t *m)
{
    uint8_t tx[1] = {MCP2515_CMD_RESET};
    return Mcp2515_StartSpi(m, MCP2515_OP_RESET, tx, 1U);
}

HAL_StatusTypeDef Mcp2515_StartReadReg(Mcp2515_t *m, uint8_t reg)
{
    uint8_t tx[3] = {MCP2515_CMD_READ, reg, 0x00U};
    return Mcp2515_StartSpi(m, MCP2515_OP_READ_REG, tx, 3U);
}

HAL_StatusTypeDef Mcp2515_StartWriteReg(Mcp2515_t *m, uint8_t reg, uint8_t value)
{
    uint8_t tx[3] = {MCP2515_CMD_WRITE, reg, value};
    return Mcp2515_StartSpi(m, MCP2515_OP_WRITE_REG, tx, 3U);
}

HAL_StatusTypeDef Mcp2515_StartBitModify(Mcp2515_t *m, uint8_t reg, uint8_t mask, uint8_t value)
{
    uint8_t tx[4] = {MCP2515_CMD_BIT_MODIFY, reg, mask, value};
    return Mcp2515_StartSpi(m, MCP2515_OP_BIT_MODIFY, tx, 4U);
}

HAL_StatusTypeDef Mcp2515_StartLoadTxb0Std(Mcp2515_t *m,
                                           uint16_t id,
                                           uint8_t dlc,
                                           const uint8_t *data)
{
    uint8_t tx[14];
    uint8_t sidh;
    uint8_t sidl;

    if ((m == NULL) || (data == NULL) || (dlc > 8U))
    {
        return HAL_ERROR;
    }

    sidh = (uint8_t)((id >> 3) & 0xFFU);
    sidl = (uint8_t)((id & 0x07U) << 5);

    memset(tx, 0, sizeof(tx));

    tx[0] = MCP2515_CMD_LOAD_TXB0;
    tx[1] = sidh;
    tx[2] = sidl;
    tx[3] = 0x00U;
    tx[4] = 0x00U;
    tx[5] = (uint8_t)(dlc & 0x0FU);

    memcpy(&tx[6], data, dlc);

    return Mcp2515_StartSpi(m, MCP2515_OP_LOAD_TXB0, tx, (uint16_t)(6U + dlc));
}

HAL_StatusTypeDef Mcp2515_StartRtsTxb0(Mcp2515_t *m)
{
    uint8_t tx[1] = {MCP2515_CMD_RTS_TXB0};
    return Mcp2515_StartSpi(m, MCP2515_OP_RTS_TXB0, tx, 1U);
}

HAL_StatusTypeDef Mcp2515_StartReadRxb0(Mcp2515_t *m)
{
    uint8_t tx[14];

    if (m == NULL)
    {
        return HAL_ERROR;
    }

    memset(tx, 0, sizeof(tx));
    tx[0] = MCP2515_CMD_READ_RXB0;

    return Mcp2515_StartSpi(m, MCP2515_OP_READ_RXB0, tx, sizeof(tx));
}

uint8_t Mcp2515_IsBusy(const Mcp2515_t *m)
{
    if (m == NULL)
    {
        return 0U;
    }

    return m->busy;
}

uint8_t Mcp2515_HasError(const Mcp2515_t *m)
{
    if (m == NULL)
    {
        return 1U;
    }

    return m->err;
}

uint8_t Mcp2515_ConsumeDone(Mcp2515_t *m)
{
    if ((m == NULL) || (m->done == 0U))
    {
        return 0U;
    }

    m->done = 0U;
    return 1U;
}

void Mcp2515_ClearError(Mcp2515_t *m)
{
    if (m == NULL)
    {
        return;
    }

    m->err = 0U;
}

const uint8_t *Mcp2515_GetRxBuf(const Mcp2515_t *m)
{
    if (m == NULL)
    {
        return NULL;
    }

    return m->rx_buf;
}

void Mcp2515_SpiTxRxCpltCallback(Mcp2515_t *m, SPI_HandleTypeDef *hspi)
{
    if ((m == NULL) || (hspi != m->hspi))
    {
        return;
    }

    Mcp2515_CsHigh(m);

    m->busy = 0U;
    m->done = 1U;
}

void Mcp2515_SpiErrorCallback(Mcp2515_t *m, SPI_HandleTypeDef *hspi)
{
    if ((m == NULL) || (hspi != m->hspi))
    {
        return;
    }

    Mcp2515_CsHigh(m);

    m->busy = 0U;
    m->err  = 1U;
}
