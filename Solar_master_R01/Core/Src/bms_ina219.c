


#include "bms_ina219.h"

/* =========================================================
 * INA219 address / registers
 * ========================================================= */
#define INA219_ADDR                  (0x40 << 1)

#define INA219_REG_CONFIG            0x00
#define INA219_REG_SHUNTVOLT         0x01
#define INA219_REG_BUSVOLT           0x02
#define INA219_REG_POWER             0x03
#define INA219_REG_CURRENT           0x04
#define INA219_REG_CALIB             0x05

/* =========================================================
 * threshold
 * ========================================================= */
#define CURRENT_WARNING              1500		// 원래값 1500
#define CURRENT_DANGER               2000		// 원래값 2000

#define VOLTAGE_UNDER_DANGER         5000
#define VOLTAGE_UNDER_WARNING        8000

#define VOLTAGE_OVER_WARNING         12000
#define VOLTAGE_OVER_DANGER          12500

/* =========================================================
 * state machine setting
 * ========================================================= */
#define INA219_BMS_UPDATE_PERIOD_MS  100U
#define INA219_BMS_TIMEOUT_MS        20U

/* =========================================================
 * internal run state
 * ========================================================= */
typedef enum
{
    INA219_BMS_RUN_INIT_CFG = 0,  /* CONFIG 레지스터 DMA 쓰기 시작 */
    INA219_BMS_RUN_WAIT_CFG,      /* CONFIG 쓰기 완료 대기 */
    INA219_BMS_RUN_INIT_CAL,      /* CALIB 레지스터 DMA 쓰기 시작 */
    INA219_BMS_RUN_WAIT_CAL,      /* CALIB 쓰기 완료 대기 */
    INA219_BMS_RUN_IDLE,          /* 주기 대기 */
    INA219_BMS_RUN_WAIT_BUS,      /* BUS 전압 읽기 대기 */
    INA219_BMS_RUN_WAIT_SHUNT     /* SHUNT 전압 읽기 대기 */
} INA219_BMS_RUN_STATE;

/* =========================================================
 * 내부 변수
 * ========================================================= */
static volatile uint8_t ina219_bms_dma_done = 0U;
static volatile uint8_t ina219_bms_dma_error = 0U;

static INA219_BMS_t s_bms = {
    .hi2c = &hi2c1,
    .addr = INA219_ADDR,
    .rshunt_mohm = 100   /* 0.1Ω = 100mΩ */
};

static INA219_BMS_RUN_STATE s_run = INA219_BMS_RUN_IDLE;

static uint8_t s_rx_buf[2];
static uint8_t s_tx_buf[2];  /* DMA 쓰기용 — 전송 완료까지 유효해야 하므로 static */
static uint8_t s_is_valid = 0U;

static uint16_t s_raw_bus = 0U;
static uint16_t s_raw_shunt = 0U;

static int32_t s_voltage_mV = -1;
static int32_t s_current_mA = -1;

static uint32_t s_prev_tick = 0U;
static uint32_t s_start_tick = 0U;

/* =========================================================
 * 내부 함수
 * ========================================================= */
static HAL_StatusTypeDef INA219_BMS_StartWriteDMA(uint8_t reg, uint16_t data);
static HAL_StatusTypeDef INA219_BMS_StartReadDMA(uint8_t reg);
static void INA219_BMS_ResetTransferFlags(void);
static void INA219_BMS_SetFault(void);

/* =========================================================
 * DMA reg write
 * ---------------------------------------------------------
 * s_tx_buf에 데이터를 채운 뒤 DMA 전송 시작.
 * 전송 완료는 MemTxCpltCallback에서 ina219_bms_dma_done으로 알림.
 * s_tx_buf는 전송 완료까지 유효해야 하므로 static 유지.
 * ========================================================= */
static HAL_StatusTypeDef INA219_BMS_StartWriteDMA(uint8_t reg, uint16_t data)
{
    s_tx_buf[0] = (uint8_t)((data >> 8) & 0xFFU);
    s_tx_buf[1] = (uint8_t)(data & 0xFFU);

    return HAL_I2C_Mem_Write_DMA(s_bms.hi2c,
                                 s_bms.addr,
                                 reg,
                                 I2C_MEMADD_SIZE_8BIT,
                                 s_tx_buf,
                                 2);
}

/* =========================================================
 * DMA read 시작
 * ========================================================= */
static HAL_StatusTypeDef INA219_BMS_StartReadDMA(uint8_t reg)
{
    return HAL_I2C_Mem_Read_DMA(s_bms.hi2c,
                                s_bms.addr,
                                reg,
                                I2C_MEMADD_SIZE_8BIT,
                                s_rx_buf,
                                2);
}

/* =========================================================
 * transfer flag 초기화
 * ========================================================= */
static void INA219_BMS_ResetTransferFlags(void)
{
    ina219_bms_dma_done = 0U;
    ina219_bms_dma_error = 0U;
}

/* =========================================================
 * fault 처리
 * ========================================================= */
static void INA219_BMS_SetFault(void)
{
    s_is_valid = 0U;
    s_voltage_mV = -1;
    s_current_mA = -1;
    s_run = INA219_BMS_RUN_INIT_CFG;  /* fault 후 재초기화부터 다시 */
    INA219_BMS_ResetTransferFlags();
}

/* =========================================================
 * Init
 * ---------------------------------------------------------
 * 상태머신을 INIT_CFG 상태로 진입시키고 즉시 반환 (non-blocking).
 * CONFIG / CALIB 레지스터 쓰기는 Task() 안에서 DMA로 수행됨.
 *  - 32V range, PGA ±320mV, 12-bit, continuous
 *  - Calibration = 4096  (Rshunt=0.1Ω, Current_LSB=100uA)
 * ========================================================= */
void INA219_BMS_Init(void)
{
    s_is_valid = 0U;
    s_raw_bus = 0U;
    s_raw_shunt = 0U;
    s_voltage_mV = -1;
    s_current_mA = -1;
    s_prev_tick = HAL_GetTick();
    s_start_tick = HAL_GetTick();

    INA219_BMS_ResetTransferFlags();

    s_run = INA219_BMS_RUN_INIT_CFG;
}

/* =========================================================
 * Task
 * ---------------------------------------------------------
 * non-blocking state machine
 * ========================================================= */
void INA219_BMS_Task(void)
{
    uint32_t now;
    HAL_StatusTypeDef st;
    int32_t vbus_mV;
    int32_t shunt_uV;

    now = HAL_GetTick();

    switch (s_run)
    {
        case INA219_BMS_RUN_INIT_CFG:
        {
            INA219_BMS_ResetTransferFlags();
            s_start_tick = now;

            if (INA219_BMS_StartWriteDMA(INA219_REG_CONFIG, 0x399FU) == HAL_OK)
            {
                s_run = INA219_BMS_RUN_WAIT_CFG;
            }
            return;
        }

        case INA219_BMS_RUN_WAIT_CFG:
        {
            if (ina219_bms_dma_error != 0U)
            {
                INA219_BMS_SetFault();
                return;
            }

            if ((now - s_start_tick) > INA219_BMS_TIMEOUT_MS)
            {
                INA219_BMS_SetFault();
                return;
            }

            if (ina219_bms_dma_done == 0U)
            {
                return;
            }

            INA219_BMS_ResetTransferFlags();
            s_start_tick = now;

            if (INA219_BMS_StartWriteDMA(INA219_REG_CALIB, 4096U) == HAL_OK)
            {
                s_run = INA219_BMS_RUN_WAIT_CAL;
            }
            return;
        }

        case INA219_BMS_RUN_INIT_CAL:
        {
            /* WAIT_CFG에서 직접 CALIB 시작하므로 여기는 통과하지 않음 */
            s_run = INA219_BMS_RUN_IDLE;
            return;
        }

        case INA219_BMS_RUN_WAIT_CAL:
        {
            if (ina219_bms_dma_error != 0U)
            {
                INA219_BMS_SetFault();
                return;
            }

            if ((now - s_start_tick) > INA219_BMS_TIMEOUT_MS)
            {
                INA219_BMS_SetFault();
                return;
            }

            if (ina219_bms_dma_done == 0U)
            {
                return;
            }

            INA219_BMS_ResetTransferFlags();
            s_prev_tick = now;
            s_run = INA219_BMS_RUN_IDLE;
            return;
        }

        case INA219_BMS_RUN_IDLE:
        {
            if ((now - s_prev_tick) < INA219_BMS_UPDATE_PERIOD_MS)
            {
                return;
            }

            s_prev_tick = now;
            s_start_tick = now;
            INA219_BMS_ResetTransferFlags();

            st = INA219_BMS_StartReadDMA(INA219_REG_BUSVOLT);
            if (st != HAL_OK)
            {
                INA219_BMS_SetFault();
                return;
            }

            s_run = INA219_BMS_RUN_WAIT_BUS;
            return;
        }

        case INA219_BMS_RUN_WAIT_BUS:
        {
            if (ina219_bms_dma_error != 0U)
            {
                INA219_BMS_SetFault();
                return;
            }

            if ((now - s_start_tick) > INA219_BMS_TIMEOUT_MS)
            {
                INA219_BMS_SetFault();
                return;
            }

            if (ina219_bms_dma_done == 0U)
            {
                return;
            }

            ina219_bms_dma_done = 0U;

            s_raw_bus = ((uint16_t)s_rx_buf[0] << 8) | (uint16_t)s_rx_buf[1];

            s_start_tick = now;
            st = INA219_BMS_StartReadDMA(INA219_REG_SHUNTVOLT);
            if (st != HAL_OK)
            {
                INA219_BMS_SetFault();
                return;
            }

            s_run = INA219_BMS_RUN_WAIT_SHUNT;
            return;
        }

        case INA219_BMS_RUN_WAIT_SHUNT:
        {
            if (ina219_bms_dma_error != 0U)
            {
                INA219_BMS_SetFault();
                return;
            }

            if ((now - s_start_tick) > INA219_BMS_TIMEOUT_MS)
            {
                INA219_BMS_SetFault();
                return;
            }

            if (ina219_bms_dma_done == 0U)
            {
                return;
            }

            ina219_bms_dma_done = 0U;

            s_raw_shunt = ((uint16_t)s_rx_buf[0] << 8) | (uint16_t)s_rx_buf[1];

            /* BUS register
             * bit[15:3] data, LSB = 4mV
             */
            vbus_mV = (int32_t)(s_raw_bus >> 3) * 4;

            /* SHUNT register
             * signed, LSB = 10uV
             */
            shunt_uV = (int32_t)((int16_t)s_raw_shunt) * 10;

            /* 측정 대상 전압 = IN+ = IN- + shunt */
            s_voltage_mV = vbus_mV + (shunt_uV / 1000);

            /* current(mA) = shunt_uV / rshunt_mohm */
            s_current_mA = shunt_uV / s_bms.rshunt_mohm;

            s_is_valid = 1U;
            s_run = INA219_BMS_RUN_IDLE;
            INA219_BMS_ResetTransferFlags();
            return;
        }

        default:
        {
            INA219_BMS_SetFault();
            return;
        }
    }
}

/* =========================================================
 * 최신 전압 반환
 * ---------------------------------------------------------
 * 새 I2C read를 시작하는 함수가 아니라
 * 마지막 측정값 반환 함수이다.
 * ========================================================= */
int32_t INA219_BMS_ReadVoltage_mV(void)
{
    return s_voltage_mV;
}

/* =========================================================
 * 최신 전류 반환
 * ========================================================= */
int32_t INA219_BMS_ReadCurrent_mA(void)
{
    return s_current_mA;
}

/* =========================================================
 * 데이터 유효 여부
 * ========================================================= */
uint8_t INA219_BMS_IsValid(void)
{
    return s_is_valid;
}

/* =========================================================
 * 전류 상태 판정
 * ========================================================= */
CURRENT_STATE INA219_BMS_GetCurrentState(int32_t current_mA)
{
    if (current_mA < 0)
    {
        return CURRENT_STATE_SAFE;
    }

    if (current_mA >= CURRENT_DANGER)
    {
        return CURRENT_STATE_DANGER;
    }

    if (current_mA >= CURRENT_WARNING)
    {
        return CURRENT_STATE_WARNING;
    }

    return CURRENT_STATE_SAFE;
}

/* =========================================================
 * 전압 상태 판정
 * ========================================================= */
VOLTAGE_STATE INA219_BMS_GetVoltageState(int32_t voltage_mV)
{
    if (voltage_mV <= 0)
    {
        return VOLTAGE_STATE_SAFE;
    }

    if (voltage_mV <= VOLTAGE_UNDER_DANGER)
    {
        return VOLTAGE_STATE_UNDER_DANGER;
    }

    if (voltage_mV <= VOLTAGE_UNDER_WARNING)
    {
        return VOLTAGE_STATE_UNDER_WARNING;
    }

    if (voltage_mV >= VOLTAGE_OVER_DANGER)
    {
        return VOLTAGE_STATE_OVER_DANGER;
    }

    if (voltage_mV >= VOLTAGE_OVER_WARNING)
    {
        return VOLTAGE_STATE_OVER_WARNING;
    }

    return VOLTAGE_STATE_SAFE;
}

/* =========================================================
 * HAL callback router
 * ---------------------------------------------------------
 * 사용자 전역 HAL callback 안에서 호출
 * ========================================================= */
void INA219_BMS_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == s_bms.hi2c)
    {
        ina219_bms_dma_done = 1U;
    }
}

void INA219_BMS_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == s_bms.hi2c)
    {
        ina219_bms_dma_error = 1U;
    }
}

void INA219_BMS_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == s_bms.hi2c)
    {
        ina219_bms_dma_done = 1U;
    }
}
