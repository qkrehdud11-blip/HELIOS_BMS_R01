/*
 * solar_sensing.c
 *
 *  Created on: 2026. 3. 20.
 */

#include "solar_sensing.h"
#include <math.h>
#include <string.h>

/* =========================================================
 * 내부 등록 테이블
 * ---------------------------------------------------------
 * HAL callback에서는 hi2c만 넘어오므로,
 * 어느 sensor 객체가 active 상태인지 찾기 위해 사용한다.
 * ========================================================= */
#define SOLAR_SENSING_MAX_INSTANCES         4U

static SolarSensing_t *g_solar_sensing_list[SOLAR_SENSING_MAX_INSTANCES] = {0};

/* =========================================================
 * 내부용: 센서 등록
 * ========================================================= */
static void SolarSensing_RegisterInstance(SolarSensing_t *sensor)
{
    uint32_t i;

    if (sensor == NULL)
    {
        return;
    }

    for (i = 0U; i < SOLAR_SENSING_MAX_INSTANCES; i++)
    {
        if (g_solar_sensing_list[i] == sensor)
        {
            return;
        }
    }

    for (i = 0U; i < SOLAR_SENSING_MAX_INSTANCES; i++)
    {
        if (g_solar_sensing_list[i] == NULL)
        {
            g_solar_sensing_list[i] = sensor;
            return;
        }
    }
}

/* =========================================================
 * 내부용: 해당 I2C에서 진행중인 sensor 찾기
 * ========================================================= */
static SolarSensing_t *SolarSensing_FindActiveByI2C(I2C_HandleTypeDef *hi2c)
{
    uint32_t i;

    if (hi2c == NULL)
    {
        return NULL;
    }

    for (i = 0U; i < SOLAR_SENSING_MAX_INSTANCES; i++)
    {
        if ((g_solar_sensing_list[i] != NULL) &&
            (g_solar_sensing_list[i]->hi2c == hi2c) &&
            (g_solar_sensing_list[i]->dma.busy != 0U))
        {
            return g_solar_sensing_list[i];
        }
    }

    return NULL;
}

/* =========================================================
 * 내부용 16-bit register write (blocking)
 * ========================================================= */
static HAL_StatusTypeDef SolarSensing_WriteReg16_Blocking(SolarSensing_t *sensor,
                                                          uint8_t reg,
                                                          uint16_t value)
{
    uint8_t data[2];

    if ((sensor == NULL) || (sensor->hi2c == NULL))
    {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef ret;

    data[0] = (uint8_t)((value >> 8) & 0xFFU);
    data[1] = (uint8_t)(value & 0xFFU);

    /* 블로킹 쓰기 중 I2C EV IRQ가 hdmatx(NULL)에 접근해 HardFault 방지 */
    __HAL_I2C_DISABLE_IT(sensor->hi2c, I2C_IT_EVT | I2C_IT_ERR | I2C_IT_BUF);
    ret = HAL_I2C_Mem_Write(sensor->hi2c,
                            (uint16_t)(sensor->address_7bit << 1),
                            reg,
                            I2C_MEMADD_SIZE_8BIT,
                            data,
                            2,
                            100);
    /* EVT/ERR 재활성화 하지 않음:
     * DMA 모드(HAL_I2C_Mem_Read_DMA)가 필요할 때 스스로 EVT|ERR를 켠다.
     * 여기서 켜두면 다음 I2CScanBus / IsDeviceReady 중 EV IRQ가 발화해
     * I2C 상태를 오염시켜 WriteReg16 실패 원인이 됨. */

    return ret;
}

/* =========================================================
 * 내부용 16-bit register read (blocking)
 * ========================================================= */
static HAL_StatusTypeDef SolarSensing_ReadReg16_Blocking(SolarSensing_t *sensor,
                                                         uint8_t reg,
                                                         uint16_t *value)
{
    uint8_t data[2];

    if ((sensor == NULL) || (sensor->hi2c == NULL) || (value == NULL))
    {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef rd;

    __HAL_I2C_DISABLE_IT(sensor->hi2c, I2C_IT_EVT | I2C_IT_ERR | I2C_IT_BUF);
    rd = HAL_I2C_Mem_Read(sensor->hi2c,
                          (uint16_t)(sensor->address_7bit << 1),
                          reg,
                          I2C_MEMADD_SIZE_8BIT,
                          data,
                          2,
                          100);
    /* EVT/ERR 재활성화 하지 않음:
     * DMA 모드(HAL_I2C_Mem_Read_DMA)가 필요할 때 스스로 EVT|ERR를 켠다.
     * 여기서 켜두면 다음 I2CScanBus / IsDeviceReady 중 EV IRQ가 발화해
     * I2C 상태를 오염시켜 WriteReg16 실패 원인이 됨. */

    if (rd != HAL_OK)
    {
        return HAL_ERROR;
    }

    *value = ((uint16_t)data[0] << 8) | (uint16_t)data[1];
    return HAL_OK;
}

/* =========================================================
 * 내부 상태 초기화
 * ========================================================= */
static void SolarSensing_ClearState(SolarSensing_t *sensor)
{
    if (sensor == NULL)
    {
        return;
    }

    memset(sensor, 0, sizeof(SolarSensing_t));

    sensor->scale.config_reg  = SOLAR_SENSING_DEFAULT_CONFIG;
    sensor->scale.calib_reg   = SOLAR_SENSING_CALIBRATION_DEFAULT;
    sensor->scale.r_shunt_ohm = SOLAR_SENSING_RSHUNT_OHM;

    sensor->status.last_status    = HAL_ERROR;
    sensor->status.last_i2c_error = HAL_I2C_ERROR_NONE;
    sensor->dma.stage             = SOLAR_SENSING_DMA_STAGE_IDLE;
}

/* =========================================================
 * raw 값으로부터 파생 물리량 계산
 * ---------------------------------------------------------
 * current:
 *   shunt voltage / r_shunt
 *
 * power:
 *   V * I
 * ========================================================= */
static void SolarSensing_UpdateDerivedRaw(SolarSensing_t *sensor)
{
    sensor->raw.source_v =
        sensor->raw.bus_v + (sensor->raw.shunt_mv * 0.001f);

    sensor->raw.current_ma =
        sensor->raw.shunt_mv / sensor->scale.r_shunt_ohm;

    sensor->raw.power_bus_w =
        sensor->raw.bus_v * (sensor->raw.current_ma / 1000.0f);

    sensor->raw.power_source_w =
        sensor->raw.source_v * (sensor->raw.current_ma / 1000.0f);
}

/* =========================================================
 * raw 값을 out 값으로 복사
 * ---------------------------------------------------------
 * solar 센서는 보통 raw -> out 그대로 사용한다.
 * battery 센서는 이후 BatteryFilterUpdate()가 out을 덮어쓴다.
 * ========================================================= */
static void SolarSensing_CopyRawToOutput(SolarSensing_t *sensor)
{
    if (sensor == NULL)
    {
        return;
    }

    sensor->out = sensor->raw;
}

/* =========================================================
 * 내부용: 물리 범위 valid 검사
 * ========================================================= */
static uint8_t SolarSensing_IsPhysicalRangeValid(const SolarSensing_t *sensor)
{
    if (sensor == NULL)
    {
        return 0U;
    }

    if ((sensor->raw.bus_v < 0.0f) || (sensor->raw.bus_v > 26.0f))
    {
        return 0U;
    }

    if (fabsf(sensor->raw.shunt_mv) > 320.0f)
    {
        return 0U;
    }

    if ((sensor->raw.source_v < 0.0f) || (sensor->raw.source_v > 26.5f))
    {
        return 0U;
    }

    return 1U;
}

/* =========================================================
 * 내부용: DMA read 시작
 * ========================================================= */
static HAL_StatusTypeDef SolarSensing_StartReadDMA(SolarSensing_t *sensor,
                                                   uint8_t reg,
                                                   SolarSensingDmaStage_t stage)
{
    HAL_StatusTypeDef st;

    if ((sensor == NULL) || (sensor->hi2c == NULL))
    {
        return HAL_ERROR;
    }

    sensor->dma.stage = stage;

    st = HAL_I2C_Mem_Read_DMA(sensor->hi2c,
                              (uint16_t)(sensor->address_7bit << 1),
                              reg,
                              I2C_MEMADD_SIZE_8BIT,
                              sensor->dma.rx_buf,
                              2);

    return st;
}

/* =========================================================
 * 내부용: DMA 수신 버퍼에서 SHUNT 해석
 * ========================================================= */
static void SolarSensing_DecodeShuntFromBuf(SolarSensing_t *sensor)
{
    uint16_t raw_reg;
    int16_t raw_signed;

    raw_reg = ((uint16_t)sensor->dma.rx_buf[0] << 8) |
              (uint16_t)sensor->dma.rx_buf[1];

    sensor->status.reg_shunt_raw = raw_reg;

    raw_signed = (int16_t)raw_reg;
    sensor->raw.shunt_mv = ((float)raw_signed) * SOLAR_SENSING_SHUNT_LSB_MV;
}

/* =========================================================
 * 내부용: DMA 수신 버퍼에서 BUS 해석
 * ========================================================= */
static void SolarSensing_DecodeBusFromBuf(SolarSensing_t *sensor)
{
    uint16_t raw_reg;

    raw_reg = ((uint16_t)sensor->dma.rx_buf[0] << 8) |
              (uint16_t)sensor->dma.rx_buf[1];

    sensor->status.reg_bus_raw = raw_reg;

    sensor->raw.bus_v = ((float)(raw_reg >> 3)) * SOLAR_SENSING_BUS_LSB_V;
}

/* =========================================================
 * 내부용: battery filter 결과를 out에 반영
 * ========================================================= */
static void SolarSensing_BatteryApplyFilterToOutput(SolarSensing_t *batt_sensor)
{
    batt_sensor->out.bus_v      = batt_sensor->batt_filter.v;
    batt_sensor->out.current_ma = batt_sensor->batt_filter.i;

    batt_sensor->out.shunt_mv =
        batt_sensor->out.current_ma * batt_sensor->scale.r_shunt_ohm;

    batt_sensor->out.source_v =
        batt_sensor->out.bus_v + (batt_sensor->out.shunt_mv * 0.001f);

    batt_sensor->out.power_bus_w =
        batt_sensor->out.bus_v * (batt_sensor->out.current_ma / 1000.0f);

    batt_sensor->out.power_source_w =
        batt_sensor->out.source_v * (batt_sensor->out.current_ma / 1000.0f);
}

/* =========================================================
 * 내부용: battery out 값 0으로 초기화
 * ========================================================= */
static void SolarSensing_BatteryApplyZeroOutput(SolarSensing_t *batt_sensor)
{
    batt_sensor->out.shunt_mv       = 0.0f;
    batt_sensor->out.bus_v          = 0.0f;
    batt_sensor->out.source_v       = 0.0f;
    batt_sensor->out.current_ma     = 0.0f;
    batt_sensor->out.power_bus_w    = 0.0f;
    batt_sensor->out.power_source_w = 0.0f;
}

/* =========================================================
 * 내부용: DMA update 정상 종료 처리
 * ========================================================= */
static void SolarSensing_FinalizeDMAUpdate(SolarSensing_t *sensor)
{
    SolarSensing_CopyRawToOutput(sensor);

    sensor->status.range_valid    = SolarSensing_IsPhysicalRangeValid(sensor);
    sensor->status.last_status    = HAL_OK;
    sensor->status.last_i2c_error = HAL_I2C_ERROR_NONE;

    sensor->dma.busy        = 0U;
    sensor->dma.update_done = 1U;
    sensor->dma.stage       = SOLAR_SENSING_DMA_STAGE_IDLE;
}

/* =========================================================
 * 내부용: DMA 에러 처리
 * ========================================================= */
static void SolarSensing_HandleDMAError(SolarSensing_t *sensor)
{
    if (sensor == NULL)
    {
        return;
    }

    sensor->status.range_valid = 0U;
    sensor->status.last_status = HAL_ERROR;

    if (sensor->hi2c != NULL)
    {
        sensor->status.last_i2c_error = sensor->hi2c->ErrorCode;
    }

    sensor->dma.busy        = 0U;
    sensor->dma.update_done = 0U;
    sensor->dma.stage       = SOLAR_SENSING_DMA_STAGE_IDLE;
}

/* =========================================================
 * SolarSensing_Init
 * ========================================================= */
HAL_StatusTypeDef SolarSensing_Init(SolarSensing_t *sensor,
                                    I2C_HandleTypeDef *hi2c,
                                    uint16_t addr_7bit)
{
    if ((sensor == NULL) || (hi2c == NULL))
    {
        return HAL_ERROR;
    }

    SolarSensing_ClearState(sensor);

    sensor->hi2c = hi2c;
    sensor->address_7bit = addr_7bit;

    if (SolarSensing_WriteReg16_Blocking(sensor,
                                         INA219_REG_CALIBRATION,
                                         sensor->scale.calib_reg) != HAL_OK)
    {
        sensor->status.last_status = HAL_ERROR;
        sensor->status.last_i2c_error = hi2c->ErrorCode;
        return HAL_ERROR;
    }

    if (SolarSensing_WriteReg16_Blocking(sensor,
                                         INA219_REG_CONFIG,
                                         sensor->scale.config_reg) != HAL_OK)
    {
        sensor->status.last_status = HAL_ERROR;
        sensor->status.last_i2c_error = hi2c->ErrorCode;
        return HAL_ERROR;
    }

    if (SolarSensing_ReadReg16_Blocking(sensor,
                                        INA219_REG_CONFIG,
                                        &sensor->status.reg_config_raw) != HAL_OK)
    {
        sensor->status.last_status = HAL_ERROR;
        sensor->status.last_i2c_error = hi2c->ErrorCode;
        return HAL_ERROR;
    }

    if (SolarSensing_ReadReg16_Blocking(sensor,
                                        INA219_REG_CALIBRATION,
                                        &sensor->status.reg_calib_raw) != HAL_OK)
    {
        sensor->status.last_status = HAL_ERROR;
        sensor->status.last_i2c_error = hi2c->ErrorCode;
        return HAL_ERROR;
    }

    SolarSensing_RegisterInstance(sensor);

    sensor->is_initialized         = 1U;
    sensor->status.range_valid     = 1U;
    sensor->status.last_status     = HAL_OK;
    sensor->status.last_i2c_error  = HAL_I2C_ERROR_NONE;

    return HAL_OK;
}

/* =========================================================
 * SolarSensing_StartUpdateDMA
 * ========================================================= */
HAL_StatusTypeDef SolarSensing_StartUpdateDMA(SolarSensing_t *sensor)
{
    HAL_StatusTypeDef st;

    if ((sensor == NULL) || (sensor->is_initialized == 0U))
    {
        return HAL_ERROR;
    }

    if (sensor->dma.busy != 0U)
    {
        return HAL_BUSY;
    }

    sensor->dma.busy        = 1U;
    sensor->dma.update_done = 0U;
    sensor->dma.stage       = SOLAR_SENSING_DMA_STAGE_IDLE;
    sensor->dma.start_tick  = HAL_GetTick();

    sensor->status.last_status    = HAL_BUSY;
    sensor->status.last_i2c_error = HAL_I2C_ERROR_NONE;

    st = SolarSensing_StartReadDMA(sensor,
                                   INA219_REG_SHUNT_VOLTAGE,
                                   SOLAR_SENSING_DMA_STAGE_SHUNT);

    if (st == HAL_BUSY)
    {
        sensor->dma.busy  = 0U;
        sensor->dma.stage = SOLAR_SENSING_DMA_STAGE_IDLE;
        sensor->status.last_status = HAL_BUSY;
        return HAL_BUSY;
    }

    if (st != HAL_OK)
    {
        SolarSensing_HandleDMAError(sensor);
        return HAL_ERROR;
    }

    return HAL_OK;
}

/* =========================================================
 * SolarSensing_Service
 * ========================================================= */
void SolarSensing_Service(SolarSensing_t *sensor)
{
    uint32_t now_tick;
    uint32_t elapsed;

    if ((sensor == NULL) || (sensor->hi2c == NULL))
    {
        return;
    }

    if (sensor->dma.busy == 0U)
    {
        return;
    }

    now_tick = HAL_GetTick();
    elapsed = now_tick - sensor->dma.start_tick;

    if (elapsed > SOLAR_SENSING_DMA_TIMEOUT_MS)
    {
        SolarSensing_HandleDMAError(sensor);
    }
}

/* =========================================================
 * SolarSensing_BatteryFilterUpdate
 *
 * 목적:
 *   battery 측 out 값을 안정화하고,
 *   invalid / recover debounce를 적용한다.
 *
 * 핵심:
 *   - 이상 샘플 1회로 즉시 range_valid=0 하지 않는다.
 *   - 이전 filter 값을 유지한다.
 *   - 연속 invalid 누적 후에만 invalid 확정
 *   - recover도 연속 정상 샘플 후에만 valid 복귀
 * ========================================================= */
void SolarSensing_BatteryFilterUpdate(SolarSensing_t *batt_sensor)
{
    float dv;
    uint8_t sample_invalid = 0U;

    if (batt_sensor == NULL)
    {
        return;
    }

    /* -----------------------------------------------------
     * 1) 이번 raw 샘플의 유효성 검사
     * ----------------------------------------------------- */
    if (batt_sensor->status.last_status != HAL_OK)
    {
        sample_invalid = 1U;
    }
    else if ((batt_sensor->raw.bus_v < SOLAR_SENSING_BATT_VALID_MIN_V) ||
             (batt_sensor->raw.bus_v > SOLAR_SENSING_BATT_VALID_MAX_V))
    {
        sample_invalid = 1U;
    }
    else if (batt_sensor->batt_filter.initialized != 0U)
    {
        dv = batt_sensor->raw.bus_v - batt_sensor->batt_filter.v;

        if (fabsf(dv) > SOLAR_SENSING_BATT_MAX_STEP_V)
        {
            sample_invalid = 1U;
        }
    }
    else
    {
        /* 첫 정상 샘플 전에는 step 검사하지 않음 */
    }

    /* -----------------------------------------------------
     * 2) invalid 샘플 처리
     * ----------------------------------------------------- */
    if (sample_invalid != 0U)
    {
        batt_sensor->batt_filter.valid_count = 0U;

        if (batt_sensor->batt_filter.invalid_count < 255U)
        {
            batt_sensor->batt_filter.invalid_count++;
        }

        /* 아직 필터 기준이 없으면 이전 값을 유지할 수 없으므로 0 출력 */
        if (batt_sensor->batt_filter.initialized == 0U)
        {
            batt_sensor->status.range_valid = 0U;
            SolarSensing_BatteryApplyZeroOutput(batt_sensor);
            return;
        }

        /* 이전 filter 값 유지 */
        SolarSensing_BatteryApplyFilterToOutput(batt_sensor);

        /* 연속 invalid가 충분히 누적될 때만 invalid 확정 */
        if (batt_sensor->batt_filter.invalid_count >=
            SOLAR_SENSING_BATT_INVALID_ASSERT_COUNT)
        {
            batt_sensor->status.range_valid = 0U;
        }

        return;
    }

    /* -----------------------------------------------------
     * 3) valid 샘플 처리
     * ----------------------------------------------------- */
    batt_sensor->batt_filter.invalid_count = 0U;

    if (batt_sensor->batt_filter.valid_count < 255U)
    {
        batt_sensor->batt_filter.valid_count++;
    }

    if (batt_sensor->batt_filter.initialized == 0U)
    {
        batt_sensor->batt_filter.v = batt_sensor->raw.bus_v;
        batt_sensor->batt_filter.i = batt_sensor->raw.current_ma;
        batt_sensor->batt_filter.initialized = 1U;
    }
    else
    {
        batt_sensor->batt_filter.v =
            (SOLAR_SENSING_BATT_ALPHA_V * batt_sensor->raw.bus_v) +
            ((1.0f - SOLAR_SENSING_BATT_ALPHA_V) * batt_sensor->batt_filter.v);

        batt_sensor->batt_filter.i =
            (SOLAR_SENSING_BATT_ALPHA_I * batt_sensor->raw.current_ma) +
            ((1.0f - SOLAR_SENSING_BATT_ALPHA_I) * batt_sensor->batt_filter.i);
    }

    SolarSensing_BatteryApplyFilterToOutput(batt_sensor);

    /* 이미 valid 상태였으면 유지 */
    if (batt_sensor->status.range_valid != 0U)
    {
        batt_sensor->status.range_valid = 1U;
        return;
    }

    /* invalid 상태였다면 recover debounce 후 valid 복귀 */
    if (batt_sensor->batt_filter.valid_count >=
        SOLAR_SENSING_BATT_VALID_RECOVER_COUNT)
    {
        batt_sensor->status.range_valid = 1U;
    }
    else
    {
        batt_sensor->status.range_valid = 0U;
    }
}

/* =========================================================
 * update_done 플래그를 안전하게 읽고 clear
 * ========================================================= */
uint8_t SolarSensing_FetchUpdateDone(SolarSensing_t *sensor)
{
    uint8_t done = 0U;

    if (sensor == NULL)
    {
        return 0U;
    }

    __disable_irq();

    done = sensor->dma.update_done;
    if (done != 0U)
    {
        sensor->dma.update_done = 0U;
    }

    __enable_irq();

    return done;
}

/* =========================================================
 * SolarSensing_GetSnapshot
 * ========================================================= */
void SolarSensing_GetSnapshot(const SolarSensing_t *sensor,
                              SolarSensingSnapshot_t *snapshot)
{
    if ((sensor == NULL) || (snapshot == NULL))
    {
        return;
    }

    __disable_irq();

    snapshot->shunt_mv       = sensor->out.shunt_mv;
    snapshot->bus_v          = sensor->out.bus_v;
    snapshot->source_v       = sensor->out.source_v;
    snapshot->current_ma     = sensor->out.current_ma;
    snapshot->power_bus_w    = sensor->out.power_bus_w;
    snapshot->power_source_w = sensor->out.power_source_w;

    snapshot->range_valid    = sensor->status.range_valid;
    snapshot->last_status    = sensor->status.last_status;

    __enable_irq();
}

/* =========================================================
 * SolarSensing_GetDebug
 * ========================================================= */
void SolarSensing_GetDebug(const SolarSensing_t *sensor,
                           SolarSensingDebug_t *debug_info)
{
    if ((sensor == NULL) || (debug_info == NULL))
    {
        return;
    }

    __disable_irq();

    debug_info->raw_bus_v          = sensor->raw.bus_v;
    debug_info->raw_source_v       = sensor->raw.source_v;
    debug_info->raw_current_ma     = sensor->raw.current_ma;

    debug_info->filt_v             = sensor->batt_filter.v;
    debug_info->filt_i             = sensor->batt_filter.i;

    debug_info->out_bus_v          = sensor->out.bus_v;
    debug_info->out_source_v       = sensor->out.source_v;
    debug_info->out_current_ma     = sensor->out.current_ma;

    debug_info->filter_initialized = sensor->batt_filter.initialized;
    debug_info->invalid_count      = sensor->batt_filter.invalid_count;
    debug_info->valid_count        = sensor->batt_filter.valid_count;

    debug_info->range_valid        = sensor->status.range_valid;
    debug_info->last_status        = sensor->status.last_status;
    debug_info->last_i2c_error     = sensor->status.last_i2c_error;

    __enable_irq();
}

/* =========================================================
 * SolarSensing_I2C_MemRxCpltCallback
 * ========================================================= */
void SolarSensing_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    SolarSensing_t *sensor;
    HAL_StatusTypeDef st;

    sensor = SolarSensing_FindActiveByI2C(hi2c);
    if (sensor == NULL)
    {
        return;
    }

    switch (sensor->dma.stage)
    {
        case SOLAR_SENSING_DMA_STAGE_SHUNT:
        {
            SolarSensing_DecodeShuntFromBuf(sensor);

            st = SolarSensing_StartReadDMA(sensor,
                                           INA219_REG_BUS_VOLTAGE,
                                           SOLAR_SENSING_DMA_STAGE_BUS);

            if (st != HAL_OK)
            {
                SolarSensing_HandleDMAError(sensor);
            }
            break;
        }

        case SOLAR_SENSING_DMA_STAGE_BUS:
        {
            SolarSensing_DecodeBusFromBuf(sensor);
            SolarSensing_UpdateDerivedRaw(sensor);
            SolarSensing_FinalizeDMAUpdate(sensor);
            break;
        }

        case SOLAR_SENSING_DMA_STAGE_IDLE:
        default:
        {
            SolarSensing_HandleDMAError(sensor);
            break;
        }
    }
}

/* =========================================================
 * SolarSensing_I2C_ErrorCallback
 * ========================================================= */
void SolarSensing_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    SolarSensing_t *sensor;

    sensor = SolarSensing_FindActiveByI2C(hi2c);
    if (sensor == NULL)
    {
        return;
    }

    SolarSensing_HandleDMAError(sensor);
}
