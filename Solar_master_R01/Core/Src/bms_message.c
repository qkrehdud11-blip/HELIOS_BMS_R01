#include "bms_message.h"
#include <stdio.h>
#include <string.h>

#include "app_bms.h"

/* 외부 UART 핸들 */
extern UART_HandleTypeDef huart6;

/* =========================================================
 * 로그 설정
 * ========================================================= */
#define BMS_UART_LOG_PERIOD_MS      1000U
#define BMS_UART6_MSG_SIZE          256U

/* =========================================================
 * ADC 표시용 설정
 * ---------------------------------------------------------
 * 가스 ADC를 전압으로 바꿀 때 사용
 * ========================================================= */
#define BMS_ADC_MAX_VALUE           4095U
#define BMS_ADC_VREF_mV             3300U

/* =========================================================
 * DMA TX 버퍼 / busy flag
 * ========================================================= */
static char uart6_msg[BMS_UART6_MSG_SIZE];

static volatile uint8_t uart6_tx_busy = 0U;

/* =========================================================
 * 내부 유틸
 * ========================================================= */

/* snprintf 결과를 실제 버퍼 길이 안으로 제한 */
static uint16_t BMS_MESSAGE_GetSafeLen(int len, uint16_t buf_size)
{
    if (len <= 0)
    {
        return 0U;
    }

    if (len >= (int)buf_size)
    {
        return (uint16_t)(buf_size - 1U);
    }

    return (uint16_t)len;
}

/* ADC 값을 mV로 변환 */
static uint32_t BMS_MESSAGE_AdcTo_mV(uint16_t adc)
{
    return ((uint32_t)adc * BMS_ADC_VREF_mV + (BMS_ADC_MAX_VALUE / 2U)) / BMS_ADC_MAX_VALUE;
}

/* baseline 대비 가스 변화율 [%] 계산
 * ---------------------------------------------------------
 * ppm이 아니라 baseline 대비 상대 변화율이다.
 */
static int32_t BMS_MESSAGE_GetGasChangePct(uint32_t now_mv, uint32_t base_mv)
{
    int64_t diff;

    if (base_mv == 0U)
    {
        return 0;
    }

    diff = (int64_t)now_mv - (int64_t)base_mv;

    return (int32_t)((diff * 100LL) / (int64_t)base_mv);
}

/* mV -> "x.xxxV" */
static void BMS_MESSAGE_Format_mV(char *buf, uint16_t buf_size, int32_t mv)
{
    int32_t abs_mv;

    if ((buf == NULL) || (buf_size == 0U))
    {
        return;
    }

    abs_mv = (mv < 0) ? -mv : mv;

    if (mv < 0)
    {
        (void)snprintf(buf, buf_size,
                       "-%ld.%03ldV",
                       (long)(abs_mv / 1000),
                       (long)(abs_mv % 1000));
    }
    else
    {
        (void)snprintf(buf, buf_size,
                       "%ld.%03ldV",
                       (long)(abs_mv / 1000),
                       (long)(abs_mv % 1000));
    }
}

/* 전류 표시
 * ---------------------------------------------------------
 * 1000mA 미만은 mA
 * 1000mA 이상은 A
 */
static void BMS_MESSAGE_Format_Current(char *buf, uint16_t buf_size, int32_t ma)
{
    int32_t abs_ma;

    if ((buf == NULL) || (buf_size == 0U))
    {
        return;
    }

    abs_ma = (ma < 0) ? -ma : ma;

    if (abs_ma < 1000)
    {
        (void)snprintf(buf, buf_size, "%ldmA", (long)ma);
    }
    else
    {
        if (ma < 0)
        {
            (void)snprintf(buf, buf_size,
                           "-%ld.%03ldA",
                           (long)(abs_ma / 1000),
                           (long)(abs_ma % 1000));
        }
        else
        {
            (void)snprintf(buf, buf_size,
                           "%ld.%03ldA",
                           (long)(abs_ma / 1000),
                           (long)(abs_ma % 1000));
        }
    }
}

/* 부호 있는 퍼센트 문자열 */
static void BMS_MESSAGE_FormatSignedPct(char *buf, uint16_t buf_size, int32_t pct)
{
    if ((buf == NULL) || (buf_size == 0U))
    {
        return;
    }

    if (pct > 0)
    {
        (void)snprintf(buf, buf_size, "+%ld%%", (long)pct);
    }
    else
    {
        (void)snprintf(buf, buf_size, "%ld%%", (long)pct);
    }
}

/* =========================================================
 * 상태 문자열
 * ========================================================= */
static const char *TEMP_STATE_STR(TEMP_STATE st)
{
    switch (st)
    {
        case TEMP_STATE_SAFE:    return "SAFE";
        case TEMP_STATE_WARNING: return "WARN";
        case TEMP_STATE_DANGER:  return "DANGER";
        case TEMP_STATE_FAULT:   return "FAULT";
        default:                 return "UNKNOWN";
    }
}

static const char *GAS_STATE_STR(GasLevel_t st)
{
    switch (st)
    {
        case GAS_SAFE:    return "SAFE";
        case GAS_WARNING: return "WARN";
        case GAS_DANGER:  return "DANGER";
        case GAS_FAULT:   return "FAULT";
        default:          return "UNKNOWN";
    }
}

static const char *VOLTAGE_STATE_STR(VOLTAGE_STATE st)
{
    switch (st)
    {
        case VOLTAGE_STATE_SAFE:           return "NORMAL";
        case VOLTAGE_STATE_OVER_WARNING:   return "VOLT_OVER_WARNING";
        case VOLTAGE_STATE_OVER_DANGER:    return "VOLT_OVER_DANGER";
        case VOLTAGE_STATE_UNDER_WARNING:  return "VOLT_UNDER_WARNING";
        case VOLTAGE_STATE_UNDER_DANGER:   return "VOLT_UNDER_DANGER";
        default:                           return "UNKNOWN";
    }
}

static const char *CURRENT_STATE_STR(CURRENT_STATE st)
{
    switch (st)
    {
        case CURRENT_STATE_SAFE:    return "NORMAL";
        case CURRENT_STATE_WARNING: return "CURR_OVER_WARNING";
        case CURRENT_STATE_DANGER:  return "CURR_OVER_DANGER";
        default:                    return "UNKNOWN";
    }
}

/* =========================================================
 * STATE 줄 문자열
 * ---------------------------------------------------------
 * NOW 상태와 LATCH 상태를 분리해서 보여준다.
 * ========================================================= */
static const char *BMS_MESSAGE_GetStateLine(void)
{
    static char msg[112];
    const char *now_reason;
    const char *latched_reason;

    now_reason = BMS_SAFETY_GetNowReason();
    latched_reason = BMS_SAFETY_GetLatchedReason();

    if (now_reason == NULL)
    {
        now_reason = "SAFE";
    }

    if (latched_reason == NULL)
    {
        latched_reason = "NONE";
    }

    if (BMS_SAFETY_IsDangerNow() != 0U)
    {
        (void)snprintf(msg, sizeof(msg),
                       "NOW:DANGER(%s) | LATCH:%s",
                       now_reason,
                       latched_reason);
        return msg;
    }

    if (BMS_SAFETY_GetWarningCount() > 0U)
    {
        (void)snprintf(msg, sizeof(msg),
                       "NOW:WARN(%s) | LATCH:%s",
                       now_reason,
                       (BMS_SAFETY_IsDangerLatched() != 0U) ? latched_reason : "NONE");
        return msg;
    }

    (void)snprintf(msg, sizeof(msg),
                   "NOW:SAFE | LATCH:%s",
                   (BMS_SAFETY_IsDangerLatched() != 0U) ? latched_reason : "NONE");
    return msg;
}

/* =========================================================
 * UART callback router
 * ========================================================= */
void BMS_MESSAGE_UartTxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart6)
    {
        uart6_tx_busy = 0U;
    }
}

void BMS_MESSAGE_UartErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart6)
    {
        uart6_tx_busy = 0U;
    }
}

/* =========================================================
 * UART별 버퍼 / busy flag
 * ========================================================= */
static char *BMS_MESSAGE_GetTxBuf(UART_HandleTypeDef *huart, uint16_t *buf_size)
{
    (void)huart;
    *buf_size = BMS_UART6_MSG_SIZE;
    return uart6_msg;
}

static uint8_t BMS_MESSAGE_IsTxBusy(UART_HandleTypeDef *huart)
{
    (void)huart;
    return uart6_tx_busy;
}

static void BMS_MESSAGE_SetTxBusy(UART_HandleTypeDef *huart, uint8_t busy)
{
    (void)huart;
    uart6_tx_busy = busy;
}

/* =========================================================
 * 공통 로그 출력
 * ========================================================= */
static void SEND_BMS_STATUS(UART_HandleTypeDef *huart, uint32_t *prev_time)
{
    uint32_t now;
    uint16_t buf_size;
    uint16_t tx_len;
    char *tx_buf;
    int len;

    uint8_t gas_ready;
    uint8_t elec_valid;

    int32_t batt_mv;
    int32_t batt_ma;

    uint32_t gas_now_mv;
    uint32_t gas_base_mv;
    int32_t gas_pct;

    char batt_v_str[16];
    char batt_i_str[16];
    char gas_pct_str[12];

    now = HAL_GetTick();

    if ((now - *prev_time) < BMS_UART_LOG_PERIOD_MS)
    {
        return;
    }

    if (BMS_MESSAGE_IsTxBusy(huart) != 0U)
    {
        return;
    }

    *prev_time = now;

    tx_buf = BMS_MESSAGE_GetTxBuf(huart, &buf_size);

    gas_ready = IsGasBaselineReady();
    elec_valid = IsBmsElectricalValid();

    batt_mv = GetVoltagemV();
    batt_ma = GetCurrentmA();

    gas_now_mv = BMS_MESSAGE_AdcTo_mV(GetGasADC());
    gas_base_mv = BMS_MESSAGE_AdcTo_mV(GetGasBaseADC());
    gas_pct = BMS_MESSAGE_GetGasChangePct(gas_now_mv, gas_base_mv);

    BMS_MESSAGE_Format_mV(batt_v_str, sizeof(batt_v_str), batt_mv);
    BMS_MESSAGE_Format_Current(batt_i_str, sizeof(batt_i_str), batt_ma);
    BMS_MESSAGE_FormatSignedPct(gas_pct_str, sizeof(gas_pct_str), gas_pct);

    if (gas_ready != 0U)
    {
        len = snprintf(tx_buf, buf_size,
                	   "[APP]   MODE:%s | FORCE:%s | LAST:%c\r\n"
                       "[STATE] %s\r\n"
                       "[T/G]   T:%dC %s | G:%s %s\r\n"
                       "[ELEC]  V:%s %s | I:%s %s\r\n"
                       "[LIM]   %u->%u\r\n",
		               App_Bms_IsAutoMode() ? "AUTO" : "MANUAL",
		               App_Bms_IsForceStopLocked() ? "ON" : "OFF",
		               App_Bms_GetLastRemoteCmd() ? App_Bms_GetLastRemoteCmd() : '-',
                       BMS_MESSAGE_GetStateLine(),
                       GetTempC(),
                       TEMP_STATE_STR(GetTempState()),
                       gas_pct_str,
                       GAS_STATE_STR(GetGasState()),
                       batt_v_str,
                       (elec_valid != 0U) ? VOLTAGE_STATE_STR(GetVoltageState()) : "INVALID",
                       batt_i_str,
                       (elec_valid != 0U) ? CURRENT_STATE_STR(GetCurrentState()) : "INVALID",
					   BMS_SAFETY_GetAppliedLimitPct(),
					   BMS_SAFETY_GetTargetLimitPct());
    }
    else
    {
        len = snprintf(tx_buf, buf_size,
                       "[STATE] %s\r\n"
                       "[T/G]   T:%dC %s | G:CAL %s\r\n"
                       "[ELEC]  V:%s %s | I:%s %s\r\n"
                       "[LIM]   %u->%u\r\n",
                       BMS_MESSAGE_GetStateLine(),
                       GetTempC(),
                       TEMP_STATE_STR(GetTempState()),
                       GAS_STATE_STR(GetGasState()),
                       batt_v_str,
                       (elec_valid != 0U) ? VOLTAGE_STATE_STR(GetVoltageState()) : "INVALID",
                       batt_i_str,
                       (elec_valid != 0U) ? CURRENT_STATE_STR(GetCurrentState()) : "INVALID",
					   BMS_SAFETY_GetAppliedLimitPct(),
                       BMS_SAFETY_GetTargetLimitPct());
    }

    tx_len = BMS_MESSAGE_GetSafeLen(len, buf_size);
    if (tx_len == 0U)
    {
        return;
    }

    BMS_MESSAGE_SetTxBusy(huart, 1U);

    if (HAL_UART_Transmit_DMA(huart, (uint8_t *)tx_buf, tx_len) != HAL_OK)
    {
        BMS_MESSAGE_SetTxBusy(huart, 0U);
    }
}

void SHOW_UART6_BMS(void)
{
    static uint32_t prev_time_uart6 = 0U;
    SEND_BMS_STATUS(&huart6, &prev_time_uart6);
}

