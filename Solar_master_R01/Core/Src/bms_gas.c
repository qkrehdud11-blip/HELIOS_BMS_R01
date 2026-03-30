/*
 * gas2.c
 *
 *  Created on: 2026. 3. 20.
 *      Author: kimsuyeon
 */

#include "bms_gas.h"

/* main.c 에서 실제 정의 */
extern volatile uint16_t adcValue[2];

/* =========================================================
 * 설정값
 * ========================================================= */
#define GAS_ADC_INDEX               1U

/* ADC extreme fault */
#define GAS_ADC_FAULT_LOW           3U
#define GAS_ADC_FAULT_HIGH          4092U

/* moving average */
#define GAS_AVG_COUNT               8U

/* baseline 수집 샘플 수
 * 전원 켠 뒤 센서가 어느 정도 안정된 다음
 * 깨끗한 공기 기준값을 한 번 잡는다.
 */
#define GAS_BASELINE_SAMPLE_CNT     32U

/* delta threshold
 * delta = adc_now - adc_base
 */
#define GAS_WARNING_ON_DELTA        150U
#define GAS_WARNING_OFF_DELTA       100U
#define GAS_DANGER_ON_DELTA         400U
#define GAS_DANGER_OFF_DELTA        300U

/* =========================================================
 * 내부 변수
 * ========================================================= */
static GasLevel_t s_gas_state = GAS_SAFE;
static uint16_t s_gas_adc = 0U;
static uint16_t s_gas_base_adc = 0U;
static uint16_t s_gas_delta = 0U;

static uint8_t s_base_ready = 0U;
static uint32_t s_base_sum = 0U;
static uint16_t s_base_cnt = 0U;

/* moving average */
static uint16_t s_avg_buf[GAS_AVG_COUNT] = {0U, };
static uint32_t s_avg_sum = 0U;
static uint8_t s_avg_idx = 0U;
static uint8_t s_avg_fill = 0U;

/* =========================================================
 * Gas_ReadADC
 * ========================================================= */
uint16_t Gas_ReadADC(void)
{
    return adcValue[GAS_ADC_INDEX];
}

/* =========================================================
 * Gas_ReadADC_Avg
 * ---------------------------------------------------------
 * ADC 평균값 계산
 * ========================================================= */
uint16_t Gas_ReadADC_Avg(void)
{
    uint16_t raw = Gas_ReadADC();

    if (s_avg_fill < GAS_AVG_COUNT)
    {
        s_avg_buf[s_avg_idx] = raw;
        s_avg_sum += raw;
        s_avg_idx++;
        s_avg_fill++;

        if (s_avg_idx >= GAS_AVG_COUNT)
        {
            s_avg_idx = 0U;
        }

        return (uint16_t)(s_avg_sum / s_avg_fill);
    }

    s_avg_sum -= s_avg_buf[s_avg_idx];
    s_avg_buf[s_avg_idx] = raw;
    s_avg_sum += raw;

    s_avg_idx++;
    if (s_avg_idx >= GAS_AVG_COUNT)
    {
        s_avg_idx = 0U;
    }

    return (uint16_t)(s_avg_sum / GAS_AVG_COUNT);
}

/* =========================================================
 * Gas_BaselineInit
 * ---------------------------------------------------------
 * baseline 다시 잡기
 * 깨끗한 공기 상태에서 호출
 * ========================================================= */
void Gas_BaselineInit(void)
{
    s_gas_base_adc = 0U;
    s_gas_delta = 0U;
    s_base_ready = 0U;
    s_base_sum = 0U;
    s_base_cnt = 0U;
    s_gas_state = GAS_SAFE;
}

/* =========================================================
 * Gas_UpdateBaseline
 * ---------------------------------------------------------
 * baseline이 아직 준비되지 않았으면 평균값으로 baseline 생성
 * ========================================================= */
void Gas_UpdateBaseline(void)
{
    if (s_base_ready != 0U)
    {
        return;
    }

    s_base_sum += s_gas_adc;
    s_base_cnt++;

    if (s_base_cnt >= GAS_BASELINE_SAMPLE_CNT)
    {
        s_gas_base_adc = (uint16_t)(s_base_sum / s_base_cnt);
        s_base_ready = 1U;
    }
}

/* =========================================================
 * Gas_IsBaselineReady
 * ========================================================= */
uint8_t Gas_IsBaselineReady(void)
{
    return s_base_ready;
}

/* =========================================================
 * Gas_GetBaselineADC
 * ========================================================= */
uint16_t Gas_GetBaselineADC(void)
{
    return s_gas_base_adc;
}

/* =========================================================
 * Gas_ComputeDelta
 * ========================================================= */
uint16_t Gas_ComputeDelta(uint16_t adc_now, uint16_t adc_base)
{
    if (adc_now <= adc_base)
    {
        return 0U;
    }

    return (uint16_t)(adc_now - adc_base);
}

/* =========================================================
 * Gas_GetLevelFromADC
 * ---------------------------------------------------------
 * baseline 대비 delta + hysteresis
 * ========================================================= */
GasLevel_t Gas_GetLevelFromADC(uint16_t adc)
{
    uint16_t delta;

    if ((adc <= GAS_ADC_FAULT_LOW) || (adc >= GAS_ADC_FAULT_HIGH))
    {
        return GAS_FAULT;
    }

    if ((s_base_ready == 0U) || (s_gas_base_adc == 0U))
    {
        return GAS_SAFE;
    }

    delta = Gas_ComputeDelta(adc, s_gas_base_adc);

    switch (s_gas_state)
    {
        case GAS_SAFE:
        {
            if (delta >= GAS_DANGER_ON_DELTA)
            {
                return GAS_DANGER;
            }
            else if (delta >= GAS_WARNING_ON_DELTA)
            {
                return GAS_WARNING;
            }
            else
            {
                return GAS_SAFE;
            }
        }

        case GAS_WARNING:
        {
            if (delta >= GAS_DANGER_ON_DELTA)
            {
                return GAS_DANGER;
            }
            else if (delta <= GAS_WARNING_OFF_DELTA)
            {
                return GAS_SAFE;
            }
            else
            {
                return GAS_WARNING;
            }
        }

        case GAS_DANGER:
        {
            if (delta <= GAS_DANGER_OFF_DELTA)
            {
                if (delta <= GAS_WARNING_OFF_DELTA)
                {
                    return GAS_SAFE;
                }
                else
                {
                    return GAS_WARNING;
                }
            }
            else
            {
                return GAS_DANGER;
            }
        }

        case GAS_FAULT:
        default:
        {
            if (delta >= GAS_DANGER_ON_DELTA)
            {
                return GAS_DANGER;
            }
            else if (delta >= GAS_WARNING_ON_DELTA)
            {
                return GAS_WARNING;
            }
            else
            {
                return GAS_SAFE;
            }
        }
    }
}

/* =========================================================
 * Gas_TaskLevel
 * ---------------------------------------------------------
 * 간단 논블로킹 구조
 *   1) ADC 평균값 읽기
 *   2) baseline 없으면 baseline 생성
 *   3) delta 계산
 *   4) 상태 판정
 * ========================================================= */
GasLevel_t Gas_TaskLevel(void)
{
    s_gas_adc = Gas_ReadADC_Avg();

    if ((s_gas_adc <= GAS_ADC_FAULT_LOW) || (s_gas_adc >= GAS_ADC_FAULT_HIGH))
    {
        s_gas_delta = 0U;
        s_gas_state = GAS_FAULT;
        return s_gas_state;
    }

    Gas_UpdateBaseline();

    if ((s_base_ready == 0U) || (s_gas_base_adc == 0U))
    {
        s_gas_delta = 0U;
        s_gas_state = GAS_SAFE;
        return s_gas_state;
    }

    s_gas_delta = Gas_ComputeDelta(s_gas_adc, s_gas_base_adc);
    s_gas_state = Gas_GetLevelFromADC(s_gas_adc);

    return s_gas_state;
}

/* =========================================================
 * 현재 값 반환
 * ========================================================= */
uint16_t Gas_GetNowADC(void)
{
    return s_gas_adc;
}

uint16_t Gas_GetNowDelta(void)
{
    return s_gas_delta;
}

GasLevel_t Gas_GetNowState(void)
{
    return s_gas_state;
}
