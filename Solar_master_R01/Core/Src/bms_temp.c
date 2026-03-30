/*
 * temp.c
 *
 *  Created on: 2026. 3. 18.
 *      Author: kimsuyeon
 *
 */

#include "bms_temp.h"
#include <math.h>

/* =========================================================
 * ADC DMA buffer
 * ---------------------------------------------------------
 * ADC1 Rank1=temp, Rank2=gas
 * ========================================================= */
extern volatile uint16_t adcValue[2];


/* =========================================================
 * 설정값
 * ========================================================= */
#define TEMP_ADC_INDEX          0U

#define TEMP_ADC_MAX            4095.0f
#define TEMP_VREF_V             3.3f

#define TEMP_PULLUP_OHM         10000.0f
#define TEMP_R25_OHM            10000.0f
#define TEMP_BETA_K             3976.0f
#define TEMP_T0_K				298.15f   // 25C = 298.15K * 100


/* warning / danger 임계값 */
#define TEMP_WARNING_ON_C             45
#define TEMP_WARNING_OFF_C            42
#define TEMP_DANGER_ON_C              55
#define TEMP_DANGER_OFF_C             52


/* ADC extreme fault */
#define TEMP_ADC_FAULT_LOW            3U
#define TEMP_ADC_FAULT_HIGH           4092U


/* =========================================================
 * 내부 변수
 * ========================================================= */
static TEMP_STATE s_temp_state = TEMP_STATE_FAULT;
static int16_t s_temp_c = 0;



/* =========================================================
 * Temp_ReadADC
 * ---------------------------------------------------------
 * 현재 temp 채널 raw ADC 반환
 * ========================================================= */

uint16_t Temp_ReadADC(void)
{
    return adcValue[TEMP_ADC_INDEX];
}


/* =========================================================
 * Temp_CalcCelsius
 * ---------------------------------------------------------
 * ADC raw -> Celsius 변환
 *
 * 반환
 *   정상 계산 : 섭씨값
 *   계산 실패 : -1000
 * ========================================================= */

int16_t Temp_CalcCelsius(uint16_t adc)
{
    float vout;
    float r_ntc;
    float inv_t;
    float temp_k;
    float temp_c;

    /* ADC extreme 값은 계산하지 않음 */
    if ((adc <= TEMP_ADC_FAULT_LOW) || (adc >= TEMP_ADC_FAULT_HIGH))
    {
        return -1000;
    }

    vout = ((float)adc / TEMP_ADC_MAX) * TEMP_VREF_V;

    if ((vout <= 0.0f) || (vout >= TEMP_VREF_V))
    {
        return -1000;
    }

    /* pull-up divider 기준 NTC 저항 계산 */
    r_ntc = (TEMP_PULLUP_OHM * vout) / (TEMP_VREF_V - vout);

    if (r_ntc <= 0.0f)
    {
        return -1000;
    }

    /* Beta equation */
    inv_t = (1.0f / TEMP_T0_K) + (logf(r_ntc / TEMP_R25_OHM) / TEMP_BETA_K);

    if (inv_t <= 0.0f)
    {
        return -1000;
    }

    temp_k = 1.0f / inv_t;
    temp_c = temp_k - 273.15f;

    /* 반올림 */
    if (temp_c >= 0.0f)
    {
        return (int16_t)(temp_c + 0.5f);
    }
    else
    {
        return (int16_t)(temp_c - 0.5f);
    }
}






/* =========================================================
 * Temp_GetState
 * ---------------------------------------------------------
 * 현재 상태 기준 hysteresis 적용
 * ========================================================= */
TEMP_STATE Temp_GetState(int16_t temp_c)
{
    /* 계산 실패면 fault */
    if (temp_c <= -1000)
    {
        return TEMP_STATE_FAULT;
    }

    switch (s_temp_state)
    {
        case TEMP_STATE_SAFE:
        {
            if (temp_c >= TEMP_DANGER_ON_C)
            {
                return TEMP_STATE_DANGER;
            }
            else if (temp_c >= TEMP_WARNING_ON_C)
            {
                return TEMP_STATE_WARNING;
            }
            else
            {
                return TEMP_STATE_SAFE;
            }
        }

        case TEMP_STATE_WARNING:
        {
            if (temp_c >= TEMP_DANGER_ON_C)
            {
                return TEMP_STATE_DANGER;
            }
            else if (temp_c <= TEMP_WARNING_OFF_C)
            {
                return TEMP_STATE_SAFE;
            }
            else
            {
                return TEMP_STATE_WARNING;
            }
        }

        case TEMP_STATE_DANGER:
        {
            if (temp_c <= TEMP_DANGER_OFF_C)
            {
                if (temp_c <= TEMP_WARNING_OFF_C)
                {
                    return TEMP_STATE_SAFE;
                }
                else
                {
                    return TEMP_STATE_WARNING;
                }
            }
            else
            {
                return TEMP_STATE_DANGER;
            }
        }

        case TEMP_STATE_FAULT:
        default:
        {
            if (temp_c >= TEMP_DANGER_ON_C)
            {
                return TEMP_STATE_DANGER;
            }
            else if (temp_c >= TEMP_WARNING_ON_C)
            {
                return TEMP_STATE_WARNING;
            }
            else
            {
                return TEMP_STATE_SAFE;
            }
        }
    }
}


/* =========================================================
 * Temp_Task
 * ---------------------------------------------------------
 * 주기적으로 호출
 *
 * DMA circular 이므로 그냥 최신 adcValue[0]를 읽어
 * 온도 계산 후 상태를 갱신하면 된다.
 * ========================================================= */
TEMP_STATE Temp_Task(void)
{
    uint16_t adc;
    int16_t temp_c;

    adc = Temp_ReadADC();
    temp_c = Temp_CalcCelsius(adc);

    s_temp_c = temp_c;
    s_temp_state = Temp_GetState(temp_c);

    return s_temp_state;
}


/* =========================================================
 * 현재 온도 반환
 * ========================================================= */
int16_t Temp_GetNowCelsius(void)
{
    return s_temp_c;
}


/* =========================================================
 * 현재 상태 반환
 * ========================================================= */
TEMP_STATE Temp_GetNowState(void)
{
    return s_temp_state;
}
