/*
 * bms_sensor.c
 *
 *  Created on: 2026. 3. 21.
 *      Author: kimsuyeon
 *
 *			TEMP task, GAS task, INA219(BMS 전압/전류) task
 *			각 상태값, 측정값 저장
 *			getter 제공
 */




#include "bms_sensor.h"

/* =========================================================
 * 내부 주기
 * ---------------------------------------------------------
 * INA219_BMS_Task() 는 BMS_SENSOR_Service()에서 매 loop 진행
 * 여기 Task는 "최신 결과를 캐시에 복사"하는 주기만 담당한다.
 * ========================================================= */
#define BMS_SENSOR_TASK_PERIOD_MS      100U

/* ===== TEMP 상태 관리 ===== */
static TEMP_STATE temp_state = TEMP_STATE_SAFE;
static uint16_t temp_adc = 0U;
static int16_t temp_c = 0;

/* ===== GAS 상태 관리 ===== */
static GasLevel_t gas_state = GAS_SAFE;
static uint16_t gas_adc = 0U;
static uint16_t gas_delta = 0U;
static uint8_t gas_base_ready = 0U;
static uint16_t gas_base_adc = 0U;

/* ===== VOLTAGE, CURRENT 상태 관리 ===== */
static uint8_t bms_valid = 0U;
static VOLTAGE_STATE bms_voltage_state = VOLTAGE_STATE_SAFE;
static CURRENT_STATE bms_current_state = CURRENT_STATE_SAFE;
static int32_t bms_voltage_mV = -1;
static int32_t bms_current_mA = -1;

/* ===== 내부 tick ===== */
static uint32_t sensor_prev_tick = 0U;

/* =========================================================
 * INIT
 * ========================================================= */
void BMS_SENSOR_Init(void)
{
    Gas_BaselineInit();
    INA219_BMS_Init();

    sensor_prev_tick = HAL_GetTick();
}

/* =========================================================
 * SERVICE
 * ---------------------------------------------------------
 * non-blocking INA219 DMA state machine 진행
 * 매 loop 호출
 * ========================================================= */
void BMS_SENSOR_Service(void)
{
    INA219_BMS_Task();
}

/* =========================================================
 * TEMP update
 * ========================================================= */
static void BMS_SENSOR_UpdateTemp(void)
{
    temp_state = Temp_Task();
    temp_adc = Temp_ReadADC();
    temp_c = Temp_GetNowCelsius();
}

/* =========================================================
 * GAS update
 * ========================================================= */
static void BMS_SENSOR_UpdateGas(void)
{
    gas_state = Gas_TaskLevel();
    gas_adc = Gas_GetNowADC();
    gas_delta = Gas_GetNowDelta();
    gas_base_ready = Gas_IsBaselineReady();
    gas_base_adc = Gas_GetBaselineADC();
}

/* =========================================================
 * ELECTRICAL update
 * ---------------------------------------------------------
 * INA219는 Service()에서 계속 진행되고,
 * 여기서는 최신 캐시값만 읽어서 상태를 갱신한다.
 * ========================================================= */
static void BMS_SENSOR_UpdateElectrical(void)
{
    bms_valid = INA219_BMS_IsValid();
    bms_voltage_mV = INA219_BMS_ReadVoltage_mV();
    bms_current_mA = INA219_BMS_ReadCurrent_mA();

    if (bms_valid != 0U)
    {
        bms_voltage_state = INA219_BMS_GetVoltageState(bms_voltage_mV);
        bms_current_state = INA219_BMS_GetCurrentState(bms_current_mA);
    }
    else
    {
        /* 센서값이 invalid일 때 이전 판정에 끌려가지 않도록 safe로 초기화
         * 실제 안전 판단에서는 valid 플래그를 별도로 사용한다.
         */
        bms_voltage_state = VOLTAGE_STATE_SAFE;
        bms_current_state = CURRENT_STATE_SAFE;
    }
}

/* =========================================================
 * PUBLIC TASK
 * ---------------------------------------------------------
 * 매 loop 호출 가능
 * 내부 100ms 주기로 캐시 갱신
 * ========================================================= */
void BMS_SENSOR_Task(void)
{
    uint32_t now = HAL_GetTick();

    if ((now - sensor_prev_tick) < BMS_SENSOR_TASK_PERIOD_MS)
    {
        return;
    }

    sensor_prev_tick = now;

    BMS_SENSOR_UpdateTemp();
    BMS_SENSOR_UpdateGas();
    BMS_SENSOR_UpdateElectrical();
}

/* ================= TEMP getter ================= */

uint16_t GetTempADC(void)
{
    return temp_adc;
}

int16_t GetTempC(void)
{
    return temp_c;
}

TEMP_STATE GetTempState(void)
{
    return temp_state;
}

/* ================= GAS getter ================= */

uint16_t GetGasADC(void)
{
    return gas_adc;
}

uint16_t GetGasDelta(void)
{
    return gas_delta;
}

uint8_t IsGasBaselineReady(void)
{
    return gas_base_ready;
}

uint16_t GetGasBaseADC(void)
{
    return gas_base_adc;
}

GasLevel_t GetGasState(void)
{
    return gas_state;
}

/* ================= ELECTRICAL getter ================= */

uint8_t IsBmsElectricalValid(void)
{
    return bms_valid;
}

int32_t GetVoltagemV(void)
{
    return bms_voltage_mV;
}

VOLTAGE_STATE GetVoltageState(void)
{
    return bms_voltage_state;
}

int32_t GetCurrentmA(void)
{
    return bms_current_mA;
}

CURRENT_STATE GetCurrentState(void)
{
    return bms_current_state;
}


