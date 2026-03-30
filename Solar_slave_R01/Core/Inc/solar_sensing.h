/*
 * solar_sensing.h
 *
 *  Created on: 2026. 3. 20.
 *
 *  설명:
 *  ---------------------------------------------------------
 *  INA219 기반 전압/전류 센싱 모듈
 *
 *  이 모듈은 같은 코드를
 *    1) 태양전지 입력측 센서
 *    2) 배터리 충전측 센서
 *  둘 다에 공통으로 사용할 수 있다.
 *
 *  [INA219 기본 개념]
 *    Vshunt = V(IN+) - V(IN-)
 *    Vbus   = V(IN-)
 *
 *    따라서
 *      V(IN+) = Vbus + Vshunt
 *
 *  [태양전지 입력측]
 *    PV+ ---- IN+ [Rshunt] IN- ---- Buck input+
 *
 *    source_v = PV 실제 전압
 *    bus_v    = Buck 입력 노드 전압
 *    current  = 태양전지 입력 전류
 *
 *  [배터리측]
 *    Buck out+ ---- IN+ [Rshunt] IN- ---- Battery+
 *
 *    source_v = Buck 출력측 전압
 *    bus_v    = Battery 순수 전압
 *    current  = 충전 전류
 *
 *  ---------------------------------------------------------
 *  DMA 기반 사용 순서
 *
 *    1) SolarSensing_Init(...)
 *    2) 주기적으로 SolarSensing_StartUpdateDMA(...)
 *    3) 주기적으로 SolarSensing_Service(...)
 *    4) update_done 완료 샘플을 회수해서 사용
 *    5) battery 센서는 완료 후 SolarSensing_BatteryFilterUpdate() 추가 적용
 *
 *  ---------------------------------------------------------
 *  주의:
 *  - 이 버전은 "주기 업데이트"만 DMA non-blocking 이다.
 *  - Init은 단순성과 안정성을 위해 blocking 방식이다.
 *  - 전류는 INA219 CURRENT register를 읽지 않고
 *    shunt voltage / shunt 저항으로 직접 계산한다.
 *  - 전력은 V * I로 계산한다.
 *
 *  ---------------------------------------------------------
 *  리팩토링 포인트
 *  ---------------------------------------------------------
 *  기존에는 설정값 / raw / final / filter / 상태 / DMA 변수가
 *  SolarSensing_t 최상위에 평면적으로 퍼져 있었다.
 *
 *  이번 버전에서는 아래처럼 의미별로 묶는다.
 *
 *    scale      : 설정값 / 환산값
 *    raw        : 이번 DMA 업데이트에서 얻은 raw 기반 물리값
 *    out        : 외부에서 사용하는 최종 출력값
 *    batt_filter: battery 센서 필터 상태
 *    status     : 레지스터 / 상태 / 에러
 *    dma        : DMA 진행 상태
 */

#ifndef INC_SOLAR_SENSING_H_
#define INC_SOLAR_SENSING_H_

#include "stm32f4xx_hal.h"
#include <stdint.h>

/* =========================================================
 * INA219 기본 주소
 * ---------------------------------------------------------
 * A0 = GND, A1 = GND 일 때 7-bit 주소는 0x40
 * HAL I2C 호출 시에는 내부에서 (addr << 1) 형태 사용
 * ========================================================= */
#define INA219_ADDR_7BIT_DEFAULT            0x40U

/* =========================================================
 * INA219 Register Map
 * ========================================================= */
#define INA219_REG_CONFIG                   0x00U
#define INA219_REG_SHUNT_VOLTAGE            0x01U
#define INA219_REG_BUS_VOLTAGE              0x02U
//#define INA219_REG_POWER                    0x03U
//#define INA219_REG_CURRENT                  0x04U
#define INA219_REG_CALIBRATION              0x05U

/* =========================================================
 * BUS VOLTAGE Register 상태 비트
 *
 * bit[15:3] : Bus Voltage Data
 * bit[2]    : CNVR (Conversion Ready)
 * bit[1]    : OVF  (Math Overflow)
 * bit[0]    : 0
 * ========================================================= */
//#define INA219_BUS_CNVR_MASK                0x0004U
//#define INA219_BUS_OVF_MASK                 0x0002U

/* =========================================================
 * CONFIG Register bit field
 * ========================================================= */
#define INA219_CFG_RST                      (1U << 15)

/* bus voltage range */
//#define INA219_CFG_BRNG_16V                 (0U << 13)
#define INA219_CFG_BRNG_32V                 (1U << 13)

/* PGA gain / shunt full-scale range */
//#define INA219_CFG_PG_40MV                  (0U << 11)
//#define INA219_CFG_PG_80MV                  (1U << 11)
//#define INA219_CFG_PG_160MV                 (2U << 11)
#define INA219_CFG_PG_320MV                 (3U << 11)

/* ADC resolution / averaging code
 * ---------------------------------------------------------
 * 현재 사용:
 *   12-bit 2S
 *
 * 참고용 옵션은 이후 튜닝 시 바로 바꿀 수 있도록 남겨둔다.
 */
//#define INA219_ADC_9BIT                     0x0U
//#define INA219_ADC_10BIT                    0x1U
//#define INA219_ADC_11BIT                    0x2U
#define INA219_ADC_12BIT                    0x3U

//#define INA219_ADC_12BIT_1S                 0x8U
#define INA219_ADC_12BIT_2S                 0x9U
//#define INA219_ADC_12BIT_4S                 0xAU
//#define INA219_ADC_12BIT_8S                 0xBU
//#define INA219_ADC_12BIT_16S                0xCU
//#define INA219_ADC_12BIT_32S                0xDU
//#define INA219_ADC_12BIT_64S                0xEU
//#define INA219_ADC_12BIT_128S               0xFU

#define INA219_CFG_BADC(code)               (((uint16_t)((code) & 0x0FU)) << 7)
#define INA219_CFG_SADC(code)               (((uint16_t)((code) & 0x0FU)) << 3)

/* operating mode
 * ---------------------------------------------------------
 * 현재는 Shunt + Bus Continuous만 사용한다.
 * 다른 모드는 참고용으로 남겨둔다.
 */
//#define INA219_MODE_POWERDOWN               0x0U
//#define INA219_MODE_SHUNT_TRIG              0x1U
//#define INA219_MODE_BUS_TRIG                0x2U
//#define INA219_MODE_SHUNT_BUS_TRIG          0x3U
//#define INA219_MODE_ADC_OFF                 0x4U
//#define INA219_MODE_SHUNT_CONT              0x5U
//#define INA219_MODE_BUS_CONT                0x6U
#define INA219_MODE_SHUNT_BUS_CONT          0x7U

/* =========================================================
 * 기본 추천 설정
 * ---------------------------------------------------------
 * 현재 기준:
 *   32V range
 *   ±320mV
 *   Bus ADC   = 12-bit 2S
 *   Shunt ADC = 12-bit 2S
 *   Shunt + Bus Continuous
 *
 * 최종 Config 값:
 *   0x3CCF
 * ========================================================= */
#define SOLAR_SENSING_DEFAULT_BRNG          INA219_CFG_BRNG_32V
#define SOLAR_SENSING_DEFAULT_PGA           INA219_CFG_PG_320MV
#define SOLAR_SENSING_DEFAULT_BADC          INA219_ADC_12BIT_2S
#define SOLAR_SENSING_DEFAULT_SADC          INA219_ADC_12BIT_2S
#define SOLAR_SENSING_DEFAULT_MODE          INA219_MODE_SHUNT_BUS_CONT

#define SOLAR_SENSING_DEFAULT_CONFIG \
    ((uint16_t)(SOLAR_SENSING_DEFAULT_BRNG | \
                SOLAR_SENSING_DEFAULT_PGA  | \
                INA219_CFG_BADC(SOLAR_SENSING_DEFAULT_BADC) | \
                INA219_CFG_SADC(SOLAR_SENSING_DEFAULT_SADC) | \
                SOLAR_SENSING_DEFAULT_MODE))

/* =========================================================
 * Calibration / scaling
 * ---------------------------------------------------------
 * Rshunt = 0.1 ohm 기준
 * Calibration = 4096 = 0x1000
 *
 * Shunt voltage LSB = 10uV = 0.01mV
 * Bus voltage   LSB = 4mV  = 0.004V
 * ========================================================= */
#define SOLAR_SENSING_RSHUNT_OHM            0.100f
#define SOLAR_SENSING_CALIBRATION_DEFAULT   0x1000U

#define SOLAR_SENSING_SHUNT_LSB_MV          0.01f
#define SOLAR_SENSING_BUS_LSB_V             0.004f

/* =========================================================
 * battery filter parameters
 * ========================================================= */
#define SOLAR_SENSING_BATT_ALPHA_V          0.08f
#define SOLAR_SENSING_BATT_ALPHA_I          0.10f

#define SOLAR_SENSING_BATT_VALID_MIN_V      2.50f
#define SOLAR_SENSING_BATT_VALID_MAX_V      4.35f
#define SOLAR_SENSING_BATT_MAX_STEP_V       0.25f

/* ---------------------------------------------------------
 * battery invalid / recover debounce
 * ---------------------------------------------------------
 * 기존 문제:
 *   이상 샘플 1회로 range_valid=0 이 되면서
 *   false BATT_RANGE가 너무 쉽게 발생했다.
 *
 * 수정 방향:
 *   - invalid 샘플이 몇 회 연속 나와야 invalid 확정
 *   - valid 샘플도 몇 회 연속 나와야 recover
 * --------------------------------------------------------- */
#define SOLAR_SENSING_BATT_INVALID_ASSERT_COUNT   4U
#define SOLAR_SENSING_BATT_VALID_RECOVER_COUNT    3U

/* =========================================================
 * DMA timeout
 * ---------------------------------------------------------
 * INA219 내부 변환 시간(12-bit 2S, shunt+bus)을 고려해도
 * 정상 상황에서 수십 ms까지 갈 이유는 거의 없다.
 * 너무 짧으면 false timeout 이 날 수 있으므로
 * 여유 있게 20ms로 둔다.
 * ========================================================= */
#define SOLAR_SENSING_DMA_TIMEOUT_MS        20U

/* =========================================================
 * DMA update stage
 * ---------------------------------------------------------
 * 현재는 SHUNT -> BUS 두 단계만 사용한다.
 * ========================================================= */
typedef enum
{
    SOLAR_SENSING_DMA_STAGE_IDLE = 0,
    SOLAR_SENSING_DMA_STAGE_SHUNT,
    SOLAR_SENSING_DMA_STAGE_BUS
} SolarSensingDmaStage_t;


/* =========================================================
 * 설정 / 환산값
 * ========================================================= */
typedef struct
{
    uint16_t config_reg;
    uint16_t calib_reg;
    float r_shunt_ohm;
} SolarSensingScale_t;

/* =========================================================
 * 물리량 묶음
 * ---------------------------------------------------------
 * raw / out / snapshot에서 공통으로 사용한다.
 *
 * current_ma:
 *   shunt_mv / r_shunt_ohm 로 직접 계산한 전류
 *
 * power_*:
 *   V * I 로 계산한 전력
 *   downstream 코드와의 호환성을 위해 유지한다.
 * ========================================================= */
typedef struct
{
    float shunt_mv;
    float bus_v;
    float source_v;
    float current_ma;
    float power_bus_w;
    float power_source_w;
} SolarSensingData_t;

/* =========================================================
 * battery filter 상태
 * ========================================================= */
typedef struct
{
    float v;
    float i;

    uint8_t initialized;
    uint8_t invalid_count;
    uint8_t valid_count;
} SolarSensingBattFilter_t;

/* =========================================================
 * 진단 / 상태 / 레지스터 캐시
 * ========================================================= */
typedef struct
{
    uint16_t reg_config_raw;
    uint16_t reg_calib_raw;
    uint16_t reg_shunt_raw;
    uint16_t reg_bus_raw;

    uint8_t range_valid;

    HAL_StatusTypeDef last_status;
    uint32_t last_i2c_error;
} SolarSensingStatus_t;

/* =========================================================
 * DMA 상태
 * ========================================================= */
typedef struct
{
    volatile uint8_t busy;                  /* 1 = DMA update 진행중 */
    volatile uint8_t update_done;           /* 1 = 새 샘플 완료 */
    volatile SolarSensingDmaStage_t stage;  /* 현재 DMA 단계 */
    volatile uint32_t start_tick;           /* timeout 감시 시작 tick */

    uint8_t rx_buf[2];                      /* 16-bit register read buffer */
} SolarSensingDma_t;

/* =========================================================
 * snapshot 구조체
 * ========================================================= */
typedef struct
{
    float shunt_mv;
    float bus_v;
    float source_v;
    float current_ma;
    float power_bus_w;
    float power_source_w;

    uint8_t range_valid;
    HAL_StatusTypeDef last_status;
} SolarSensingSnapshot_t;

/* =========================================================
 * 디버그 snapshot
 * ---------------------------------------------------------
 * app 로그에서 raw / filt / final을 출력할 때
 * 내부 멤버를 직접 접근하지 않도록 별도 getter용 구조체를 둔다.
 * ========================================================= */
typedef struct
{
    float raw_bus_v;
    float raw_source_v;
    float raw_current_ma;

    float filt_v;
    float filt_i;

    float out_bus_v;
    float out_source_v;
    float out_current_ma;

    uint8_t filter_initialized;
    uint8_t invalid_count;
    uint8_t valid_count;

    uint8_t range_valid;
    HAL_StatusTypeDef last_status;
    uint32_t last_i2c_error;
} SolarSensingDebug_t;

/* =========================================================
 * 센서 객체
 * ========================================================= */
typedef struct
{
    I2C_HandleTypeDef *hi2c;
    uint16_t address_7bit;
    uint8_t is_initialized;

    SolarSensingScale_t      scale;
    SolarSensingData_t       raw;
    SolarSensingData_t       out;
    SolarSensingBattFilter_t batt_filter;
    SolarSensingStatus_t     status;
    SolarSensingDma_t        dma;
} SolarSensing_t;

/* =========================================================
 * public API
 * ========================================================= */

/* 1회 초기화: blocking */
HAL_StatusTypeDef SolarSensing_Init(SolarSensing_t *sensor,
                                    I2C_HandleTypeDef *hi2c,
                                    uint16_t addr_7bit);

/* 주기 업데이트 시작: DMA non-blocking
 * 반환:
 *   HAL_OK   : DMA 시작 성공
 *   HAL_BUSY : 이미 진행중이거나 I2C bus busy
 *   HAL_ERROR: 시작 실패
 */
HAL_StatusTypeDef SolarSensing_StartUpdateDMA(SolarSensing_t *sensor);

/* timeout / stuck 감시
 * 주기적으로 호출해서 DMA가 오래 걸린 경우 복구 처리
 */
void SolarSensing_Service(SolarSensing_t *sensor);

/* battery 측 final 값 안정화 */
void SolarSensing_BatteryFilterUpdate(SolarSensing_t *batt_sensor);

/* 안전한 snapshot 복사 */
void SolarSensing_GetSnapshot(const SolarSensing_t *sensor,
                              SolarSensingSnapshot_t *snapshot);

/* 디버그용 상세 snapshot */
void SolarSensing_GetDebug(const SolarSensing_t *sensor,
                           SolarSensingDebug_t *debug_info);

/* update_done 플래그를 안전하게 읽고 clear */
uint8_t SolarSensing_FetchUpdateDone(SolarSensing_t *sensor);

/* 사용자 HAL callback 에서 호출할 라우터 함수 */
void SolarSensing_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c);
void SolarSensing_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c);

#endif /* INC_SOLAR_SENSING_H_ */
