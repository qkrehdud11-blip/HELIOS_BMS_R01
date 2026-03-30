

#include "ultrasonic.h"
#include "gpio.h"
#include "tim.h"
#include "delay.h"
#include "stdlib.h"


/* ====== TRIG 핀 매핑 ====== */
#define TRIG_PORT_LEFT    GPIOB
#define TRIG_PIN_LEFT     GPIO_PIN_2

#define TRIG_PORT_CENTER  GPIOB
#define TRIG_PIN_CENTER   GPIO_PIN_1

#define TRIG_PORT_RIGHT   GPIOB
#define TRIG_PIN_RIGHT    GPIO_PIN_15


/* ====== 내부 채널 테이블 ====== */
static ultrasonic_ch_t s_us[US_COUNT];
static uint32_t prev_tick = 0;
static int st = 0;


static void trig_pulse(GPIO_TypeDef *port, uint16_t pin)
{
  HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
  delay_us(1);
  HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
  delay_us(10);
  HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
}


/* ====== 캡처 처리 ====== */

static void process_capture(ultrasonic_ch_t *ch, TIM_HandleTypeDef *htim)
{
  uint32_t captured = HAL_TIM_ReadCapturedValue(htim, ch->tim_channel);
  if (ch->capture_flag == 0) // Rising edge
  {
    ch->ic_v1 = captured;
    ch->capture_flag = 1;
    __HAL_TIM_SET_CAPTUREPOLARITY(htim, ch->tim_channel, TIM_INPUTCHANNELPOLARITY_FALLING);
  }
  else // Falling edge
  {
    ch->ic_v2 = captured;
    if (ch->ic_v2 >= ch->ic_v1)
        ch->echo_time_us = ch->ic_v2 - ch->ic_v1;
    else // 오버플로우 처리
        ch->echo_time_us = (0xFFFF - ch->ic_v1) + ch->ic_v2;

    ch->distance_cm = (uint8_t)(ch->echo_time_us / 58u);
    ch->capture_flag = 0;
    __HAL_TIM_SET_CAPTUREPOLARITY(htim, ch->tim_channel, TIM_INPUTCHANNELPOLARITY_RISING);
  }
}








void Ultrasonic_Init(void)
{
  s_us[US_LEFT] = (ultrasonic_ch_t){
    .trig_port = TRIG_PORT_LEFT,
    .trig_pin  = TRIG_PIN_LEFT,
    .htim      = &htim3,
    .tim_channel = TIM_CHANNEL_4,
    .tim_it_cc   = TIM_IT_CC4,
    .capture_flag = 0,
	.distance_cm = 200,
  };

  s_us[US_CENTER] = (ultrasonic_ch_t){
    .trig_port = TRIG_PORT_CENTER,
    .trig_pin  = TRIG_PIN_CENTER,
    .htim      = &htim3,
    .tim_channel = TIM_CHANNEL_3,
    .tim_it_cc   = TIM_IT_CC3,
    .capture_flag = 0,
	.distance_cm = 200,
  };

  s_us[US_RIGHT] = (ultrasonic_ch_t){
    .trig_port = TRIG_PORT_RIGHT,
    .trig_pin  = TRIG_PIN_RIGHT,
    .htim      = &htim3,
    .tim_channel = TIM_CHANNEL_1,
    .tim_it_cc   = TIM_IT_CC1,
    .capture_flag = 0,
	.distance_cm = 200,
  };

  /* Input Capture Start */
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_4);
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_3);
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);

  /* Rising edge 시작 */
  __HAL_TIM_SET_CAPTUREPOLARITY(&htim3, TIM_CHANNEL_4, TIM_INPUTCHANNELPOLARITY_RISING);
  __HAL_TIM_SET_CAPTUREPOLARITY(&htim3, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_RISING);
  __HAL_TIM_SET_CAPTUREPOLARITY(&htim3, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
}




void Ultrasonic_TriggerAll(void)
{
	uint32_t current_tick = HAL_GetTick();

	if (current_tick - prev_tick < 10) return;




	switch (st) {
		case 0:
		    Ultrasonic_TriggerOne(US_CENTER);
		    st = 1;
			break;
		case 1:
		    Ultrasonic_TriggerOne(US_LEFT);
		    st = 2;
			break;
		case 2:
		    Ultrasonic_TriggerOne(US_RIGHT);
		    st = 0;
			break;
		default:
			break;
	}

	prev_tick = current_tick;

}





void Ultrasonic_TriggerOne(ultrasonic_id_t id)
{
  if (id >= US_COUNT) return;
  trig_pulse(s_us[id].trig_port, s_us[id].trig_pin);
}



uint8_t Ultrasonic_GetDistanceCm(ultrasonic_id_t id)
{
  if (id >= US_COUNT) return 0;
  return s_us[id].distance_cm;
}


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{

  Ultrasonic_IC_CaptureCallback(htim);
}

void Ultrasonic_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM3 &&
      htim->Channel  == HAL_TIM_ACTIVE_CHANNEL_4)
  {
    process_capture(&s_us[US_LEFT], htim);
    return;
  }

  if (htim->Instance == TIM3 &&
      htim->Channel  == HAL_TIM_ACTIVE_CHANNEL_3)
  {
    process_capture(&s_us[US_CENTER], htim);
    return;
  }

  if (htim->Instance == TIM3 &&
      htim->Channel  == HAL_TIM_ACTIVE_CHANNEL_1)
  {
    process_capture(&s_us[US_RIGHT], htim);
    return;
  }
}

