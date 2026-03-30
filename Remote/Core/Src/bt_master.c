#include "bt_master.h"
#include <string.h>
#include "gpio.h"

extern UART_HandleTypeDef huart1;





// 내부 송신 함수
static void BT_Send(const char* cmd, uint32_t wait_ms)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)cmd, strlen(cmd), 1000);
    if (wait_ms > 0) HAL_Delay(wait_ms);
}



void BT_Connect(void)
{


	// AT 모드용 38400 설정
	huart1.Init.BaudRate = 38400;
	HAL_UART_Init(&huart1);
	HAL_Delay(500);

	// 자동 연결 설정 (딱 1번만)
	BT_Send("AT+ORGL\r\n", 500);     // 공장값(선택이지만 새 모듈이면 추천)
	BT_Send("AT+ROLE=1\r\n", 300);
	BT_Send("AT+CMODE=0\r\n", 300);
	BT_Send("AT+PSWD=1234\r\n", 300); // 슬레이브 PIN에 맞추기(1234/0000 중 하나)
	BT_Send("AT+BIND=98DA,60,0CDF94\r\n", 300);	// MAC주소는 알아서 바꾸기
	BT_Send("AT+RESET\r\n", 500);

}
