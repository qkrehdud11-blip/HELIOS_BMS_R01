/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <string.h>
#include "bt_master.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint16_t adc_y = 0;  // S-Y1
uint16_t adc_x = 0;  // S-X2
uint8_t  bt_cmd = 'S';
uint8_t  bt_cmd_prev = 'S';


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */



//  BT_Connect();




  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  // 1. 버튼 상태 먼저 읽기
	      if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == GPIO_PIN_RESET) // SWA: 오토 모드
	      {
	          bt_cmd = 'A';
	          HAL_UART_Transmit(&huart1, &bt_cmd, 1, 10);
	          bt_cmd_prev = bt_cmd;
	          HAL_Delay(300); // 500ms보다 조금 짧은 300ms 정도로도 충분합니다.
	          continue; // 버튼 처리 후 루프 처음으로 돌아가서 조이스틱 로직 건너뜀
	      }
	      else if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == GPIO_PIN_RESET) // SWB: 매뉴얼 모드
	      {
	          bt_cmd = 'P';
	          HAL_UART_Transmit(&huart1, &bt_cmd, 1, 10);
	          bt_cmd_prev = bt_cmd;
	          HAL_Delay(300);
	          continue;
	      }
        else if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == GPIO_PIN_RESET) // K1: 태양 추적 초기화 모드
        {
            bt_cmd = 'D';
            HAL_UART_Transmit(&huart1, &bt_cmd, 1, 10);
            bt_cmd_prev = bt_cmd;
            HAL_Delay(300);
            continue;
        }
        else if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5) == GPIO_PIN_RESET) // K2: [공백]
        {
            bt_cmd = 'K';
            HAL_UART_Transmit(&huart1, &bt_cmd, 1, 10);
            bt_cmd_prev = bt_cmd;
            HAL_Delay(300);
            continue;
        }


	  // 1. ADC 변환 시작 (Rank 1, 2 순차 실행)
	      HAL_ADC_Start(&hadc1);

	      // 2. Rank 1 (PA0 - X축) 변환 완료 대기 및 읽기
	      if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
	      {
	          adc_x = HAL_ADC_GetValue(&hadc1);
	      }

	      // 3. Rank 2 (PA1 - Y축) 변환 완료 대기 및 읽기
	      if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
	      {
	          adc_y = HAL_ADC_GetValue(&hadc1);
	      }

	      // 4. ADC 변환 중단
	      HAL_ADC_Stop(&hadc1);

	      // --- 조이스틱 방향 판별 및 전송 로직 ---
	      if (adc_y > 3000)
		  {
	    	  if(adc_y > 4000) bt_cmd = 'F';	//전진
	    	  else bt_cmd = 'Q';				//약전진
		  }
	      else if (adc_y < 1000)
		  {
			  if(adc_y < 200) bt_cmd = 'B';		//후진
			  else bt_cmd = 'W';				//약후진
		  }

	      else if (adc_x > 3000)
		  {
			  if(adc_x > 4000) bt_cmd = 'L';	//전진
			  else bt_cmd = 'E';				//약전진
		  }
		  else if (adc_x < 1000)
		  {
			  if(adc_x < 200) bt_cmd = 'R';		//후진
			  else bt_cmd = 'T';				//약후진
		  }
		  else bt_cmd = 'S';



	      if (bt_cmd != bt_cmd_prev)
	      {
	          HAL_UART_Transmit(&huart1, &bt_cmd, 1, 10);
	          bt_cmd_prev = bt_cmd;
	      }


	      HAL_Delay(50);




    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
