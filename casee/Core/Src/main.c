/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
#include "string.h"
#include "stdlib.h"
#include "config.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct UART_HANDLER{
	char rxBuf[1024];
	char txBuf[1024];
	int rxIndex;
	char rxTmp[1];
}UART_HANDLER;

typedef struct TIM_HANDLER_T{
	volatile uint32_t counterValue;
	volatile uint32_t targetValue;
	bool interruptFlag;
}TIM_HANDLER_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
UART_HANDLER uart1;

bool isLedOn = false;
bool isWorking = true;
bool isSecondLedOn = false;
volatile uint32_t sysTimeRising, sysTimeFalling;
int readValue;

TIM_HANDLER_t timStatusLed = {0,NULL,false};
TIM_HANDLER_t timCom = {0,500,false};
TIM_HANDLER_t timRun = {0,UINT32_MAX-1,false};

bool timerHandler(TIM_HANDLER_t* tim)
{
	if ( tim->interruptFlag ){
		tim->interruptFlag = false;
		return true;
	}
	return false;
}

typedef struct MY_PARAMS_T{
	int a ;
}MY_PARAMS;

void timerInterruptHandler(TIM_HANDLER_t* tim,void (*CALLBACK)(MY_PARAMS*),MY_PARAMS* params)
{
	tim->counterValue++;
	if ( tim->counterValue == tim->targetValue ){
		tim->counterValue = 0 ;
		tim->interruptFlag = true;
		if ( timerHandler(tim)){
			CALLBACK(params == NULL ? NULL : params);
		}
	}
}

void timComInterruptCallback(MY_PARAMS* param)
{
	sprintf(uart1.txBuf,"%d,%d,%d,%d,%d\n", isLedOn ,timStatusLed.counterValue, timStatusLed.targetValue,(timStatusLed.targetValue - timStatusLed.counterValue),timRun.counterValue);
	HAL_UART_Transmit_IT(&huart1, uart1.txBuf, strlen(uart1.txBuf));
}

void timStatusLedInterruptCallback(MY_PARAMS* param)
{
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_11);
	isLedOn = !isLedOn;
}

void timRunInterruptCallback(MY_PARAMS* param){
	HAL_UART_Transmit_IT(&huart1, "\n\nSYSTEM TIME LIMIT HAS BEEN REACHED. SYSTEM TIME IS BEING RESET !\n\n", strlen("\n\nSYSTEM TIME LIMIT HAS BEEN REACHED. SYSTEM TIME IS BEING RESET !\n\n"));
	timRun.counterValue = 0;
}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance == USART1){
		if(uart1.rxIndex == 0){
			memset(uart1.rxBuf, '\0', strlen(uart1.rxBuf));
		}
		if(uart1.rxIndex < 1023){
			if(uart1.rxTmp[0] != '!'){
				uart1.rxBuf[uart1.rxIndex++] = uart1.rxTmp[0];
			}
			else{
				uart1.rxBuf[uart1.rxIndex] = '\0';
				int tmp = atoi(uart1.rxBuf);
				if(tmp>0){
					timStatusLed.targetValue = tmp;
					char tmpBuf[1024];
					sprintf(tmpBuf, "NEW LED TARGET VALUE : %d\n STARTING AGAIN!\n", timStatusLed.targetValue);
					HAL_UART_Transmit_IT(&huart1, tmpBuf, strlen(tmpBuf));
					timStatusLed.counterValue = 0;
					timCom.counterValue = 0;
					uart1.rxIndex = 0;
					if(isLedOn){
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, 0);
					}
					isLedOn = false;
					Erase_Flash(FLASH_USER_START_ADDR);
					Write_Flash(FLASH_USER_START_ADDR,timStatusLed.targetValue);
				}
				else{
					HAL_UART_Transmit_IT(&huart1, "PLEASE ENTER A VALID VALUE", strlen("PLEASE ENTER A VALID VALUE"));
				}
			}
		}
		else{
			uart1.rxIndex = 0;
			memset(uart1.rxBuf, '\0', uart1.rxIndex);
		}
		HAL_UART_Receive_IT(&huart1, uart1.rxTmp, 1);
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance == htim1.Instance){
		timerInterruptHandler(&timRun, timRunInterruptCallback, NULL);
		if(isWorking){
			timerInterruptHandler(&timStatusLed, timStatusLedInterruptCallback, NULL);
			timerInterruptHandler(&timCom, timComInterruptCallback, NULL);
		}
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == GPIO_PIN_12){
		if(isWorking){
			isWorking = false;
			sysTimeRising = HAL_GetTick();
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, 0);
			sprintf(uart1.txBuf,"STOPPED!\nValues when stopped : %d,%d,%d,%d,%d\n", isLedOn ,timStatusLed.counterValue, timStatusLed.targetValue,(timStatusLed.targetValue - timStatusLed.counterValue),timRun.counterValue);
			HAL_UART_Transmit_IT(&huart1, uart1.txBuf, strlen(uart1.txBuf));

		}
		else{
			isWorking = true;
			sysTimeFalling = HAL_GetTick();
			if(sysTimeFalling-sysTimeRising < FACTORY_MODE_TIMEOUT){
				HAL_UART_Transmit_IT(&huart1, "STARTED AGAIN!\n", strlen("STARTED AGAIN!\n"));
				if(isLedOn){
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, 1);
				}
			}
			else{
				HAL_UART_Transmit(&huart1, "\n\nRETURNING TO FACTORY MODE !!\n\n", strlen("\n\nRETURNING TO FACTORY MODE !!\n\n"),200);
				Erase_Flash(FLASH_USER_START_ADDR);
				NVIC_SystemReset();
			}
		}
	}
}

uint16_t Read_Flash(uint32_t  adr)
{
  uint16_t * Pntr = (uint16_t *)adr;
  return(*Pntr);
}

void Erase_Flash (uint32_t adr)
{
  FLASH->KEYR=0x45670123;
  FLASH->KEYR=0xCDEF89AB;
  FLASH->CR|=0x00000002;
  FLASH->AR=adr;
  FLASH->CR|=0x00000040;
  while((FLASH->SR&0x00000001));
  FLASH->CR &= ~0x00000042;
  FLASH->CR=0x00000080;
}

void Write_Flash (uint32_t adr, uint16_t data)
{
  FLASH->KEYR=0x45670123;
  FLASH->KEYR=0xCDEF89AB;
  FLASH->CR|=0x00000001;
  *(__IO uint16_t*)adr = data;
  while((FLASH->SR&0x00000001));
  FLASH->CR=0x00000080;
}

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
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  readValue = Read_Flash(FLASH_USER_START_ADDR);
  if(ERASED_MEMORY_VALUE == readValue){
	  timStatusLed.targetValue = DEFAULT_LED_TARGET_VALUE;
  }
  else{
	  timStatusLed.targetValue = readValue;
  }

  uart1.rxIndex = 0;
  HAL_UART_Receive_IT(&huart1, uart1.rxTmp, 1);
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 72;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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

#ifdef  USE_FULL_ASSERT
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
