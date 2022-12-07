/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


#define wheelDrop1_duration 500

#define pillDrop_MAX_duration 5

// out of 4096-1
#define LDR_threshold 2048

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM16_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// RTC
uint8_t buffer[15] = "skjsjdsks";
char tem[15];
uint8_t mytx[15] = "Yojak";
uint8_t myrx[15] = "1234509678";

// Drop Pill

uint32_t buffer = 0;

int tim16Overflown = 0;		// flag to indicate timer 16 has overflown in past.
int tim16Available = 1;		// do not access timer till it is available (binary semaphore)

int dropFlagBtn = 0;

// RTC
uint8_t decToBcd(int val)
{
	return (uint8_t)((val/10)*16)+(val%10);
}


uint8_t bcdToDec(int val)
{
	
	return (uint8_t)((val/16)*10)+(val%16);
}


void swapp()
{
	for(int i=0; i<15; ++i)
	{
		buffer[i] = tem[i];
	}
}

void spitstring(struct Cassette *cassette,uint8_t *st)
{
	cassette->Mor[0] = st[0]-'0';
	cassette->Aft[0] = st[5]-'0';
	cassette->Eve[0] = st[10]-'0';

	cassette->Mor[1] = (st[1]-'0')*10+(st[2]-'0');
	cassette->Aft[1] = (st[6]-'0')*10+(st[7]-'0');
	cassette->Eve[1] = (st[11]-'0')*10+(st[12]-'0');

	cassette->Mor[2] = (st[3]-'0')*10+(st[4]-'0');
	cassette->Aft[2] = (st[8]-'0')*10+(st[9]-'0');
	cassette->Eve[2] = (st[13]-'0')*10+(st[14]-'0');
}


// Drop Pill
struct Cassette {
	uint8_t cassetteID;

	// Control Pins
	GPIO_TypeDef* wheelControlPort;
	uint16_t wheelControlPins[2];	// [0] = Forward;	[1] = Reverse

	GPIO_TypeDef* trayControlPort;
	uint16_t trayControlPins[2];	// [0] = Open Tray;	[1] = Rewind Pills

//	uint16_t ldrADCPin;				// Pin for ADC dma input
	uint32_t *ldrADC;				// Stores the Address of variable storing ADC DMA from LDR

	// Timing details
  // spit struct
	uint8_t Mor[3];
	uint8_t Aft[3];
	uint8_t Eve[3];

 // int (*dropPills)(uint8_t numberOfPills);

};



  // Setting up cassette 1
  struct Cassette cass1;


  

	int dropPills(struct Cassette *cassette, uint8_t numberOfPills){

		HAL_Delay(200);
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		
		HAL_Delay(200);
		
		// wait for pill drop using ADC polling from DMA for dynamic comparing
		uint8_t medCounter = 0;
		uint8_t alreadyBlocked = 0;
		uint8_t errorCounts = 0;	// exit procedure when it exceeds 3 attempts.

		// init timer to measure max wait

		// wait for timer 16 to free.
		// while(!tim16Available){};	// wait to acquire lock timer 16

		tim16Available = 0;			// lock timer 16
		tim16Overflown = 0;     	// reset overflown state


		MX_TIM16_Init();
		HAL_TIM_Base_Start_IT(&htim16);

		while(medCounter < numberOfPills){

			// drop more only if less than the needed amount
			// start wheel servo forward rotation
			HAL_GPIO_WritePin(cassette->wheelControlPort, (cassette->wheelControlPins)[0], 0);
			HAL_Delay(wheelDrop1_duration);
			HAL_GPIO_WritePin(cassette->wheelControlPort, (cassette -> wheelControlPins)[0], 1);


			// finding how many dropped
			while(!tim16Overflown){   // wait for timer 16 to complete

				if(*(cassette -> ldrADC) < 2048){		// beam is blocked
					if(alreadyBlocked){	// pill was already blocking beam

					}else{				// pill just started blocking beam
						alreadyBlocked = 1;
						medCounter++;

						// reset timer value
						MX_TIM16_Init();
						HAL_TIM_Base_Start_IT(&htim16);
					}
				}else{					// beam is not blocked
					if(alreadyBlocked){	// pill just finished crossing beam
						alreadyBlocked = 0;

						// reset timer value
						MX_TIM16_Init();
						HAL_TIM_Base_Start_IT(&htim16);
					}else{				// waiting for medicine to fall.

					}
				}

			}
			//HAL_Delay(1000);

			//medCounter = 2;

			// checking block states and med counter
			if(alreadyBlocked){		// the pill is stuck in chamber
				return -1;
			}
			if(medCounter > numberOfPills){		// dropped more then needed.
				// return them back to chamber by rewinding tray.
				HAL_GPIO_WritePin(cassette->trayControlPort, cassette->trayControlPins[1], 0);
				HAL_Delay(2000);
				HAL_GPIO_WritePin(cassette->trayControlPort, cassette->trayControlPins[1], 1);

				// reset to medcounter to start from scratch.
				medCounter = 0;
				errorCounts++;
				if(errorCounts > 3){
					return -1;		// unable to drop equal to specified.
				}

			}

		}

		tim16Available = 1;		// release timer 16 lock
		tim16Overflown = 0;     // reset overflown state
		
		
		// Open Tray & close
		HAL_GPIO_WritePin(cassette->trayControlPort, cassette->trayControlPins[0], 0);
		HAL_Delay(1000);
		HAL_GPIO_WritePin(cassette->trayControlPort, cassette->trayControlPins[0], 1);








	//	//wait for pill drop using interrupts
	//	HAL_Delay(pillDrop_MAX_duration);


		return 0; // 0 = Success
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef * htim){
	if(htim==&htim16){
		// HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		tim16Overflown = 1;
	}
}

void HAL_GPIO_EXTI_Callback (uint16_t GPIO_Pin){
	if(GPIO_Pin == GPIO_PIN_13){
		//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		//HAL_Delay(200);
		dropFlagBtn = 1;
	}
}

typedef struct {
	uint8_t seconds;
	uint8_t minutes;
	uint8_t hour;
	uint8_t dayofweek;
	uint8_t dayofmonth;
	uint8_t month;
	uint8_t year;
	
}Time;

Time time1;

void Set_Time(uint8_t sec, uint8_t min, uint8_t hour, uint8_t dow, uint8_t dom, uint8_t month, uint8_t year)
{
	uint8_t set_time[7];
	set_time[0] = decToBcd(sec);
	set_time[1] = decToBcd(min);
	set_time[2] = decToBcd(hour);
	set_time[3] = decToBcd(dow);
	set_time[4] = decToBcd(dom);
	set_time[5] = decToBcd(month);
	set_time[6] = decToBcd(year);

	HAL_I2C_Mem_Write(&hi2c1, 0xD0, 0x00, 1, set_time, 7, 1000);
}

void Get_Time(void)
{
	uint8_t get_time[7];
	HAL_I2C_Mem_Read(&hi2c1, 0xD0, 0x00, 1, get_time, 7, 1000);
	time1.seconds = bcdToDec(get_time[0]);
	time1.minutes = bcdToDec(get_time[1]);
	time1.hour = bcdToDec(get_time[2]);
	time1.dayofweek = bcdToDec(get_time[3]);
	time1.dayofmonth = bcdToDec(get_time[4]);
	time1.month = bcdToDec(get_time[5]);
	time1.year = bcdToDec(get_time[6]);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) 
{
	HAL_UART_Transmit(&huart2,myrx,sizeof(myrx),10);
	HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_5);
	spitstring(&cass1,myrx);
	
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_ADC_Init();
  MX_TIM16_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  // RTC
  		//Set_Time(uint8_t sec, uint8_t min, uint8_t hour, uint8_t dow, uint8_t dom, uint8_t month, uint8_t year)
	 //Set_Time(20,48,16,1,5,11,22);
		int check = 1;
	

  // Drop Pill
	// setting pins high
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, 1);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, 1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, 1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 1);


  cass1.cassetteID = 0;

  cass1.wheelControlPort = GPIOB;
  cass1.wheelControlPins[0] = GPIO_PIN_11;
  cass1.wheelControlPins[1] = GPIO_PIN_12;

  cass1.trayControlPort = GPIOA;
  cass1.trayControlPins[0] = GPIO_PIN_11;
  cass1.trayControlPins[1] = GPIO_PIN_12;

  cass1.ldrADC = &buffer;



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    // RTC
    Get_Time();
		sprintf(tem, "%02d:%02d:%02d\n", time1.hour, time1.minutes, time1.seconds);
		swapp();
		HAL_UART_Transmit(&huart2,buffer,15,10);
		HAL_UART_Receive_IT(&huart3,myrx,10);
		
		if(time1.hour==cass1.Mor[1] &&  time1.minutes==cass1.Mor[2] && check)
		{
				uint8_t tt[10] = "hahahahaha";
			 HAL_UART_Transmit(&huart2,tt,10,10);
			//@@@@@@@@@@@@@@@@@@@@@@@@@@@ call xyz @@@@@@@@@@@@@@@@@@@@@@
			check = 0;
			
		}
		
		if(time1.minutes!=cass1.Mor[2])
		{
				check = 1;
		}
		
		HAL_Delay(1000);


    // // Drop Pill

		// //HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		// 	HAL_Delay(1);
		// if(dropFlagBtn == 1){
			
		// 	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		// 	HAL_Delay(200);
			
			
		// 	// HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_12);
		
		
		// 	dropPills(&cass1, 2);
		// //HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		//   dropFlagBtn = 0;
	  // }
		// 		 		  dropFlagBtn = dropFlagBtn;

		
		
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 48000-1;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 1000-1;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 38400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin PA11 PA12 */
  GPIO_InitStruct.Pin = LD2_Pin|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB11 PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

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
