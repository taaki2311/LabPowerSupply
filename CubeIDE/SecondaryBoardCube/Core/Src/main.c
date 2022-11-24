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
#include "string.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define P 0.1
#define I 0.1
#define D 0.1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

DAC_HandleTypeDef hdac;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */

uint8_t txbuffer[64];//Uart TX Buffer
uint8_t rxbuffer[64];//Uart RX Buffer

//Channel numbers
float voltnum1 = 0.0;
float ampnum1 = 0.0;
//float voltnum2 = 0.0;
//float ampnum2 = 0.0;

float correctedvoltnum1;

uint8_t first_shot = 1;

//Array for the adc values and vars to hold them
uint16_t adcvalues[5];
uint16_t* adc_current;
uint16_t* adc_linear;
uint16_t* adc_opamp;
uint16_t* adc_switching;
uint16_t* adc_vref;
uint16_t* vrefptr = ((uint16_t*)VREFINT_CAL_ADDR_CMSIS);
int chstat = 0;

//Globals for adc values
uint16_t v1;
float lin_num;
float cur_num;
float op_num;
float swi_num;

// The value for the Intgral part of the PID controller
float error = 0;
float derivative = 0;
float integral = 0;
float error_previous = 0;
float correction = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC_Init(void);
static void MX_DAC_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

void our_init(void);//Runs several Inits

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
  MX_ADC_Init();
  MX_DAC_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  our_init();
  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 4095);

  adc_current = &adcvalues[2];
  adc_linear = &adcvalues[1];
  adc_opamp = &adcvalues[0];
  adc_switching = &adcvalues[3];
  adc_vref = &adcvalues[4];
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  //Control channel here

	  if(voltnum1 <= 0.00){
		  HAL_GPIO_WritePin(Channel_Shutdown_GPIO_Port, Channel_Shutdown_Pin, GPIO_PIN_SET);
		  chstat = 0;
		  //HAL_GPIO_WritePin(Status_LED_1_GPIO_Port, Status_LED_1_Pin, GPIO_PIN_RESET);
	  }

	  uint16_t vrefvalue = (uint16_t)*vrefptr;
	  float vddcalc = (float)3.0 * ((float)vrefvalue / (float)*adc_vref);

	  float cur_num_temp = ((((float)3.0 * (float)*adc_current * (float)vrefvalue)/((float)*adc_vref * (float)4095) / (float)20) / (float)0.15);
	  if(cur_num_temp >= 0.0000){
		  cur_num = cur_num_temp;
	  }
	  else{
		  cur_num = 0.0000;
	  }
	  //float cur_num = (((float)vddcalc * (float)*adc_current * (float)4095) / (float)20) / (float)0.3;
	  float op_num_temp = ((float)3.0 * ((float)*adc_opamp * (float)4.0) * (float)vrefvalue)/((float)*adc_vref * (float)4095) - ((float)cur_num * (float)0.35);
	  if(op_num_temp >= 0.0000){
		  op_num = op_num_temp;
	  }
	  else{
		  op_num = 0.0000;
	  }
	  float lin_num_temp = ((float)3.0 * ((float)*adc_linear * (float)4.0) * (float)vrefvalue)/((float)*adc_vref * (float)4095) - ((float)cur_num * (float)0.35);
	  if(lin_num_temp >= 0.0000){
		  lin_num = lin_num_temp;
	  }
	  else{
		  lin_num = 0.0000;
	  }
	  //float lin_num = ((float)vddcalc * (float)*adc_linear * (float)4095) * (float)4;
	  float swi_num_temp = ((float)3.0 * ((float)*adc_switching * (float)5.0) * (float)vrefvalue)/((float)*adc_vref * (float)4095);
	  if(swi_num_temp >= 0.0000){
		  swi_num = swi_num_temp;
	  }
	  else{
		  swi_num = 0.0000;
	  }

	  if(first_shot){
		  v1 = (uint16_t)((( (((float)voltnum1) / (float)4.0) + ((float)0.446974063 / (float)4.0)) * (float)4095) / (float)vddcalc);
		  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, v1);
		  correctedvoltnum1 = voltnum1;
		  integral = 0;
		  error = 0;
		  derivative = 0;
		  first_shot = 0;
	  }

/*
	  else{ // Bang-Bang Controller
		  //Try really hard to get the voltage right
		  const float margin = 0.05;
		  if(lin_num > voltnum1 + margin){
			  v1--;
			  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, v1);
		  }
		  else if(lin_num < voltnum1 - margin){
			  v1++;
			  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, v1);
		  }
	  }
*/

	  else {
		  error = lin_num - voltnum1;
		  integral += error;
		  if (integral > (float)4095.0) {
			  integral = (float)4095;
		  } else if (integral < (float)-4095.0) {
			  integral = (float)-4095.0;
		  }
		  derivative = error - error_previous;
		  error_previous = error;
		  correction = P * error + I * integral + D * derivative;
		  correctedvoltnum1 = voltnum1 - correction;
		  if(correctedvoltnum1 > 12.0){
			  correctedvoltnum1 = 12.0;
		  }
		  else if(correctedvoltnum1 < 0.0){
			  correctedvoltnum1 = 0.0;
		  }
		  v1 = (uint16_t)((( (correctedvoltnum1 / (float)4.0) + ((float)0.446974063 / (float)4.0)) * (float)4095) / (float)vddcalc);
		  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, v1);
	  }

	  if(voltnum1 > 0.00){
		  HAL_GPIO_WritePin(Channel_Shutdown_GPIO_Port, Channel_Shutdown_Pin, GPIO_PIN_RESET);
		  chstat = 1;
		  //HAL_GPIO_WritePin(Status_LED_1_GPIO_Port, Status_LED_1_Pin, GPIO_PIN_SET);
	  }
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV4;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc.Init.LowPowerAutoWait = ADC_AUTOWAIT_DISABLE;
  hadc.Init.LowPowerAutoPowerOff = ADC_AUTOPOWEROFF_DISABLE;
  hadc.Init.ChannelsBank = ADC_CHANNELS_BANK_A;
  hadc.Init.ContinuousConvMode = ENABLE;
  hadc.Init.NbrOfConversion = 5;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = ENABLE;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_192CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */

  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT2 config
  */
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

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
  huart1.Init.BaudRate = 4800;
  huart1.Init.WordLength = UART_WORDLENGTH_9B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_ODD;
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
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Channel_Shutdown_GPIO_Port, Channel_Shutdown_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : Channel_Shutdown_Pin */
  GPIO_InitStruct.Pin = Channel_Shutdown_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Channel_Shutdown_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void our_init(void){
	HAL_GPIO_WritePin(Channel_Shutdown_GPIO_Port, Channel_Shutdown_Pin, GPIO_PIN_SET);	//Ensure shutdown is enabled
	/*
	 * The HAL library is dumb and tries to init the adc before the DMA which does not work
	 * if the ADC is using the DMA. This init code is placed in space reserved for CubeMx so
	 * manually reordering it will be overwritten every time we regenerate code.
	 */
	HAL_ADC_DeInit(&hadc);
	HAL_DMA_DeInit(&hdma_adc);
	MX_DMA_Init();
	MX_ADC_Init();

	//Actually our init now
	HAL_ADC_Start_DMA(&hadc, (uint32_t*)&adcvalues, 5);// start the adc in dma mode
	HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
	HAL_DAC_Start(&hdac, DAC_CHANNEL_2);
	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);
	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 0);
	//USB_EXTIinit();
	//Ensure keypad columns output 0 by default
	//HAL_GPIO_WritePin(Col_1_GPIO_Port, Col_1_Pin|Col_2_Pin|Col_3_Pin|Col_4_Pin, GPIO_PIN_RESET);

	//LCD Init
	//lcd_psu_init();
	//Start display timer
	//HAL_TIM_Base_Start_IT(&htim3);

	memset (txbuffer, '\0', 64);  // clear the buffer
	memset (rxbuffer, '\0', 64);  // clear the buffer

	//strncpy((char*)txbuffer, "Hello World From Second MCU\n", 64);

	snprintf((char*)txbuffer, 26, "*STRT,%5.2f,%5.3f,%d,FNSH!", lin_num, cur_num, chstat);

	HAL_UART_Transmit_DMA(&huart1, txbuffer, 64);

	HAL_UART_Receive_DMA (&huart1, rxbuffer, 64);  // Receive 64 Bytes of data
}
 /*
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	adc_current = adcvalues[2];
	adc_linear = adcvalues[1];
	adc_opamp = adcvalues[0];
	adc_switching = adcvalues[3];
	adc_vref = adcvalues[4];
}


void HAL_UART_TxHalfCpltCallback(UART_HandleTypeDef *huart){

}
*/

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	memset (txbuffer, '\0', 64);  // clear the buffer
	snprintf((char*)txbuffer, 26, "*STRT,%05.2f,%5.3f,%d,FNSH!", lin_num, cur_num, chstat);
	HAL_UART_Transmit_DMA(&huart1, txbuffer, 64);
}

/*
void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart){
  //do nothing cause we're a lazy receiver
}
*/

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
  /*
   * Messages are structured as follows *STRT,00.00,0.000,0,FNSH!
   * Here we will search for the unique start character * and check that the
   * full 25 character string is correct. We may have to wrap around our buffer
   * since we may begin reading at any point in the transmission.
   */
	uint8_t rxiter = 0;
	for(int i = 0; i < 64; i++){
		if(rxbuffer[i] == '*'){
			rxiter = i;//Found start condition
			break;
		}
	}
	uint8_t rxbuffercpy[32];
	memset (rxbuffercpy, '\0', 32);  // clear the buffer
	//Copy our message into a buffer that isn't offset
	for(int i = 0; i < 25; i++){
		rxbuffercpy[i] = rxbuffer[rxiter];
		rxiter++;
		if(rxiter >= 64){
			rxiter = 0;
		}
	}

	if( //Check start condition
		(rxbuffercpy[0] == '*' && rxbuffercpy[1] == 'S' && rxbuffercpy[2] == 'T' && rxbuffercpy[3] == 'R' && rxbuffercpy[4] == 'T') &&
		//Check finish condition
		(rxbuffercpy[20] == 'F' && rxbuffercpy[21] == 'N' && rxbuffercpy[22] == 'S' && rxbuffercpy[23] == 'H' && rxbuffercpy[24] == '!') &&
		//Check commas
		(rxbuffercpy[5] == ',' && rxbuffercpy[11] == ',' && rxbuffercpy[17] == ',' && rxbuffercpy[19] == ',') &&
		//Check voltage
		((rxbuffercpy[6] >= '0' && rxbuffercpy[6] <= '9') && (rxbuffercpy[7] >= '0' && rxbuffercpy[7] <= '9') && rxbuffercpy[8] == '.' &&
		(rxbuffercpy[9] >= '0' && rxbuffercpy[9] <= '9') && (rxbuffercpy[10] >= '0' && rxbuffercpy[10] <= '9')) &&
		//Check amperage
		((rxbuffercpy[12] >= '0' && rxbuffercpy[12] <= '9') && rxbuffercpy[13] == '.' && (rxbuffercpy[14] >= '0' && rxbuffercpy[14] <= '9') &&
		(rxbuffercpy[15] >= '0' && rxbuffercpy[15] <= '9') && (rxbuffercpy[16] >= '0' && rxbuffercpy[16] <= '9')) &&
		//Check chstat
		(rxbuffercpy[18] == '0' || rxbuffercpy[18] == '1' || rxbuffercpy[18] == '2')
			){
		//Valid message
		float tempv2 = 0;
		float tempa2 = 0;

		tempv2 += (float)(rxbuffercpy[6]-48) * (float)10.0;
		tempv2 += (float)(rxbuffercpy[7]-48) * (float)1.0;
		tempv2 += (float)(rxbuffercpy[9]-48) / (float)10.0;
		tempv2 += (float)(rxbuffercpy[10]-48) / (float)100.0;

		tempa2 += (float)(rxbuffercpy[12]-48) * (float)1.0;
		tempa2 += (float)(rxbuffercpy[14]-48) / (float)10.0;
		tempa2 += (float)(rxbuffercpy[15]-48) / (float)100.0;
		tempa2 += (float)(rxbuffercpy[16]-48) / (float)1000.0;

		if(tempv2 >= 0.00 && tempv2 <= 12.00){
			first_shot = 1;
			voltnum1 = tempv2;
		}
		if(tempa2 >= 0.00 && tempa2 <= 0.800){
			ampnum1 = tempa2;
		}
		//chstat = rxbuffercpy[18]-48;
	}

	memset (rxbuffer, '\0', 64);  // clear the buffer
	HAL_UART_Receive_DMA (&huart1, rxbuffer, 64);  // Receive 64 Bytes of data
}


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
