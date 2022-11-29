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
#define ADC_OPAMP adc_values_cpy[0]
#define ADC_LINEAR adc_values_cpy[1]
#define ADC_CURRENT adc_values_cpy[2]
#define ADC_SWITCHING adc_values_cpy[3]
#define ADC_VREF adc_values_cpy[4]

/* PID Control for the Linear Voltage Regulator ------------------------------*/
#define P 0.15
#define I 0.15
#define D 0.2
/* PID Linear Section End ----- */

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
uint8_t txbuffer_cpy[64];//Uart TX Buffer copy because transfer is circular
uint8_t rxbuffer[64];//Uart RX Buffer

//Channel numbers
float volt_set_main = 0.0;
float amp_set_main = 0.0;
//volatile float volt_set_aux = 0.0;
//volatile float amp_set_aux = 0.0;

float volt_set_main_old = 0.0;
float amp_set_main_old = 0.0;
//volatile float volt_set_aux_old = 0.0;
//volatile float amp_set_aux_old = 0.0;

//Array for the adc values and vars to hold them
volatile uint16_t adc_values[6];
uint16_t adc_values_cpy[6];
uint16_t* vrefptr = ((uint16_t*)VREFINT_CAL_ADDR_CMSIS);
int8_t chstat_main = 0;
//volatile int8_t chstat_aux_tx = 0;
//volatile int8_t chstat_aux_rx = 0;

//Globals for adc values
float lin_num = 0;
float cur_num = 0;
float op_num = 0;
float swi_num = 0;

//volatile float lin_num_aux = 0;
//volatile float cur_num_aux = 0;

//Global for dac value
uint16_t v1;//dac channel 1 is linear
uint16_t v2;//dac channel 2 is switching

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC_Init(void);
static void MX_DAC_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

void ourInit(void);//Runs several Inits

/* ADC Section Begin ---------------------------------------------------------*/
void update_ADC_watchdog(float val);
/* ADC Section End -----------------------------------------------------------*/

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
  ourInit();
  float error = 0;
  volatile float derivative = 0;
  float integral = 0;
  float error_previous = 0;
  float correction = 0;
  float corrected_volt_set_main;
  float tmpv1;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  //Control channel here

	  uint16_t vrefvalue = (uint16_t) *vrefptr;
	  float vddcalc = (float)3.0 * ((float)vrefvalue / (float)ADC_VREF);

	  float cur_num_temp = ((((float)3.0 * (float)ADC_CURRENT * (float)vrefvalue)/((float)ADC_VREF * (float)4095) / (float)20) / (float)0.15);
	  cur_num  = (cur_num_temp >= 0.0000) ? cur_num_temp : 0.0000;


	  //float cur_num = (((float)vddcalc * (float)*ADC_CURRENT * (float)4095) / (float)20) / (float)0.3;
	  float op_num_temp = ((float)3.0 * ((float)ADC_OPAMP * (float)4.0) * (float)vrefvalue)/((float)ADC_VREF * (float)4095) - ((float)cur_num * (float)0.35);
	  op_num  = (op_num_temp >= 0.0000) ? op_num_temp : 0.0000;


	  float lin_num_temp = ((float)3.0 * ((float)ADC_LINEAR * (float)4.0) * (float)vrefvalue)/((float)ADC_VREF * (float)4095) - ((float)cur_num * (float)0.35);
	  lin_num  = (lin_num_temp >= 0.0000) ? lin_num_temp : 0.0000;

	  //float lin_num = ((float)vddcalc * (float)*ADC_LINEAR * (float)4095) * (float)4;
	  float swi_num_temp = ((float)3.0 * ((float)ADC_SWITCHING * (float)5.0) * (float)vrefvalue)/((float)ADC_VREF * (float)4095);
	  swi_num  = (swi_num_temp >= 0.0000) ? swi_num_temp : 0.0000;

	  /*
	  // Bang-Bang Controller
	  //Try really hard to get the voltage right
	  const float margin = 0.002;
	  //If we are active watch the output adc
	  if(chstat_main == 1){
		  if(lin_num > volt_set_main + margin){
			  if(v1 >= 1){
				  v1--;
			  }
			  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, v1);
		  }
		  else if(lin_num < volt_set_main - margin){
			  if(v1 <= 4094){
				  v1++;
			  }
			  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, v1);
		  }
	  }
	  //if we are inactive watch the opamp output, remove half a volt because it's better to undershoot than overshoot
	  else{
		  if(op_num > (volt_set_main - 0.5) + margin){
			  if(v1 >= 1){
				  v1--;
			  }
			  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, v1);
		  }
		  else if(op_num < (volt_set_main - 0.5) - margin){
			  if(v1 <= 4094){
				  v1++;
			  }
			  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, v1);
		  }
	  }
	   */

//	  /*
	  //PID
	  if (chstat_main) {
		  error = lin_num - volt_set_main;
		  integral += error;
		  if (integral > (float)4095.0) {
			  integral = (float)4095;
		  } else if (integral < (float)-4095.0) {
			  integral = (float)-4095.0;
		  }
		  derivative = error - error_previous;
		  error_previous = error;
		  correction = P * error + I * integral + D * derivative;
		  corrected_volt_set_main = volt_set_main - correction;
		  tmpv1 = (((((float)corrected_volt_set_main / (float)4.0) + ((float)0.446974063 / (float)4.0)) * (float)4095) / (float)vddcalc);
		  if (tmpv1 > 4095) {
			  tmpv1 = 4095;
		  } else if (tmpv1 < 0) {
			  tmpv1 = 0;
		  }
		  v1 = (uint16_t) tmpv1;
	  } else {
		  if(op_num > (volt_set_main - 1)){
			  if(v1 >= 1){
				  v1--;
			  }
			  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, v1);
		  }
		  else if(op_num < (volt_set_main - 1)){
			  if(v1 <= 4094){
				  v1++;
			  }
			  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, v1);
		  }
	  }
//	*/

	  /*
	   * The calculation we do to determine what we need to set the dac for the switching regulator to is determined by the following formula
	   * 1.235 = (R3*R2*Vout + R1*R2Vdac) / (R3*R2 + R1*R3 + R1*R2) where R1 = 10k, R2 = 1.2k, R3 = 2.4K
	   * This equation would be a little cumbersome to calculate every time we loop so I have simplified it on paper to the following formula
	   * Vdac = 4.001400 - 0.240000*Vout
	   */

	  volatile float temp = ( ((float)4.001400 - ((float)0.240000*((float)volt_set_main + (float)0.5))) * (float)4095 / (float)vddcalc);
	  if(temp <= 0){
		  v2 = 0;
	  }
	  else if(temp >= 4095){
		  v2 = 4095;
	  }
	  else{
		  v2 = (uint16_t)temp;
	  }

	  if(volt_set_main > volt_set_main_old){
		  //If the new voltage is greater than the old voltage set we need to increase the switching regulators voltage first
		  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, v2);
		  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, v1);
	  }
	  else if(volt_set_main < volt_set_main_old){
		  //If the new voltage is lower than the old voltage set we need to decrease the switching regulators voltage last
		  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, v1);
		  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, v2);
	  }
	  else{
		  //We shouldn't have to do anything in the case that the old voltage is equal to the new voltage but we can allow PID to keep going
		  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, v1);
		  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, v2);
	  }

	  if(chstat_main == 1 && ADC_OPAMP >= 5){
		  HAL_GPIO_WritePin(Channel_Shutdown_GPIO_Port, Channel_Shutdown_Pin, GPIO_PIN_RESET);
	  }
	  else{
		  HAL_GPIO_WritePin(Channel_Shutdown_GPIO_Port, Channel_Shutdown_Pin, GPIO_PIN_SET);
	  }

	  //Keep the watchdog threshold up to date with changes to vdd
	  update_ADC_watchdog(amp_set_main);

	  HAL_Delay(1);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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

  ADC_AnalogWDGConfTypeDef AnalogWDGConfig = {0};
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
  hadc.Init.NbrOfConversion = 6;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = ENABLE;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the analog watchdog
  */
  AnalogWDGConfig.WatchdogMode = ADC_ANALOGWATCHDOG_SINGLE_REG;
  AnalogWDGConfig.Channel = ADC_CHANNEL_2;
  AnalogWDGConfig.ITMode = ENABLE;
  AnalogWDGConfig.HighThreshold = 4095;
  AnalogWDGConfig.LowThreshold = 0;
  if (HAL_ADC_AnalogWDGConfig(&hadc, &AnalogWDGConfig) != HAL_OK)
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

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = ADC_REGULAR_RANK_6;
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
  huart1.Init.BaudRate = 9600;
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Channel_Shutdown_GPIO_Port, Channel_Shutdown_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : Unused_Pin_1_Pin Unused_Pin_2_Pin Unused_Pin_3_Pin */
  GPIO_InitStruct.Pin = Unused_Pin_1_Pin|Unused_Pin_2_Pin|Unused_Pin_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : Channel_Shutdown_Pin */
  GPIO_InitStruct.Pin = Channel_Shutdown_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Channel_Shutdown_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Unused_Pin_4_Pin Unused_Pin_14_Pin Unused_Pin_15_Pin Unused_Pin_16_Pin
                           Unused_Pin_17_Pin */
  GPIO_InitStruct.Pin = Unused_Pin_4_Pin|Unused_Pin_14_Pin|Unused_Pin_15_Pin|Unused_Pin_16_Pin
                          |Unused_Pin_17_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Unused_Pin_5_Pin Unused_Pin_6_Pin Unused_Pin_7_Pin Unused_Pin_8_Pin
                           Unused_Pin_9_Pin Unused_Pin_10_Pin Unused_Pin_11_Pin Unused_Pin_12_Pin
                           Unused_Pin_13_Pin Unused_Pin_18_Pin Unused_Pin_19_Pin Unused_Pin_20_Pin
                           Unused_Pin_21_Pin Unused_Pin_22_Pin Unused_Pin_23_Pin Unused_Pin_24_Pin */
  GPIO_InitStruct.Pin = Unused_Pin_5_Pin|Unused_Pin_6_Pin|Unused_Pin_7_Pin|Unused_Pin_8_Pin
                          |Unused_Pin_9_Pin|Unused_Pin_10_Pin|Unused_Pin_11_Pin|Unused_Pin_12_Pin
                          |Unused_Pin_13_Pin|Unused_Pin_18_Pin|Unused_Pin_19_Pin|Unused_Pin_20_Pin
                          |Unused_Pin_21_Pin|Unused_Pin_22_Pin|Unused_Pin_23_Pin|Unused_Pin_24_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void ourInit(void){
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
	HAL_ADC_Start_DMA(&hadc, (uint32_t*)&adc_values, 6);// start the adc in dma mode
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
	memset (txbuffer_cpy, '\0', 64);  // clear the buffer
	memset (rxbuffer, '\0', 64);  // clear the buffer

	//strncpy((char*)txbuffer, "Hello World From Second MCU\n", 64);

	snprintf((char*)txbuffer, 32, "*STRT,%5.2f,%5.3f,%d,FNSH!", lin_num, cur_num, chstat_main);

	HAL_UART_Transmit_DMA(&huart1, txbuffer, 64);

	HAL_UART_Receive_DMA (&huart1, rxbuffer, 64);  // Receive 64 Bytes of data
}

/* ADC Section Begin ---------------------------------------------------------*/
void update_ADC_watchdog(float val){
	//Don't want to stop the dma/adc and deinit/reinit it to change the watchdog, plus we need to calculate vdd to get a good value

	uint16_t vrefvalue = (uint16_t) *vrefptr;
	float vddcalc = (float)3.0 * ((float)vrefvalue / (float)ADC_VREF);
	volatile uint16_t amp = (uint16_t)( ((float)val * (float)0.15 * (float)20.0) * (float)4095 / (float)vddcalc);

	//Special case for "unlimited" current
	if(val == 0.0){
		ADC1->HTR = 4095;
	}
	else if(amp >= 4095.0){
		ADC1->HTR = 4095;
	}
	else if(amp < 0.0){
		//How?
		ADC1->HTR = 0;
	}
	else{
		ADC1->HTR = amp;
	}
}
/* ADC Section End -----------------------------------------------------------*/

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc){
	adc_values_cpy[0] = adc_values[0];
	adc_values_cpy[1] = adc_values[1];
	adc_values_cpy[2] = adc_values[2];
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	adc_values_cpy[3] = adc_values[3];
	adc_values_cpy[4] = adc_values[4];
	adc_values_cpy[5] = adc_values[5];
}

void HAL_ADC_LevelOutOfWindowCallback(ADC_HandleTypeDef* hadc)
{
	HAL_GPIO_WritePin(Channel_Shutdown_GPIO_Port, Channel_Shutdown_Pin, GPIO_PIN_SET);
	chstat_main = 2;
}

/* Interrupt Callback Section Begin ------------------------------------------*/

void HAL_UART_TxHalfCpltCallback(UART_HandleTypeDef *huart){
	/*
	 * The buffer is 64 bytes wide, the message is 32 bytes wide, this allows us to do some processing between messages
	 * as well as catch messages we are receiving if they aren't synchronized
	 * When half complete we have just sent out the message we want to send and the remaining 32 bytes are all 0
	 * During this stage of sending 0's we can copy the next message into our buffer because we don't care about
	 * overwriting 0s
	 */
	memcpy(txbuffer, txbuffer_cpy, 64);  // copy the data to the buffer
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	/*
	 * The buffer is 64 bytes wide, the message is 32 bytes wide, this allows us to do some processing between messages
	 * as well as catch messages we are receiving if they aren't synchronized
	 * When complete we have just sent out a bunch of 0s and are preparing to send out the next message
	 * During this stage we don't want to overwrite the first half of the buffer because we are currently sending it
	 * but we can prepare the next message to send
	 */
	memset (txbuffer_cpy, '\0', 64);  // clear the buffer
	snprintf((char*)txbuffer_cpy, 32, "*STRT,%05.2f,%5.3f,%d,FNSH!", lin_num, cur_num, chstat_main);
}

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart){
  //do nothing cause we're a lazy receiver
}

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
		//Check chstat_main
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
			volt_set_main_old = volt_set_main;
			volt_set_main = tempv2;
		}
		if(tempa2 >= 0.00 && tempa2 <= 0.8001){
			amp_set_main_old = amp_set_main;
			amp_set_main = tempa2;
		}

		int8_t chstat_temp = rxbuffercpy[18]-48;
		//If we are shutdown we can accept a channel enable request
		if(chstat_main == 0){
			if(chstat_temp == 1){
				chstat_main = chstat_temp;
			}
		}
		//If we are active we can accept a channel shutdown request
		else if(chstat_main == 1){
			if(chstat_temp == 0){
				chstat_main = 0;
			}
		}
		//If we are current limited we can accept a channel shutdown request
		else if(chstat_main == 2){
			if(chstat_temp == 0){
				chstat_main = 0;
			}
		}
	}

	memset (rxbuffer, '\0', 64);  // clear the buffer
	HAL_UART_Receive_DMA (&huart1, rxbuffer, 64);  // Receive 64 Bytes of data
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart){
	HAL_UART_Receive_DMA (&huart1, rxbuffer, 64); //Try again!
}
/* Interrupt Callback Section End ----------------------------------------*/

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
