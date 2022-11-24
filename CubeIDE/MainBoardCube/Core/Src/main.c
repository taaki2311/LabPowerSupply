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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "string.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
enum KEYPAD_ENUM { WAIT, V1, A1, V2, A2 };
enum ROTARY_STATE { NOTURN, CWTURN, CCWTURN };
enum ROTARY_MODE { VALUE, INDEX };
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* LCD Section Begin ---------------------------------------------------------*/
#define SLAVE_ADDRESS_LCD 0x4E // change this according to your setup
const uint8_t customChar[64] = {
	0x08,0x04,0x02,0x1F,0x02,0x04,0x08,0x00, // Right Arrow
	0x04,0x04,0x04,0x4,0x04,0x04,0x04,0x04,  //	Vertical Line
	0x00,0x00,0x00,0x0,0x00,0x00,0x00,0x00,  // Custom Char 2
	0x00,0x00,0x00,0x0,0x00,0x00,0x00,0x00,  // Custom Char 3
	0x00,0x00,0x00,0x0,0x00,0x00,0x00,0x00,  // Custom Char 4
	0x00,0x00,0x00,0x0,0x00,0x00,0x00,0x00,  // Custom Char 5
	0x00,0x00,0x00,0x0,0x00,0x00,0x00,0x00,  // Custom Char 6
	0x00,0x00,0x00,0x0,0x00,0x00,0x00,0x00   // Custom Char 7
};
/* LCD Section End -----------------------------------------------------------*/

/* PID Control for the Linear Voltage Regulator ------------------------------*/
#define P 0.1
#define I 0.1
#define D 0.1
/* PID Linear Section End ----------------------------------------------------*/

/* Button States Section Begin -----------------------------------------------*/
#define PRESSED GPIO_PIN_RESET
#define RELEASED GPIO_PIN_SET
/* Button States Section End -------------------------------------------------*/

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

DAC_HandleTypeDef hdac;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */

/* USB Section Begin ---------------------------------------------------------*/
EXTI_HandleTypeDef hexti1;
uint8_t usbbuffer[128]; // USB buffer
uint8_t txbuffer[64]; // UART TX Buffer
uint8_t rxbuffer[64]; // UART RX Buffer
/* USB Section End -----------------------------------------------------------*/

/* Keypad Section Begin ------------------------------------------------------*/
volatile int8_t rowpin = -1;
volatile uint8_t kpedge = 1; // 0 = falling edge, 1 = rising edge

const uint8_t keypadlength = 5;
volatile char keypadarr[5] = {'z','z','z','z','z'}; // 'z' is null
volatile uint8_t keypaditerator = keypadlength - 1;
volatile uint8_t keypaddecimal = 0;
volatile enum KEYPAD_ENUM kpenum = WAIT;
/* Keypad Section End --------------------------------------------------------*/

/* Rotary Encoder Section Begin ----------------------------------------------*/
volatile enum ROTARY_STATE Rot_State = NOTURN;
volatile GPIO_PinState Rot_SW_State = RELEASED;
volatile enum ROTARY_MODE Rot_Mode = VALUE;
/* Rotary Encoder End --------------------------------------------------------*/

//Channel numbers
volatile float voltnum1 = 0.0;
volatile float ampnum1 = 0.0;
volatile float voltnum2 = 0.0;
volatile float ampnum2 = 0.0;

// Additional Control Variables
volatile uint8_t first_shot = 0;
volatile uint8_t chstat2 = 0;

//Array for the ADC values and the variables to hold them
uint16_t adcvalues[5];
uint16_t adc_current = 0;
uint16_t adc_linear = 0;
uint16_t adc_opamp = 0;
uint16_t adc_switching = 0;
uint16_t adc_vref = 0;
uint16_t* vrefptr = ((uint16_t*)VREFINT_CAL_ADDR_CMSIS);

//Globals for ADC values
uint16_t v1;
float lin_num = 0;
float cur_num = 0;
float op_num = 0;
float swi_num = 0;

float slin_num = 0;
float scur_num = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC_Init(void);
static void MX_DAC_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM9_Init(void);
static void MX_TIM10_Init(void);
static void MX_TIM11_Init(void);
/* USER CODE BEGIN PFP */

void our_init(void); //Runs several Inits

/* USB Section Begin ---------------------------------------------------------*/
void USB_EXTIinit(void);
void USB_Interrupt_Callback(void);
void EXTI1_IRQHandler(void);
/* USB Section End -----------------------------------------------------------*/

/* LCD Section Begin ---------------------------------------------------------*/
void lcd_init (void);   // initialize LCD
void lcd_send_cmd (char cmd);  // send command to the LCD
void lcd_send_data (char data);  // send data to the LCD
void lcd_send_string (char *str);  // send string to the LCD
void lcd_put_cur(int row, int col);  // put cursor at the entered position row (0 or 1), col (0-15);
void lcd_clear (void);
void lcd_cursorblink_onoff( uint8_t cursor_status, uint8_t blink_status );
void lcd_createChar(void);
void lcd_psu_init(void);
void lcd_update_voltage(uint8_t channel, float num);
void lcd_update_amperage(uint8_t channel, float num);
void lcd_psu_update(void);
/* LCD Section End -----------------------------------------------------------*/

/* Keypad Section Begin ------------------------------------------------------*/
void update_keypad(char num);
void clear_keypad(void);
float translate_keypad(void);
uint8_t check_keypad(uint8_t which);
void keypad_sm(char num);
void row_input(void);
void column_input(void);
void keypad_decode(uint8_t row_pin);
void set_rowpin(uint8_t row_num);
/* Keypad Section End --------------------------------------------------------*/

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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_USB_DEVICE_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM9_Init();
  MX_TIM10_Init();
  MX_TIM11_Init();
  /* USER CODE BEGIN 2 */
  our_init();
  //HAL_GPIO_WritePin(Channel_Shutdown_GPIO_Port, Channel_Shutdown_Pin, GPIO_PIN_RESET);
  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 4095);

  float error = 0;
  float derivative = 0;
  float integral = 0;
  float error_previous = 0;
  float correction = 0;
  float correctedvoltnum1;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  if(voltnum1 <= 0.00){
		  HAL_GPIO_WritePin(Channel_Shutdown_GPIO_Port, Channel_Shutdown_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(Status_LED_1_GPIO_Port, Status_LED_2_Pin, GPIO_PIN_RESET);
	  }

	  uint16_t vrefvalue = (uint16_t)*vrefptr;
	  float vddcalc = (float)3.0 * ((float)vrefvalue / (float)adc_vref);

	  float cur_num_temp = ((((float)3.0 * (float)adc_current * (float)vrefvalue)/((float)adc_vref * (float)4095) / (float)20) / (float)0.15);
	  if (cur_num_temp >= 0.0000) {
		  cur_num = cur_num_temp;
	  } else {
		  cur_num = 0.0000;
	  }

	  //float cur_num = (((float)vddcalc * (float)adc_current * (float)4095) / (float)20) / (float)0.3;
	  float op_num_temp = ((float)3.0 * ((float)adc_opamp * (float)4.0) * (float)vrefvalue)/((float)adc_vref * (float)4095) - ((float)cur_num * (float)0.35);
	  if (op_num_temp >= 0.0000) {
		  op_num = op_num_temp;
	  } else {
		  op_num = 0.0000;
	  }

	  float lin_num_temp = ((float)3.0 * ((float)adc_linear * (float)4.0) * (float)vrefvalue)/((float)adc_vref * (float)4095) - ((float)cur_num * (float)0.35);
	  if (lin_num_temp >= 0.0000) {
		  lin_num = lin_num_temp;
	  } else {
		  lin_num = 0.0000;
	  }
	  //float lin_num = ((float)vddcalc * (float)adc_linear * (float)4095) * (float)4;
	  float swi_num_temp = ((float)3.0 * ((float)adc_switching * (float)5.0) * (float)vrefvalue)/((float)adc_vref * (float)4095);
	  if(swi_num_temp >= 0.0000){
		  swi_num = swi_num_temp;
	  } else {
		  swi_num = 0.0000;
	  }

	  if (first_shot) {
		  v1 = (uint16_t)((( (((float)voltnum1) / (float)4.0) + ((float)0.446974063 / (float)4.0)) * (float)4095) / (float)vddcalc);
		  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, v1);
		  correctedvoltnum1 = voltnum1;
		  integral = 0;
		  error = 0;
		  derivative = 0;
		  first_shot = 0;
	  }



/*
	  else { // Bang-Bang Controller
		  //Try really hard to get the voltage right
		  const float margin = 0.05;
		  if (lin_num > (voltnum1 + margin)) {
			  v1--;
			  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, v1);
		  } else if (lin_num < (voltnum1 - margin)) {
			  v1++;
			  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, v1);
		  }
	  }
*/



	  else {
		  error = lin_num - voltnum1;
		  integral += error;
		  derivative = error - error_previous;
		  error_previous = error;
		  correction = P * error + I * integral + D * derivative;
		  correctedvoltnum1 = voltnum1 - correction;
		  if (correctedvoltnum1 > 12.0) {
			  correctedvoltnum1 = 12.0;
		  } else if(correctedvoltnum1 < 0.0) {
			  correctedvoltnum1 = 0.0;
		  }
		  v1 = (uint16_t)((((correctedvoltnum1 / (float)4.0) + ((float)0.446974063 / (float)4.0)) * (float)4095) / (float)vddcalc);
		  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, v1);
	  }

	  if(voltnum1 > 0.00){
		  HAL_GPIO_WritePin(Channel_Shutdown_GPIO_Port, Channel_Shutdown_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(Status_LED_1_GPIO_Port, Status_LED_2_Pin, GPIO_PIN_SET);
	  }
	  if(chstat2 == 1){
		  HAL_GPIO_WritePin(Status_LED_1_GPIO_Port, Status_LED_1_Pin, GPIO_PIN_SET);
	  }

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
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 156;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 32000;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 32000;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 500;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 32000;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 10;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 32000;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 20;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim9, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 32000;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 500;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim10, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 32000;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 500;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim11, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Channel_Shutdown_GPIO_Port, Channel_Shutdown_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, Status_LED_1_Pin|Status_LED_2_Pin|Col_1_Pin|Col_2_Pin
                          |Col_3_Pin|Col_4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Channel_Shutdown_Pin */
  GPIO_InitStruct.Pin = Channel_Shutdown_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Channel_Shutdown_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Status_LED_1_Pin Status_LED_2_Pin Col_1_Pin Col_2_Pin
                           Col_3_Pin Col_4_Pin */
  GPIO_InitStruct.Pin = Status_LED_1_Pin|Status_LED_2_Pin|Col_1_Pin|Col_2_Pin
                          |Col_3_Pin|Col_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : Rot_CLK_Pin Rot_SW_Pin Row_1_Pin Row_2_Pin
                           Row_3_Pin Row_4_Pin Row_5_Pin */
  GPIO_InitStruct.Pin = Rot_CLK_Pin|Rot_SW_Pin|Row_1_Pin|Row_2_Pin
                          |Row_3_Pin|Row_4_Pin|Row_5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : Rot_DT_Pin */
  GPIO_InitStruct.Pin = Rot_DT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Rot_DT_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
	USB_EXTIinit();
	//Ensure keypad columns output 0 by default
	HAL_GPIO_WritePin(Col_1_GPIO_Port, Col_1_Pin|Col_2_Pin|Col_3_Pin|Col_4_Pin, GPIO_PIN_RESET);

	//LCD Init
	lcd_psu_init();
	//Start display timer
	HAL_TIM_Base_Start_IT(&htim3);

	memset (usbbuffer, '\0', 128);  // clear the buffer
	memset (txbuffer, '\0', 64);  // clear the buffer
	memset (rxbuffer, '\0', 64);  // clear the buffer

	HAL_UART_Receive_DMA (&huart1, rxbuffer, 64);  // Receive 64 Bytes of data

	snprintf((char*)txbuffer, 32, "*STRT,%05.2f,%5.3f,%d,FNSH!", voltnum2, ampnum2, chstat2);
	HAL_UART_Transmit_DMA(&huart1, txbuffer, 64);
}

/* USB Section Begin ---------------------------------------------------------*/
void USB_EXTIinit(void)
{
	EXTI_ConfigTypeDef ExtiConfig;

	ExtiConfig.Line = EXTI_LINE_1;
	ExtiConfig.Mode = EXTI_MODE_INTERRUPT;
	ExtiConfig.Trigger = EXTI_TRIGGER_RISING_FALLING;
	HAL_EXTI_SetConfigLine(&hexti1, &ExtiConfig);

	//The function below doesn't work and I don't know why so we just call our callback in the irqhandler
	//HAL_EXTI_RegisterCallback(&hexti1, HAL_EXTI_COMMON_CB_ID, USB_Interrupt_Callback);

	HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI1_IRQn);
}

void EXTI1_IRQHandler(void)
{
	__HAL_GPIO_EXTI_CLEAR_IT(EXTI1_IRQn);
	USB_Interrupt_Callback();
}
/* USB Section End -----------------------------------------------------------*/

/* LCD Section Begin ---------------------------------------------------------*/
/*
 * The code for every function here except for lcd_createChar was taken from
 * https://controllerstech.com/lcd-20x4-using-i2c-with-stm32/
 * The remaining function was taken from
 * https://circuitdigest.com/microcontroller-projects/custom-characters-on-lcd-using-pic16f877a
 */
void lcd_send_cmd (char cmd)
{
	char data_u, data_l;
	uint8_t data_t[4];
	data_u = (cmd&0xf0);
	data_l = ((cmd<<4)&0xf0);
	data_t[0] = data_u|0x0C;  //en=1, rs=0
	data_t[1] = data_u|0x08;  //en=0, rs=0
	data_t[2] = data_l|0x0C;  //en=1, rs=0
	data_t[3] = data_l|0x08;  //en=0, rs=0
	HAL_I2C_Master_Transmit (&hi2c1, SLAVE_ADDRESS_LCD,(uint8_t *) data_t, 4, 100);
}

void lcd_send_data (char data)
{
	char data_u, data_l;
	uint8_t data_t[4];
	data_u = (data&0xf0);
	data_l = ((data<<4)&0xf0);
	data_t[0] = data_u|0x0D;  //en=1, rs=0
	data_t[1] = data_u|0x09;  //en=0, rs=0
	data_t[2] = data_l|0x0D;  //en=1, rs=0
	data_t[3] = data_l|0x09;  //en=0, rs=0
	HAL_I2C_Master_Transmit (&hi2c1, SLAVE_ADDRESS_LCD,(uint8_t *) data_t, 4, 100);
}

void lcd_cursorblink_onoff( uint8_t cursor_status, uint8_t blink_status)
{
	if (blink_status == 1 && cursor_status == 1) {
		lcd_send_cmd(0x0F);
	} else if (blink_status == 0 && cursor_status == 1) {
		lcd_send_cmd(0x0E);
	} else if (blink_status == 1 && cursor_status == 0) {
		lcd_send_cmd(0x0D);
	} else if (blink_status == 0 && cursor_status == 0) {
		lcd_send_cmd(0x0C);
	}
}

void lcd_blink_onoff( uint8_t status )
{
	(status) ? lcd_send_cmd(0x0D) : lcd_send_cmd(0x0C);
}

void lcd_clear (void)
{
	lcd_send_cmd (0x00);
	for (int i=0; i<100; i++) {
		lcd_send_data (' ');
	}
}

void lcd_put_cur(int row, int col)
{
    switch (row) {
		case 0:
            col += 0x80;
            break;
        case 1:
            col += 0xC0; //C0
            break;
        case 2:
            col += 0x94; //0x80|0x14 for row 3 col 2
            break;
        case 3:
            col += 0xD4;
            break;
        default:
        	// Error
        	break;
    }
    lcd_send_cmd (col);
}


void lcd_init (void)
{
	uint8_t i=0;
	HAL_Delay(100);
	for(i=0;i<3;i++){ //sending 3 times: select 4-bit mode
		lcd_send_cmd(0x03);
		HAL_Delay(45);
	}
	lcd_send_cmd (0x02);
	HAL_Delay(100);
	lcd_send_cmd (0x28);
	HAL_Delay(1);
	lcd_send_cmd (0x0c);
	HAL_Delay(1);
	lcd_send_cmd (0x80);
	HAL_Delay(1);
	lcd_createChar();
	lcd_clear();
}

void lcd_send_string (char *str)
{
	while (*str) lcd_send_data (*str++);
}

void lcd_createChar(void)
{
	//*** Load custom char into the CGROM***//////
	lcd_send_cmd(0x40);   // Set CGRAM Address
	HAL_Delay(1);
	lcd_send_cmd(0x00);   // .. set CGRAM Address
	HAL_Delay(1);
	for(int i = 0; i < 64 ; i++){
		lcd_send_data(customChar[i]);
		HAL_Delay(1);
	}
	lcd_send_cmd(0);      // Return to Home
	HAL_Delay(1);
	lcd_send_cmd(2);      // .. return to Home
	HAL_Delay(1);
	//*** Loading custom char complete***//////
}

void lcd_psu_init(void){
	lcd_init();

	lcd_put_cur(0, 0);
	lcd_send_string("V1:0.00V ");
	lcd_send_data((uint8_t)1); // Vertical Line
	lcd_send_string(" V1");
	lcd_send_data((uint8_t)0); // Right Arrow
	lcd_send_string("0.00V");

	lcd_put_cur(1, 0);
	lcd_send_string("A1:0.00A ");
	lcd_send_data((uint8_t)1);
	lcd_send_string(" A1");
	lcd_send_data((uint8_t)0);
	lcd_send_string("0.00A");

	lcd_put_cur(2, 0);
	lcd_send_string("V2:0.00V ");
	lcd_send_data((uint8_t)1);
	lcd_send_string(" V2");
	lcd_send_data((uint8_t)0);
	lcd_send_string("0.00V");

	lcd_put_cur(3, 0);
	lcd_send_string("A2:0.00A ");
	lcd_send_data((uint8_t)1);
	lcd_send_string(" A2");
	lcd_send_data((uint8_t)0);
	lcd_send_string("0.00A");
}

void lcd_psu_update(void){
	lcd_cursorblink_onoff(0,0);
	switch(kpedge) {

	case WAIT:
		lcd_update_voltage(1,voltnum2);
		lcd_update_amperage(1,ampnum2);
		lcd_update_voltage(2,slin_num);
		lcd_update_amperage(2,scur_num);
		lcd_update_voltage(3,voltnum1);
		lcd_update_amperage(3,ampnum1);
		lcd_update_voltage(4,lin_num);
		lcd_update_amperage(4,cur_num);
		break;

	case V2:
		//lcd_update_voltage(1,voltnum2);
		lcd_update_amperage(1,ampnum2);
		lcd_update_voltage(2,slin_num);
		lcd_update_amperage(2,scur_num);
		lcd_update_voltage(3,voltnum1);
		lcd_update_amperage(3,ampnum1);
		lcd_update_voltage(4,lin_num);
		lcd_update_amperage(4,cur_num);
		lcd_put_cur(0,3);
		lcd_send_string("      ");
		lcd_put_cur(0,3);
		lcd_cursorblink_onoff(1,1);
		//Print the keypad array
		for (uint8_t i = 0; i < keypadlength; i++) {
			if (keypadarr[i] != 'z') {
				lcd_send_data(keypadarr[i]);
			}
		}
		break;

	case V1:
		lcd_update_voltage(1,voltnum2);
		lcd_update_amperage(1,ampnum2);
		lcd_update_voltage(2,slin_num);
		lcd_update_amperage(2,scur_num);
		//lcd_update_voltage(3,voltnum1);
		lcd_update_amperage(3,ampnum1);
		lcd_update_voltage(4,lin_num);
		lcd_update_amperage(4,cur_num);
		lcd_put_cur(2,3);
		lcd_send_string("      ");
		lcd_put_cur(2,3);
		lcd_cursorblink_onoff(1,1);
		//Print the keypad array
		for (uint8_t i = 0; i < keypadlength; i++) {
			if (keypadarr[i] != 'z') {
				lcd_send_data(keypadarr[i]);
			}
		}
		break;

	case A2:
		lcd_update_voltage(1,voltnum2);
		//lcd_update_amperage(1,ampnum2);
		lcd_update_voltage(2,slin_num);
		lcd_update_amperage(2,scur_num);
		lcd_update_voltage(3,voltnum1);
		lcd_update_amperage(3,ampnum1);
		lcd_update_voltage(4,lin_num);
		lcd_update_amperage(4,cur_num);
		lcd_put_cur(1,3);
		lcd_send_string("      ");
		lcd_put_cur(1,3);
		lcd_cursorblink_onoff(1,1);
		//Print the keypad array
		for (uint8_t i = 0; i < keypadlength; i++) {
			if (keypadarr[i] != 'z') {
				lcd_send_data(keypadarr[i]);
			}
		}
		break;

	case A1:
		lcd_update_voltage(1,voltnum2);
		lcd_update_amperage(1,ampnum2);
		lcd_update_voltage(2,slin_num);
		lcd_update_amperage(2,scur_num);
		lcd_update_voltage(3,voltnum1);
		//lcd_update_amperage(3,ampnum1);
		lcd_update_voltage(4,lin_num);
		lcd_update_amperage(4,cur_num);
		lcd_put_cur(3,3);
		lcd_send_string("      ");
		lcd_put_cur(3,3);
		lcd_cursorblink_onoff(1,1);
		//Print the keypad array
		for (uint8_t i = 0; i < keypadlength; i++) {
			if (keypadarr[i] != 'z') {
				lcd_send_data(keypadarr[i]);
			}
		}
		break;

	default:
		//Error
		break;
	}
}

/* LCD Section End -----------------------------------------------------------*/

/* Keypad Section Begin ------------------------------------------------------*/
void lcd_update_voltage(uint8_t channel, float num){
	char kpbuff[8];
	snprintf(kpbuff, 6, "%.2f", num);

	switch (channel) {
	case 1: // V1 Set
		lcd_put_cur(0,3);
		lcd_send_string("      ");//Clear current number
		lcd_put_cur(0,3);
		lcd_send_string(kpbuff);
		lcd_send_string("V");
		break;

	case 2: // V1 Out
		lcd_put_cur(0,14);
		lcd_send_string("      ");//Clear current number
		lcd_put_cur(0,14);
		lcd_send_string(kpbuff);
		lcd_send_string("V");
		break;

	case 3: // V2 Set
		lcd_put_cur(2,3);
		lcd_send_string("      ");//Clear current number
		lcd_put_cur(2,3);
		lcd_send_string(kpbuff);
		lcd_send_string("V");
		break;

	case 4: // V2 Out
		lcd_put_cur(2,14);
		lcd_send_string("      ");//Clear current number
		lcd_put_cur(2,14);
		lcd_send_string(kpbuff);
		lcd_send_string("V");
		break;

	default:
		//Error
		break;
	}
}

void lcd_update_amperage(uint8_t channel, float num) {
	char kpbuff[8];
	snprintf(kpbuff, 5, "%.2f", num);

	switch (channel) {
	case 1: // A1 Set
		lcd_put_cur(1,3);
		lcd_send_string("      ");//Clear current number
		lcd_put_cur(1,3);
		lcd_send_string(kpbuff);
		lcd_send_string("A");
		break;

	case 2: // A1 Out
		lcd_put_cur(1,14);
		lcd_send_string("      ");//Clear current number
		lcd_put_cur(1,14);
		lcd_send_string(kpbuff);
		lcd_send_string("A");
		break;

	case 3: // A2 Set
		lcd_put_cur(3,3);
		lcd_send_string("      ");//Clear current number
		lcd_put_cur(3,3);
		lcd_send_string(kpbuff);
		lcd_send_string("A");
		break;

	case 4: // A2 Out
		lcd_put_cur(3,14);
		lcd_send_string("      ");//Clear current number
		lcd_put_cur(3,14);
		lcd_send_string(kpbuff);
		lcd_send_string("A");
		break;

	default:
		// Error
		break;
	}
}

/*
 * This function shifts character into and out of a 5 element array from the right.
 * This allows us to think about the number stored in the array as a normal number
 * that is read from left to right.
 * Passing a 'z' to this function will delete the last character.
 * Passing a '.' to this function will shift in a '.' if there is not already one in the array.
 * Passing a number ('1' not 1) to this function will shift in that number but will only
 * allow up to 2 numbers before a decimal and up to 2 numbers after a decimal.
 */
void update_keypad(char num){
	if (num == 'z') {
		if(keypaditerator < keypadlength-1){
			//Update decimal "bool" if we remove a decimal
			if(keypadarr[4] == '.'){
				keypaddecimal = 0;
			}
			//shift out last entry if array isnt empty
			for(int i = 3; i >= 0; i--){
				keypadarr[i+1] = keypadarr[i];
			}
			keypadarr[0] = num;
			keypaditerator++;
		}
	} else if(num == '.') {
		if (keypaditerator >= keypadlength-3 && keypaddecimal == 0) {
			//shift in new entry with up to two numbers preceding it
			for (int i = 1; i < keypadlength; i++) {
				keypadarr[i-1] = keypadarr[i];
			}
			keypadarr[keypadlength-1] = num;
			keypaddecimal = 1;
			keypaditerator--;
		}
	}
	else if(num >= '0' && num <= '9'){
		//Allow entries if only up to one number has been entered
		if (keypaditerator > 2) {
			//shift in new entry
			for (int i = 1; i < keypadlength; i++) {
				keypadarr[i-1] = keypadarr[i];
			}
			keypadarr[keypadlength-1] = num;
			keypaditerator--;
		} else if(keypaditerator <= 1 && keypaddecimal == 1 && keypadarr[1] == 'z' && keypadarr[2] != '.') {
			//shift in new entry
			for (int i = 1; i < keypadlength; i++) {
				keypadarr[i-1] = keypadarr[i];
			}
			keypadarr[keypadlength-1] = num;
			keypaditerator--;
		}
		//Disallow third decimal place
		else if (keypaditerator <= 1 && keypaddecimal == 1 && keypadarr[1] == 'z') {
			//Do nothing
		}
		//Allow numbers after decimal place
		else if (keypaditerator > 0 && keypaddecimal == 1) {
			//shift in new entry
			for (int i = 1; i < keypadlength; i++) {
				keypadarr[i-1] = keypadarr[i];
			}
			keypadarr[keypadlength-1] = num;
			keypaditerator--;
		}
		//Allow second decimal place when num > 10
		else if ((keypaditerator >= 0) && (keypaddecimal == 1) && (translate_keypad() >= 10.0)) {
			//shift in new entry
			for(int i = 1; i < keypadlength; i++){
				keypadarr[i-1] = keypadarr[i];
			}
			keypadarr[keypadlength-1] = num;
			keypaditerator--;
		}
	}
}

void clear_keypad(void)
{
	for (uint8_t i = 0; i < keypadlength; i++) {
		keypadarr[i] = 'z';
	}
}

float translate_keypad(void)
{
	float num = 0;
	int decimallocation = -1;

	for (int i = 0; i < keypadlength; i++) {
		if (keypadarr[i] == '.') {
			decimallocation = i;
		}

		if (decimallocation == -1) {
			//No decimal in array
			int count = 1;
			for (int i = keypadlength-1; i >= 0; i--) {
				if (keypadarr[i] >= '0' && keypadarr[i] <= '9') {
					num += ( (int)keypadarr[i] - (int)'0' ) * (count);
					count = count * 10;
				}
			}
		} else {
			//First do numbers to the left of the decimal
			int count = 1;
			for (int i = decimallocation-1; i >=0; i--) {
				if (keypadarr[i] >= '0' && keypadarr[i] <= '9') {
					num += ( (int)keypadarr[i] - (int)'0' ) * (count);
					count = count * 10;
				}
			}
			//Next do numbers to the right of the decimal
			count = 10;
			for (int i = decimallocation+1; i < keypadlength; i++) {
				if (keypadarr[i] >= '0' && keypadarr[i] <= '9') {
					num += ( (float)(int)keypadarr[i] - (int)'0' ) / (float)(count);
					count = count * 10;
				}
			}
		}
	}
	return num;
}

uint8_t check_keypad(uint8_t which)
{
	//which=0 for voltage which=1 for amperage
	float temp = translate_keypad();
	return (which) ? (temp >= 0 && temp <= 0.5) : (temp >= 0 && temp <= 12.00);
}

void keypad_sm(char num)
{
	//A=V1;B=A1;C=V2;D=A2;
	switch (kpenum) {
	case WAIT:
		//While in wait we only listen to letters for channel number and type
		switch(num) {
		case 'A':
			kpenum = V1;
			break;
		case 'B':
			kpenum = A1;
			break;
		case 'C':
			kpenum = V2;
			break;
		case 'D':
			kpenum = A2;
			break;
		default:
			// Error
			break;
		}
		clear_keypad();
		break;

	case V1:
		//While in channel listen to numbers/decimal and letters for ENTER
		if (num == 'A') {
			if (check_keypad(0)) {
				//Only update the value if valid
				voltnum1 = translate_keypad();
			}
			kpenum = WAIT;
			clear_keypad();
		} else if (num == 'B') {
			kpenum = WAIT;
			clear_keypad();
		} else if (num == 'C') {
			kpenum = WAIT;
			clear_keypad();
		} else if (num == 'D') {
			kpenum = WAIT;
			clear_keypad();
		} else if (num == '.') {
			//It "should" be impossible to pass in a bad decimal and we can ignore bad calls
			update_keypad(num);
		} else if (num >= '0' && num <= '9') {
			//It "should" be impossible to overfill the array but we could do additional checks here for valid numbers
			update_keypad(num);
			// TODO: Display keypad array on LCD
		} else if (num == '#') {
			update_keypad('z');
		} else if (num == '+') {
			if (check_keypad(0)) {
				//Only update the value if valid
				voltnum1 = translate_keypad();
			}
			kpenum = WAIT;
			clear_keypad();
			first_shot = 1;
		} else if(num == '-') {
			kpenum = WAIT;
			clear_keypad();
		}
		break;

	case A1:
		//While in channel listen to numbers/decimal and letters for ENTER
		if (num == 'A') {
			kpenum = WAIT;
			clear_keypad();
		} else if (num == 'B') {
			if (check_keypad(1)) {
				//Only update the value if valid
				ampnum1 = translate_keypad();
			}
			// TODO: Display set number on LCD
			kpenum = WAIT;
			clear_keypad();
		} else if (num == 'C') {
			kpenum = WAIT;
			clear_keypad();
		} else if (num == 'D') {
			kpenum = WAIT;
			clear_keypad();
		} else if (num == '.') {
			//It "should" be impossible to pass in a bad decimal and we can ignore bad calls
			update_keypad(num);
		} else if (num >= '0' && num <= '9') {
			//It "should" be impossible to overfill the array but we could do additional checks here for valid numbers
			update_keypad(num);
		} else if (num == '#') {
			update_keypad('z');
		} else if (num == '+') {
			if (check_keypad(1)) {
				//Only update the value if valid
				ampnum1 = translate_keypad();
			}
			// TODO: Display set number on LCD
			kpenum = WAIT;
			clear_keypad();
		} else if(num == '-') {
			kpenum = WAIT;
			clear_keypad();
		}
		break;

	case V2:
		//While in channel listen to numbers/decimal and letters for ENTER
		if (num == 'A') {
			kpenum = WAIT;
			clear_keypad();
		} else if (num == 'B') {
			kpenum = WAIT;
			clear_keypad();
		} else if (num == 'C') {
			if (check_keypad(0)) {
				//Only update the value if valid
				voltnum2 = translate_keypad();
			}
			kpenum = WAIT;
			clear_keypad();
		} else if (num == 'D') {
			kpenum = WAIT;
			clear_keypad();
		} else if (num == '.') {
			//It "should" be impossible to pass in a bad decimal and we can ignore bad calls
			update_keypad(num);
		} else if (num >= '0' && num <= '9') {
			//It "should" be impossible to overfill the array but we could do additional checks here for valid numbers
			update_keypad(num);
			// TODO: Display keypad array on LCD
		} else if (num == '#') {
			update_keypad('z');
		} else if (num == '+') {
			if(check_keypad(0)){
				//Only update the value if valid
				voltnum2 = translate_keypad();
			}
			kpenum = WAIT;
			clear_keypad();
		} else if (num == '-') {
			kpenum = WAIT;
			clear_keypad();
		}
		break;

	case A2:
		//While in channel listen to numbers/decimal and letters for ENTER
		if (num == 'A') {
			kpenum = WAIT;
			clear_keypad();
		} else if (num == 'B') {
			kpenum = WAIT;
			clear_keypad();
		} else if (num == 'C') {
			kpenum = WAIT;
			clear_keypad();
		} else if (num == 'D') {
			if (check_keypad(1)) {
				//Only update the value if valid
				ampnum2 = translate_keypad();
			}
			kpenum = WAIT;
			clear_keypad();
		} else if (num == '.') {
			//It "should" be impossible to pass in a bad decimal and we can ignore bad calls
			update_keypad(num);
		} else if (num >= '0' && num <= '9') {
			//It "should" be impossible to overfill the array but we could do additional checks here for valid numbers
			update_keypad(num);
			// TODO: Display keypad array on LCD
		} else if (num == '#') {
			update_keypad('z');
		} else if (num == '+') {
			if (check_keypad(1)) {
				//Only update the value if valid
				ampnum2 = translate_keypad();
			}
			kpenum = WAIT;
			clear_keypad();
		} else if (num == '-') {
			kpenum = WAIT;
			clear_keypad();
		}
		break;

	default:
		// Error
		break;
	}
}

void row_input(void) {
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	//Deinit
	HAL_GPIO_DeInit(Row_1_GPIO_Port, Row_1_Pin);
	HAL_GPIO_DeInit(Row_2_GPIO_Port, Row_2_Pin);
	HAL_GPIO_DeInit(Row_3_GPIO_Port, Row_3_Pin);
	HAL_GPIO_DeInit(Row_4_GPIO_Port, Row_4_Pin);
	HAL_GPIO_DeInit(Row_5_GPIO_Port, Row_5_Pin);

	HAL_GPIO_DeInit(Col_1_GPIO_Port, Col_1_Pin);
	HAL_GPIO_DeInit(Col_2_GPIO_Port, Col_2_Pin);
	HAL_GPIO_DeInit(Col_3_GPIO_Port, Col_3_Pin);
	HAL_GPIO_DeInit(Col_4_GPIO_Port, Col_4_Pin);

	//Write zeros to outputs, should be by default but just in case
	HAL_GPIO_WritePin(Col_1_GPIO_Port, Col_1_Pin|Col_2_Pin|Col_3_Pin|Col_4_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : Col1_Pin Col2_Pin Col3_Pin Col4_Pin */
	GPIO_InitStruct.Pin = Col_1_Pin|Col_2_Pin|Col_3_Pin|Col_4_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(Col_1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : Row1_Pin Row2_Pin Row3_Pin Row4_Pin */
	GPIO_InitStruct.Pin = Row_1_Pin|Row_2_Pin|Row_3_Pin|Row_4_Pin|Row_5_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(Row_1_GPIO_Port, &GPIO_InitStruct);

	//Write zeros to outputs, should be by default but just in case
	HAL_GPIO_WritePin(Col_1_GPIO_Port, Col_1_Pin|Col_2_Pin|Col_3_Pin|Col_4_Pin, GPIO_PIN_RESET);

	//Reenable interrupts
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

void column_input(void) {
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	//Disable interrupts
	//HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
	HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);

	//Deinit
	HAL_GPIO_DeInit(Row_1_GPIO_Port, Row_1_Pin);
	HAL_GPIO_DeInit(Row_2_GPIO_Port, Row_2_Pin);
	HAL_GPIO_DeInit(Row_3_GPIO_Port, Row_3_Pin);
	HAL_GPIO_DeInit(Row_4_GPIO_Port, Row_4_Pin);
	HAL_GPIO_DeInit(Row_5_GPIO_Port, Row_5_Pin);

	HAL_GPIO_DeInit(Col_1_GPIO_Port, Col_1_Pin);
	HAL_GPIO_DeInit(Col_2_GPIO_Port, Col_2_Pin);
	HAL_GPIO_DeInit(Col_3_GPIO_Port, Col_3_Pin);
	HAL_GPIO_DeInit(Col_4_GPIO_Port, Col_4_Pin);

	//Write zeros to outputs, should be by default but just in case
	HAL_GPIO_WritePin(Row_1_GPIO_Port, Row_1_Pin|Row_2_Pin|Row_3_Pin|Row_4_Pin|Row_5_Pin, GPIO_PIN_RESET);

	/*Switch rows to outputs */
	GPIO_InitStruct.Pin = Row_1_Pin|Row_2_Pin|Row_3_Pin|Row_4_Pin|Row_5_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(Row_1_GPIO_Port, &GPIO_InitStruct);

	/*Switch columns to inputs */
	GPIO_InitStruct.Pin = Col_1_Pin|Col_2_Pin|Col_3_Pin|Col_4_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(Col_1_GPIO_Port, &GPIO_InitStruct);

	//Write zeros to outputs, should be by default but just in case
	HAL_GPIO_WritePin(Col_1_GPIO_Port, Row_1_Pin|Row_2_Pin|Row_3_Pin|Row_4_Pin|Row_5_Pin, GPIO_PIN_RESET);
}

void keypad_decode(uint8_t row_pin) {
	static GPIO_TypeDef *col_ports[4] = { Col_1_GPIO_Port, Col_2_GPIO_Port, Col_3_GPIO_Port, Col_4_GPIO_Port };
	static uint16_t col_pins[4] = { Col_1_Pin, Col_2_Pin, Col_3_Pin, Col_4_Pin };
	static char keypad_labels[4][4] = {{ '1', '4', '7', '.' },
									   { '2', '5', '8', '0' },
									   { '3', '6', '9', '#' },
									   { 'C', 'D', 'A', 'B' }};
	if (row_pin) {
		for (uint8_t i = 0; i < 4; i++) {
			if (HAL_GPIO_ReadPin(col_ports[i], col_pins[i]) == PRESSED) {
				keypad_sm(keypad_labels[row_pin - 1][i]);
				break;
			}
		}
	} else {
		if (HAL_GPIO_ReadPin(Col_1_GPIO_Port, Col_1_Pin) == PRESSED) {
			HAL_GPIO_TogglePin(GPIOB, Status_LED_1_Pin);
		} else if (HAL_GPIO_ReadPin(Col_2_GPIO_Port, Col_2_Pin) == PRESSED) {
			keypad_sm('+'); // Confirm
		} else if (HAL_GPIO_ReadPin(Col_3_GPIO_Port, Col_3_Pin) == PRESSED) {
			keypad_sm('-'); // Cancel
		} else if (HAL_GPIO_ReadPin(Col_4_GPIO_Port, Col_4_Pin) == PRESSED) {
			HAL_GPIO_TogglePin(Status_LED_2_GPIO_Port, Status_LED_2_Pin);
		}
	}
}

void set_rowpin(uint8_t row_num) {
	static GPIO_TypeDef *row_ports[5] = { Row_1_GPIO_Port, Row_2_GPIO_Port, Row_3_GPIO_Port, Row_4_GPIO_Port, Row_5_GPIO_Port };
	static uint16_t row_pins[5] = {Row_1_Pin, Row_2_Pin, Row_3_Pin, Row_4_Pin, Row_5_Pin};
	if (HAL_GPIO_ReadPin(row_ports[row_num], row_pins[row_num]) == PRESSED) {
		//Make sure we didn't interrupt on falling edge already
		if (kpedge != 0) {
			kpedge = 0;
			//Save rowpin
			rowpin = row_num;
			//start debounce
			HAL_TIM_Base_Start_IT(&htim2);
		}
	}
	//Rising edge
	else {
		//Make sure we didn't interrupt on rising edge already
		if (kpedge != 1) {
			kpedge = 1;
			//Reset keypad detection vars
			rowpin = -1;
		}
	}
}

/* Keypad Section End --------------------------------------------------------*/

/* Interrupt Callback Section Begin ------------------------------------------*/
void USB_Interrupt_Callback(void)
{
	memset (usbbuffer, '\0', 128);  // clear the buffer
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	switch(GPIO_Pin) {
	case Row_1_Pin:
		set_rowpin(0);
		break;

	case Row_2_Pin:
		set_rowpin(1);
		break;

	case Row_3_Pin:
		set_rowpin(2);
		break;

	case Row_4_Pin:
		set_rowpin(3);
		break;

	case Row_5_Pin:
		set_rowpin(4);
		break;

	case Rot_CLK_Pin:
		if ((kpenum != WAIT) && (Rot_State != NOTURN)) {
			HAL_NVIC_DisableIRQ(Rot_CLK_EXTI_IRQn);
			Rot_State = (HAL_GPIO_ReadPin(Rot_CLK_GPIO_Port, Rot_CLK_Pin) == HAL_GPIO_ReadPin(Rot_DT_GPIO_Port, Rot_DT_Pin)) ? CWTURN : CCWTURN;
			HAL_TIM_Base_Start_IT(&htim4);
		}
		break;

	case Rot_SW_Pin:
		if (kpenum != WAIT) {
			HAL_NVIC_DisableIRQ(Rot_SW_EXTI_IRQn);
			Rot_SW_State = HAL_GPIO_ReadPin(Rot_SW_GPIO_Port, Rot_SW_Pin);
			HAL_TIM_Base_Start_IT(&htim9);
		}
		break;

	default:
		// Error
		break;
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim2) {
		//Disable timer now that we're in its interrupt
		HAL_TIM_Base_Stop_IT(&htim2);
		column_input();
		keypad_decode(rowpin);
		row_input();

	} else if (htim == &htim3) {
		//Disable timer now that we're in its interrupt
		HAL_TIM_Base_Stop_IT(&htim3);
		//Update Display
		lcd_psu_update();
		//Start timer again
		HAL_TIM_Base_Start_IT(&htim3);

	} else if (htim == &htim4) {
		HAL_TIM_Base_Stop_IT(&htim4);

		if (Rot_State != NOTURN) {
			switch (Rot_Mode) {
			case VALUE:
				switch(Rot_State) {
				case NOTURN:
					// Error
					break;

				case CWTURN:
					if ((uint8_t)keypadarr[keypaditerator] < (uint8_t)('9')) {
						(uint8_t)keypadarr[keypaditerator]++;
					} else if (keypaditerator > 0) {
						keypadarr[keypaditerator] = '0';
						(uint8_t)keypadarr[keypaditerator-1]++;
					}
					break;

				case CCWTURN:
					if ((uint8_t)keypadarr[keypaditerator] > (uint8_t)('0')) {
						(uint8_t)keypadarr[keypaditerator]--;
					} else if (keypaditerator < (keypadlength - 1)) {
						(uint8_t)keypadarr[keypaditerator + 1]--;
						keypadarr[keypaditerator] = '9';
					}
					break;

				default:
					//Error
					break;
				}

				volatile float *modifying_output = NULL;
				switch (kpenum) {
				case WAIT:
					// Error
					break;
				case V1:
					modifying_output = &voltnum1;
					break;
				case A1:
					modifying_output = &ampnum1;
					break;
				case V2:
					modifying_output = &voltnum2;
					break;
				case A2:
					modifying_output = &ampnum2;
					break;
				default:
					// Error
					break;
				}
				if (modifying_output != NULL) {
					*modifying_output = translate_keypad();
				}
				break;

			case INDEX:
				switch (Rot_State) {
				case NOTURN:
					// Error
					break;

				case CWTURN:
					if (keypaditerator < (keypadlength - 1)) {
						keypaditerator++;
						if (keypadarr[keypaditerator] == '.') {
							keypaditerator++;
						}
					}
					break;

				case CCWTURN:
					if (keypaditerator > 0) {
						keypaditerator--;
						if (keypadarr[keypaditerator] == '.') {
							keypaditerator--;
						}
					}
					break;

				default:
					//Error
					break;
				}
				break;

			default:
				// Error
				break;
			}
			Rot_State = NOTURN;
			HAL_NVIC_EnableIRQ(Rot_CLK_EXTI_IRQn);
		}

	} else if (htim == &htim9) {
		HAL_TIM_Base_Stop_IT(&htim9);
		if (HAL_GPIO_ReadPin(Rot_SW_GPIO_Port, Rot_SW_Pin) == Rot_SW_State) {
			switch (Rot_SW_State) {
			case PRESSED:
				Rot_Mode = !Rot_Mode;
				break;
			case RELEASED:
				break;
			default:
				// Error
				break;
			}
		} else {
			HAL_GPIO_EXTI_Callback(Rot_SW_Pin);
		}

		switch (Rot_Mode) {
		case VALUE:
			HAL_TIM_Base_Stop_IT(&htim10);
			break;
		case INDEX:
			HAL_TIM_Base_Start_IT(&htim10);
			break;
		default:
			// Error
			break;
		}
		HAL_NVIC_EnableIRQ(Rot_SW_EXTI_IRQn);

	} else if (htim == &htim10) {
		static uint8_t cursor_blink = 0;
		lcd_cursorblink_onoff(cursor_blink, cursor_blink);
		cursor_blink = !cursor_blink;

	} else if (htim == &htim11) {
		// TODO: Use cursor to indicate what digit is currently selected (Keypad and Rotary Encoder)
		static uint8_t cursor = 0;
		lcd_cursorblink_onoff(cursor, 0);
		cursor = !cursor;
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	adc_current = adcvalues[2];
	adc_linear = adcvalues[1];
	adc_opamp = adcvalues[0];
	adc_switching = adcvalues[3];
	adc_vref = adcvalues[4];
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

	//CDC_Transmit_FS(rxbuffer,64);
	uint8_t rxiter = 0;
	for(int i = 0; i < 64; i++){
		if (rxbuffer[i] == '*') {
			rxiter = i;//Found start condition
			break;
		}
	}
	uint8_t rxbuffercpy[32];
	memset (rxbuffercpy, '\0', 32);  // clear the buffer
	//Copy our message into a buffer that isn't offset
	for (int i = 0; i < 25; i++) {
		rxbuffercpy[i] = rxbuffer[rxiter];
		rxiter++;
		if (rxiter > 64) {
			rxiter = 0;
		}
	}

	//CDC_Transmit_FS(rxbuffercpy,32);

	if (//Check start condition
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
			) {
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

		slin_num = tempv2;
		scur_num = tempa2;
		chstat2 = rxbuffercpy[18]-48;

		//strcat((char*)rxbuffercpy, "\n");
		//CDC_Transmit_FS(rxbuffercpy,32);

		//uint8_t tstbuf[64];
		//snprintf((char*)tstbuf, 64, "%f, %f", tempv2, tempa2);
		//CDC_Transmit_FS(tstbuf,64);
	}

	memset (rxbuffer, '\0', 64);  // clear the buffer
	HAL_UART_Receive_DMA (&huart1, rxbuffer, 64);  // Receive 64 Bytes of data
}


void HAL_UART_TxHalfCpltCallback(UART_HandleTypeDef *huart)
{

}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	memset (txbuffer, '\0', 64);  // clear the buffer
	snprintf((char*)txbuffer, 32, "*STRT,%05.2f,%5.3f,%d,FNSH!", voltnum2, ampnum2, chstat2);
	HAL_UART_Transmit_DMA(&huart1, txbuffer, 64);
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
