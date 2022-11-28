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
#include "stdlib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* LCD Section Begin ---------------------------------------------------------*/
#define SLAVE_ADDRESS_LCD 0x4E // change this according to your setup

//Char 0 = Right Arrow
//Char 1 = Vertical Line
const uint8_t customChar[64] = {
	0x08,0x04,0x02,0x1F,0x02,0x04,0x08,0x00, //Custom Char 0
	0x04,0x04,0x04,0x4,0x04,0x04,0x04,0x04,  //Custom Char 1
	0x00,0x00,0x00,0x0,0x00,0x00,0x00,0x00,  //Custom Char 2
	0x00,0x00,0x00,0x0,0x00,0x00,0x00,0x00,  //Custom Char 3
	0x00,0x00,0x00,0x0,0x00,0x00,0x00,0x00,  //Custom Char 4
	0x00,0x00,0x00,0x0,0x00,0x00,0x00,0x00,  //Custom Char 5
	0x00,0x00,0x00,0x0,0x00,0x00,0x00,0x00,  //Custom Char 6
	0x00,0x00,0x00,0x0,0x00,0x00,0x00,0x00   //Custom Char 7
};
/* LCD Section End -----------------------------------------------------------*/

/* ADC Section Begin ---------------------------------------------------------*/
#define ADC_OPAMP adc_values_cpy[0]
#define ADC_LINEAR adc_values_cpy[1]
#define ADC_CURRENT adc_values_cpy[2]
#define ADC_SWITCHING adc_values_cpy[3]
#define ADC_VREF adc_values_cpy[4]
/* ADC Section End -----------------------------------------------------------*/

/* PID Control for the Linear Voltage Regulator ------------------------------*/
#define P 0.1
#define I 0.1
#define D 0.1
/* PID Linear Section End ----------------------------------------------------*/

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
uint8_t usbbuffer[64];//Usb buffer
/*
 * We need to be able to store USB commands and process them later because we may receive them too fast
 * We really only need to be able to store up to two commands, one for the write, and one for the read
 * We have enough ram left for a slightly bigger buffer though
 * A circular buffer would make sense for this but I'm lazy so here's your circular buffer
 */
#define CIRCSIZE 8
uint8_t notacircbuff[CIRCSIZE][64];
int8_t headiter = 1;//Start at element 1, we're not going to let the tailiter pass the headiter
int8_t tailiter = 0;
uint8_t MSG[64];
/* USB Section End -----------------------------------------------------------*/

/* UART Section Begin --------------------------------------------------------*/
uint8_t txbuffer[64];//Uart TX Buffer
uint8_t txbuffer_cpy[64];//Uart TX Buffer copy because transfer is circular
uint8_t rxbuffer[64];//Uart RX Buffer
/* UART Section End ----------------------------------------------------------*/

/* Keypad Section Begin ------------------------------------------------------*/
int rowpin = -1;
uint8_t kpedge = 1;	//0 == falling edge 1 == rising edge
uint8_t swedge = 1;	//0 == falling edge 1 == rising edge

const uint8_t keypadlength = 5;
char keypadarr[5] = {'z','z','z','z','z'};//z is null
int8_t keypaditerator = 4;
uint8_t keypaddecimal = 0;
enum KEYPAD_ENUM {WAIT, V1, A1, V2, A2};
enum KEYPAD_ENUM kpenum = WAIT;
enum ROTARY_STATE {NOTURN, CWTURN, CCWTURN};
enum ROTARY_STATE rotenum = NOTURN;
uint8_t encmode = 0;
int8_t encpos = 0;

//static GPIO_TypeDef* row_ports[5] = { Row_1_GPIO_Port, Row_2_GPIO_Port, Row_3_GPIO_Port, Row_4_GPIO_Port, Row_5_GPIO_Port };
static uint16_t row_pins[5] = {Row_1_Pin, Row_2_Pin, Row_3_Pin, Row_4_Pin, Row_5_Pin};
static GPIO_TypeDef *col_ports[4] = { Col_1_GPIO_Port, Col_2_GPIO_Port, Col_3_GPIO_Port, Col_4_GPIO_Port };
static uint16_t col_pins[4] = { Col_1_Pin, Col_2_Pin, Col_3_Pin, Col_4_Pin };
static const char keypad_labels[5][4] = {
	{ '*', '+', '-', '/' },
	{ '1', '4', '7', '.' },
	{ '2', '5', '8', '0' },
	{ '3', '6', '9', '#' },
	{ 'A', 'B', 'C', 'D' }
};
/* Keypad Section End --------------------------------------------------------*/

//Channel numbers
volatile float volt_set_main = 0.0;
volatile float amp_set_main = 0.0;
volatile float volt_set_aux = 0.0;
volatile float amp_set_aux = 0.0;

volatile float volt_set_main_old = 0.0;
volatile float amp_set_main_old = 0.0;
//volatile float volt_set_aux_old = 0.0;
//volatile float amp_set_aux_old = 0.0;

//Array for the adc values and vars to hold them
volatile uint16_t adc_values[6];
volatile uint16_t adc_values_cpy[6];
uint16_t* vrefptr = ((uint16_t*)VREFINT_CAL_ADDR_CMSIS);
int8_t chstat_main = 0;
int8_t chstat_aux_tx = 0;
int8_t chstat_aux_rx = 0;

//Globals for adc values
volatile float lin_num = 0;
volatile float cur_num = 0;
volatile float op_num = 0;
volatile float swi_num = 0;

volatile float lin_num_aux = 0;
volatile float cur_num_aux = 0;

//Global for dac value
uint16_t v1;//dac channel 1 is linear
uint16_t v2;//dac channel 2 is switching

uint8_t timercounter = 0;
uint8_t blink = 0;
uint8_t startmessage = 0;

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
static void MX_TIM11_Init(void);
static void MX_TIM9_Init(void);
static void MX_TIM10_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

void ourInit(void);//Runs several Inits

/* USB Section Begin ---------------------------------------------------------*/
void USB_EXTIinit(void);
void USB_Interrupt_Callback(void);
void EXTI1_IRQHandler(void);
/* USB Section End -----------------------------------------------------------*/

/* ADC Section Begin ---------------------------------------------------------*/
void update_ADC_watchdog(float val);
/* ADC Section End -----------------------------------------------------------*/

/* LCD Section Begin ---------------------------------------------------------*/
void lcd_init (void);   // initialize lcd
void lcd_send_cmd (char cmd);  // send command to the lcd
void lcd_send_data (char data);  // send data to the lcd
void lcd_send_string (char *str);  // send string to the lcd
void lcd_put_cur(int row, int col);  // put cursor at the entered position row (0-3), col (0-19);
void lcd_clear (void);
void LCD_CursorBlinkOnOff( uint8_t cursor_status, uint8_t blink_status );
void lcd_createChar(void);
void lcd_psu_init(void);
void lcd_update_voltage(uint8_t channel, float num);
void lcd_update_amperage(uint8_t channel, float num);
void lcd_psu_update(void);
void lcd_psu_welcome(void);
/* LCD Section End -----------------------------------------------------------*/

/* Keypad Section Begin ------------------------------------------------------*/
void update_keypad(char num);
void clear_keypad(void);
float translate_keypad(void);
uint8_t check_keypad(uint8_t which);
void fill_keypad(uint8_t va, float num);
void keypad_sm(char num);
void row_input(void);
void column_input(void);
void inc_arr_v(int8_t pos);
void inc_arr_a(int8_t pos);
void dec_arr_v(int8_t pos);
void dec_arr_a(int8_t pos);
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
  MX_TIM11_Init();
  MX_TIM9_Init();
  MX_TIM10_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  ourInit();
  float error = 0;
  float derivative = 0;
  float integral = 0;
  float error_previous = 0;
  float correction = 0;
  float corrected_volt_set_main;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  //Process USB Commands here
	  int8_t tempiter = tailiter;
	  tempiter++;
	  if(tempiter >= CIRCSIZE){
		  tempiter = 0;
	  }
	  //Buffer is not empty
	  if(tempiter != headiter){
			// Measure Voltage
			if ((strncmp("MEAS:VOLT?", (char*)notacircbuff[tempiter], strlen("MEAS:VOLT?")) == 0) ||
					(strncmp("MEASure:VOLTage:DC?", (char*)notacircbuff[tempiter], strlen("MEASure:VOLTage:DC?")) == 0) ||
					(strncmp("MEASure:VOLTage?", (char*)notacircbuff[tempiter], strlen("MEASure:VOLTage?")) == 0)){
				snprintf((char*)MSG, 64, "%.2f, %.2f\n", lin_num_aux, lin_num);
				CDC_Transmit_FS(MSG, 64);
				memset(MSG,'\0', 64);
			}
			// Measure Current
			else if ((strncmp("MEAS:CURR?", (char*)notacircbuff[tempiter], strlen("MEAS:CURR?")) == 0) ||
					(strncmp("MEASure:CURRent:DC?", (char*)notacircbuff[tempiter], strlen("MEASure:CURRent:DC?")) == 0) ||
					(strncmp("MEASure:CURRent?", (char*)notacircbuff[tempiter], strlen("MEASure:CURRent?")) == 0)){
				snprintf((char*)MSG, 64, "%.3f, %.3f\n", cur_num_aux, cur_num);
				CDC_Transmit_FS(MSG, 64);
				memset(MSG,'\0', 64);
			}
			// CH1 Output Status
			else if ((strncmp("OUTPut:ONE?", (char*)notacircbuff[tempiter], strlen("OUTPut:ONE?")) == 0) ||
					(strncmp("OUTP:ONE?", (char*)notacircbuff[tempiter], strlen("OUTP:ONE?")) == 0)){
				snprintf((char*)MSG, 64, "%d\n", chstat_aux_rx);
				CDC_Transmit_FS(MSG, 64);
				memset(MSG,'\0', 64);
			}
			// CH2 Output Status
			else if ((strncmp("OUTPut:TWO?", (char*)notacircbuff[tempiter], strlen("OUTPut:TWO?")) == 0) ||
					(strncmp("OUTP:TWO?", (char*)notacircbuff[tempiter], strlen("OUTP:TWO?")) == 0)){
				snprintf((char*)MSG, 64, "%d\n", chstat_main);
				CDC_Transmit_FS(MSG, 64);
				memset(MSG,'\0', 64);
			}
			// CH1 Output ON
			else if ((strncmp("OUTPUT:ONE:START", (char*)notacircbuff[tempiter], strlen("OUTPUT:ONE:START")) == 0) ||
					(strncmp("OUTP:ONE:START", (char*)notacircbuff[tempiter], strlen("OUTP:ONE:START")) == 0)){
				chstat_aux_tx = 1;
				snprintf((char*)MSG, 64, "\n");
				CDC_Transmit_FS(MSG, 64);
				memset(MSG,'\0', 64);
			}
			// CH2 Output ON
			else if ((strncmp("OUTPut:TWO:START", (char*)notacircbuff[tempiter], strlen("OUTPut:TWO:START")) == 0) ||
					(strncmp("OUTP:TWO:START", (char*)notacircbuff[tempiter], strlen("OUTP:TWO:START")) == 0)){
				chstat_main = 1;
				snprintf((char*)MSG, 64, "\n");
				CDC_Transmit_FS(MSG, 64);
				memset(MSG,'\0', 64);
			}
			// CH1 Output OFF
			else if ((strncmp("OUTPut:ONE:STOP", (char*)notacircbuff[tempiter], strlen("OUTPut:ONE:STOP")) == 0) ||
					(strncmp("OUTP:ONE:STOP", (char*)notacircbuff[tempiter], strlen("OUTP:ONE:STOP")) == 0)){
				chstat_aux_tx = 0;
				snprintf((char*)MSG, 64, "\n");
				CDC_Transmit_FS(MSG, 64);
				memset(MSG,'\0', 64);
			}
			// CH2 Output OFF
			else if ((strncmp("OUTPut:TWO:STOP", (char*)notacircbuff[tempiter], strlen("OUTPut:TWO:STOP")) == 0) ||
					(strncmp("OUTP:TWO:STOP", (char*)notacircbuff[tempiter], strlen("OUTP:TWO:STOP")) == 0)){
				chstat_main = 0;
				snprintf((char*)MSG, 64, "\n");
				CDC_Transmit_FS(MSG, 64);
				memset(MSG,'\0', 64);
			}
			// CH1 VOLT 1
			else if ((strncmp("VOLTage:ONE:", (char*)notacircbuff[tempiter], strlen("VOLTage:ONE:")) == 0)){
				if ((atof((char*)notacircbuff[tempiter] + strlen("VOLTage:ONE:")) >= 0.00) && (atof((char*)notacircbuff[tempiter] + strlen("VOLTage:ONE:")) <= 12.00)){
					//Truncate voltage inputs to 2 decimal places
					float temp = (float)atof((char*)notacircbuff[tempiter] + 12);
					uint8_t tempbuff[8] = {0};
					snprintf((char*)tempbuff, 8, "%.2f", temp);
					volt_set_aux = (float)atof((char*)tempbuff);
					snprintf((char*)MSG, 64, "\n");
				}
				else{
					snprintf((char*)MSG, 64, "ERROR: INVALID NUMBER\n");
				}
				CDC_Transmit_FS(MSG, 64);
				memset(MSG,'\0', 64);
			}
			// CH1 VOLT 2
			else if ((strncmp("VOLT:ONE:", (char*)notacircbuff[tempiter], strlen("VOLT:ONE:")) == 0)){
				if ((atof((char*)notacircbuff[tempiter] + strlen("VOLT:ONE:")) >= 0.00) && (atof((char*)notacircbuff[tempiter] + strlen("VOLT:ONE:")) <= 12.00)){
					//Truncate voltage inputs to 2 decimal places
					float temp = (float)atof((char*)notacircbuff[tempiter] + 9);
					uint8_t tempbuff[8] = {0};
					snprintf((char*)tempbuff, 8, "%.2f", temp);
					volt_set_aux = (float)atof((char*)tempbuff);
					snprintf((char*)MSG, 64, "\n");
				}
				else{
					snprintf((char*)MSG, 64, "ERROR: INVALID NUMBER\n");
				}
				CDC_Transmit_FS(MSG, 64);
				memset(MSG,'\0', 64);
			}
			// CH2 VOLT 1
			else if ((strncmp("VOLTage:TWO:", (char*)notacircbuff[tempiter], strlen("VOLTage:TWO:")) == 0)){
				if ((atof((char*)notacircbuff[tempiter] + strlen("VOLTage:TWO:")) >= 0.00) && (atof((char*)notacircbuff[tempiter] + strlen("VOLTage:TWO:")) <= 12.00)){
					//Truncate voltage inputs to 2 decimal places
					float temp = (float)atof((char*)notacircbuff[tempiter] + 12);
					uint8_t tempbuff[8] = {0};
					snprintf((char*)tempbuff, 8, "%.2f", temp);
					volt_set_main_old = volt_set_main;
					volt_set_main = (float)atof((char*)tempbuff);
					snprintf((char*)MSG, 64, "\n");
				}
				else{
					snprintf((char*)MSG, 64, "ERROR: INVALID NUMBER\n");
				}
				CDC_Transmit_FS(MSG, 64);
				memset(MSG,'\0', 64);
			}

			// CH2 VOLT 2
			else if ((strncmp("VOLT:TWO:", (char*)notacircbuff[tempiter], strlen("VOLT:TWO:")) == 0)){
				if ((atof((char*)notacircbuff[tempiter] + strlen("VOLT:TWO:")) >= 0.00) && (atof((char*)notacircbuff[tempiter] + strlen("VOLT:TWO:")) <= 12.00)){
					//Truncate voltage inputs to 2 decimal places
					float temp = (float)atof((char*)notacircbuff[tempiter] + 9);
					uint8_t tempbuff[8] = {0};
					snprintf((char*)tempbuff, 8, "%.2f", temp);
					volt_set_main_old = volt_set_main;
					volt_set_main = (float)atof((char*)tempbuff);
					snprintf((char*)MSG, 64, "\n");
				}
				else{
					snprintf((char*)MSG, 64, "ERROR: INVALID NUMBER\n");
				}
				CDC_Transmit_FS(MSG, 64);
				memset(MSG,'\0', 64);
			}
			// CH1 AMP 1
			else if ((strncmp("CURRent:ONE:", (char*)notacircbuff[tempiter], strlen("CURRent:ONE:")) == 0)){
				if ((atof((char*)notacircbuff[tempiter] + strlen("CURRent:ONE:")) >= 0.00) && (atof((char*)notacircbuff[tempiter] + strlen("CURRent:ONE:")) <= 0.80)){
					//Truncate amperage inputs to 3 decimal places
					float temp = (float)atof((char*)notacircbuff[tempiter] + 12);
					uint8_t tempbuff[8] = {0};
					snprintf((char*)tempbuff, 8, "%.3f", temp);
					amp_set_aux = (float)atof((char*)tempbuff);
					snprintf((char*)MSG, 64, "\n");
				}
				else{
					snprintf((char*)MSG, 64, "ERROR: INVALID NUMBER\n");
				}
				CDC_Transmit_FS(MSG, 64);
				memset(MSG,'\0', 64);
			}
			// CH1 AMP 2
			else if ((strncmp("CURR:ONE:", (char*)notacircbuff[tempiter], strlen("CURR:ONE:")) == 0)){
				if ((atof((char*)notacircbuff[tempiter] + strlen("CURR:ONE:")) >= 0.00) && (atof((char*)notacircbuff[tempiter] + strlen("CURR:ONE:")) <= 0.80)){
					//Truncate amperage inputs to 3 decimal places
					float temp = (float)atof((char*)notacircbuff[tempiter] + 9);
					uint8_t tempbuff[8] = {0};
					snprintf((char*)tempbuff, 8, "%.3f", temp);
					amp_set_aux = (float)atof((char*)tempbuff);
					snprintf((char*)MSG, 64, "\n");
				}
				else{
					snprintf((char*)MSG, 64, "ERROR: INVALID NUMBER\n");
				}
				CDC_Transmit_FS(MSG, 64);
				memset(MSG,'\0', 64);
			}

			// CH2 AMP 1
			else if ((strncmp("CURRent:TWO:", (char*)notacircbuff[tempiter], strlen("CURRent:TWO:")) == 0)){
				if ((atof((char*)notacircbuff[tempiter] + strlen("CURRent:TWO:")) >= 0.00) && (atof((char*)notacircbuff[tempiter] + strlen("CURRent:TWO:")) <= 0.80)){
					//Truncate amperage inputs to 3 decimal places
					float temp = (float)atof((char*)notacircbuff[tempiter] + 12);
					uint8_t tempbuff[8] = {0};
					snprintf((char*)tempbuff, 8, "%.3f", temp);
					amp_set_main_old = amp_set_main;
					amp_set_main = (float)atof((char*)tempbuff);
					snprintf((char*)MSG, 64, "\n");
				}
				else{
					snprintf((char*)MSG, 64, "ERROR: INVALID NUMBER\n");
				}
				CDC_Transmit_FS(MSG, 64);
				memset(MSG,'\0', 64);
			}

			// CH2 AMP 2
			else if ((strncmp("CURR:TWO:", (char*)notacircbuff[tempiter], strlen("CURR:TWO:")) == 0)){
				if ((atof((char*)notacircbuff[tempiter] + strlen("CURR:TWO:")) >= 0.00) && (atof((char*)notacircbuff[tempiter] + strlen("CURR:TWO:")) <= 0.80)){
					//Truncate amperage inputs to 3 decimal places
					float temp = (float)atof((char*)notacircbuff[tempiter] + 9);
					uint8_t tempbuff[8] = {0};
					snprintf((char*)tempbuff, 8, "%.3f", temp);
					amp_set_main_old = amp_set_main;
					amp_set_main = (float)atof((char*)tempbuff);
					snprintf((char*)MSG, 64, "\n");
				}
				else{
					snprintf((char*)MSG, 64, "ERROR: INVALID NUMBER\n");
				}
				CDC_Transmit_FS(MSG, 64);
				memset(MSG,'\0', 64);
			}
			// Identify
			else if ((strncmp("*IDN?", (char*)notacircbuff[tempiter], strlen("*IDN?")) == 0)){
				CDC_Transmit_FS((uint8_t*)"493 Lab Power Supply\n", strlen("493 Lab Power Supply\n"));
			}
			//Ignore read, we can add it back later if needed
			if ((strncmp("READ?", (char*)notacircbuff[tempiter], strlen("READ?")) == 0)){
				//CDC_Transmit_FS(MSG, strlen((char*)MSG));
				//memset(MSG,'\0', 64);
				CDC_Transmit_FS((uint8_t*)"\n", strlen("\n"));
			}
			// Invalid input/command not supported
			else{
				snprintf((char*)MSG, 64, "ERROR: INVALID COMMAND\n");
				CDC_Transmit_FS(MSG, 64);
				memset(MSG,'\0', 64);
			}
			memset (notacircbuff[tempiter], '\0', 64); // clear the buffer
			tailiter++;
			if(tailiter >= CIRCSIZE){
				tailiter = 0;
			}
	  }


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


/*
	  //PID
	  error = lin_num - volt_set_main;
	  integral += error;
	  if (integral > (float)4095.0) {
		  integral = 4095;
	  } else if (integral < (float)(-4095.0)) {
		  integral = (float)(-4095.0);
	  }
	  derivative = error - error_previous;
	  error_previous = error;
	  correction = P * error + I * integral + D * derivative;
	  corrected_volt_set_main = volt_set_main - correction;
	  if (corrected_volt_set_main > 12.0) {
		  corrected_volt_set_main = 12.0;
	  }
	  else if(corrected_volt_set_main < 0.0) {
		  corrected_volt_set_main = 0.0;
	  }
	  v1 = (uint16_t)(((((float)corrected_volt_set_main / (float)4.0) + ((float)0.446974063 / (float)4.0)) * (float)4095) / (float)vddcalc);
*/

	  /*
	   * The calculation we do to determine what we need to set the dac for the switching regulator to is determined by the following formula
	   * 1.235 = (R3*R2*Vout + R1*R2Vdac) / (R3*R2 + R1*R3 + R1*R2) where R1 = 10k, R2 = 1.2k, R3 = 2.4K
	   * This equation would be a little cumbersome to calculate every time we loop so I have simplified it on paper to the following formula
	   * Vdac = 4.001400 - 0.240000*Vout
	   */

	  float temp = ( ((float)4.001400 - ((float)0.240000*((float)volt_set_main + (float)0.5))) * (float)4095 / (float)vddcalc);
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
  AnalogWDGConfig.HighThreshold = 0;
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
  htim2.Init.Period = 20;
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
  htim4.Init.Period = 2500;
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
  htim9.Init.Period = 200;
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
  htim10.Init.Period = 10;
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
  htim11.Init.Period = 100;
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
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Channel_Shutdown_GPIO_Port, Channel_Shutdown_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, Status_LED_1_Pin|Status_LED_2_Pin|Col_1_Pin|Col_2_Pin
                          |Col_3_Pin|Col_4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Unused_Pin_1_Pin Unused_Pin_2_Pin Unused_Pin_3_Pin Unused_Pin_4_Pin
                           Unused_Pin_5_Pin Unused_Pin_6_Pin Unused_Pin_7_Pin Unused_Pin_12_Pin
                           Unused_Pin_13_Pin Unused_Pin_14_Pin */
  GPIO_InitStruct.Pin = Unused_Pin_1_Pin|Unused_Pin_2_Pin|Unused_Pin_3_Pin|Unused_Pin_4_Pin
                          |Unused_Pin_5_Pin|Unused_Pin_6_Pin|Unused_Pin_7_Pin|Unused_Pin_12_Pin
                          |Unused_Pin_13_Pin|Unused_Pin_14_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : Channel_Shutdown_Pin */
  GPIO_InitStruct.Pin = Channel_Shutdown_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Channel_Shutdown_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Unused_Pin_8_Pin Unused_Pin_10_Pin Unused_Pin_11_Pin */
  GPIO_InitStruct.Pin = Unused_Pin_8_Pin|Unused_Pin_10_Pin|Unused_Pin_11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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

  /*Configure GPIO pins : Unused_Pin_9_Pin Unused_Pin_16_Pin Unused_Pin_17_Pin Unused_Pin_18_Pin
                           Unused_Pin_19_Pin Unused_Pin_20_Pin */
  GPIO_InitStruct.Pin = Unused_Pin_9_Pin|Unused_Pin_16_Pin|Unused_Pin_17_Pin|Unused_Pin_18_Pin
                          |Unused_Pin_19_Pin|Unused_Pin_20_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : Unused_Pin_15_Pin */
  GPIO_InitStruct.Pin = Unused_Pin_15_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(Unused_Pin_15_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
	USB_EXTIinit();
	//Ensure keypad columns output 0 by default
	HAL_GPIO_WritePin(Col_1_GPIO_Port, Col_1_Pin|Col_2_Pin|Col_3_Pin|Col_4_Pin, GPIO_PIN_RESET);

	memset (usbbuffer, '\0', 64);  // clear the buffer
	memset (MSG, '\0', 64);  // clear the buffer
	memset (txbuffer, '\0', 64);  // clear the buffer
	memset (txbuffer_cpy, '\0', 64);  // clear the buffer
	memset (rxbuffer, '\0', 64);  // clear the buffer

	HAL_UART_Receive_DMA (&huart1, rxbuffer, 64);  // Receive 64 Bytes of data

	snprintf((char*)txbuffer, 32, "*STRT,%05.2f,%5.3f,%d,FNSH!", volt_set_aux, amp_set_aux, chstat_aux_tx);
	HAL_UART_Transmit_DMA(&huart1, txbuffer, 64);

	//LCD Init
	//lcd_psu_init();
	lcd_psu_welcome();

	//Start LED timer
	HAL_TIM_Base_Start_IT(&htim11);

	//Init the notacircbuff
	for(int i = 0; i < CIRCSIZE; i++){
		memset (notacircbuff[i], '\0', 64);  // clear the buffer
	}

	//Start display timer
	HAL_TIM_Base_Start_IT(&htim4);

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

/* ADC Section Begin ---------------------------------------------------------*/
void update_ADC_watchdog(float val){
	//Don't want to stop the dma/adc and deinit/reinit it to change the watchdog, plus we need to calculate vdd to get a good value

	uint16_t vrefvalue = (uint16_t) *vrefptr;
	float vddcalc = (float)3.0 * ((float)vrefvalue / (float)ADC_VREF);
	uint16_t amp = (uint16_t)( ((float)val * (float)0.15 * (float)20.0) * (float)4095 / (float)vddcalc);

	if(amp >= 4095){
		ADC1->HTR = 4095;
	}
	else{
		ADC1->HTR = amp;
	}
}
/* ADC Section End -----------------------------------------------------------*/

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

void LCD_CursorBlinkOnOff( uint8_t cursor_status, uint8_t blink_status){

	if( blink_status == 1 && cursor_status == 1){
		lcd_send_cmd(0x0F);
	}
	else if(blink_status == 0 && cursor_status == 1){
		lcd_send_cmd(0x0E);
	}
	else if(blink_status == 1 && cursor_status == 0){
		lcd_send_cmd(0x0D);
	}
	else {
		lcd_send_cmd(0x0C);
	}
}

void LCD_BlinkOnOff( uint8_t status ){

	if( status == 1 ){
		lcd_send_cmd(0x0D);
	}
	else {
		lcd_send_cmd(0x0C);
	}
}

void lcd_clear (void)
{
	lcd_send_cmd (0x00);
	for (int i=0; i<100; i++)
	{
		lcd_send_data (' ');
	}
}

void lcd_put_cur(int row, int col)
{
    switch (row)
    {
		case 0:
            col += 0x80;
            break;
        case 1:
            col += 0xC0;  //C0
            break;
        case 2:
            col += 0x94;	//0x80|0x14 for row 3 col 2
            break;
        case 3:
            col += 0xD4;
            break;
    }

    lcd_send_cmd (col);
}


void lcd_init (void)
{
	uint8_t i=0;
	HAL_Delay(100);
	for(i=0;i<3;i++){//sending 3 times: select 4-bit mode
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
	for(int i = 0; i <= 63 ; i++){
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
	//lcd_init();

	lcd_put_cur(0, 0);
	lcd_send_string("V1:0.00V ");
	lcd_send_data((uint8_t)1);//Custom Char 1
	lcd_send_string("V1");
	lcd_send_data((uint8_t)0);//Custom Char 0
	lcd_send_string(":0.00V");

	lcd_put_cur(1, 0);
	lcd_send_string("A1:0.000A");
	lcd_send_data((uint8_t)1);
	lcd_send_string("A1");
	lcd_send_data((uint8_t)0);
	lcd_send_string(":0.000A");

	lcd_put_cur(2, 0);
	lcd_send_string("V2:0.00V ");
	lcd_send_data((uint8_t)1);
	lcd_send_string("V2");
	lcd_send_data((uint8_t)0);
	lcd_send_string(":0.00V");

	lcd_put_cur(3, 0);
	lcd_send_string("A2:0.000A");
	lcd_send_data((uint8_t)1);
	lcd_send_string("A2");
	lcd_send_data((uint8_t)0);
	lcd_send_string(":0.000A");
}

void lcd_psu_welcome(void){
	lcd_init();

	lcd_put_cur(1, 0);
	lcd_send_string("493 Lab Power Supply");
	lcd_put_cur(2, 0);
	lcd_send_string("   Please Wait...   ");
}

void lcd_psu_update(void){
	if(startmessage){
		startmessage = 0;
		lcd_psu_init();
	}
	LCD_CursorBlinkOnOff(0,0);
	if(kpenum == WAIT){
		lcd_update_voltage(1,volt_set_aux);
		lcd_update_amperage(1,amp_set_aux);
		lcd_update_voltage(2,lin_num_aux);
		lcd_update_amperage(2,cur_num_aux);
		lcd_update_voltage(3,volt_set_main);
		lcd_update_amperage(3,amp_set_main);
		lcd_update_voltage(4,lin_num);
		lcd_update_amperage(4,cur_num);
	}
	else if(kpenum == V1){
		//lcd_update_voltage(1,volt_set_aux);
		lcd_update_amperage(1,amp_set_aux);
		lcd_update_voltage(2,lin_num_aux);
		lcd_update_amperage(2,cur_num_aux);
		lcd_update_voltage(3,volt_set_main);
		lcd_update_amperage(3,amp_set_main);
		lcd_update_voltage(4,lin_num);
		lcd_update_amperage(4,cur_num);
		lcd_put_cur(0,3);
		lcd_send_string("      ");
		lcd_put_cur(0,3);
		LCD_CursorBlinkOnOff(1,1);
		//Print the keypad array
		if(keypadarr[0] != 'z'){
			lcd_send_data(keypadarr[0]);
		}
		if(keypadarr[1] != 'z'){
			lcd_send_data(keypadarr[1]);
		}
		if(keypadarr[2] != 'z'){
			lcd_send_data(keypadarr[2]);
		}
		if(keypadarr[3] != 'z'){
			lcd_send_data(keypadarr[3]);
		}
		if(keypadarr[4] != 'z'){
			lcd_send_data(keypadarr[4]);
		}
		if(encmode){
			lcd_put_cur(0, 3 + encpos);
		}
	}
	else if(kpenum == V2){
		lcd_update_voltage(1,volt_set_aux);
		lcd_update_amperage(1,amp_set_aux);
		lcd_update_voltage(2,lin_num_aux);
		lcd_update_amperage(2,cur_num_aux);
		//lcd_update_voltage(3,volt_set_main);
		lcd_update_amperage(3,amp_set_main);
		lcd_update_voltage(4,lin_num);
		lcd_update_amperage(4,cur_num);
		lcd_put_cur(2,3);
		lcd_send_string("      ");
		lcd_put_cur(2,3);
		LCD_CursorBlinkOnOff(1,1);
		//Print the keypad array
		if(keypadarr[0] != 'z'){
			lcd_send_data(keypadarr[0]);
		}
		if(keypadarr[1] != 'z'){
			lcd_send_data(keypadarr[1]);
		}
		if(keypadarr[2] != 'z'){
			lcd_send_data(keypadarr[2]);
		}
		if(keypadarr[3] != 'z'){
			lcd_send_data(keypadarr[3]);
		}
		if(keypadarr[4] != 'z'){
			lcd_send_data(keypadarr[4]);
		}
		if(encmode){
			lcd_put_cur(2, 3 + encpos);
		}
	}
	else if(kpenum == A1){
		lcd_update_voltage(1,volt_set_aux);
		//lcd_update_amperage(1,amp_set_aux);
		lcd_update_voltage(2,lin_num_aux);
		lcd_update_amperage(2,cur_num_aux);
		lcd_update_voltage(3,volt_set_main);
		lcd_update_amperage(3,amp_set_main);
		lcd_update_voltage(4,lin_num);
		lcd_update_amperage(4,cur_num);
		lcd_put_cur(1,3);
		lcd_send_string("      ");
		lcd_put_cur(1,3);
		LCD_CursorBlinkOnOff(1,1);
		//Print the keypad array
		if(keypadarr[0] != 'z'){
			lcd_send_data(keypadarr[0]);
		}
		if(keypadarr[1] != 'z'){
			lcd_send_data(keypadarr[1]);
		}
		if(keypadarr[2] != 'z'){
			lcd_send_data(keypadarr[2]);
		}
		if(keypadarr[3] != 'z'){
			lcd_send_data(keypadarr[3]);
		}
		if(keypadarr[4] != 'z'){
			lcd_send_data(keypadarr[4]);
		}
		if(encmode){
			lcd_put_cur(1, 3 + encpos);
		}
	}
	else if(kpenum == A2){
		lcd_update_voltage(1,volt_set_aux);
		lcd_update_amperage(1,amp_set_aux);
		lcd_update_voltage(2,lin_num_aux);
		lcd_update_amperage(2,cur_num_aux);
		lcd_update_voltage(3,volt_set_main);
		//lcd_update_amperage(3,amp_set_main);
		lcd_update_voltage(4,lin_num);
		lcd_update_amperage(4,cur_num);
		lcd_put_cur(3,3);
		lcd_send_string("      ");
		lcd_put_cur(3,3);
		LCD_CursorBlinkOnOff(1,1);
		//Print the keypad array
		if(keypadarr[0] != 'z'){
			lcd_send_data(keypadarr[0]);
		}
		if(keypadarr[1] != 'z'){
			lcd_send_data(keypadarr[1]);
		}
		if(keypadarr[2] != 'z'){
			lcd_send_data(keypadarr[2]);
		}
		if(keypadarr[3] != 'z'){
			lcd_send_data(keypadarr[3]);
		}
		if(keypadarr[4] != 'z'){
			lcd_send_data(keypadarr[4]);
		}
		if(encmode){
			lcd_put_cur(3, 3 + encpos);
		}
	}
}

/* LCD Section End -----------------------------------------------------------*/

/* Keypad Section Begin ------------------------------------------------------*/
void lcd_update_voltage(uint8_t channel, float num){
	char kpbuff[8];
	snprintf(kpbuff, 6, "%.2f", num);
	//V1 Set
	if(channel == 1){
		lcd_put_cur(0,3);
		lcd_send_string("      ");//Clear current number
		lcd_put_cur(0,3);
		lcd_send_string(kpbuff);
		lcd_send_string("V");

	}
	//V1 Out
	else if(channel == 2){
		lcd_put_cur(0,14);
		lcd_send_string("      ");//Clear current number
		lcd_put_cur(0,14);
		lcd_send_string(kpbuff);
		lcd_send_string("V");
	}
	//V2 Set
	else if(channel == 3){
		lcd_put_cur(2,3);
		lcd_send_string("      ");//Clear current number
		lcd_put_cur(2,3);
		lcd_send_string(kpbuff);
		lcd_send_string("V");
	}
	//V2 Out
	else if(channel == 4){
		lcd_put_cur(2,14);
		lcd_send_string("      ");//Clear current number
		lcd_put_cur(2,14);
		lcd_send_string(kpbuff);
		lcd_send_string("V");
	}
	else{
		//Error
	}
}
void lcd_update_amperage(uint8_t channel, float num){
	char kpbuff[8];
	snprintf(kpbuff, 6, "%.3f", num);
	//A1 Set
	if(channel == 1){
		lcd_put_cur(1,3);
		lcd_send_string("      ");//Clear current number
		lcd_put_cur(1,3);
		lcd_send_string(kpbuff);
		lcd_send_string("A");

	}
	//A1 Out
	else if(channel == 2){
		lcd_put_cur(1,14);
		lcd_send_string("      ");//Clear current number
		lcd_put_cur(1,14);
		lcd_send_string(kpbuff);
		lcd_send_string("A");
	}
	//A2 Set
	else if(channel == 3){
		lcd_put_cur(3,3);
		lcd_send_string("      ");//Clear current number
		lcd_put_cur(3,3);
		lcd_send_string(kpbuff);
		lcd_send_string("A");
	}
	//A2 Out
	else if(channel == 4){
		lcd_put_cur(3,14);
		lcd_send_string("      ");//Clear current number
		lcd_put_cur(3,14);
		lcd_send_string(kpbuff);
		lcd_send_string("A");
	}
	else{
		//Error
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
	if(num == 'z'){
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
	}
	else if(num == '.'){
		if(keypaditerator >= keypadlength-3 && keypaddecimal == 0){
			//shift in new entry with up to two numbers preceeding it
			for(int i = 1; i < keypadlength; i++){
				keypadarr[i-1] = keypadarr[i];
			}
			keypadarr[keypadlength-1] = num;
			keypaddecimal = 1;
			keypaditerator--;
		}
	}
	else if(num >= '0' && num <= '9'){
		//Voltage allows xx.xx
		if(kpenum == V1 || kpenum == V2){
			//Allow entries if only up to one number has been entered
			if(keypaditerator > 2){
				//shift in new entry
				for(int i = 1; i < keypadlength; i++){
					keypadarr[i-1] = keypadarr[i];
				}
				keypadarr[keypadlength-1] = num;
				keypaditerator--;
			}
			//Allow first number after decimal place with no leading number
			else if(keypaditerator == 3 && keypadarr[4] == '.'){
				//shift in new entry
				for(int i = 1; i < keypadlength; i++){
					keypadarr[i-1] = keypadarr[i];
				}
				keypadarr[keypadlength-1] = num;
				keypaditerator--;
			}
			//Allow second number after decimal place with no leading number
			else if(keypaditerator == 2 && keypadarr[3] == '.'){
				//shift in new entry
				for(int i = 1; i < keypadlength; i++){
					keypadarr[i-1] = keypadarr[i];
				}
				keypadarr[keypadlength-1] = num;
				keypaditerator--;
			}
			//Allow first number after decimal place with one leading number
			else if(keypaditerator == 2 && keypadarr[4] == '.'){
				//shift in new entry
				for(int i = 1; i < keypadlength; i++){
					keypadarr[i-1] = keypadarr[i];
				}
				keypadarr[keypadlength-1] = num;
				keypaditerator--;
			}
			//Allow second number after decimal place with one leading number
			else if(keypaditerator == 1 && keypadarr[3] == '.'){
				//shift in new entry
				for(int i = 1; i < keypadlength; i++){
					keypadarr[i-1] = keypadarr[i];
				}
				keypadarr[keypadlength-1] = num;
				keypaditerator--;
			}
			//Allow first number after decimal place with two leading numbers
			else if(keypaditerator == 1 && keypadarr[4] == '.'){
				//shift in new entry
				for(int i = 1; i < keypadlength; i++){
					keypadarr[i-1] = keypadarr[i];
				}
				keypadarr[keypadlength-1] = num;
				keypaditerator--;
			}
			//Allow second number after decimal place with two leading numbers
			else if(keypaditerator == 0 && keypadarr[3] == '.'){
				//shift in new entry
				for(int i = 1; i < keypadlength; i++){
					keypadarr[i-1] = keypadarr[i];
				}
				keypadarr[keypadlength-1] = num;
				keypaditerator--;
			}
		}
		//Amperage allows x.xxx
		else if(kpenum == A1 || kpenum == A2){
			//Allow entries if no number has been entered
			if(keypaditerator > 3){
				//shift in new entry
				for(int i = 1; i < keypadlength; i++){
					keypadarr[i-1] = keypadarr[i];
				}
				keypadarr[keypadlength-1] = num;
				keypaditerator--;
			}
			//Allow first number after decimal place with no leading number
			else if(keypaditerator == 3 && keypadarr[4] == '.'){
				//shift in new entry
				for(int i = 1; i < keypadlength; i++){
					keypadarr[i-1] = keypadarr[i];
				}
				keypadarr[keypadlength-1] = num;
				keypaditerator--;
			}
			//Allow second number after decimal place with no leading number
			else if(keypaditerator == 2 && keypadarr[3] == '.'){
				//shift in new entry
				for(int i = 1; i < keypadlength; i++){
					keypadarr[i-1] = keypadarr[i];
				}
				keypadarr[keypadlength-1] = num;
				keypaditerator--;
			}
			//Allow third number after decimal place with no leading number
			else if(keypaditerator == 1 && keypadarr[2] == '.'){
				//shift in new entry
				for(int i = 1; i < keypadlength; i++){
					keypadarr[i-1] = keypadarr[i];
				}
				keypadarr[keypadlength-1] = num;
				keypaditerator--;
			}
			//Allow first number after decimal place with one leading number
			else if(keypaditerator == 2 && keypadarr[4] == '.'){
				//shift in new entry
				for(int i = 1; i < keypadlength; i++){
					keypadarr[i-1] = keypadarr[i];
				}
				keypadarr[keypadlength-1] = num;
				keypaditerator--;
			}
			//Allow second number after decimal place with one leading number
			else if(keypaditerator == 1 && keypadarr[3] == '.'){
				//shift in new entry
				for(int i = 1; i < keypadlength; i++){
					keypadarr[i-1] = keypadarr[i];
				}
				keypadarr[keypadlength-1] = num;
				keypaditerator--;
			}
			//Allow third number after decimal place with one leading number
			else if(keypaditerator == 0 && keypadarr[2] == '.'){
				//shift in new entry
				for(int i = 1; i < keypadlength; i++){
					keypadarr[i-1] = keypadarr[i];
				}
				keypadarr[keypadlength-1] = num;
				keypaditerator--;
			}
		}
	}
}

void clear_keypad(void){
	while(keypaditerator < 4){
		update_keypad('z');
	}
}

float translate_keypad(void){
	float num = 0;
	int decimallocation = -1;

	for(int i = 0; i < keypadlength; i++){
		if(keypadarr[i] == '.'){
			decimallocation = i;
		}
	}

	if(decimallocation == -1){
		//No decimal in array
		int count = 1;
		for(int i = keypadlength-1; i >=0; i--){
			if(keypadarr[i] >= '0' && keypadarr[i] <= '9'){
				num += ( (int)keypadarr[i] - (int)'0' ) * (count);
				count = count * 10;
			}
		}
	}
	else{
		//First do numbers to the left of the decimal
		int count = 1;
		for(int i = decimallocation-1; i >=0; i--){
			if(keypadarr[i] >= '0' && keypadarr[i] <= '9'){
				num += ( (int)keypadarr[i] - (int)'0' ) * (count);
				count = count * 10;
			}
		}
		//Next do numbers to the right of the decimal
		count = 10;
		for(int i = decimallocation+1; i < keypadlength; i++){
			if(keypadarr[i] >= '0' && keypadarr[i] <= '9'){
				num += ( (float)(int)keypadarr[i] - (int)'0' ) / (float)(count);
				count = count * 10;
			}
		}
	}

	return num;
}

uint8_t check_keypad(uint8_t which){
	//which=0 for voltage which=1 for amperage
	float temp = translate_keypad();
	return (which) ? (temp >= 0 && temp <= 0.8001) : (temp >= 0 && temp <= 12.00);
}

/*
 * This function is to be used for the rotary encoder to empty the current keypad and dump the current set voltage or amperage into it
 * va == 0 -> Dump voltage, va == 1 -> dump amperage because voltage allows xx.xx while amperage allows x.xxx
 * we take a float num as an input to allow easy but inefficient rotary control
 */
void fill_keypad(uint8_t va, float num){
	//Clear the keypad first
	clear_keypad();
	if(va){
		int temp = (int)(num * 1000);
		//special case for 0A
		if(num <= 0.0001){
			keypadarr[0] = '0';
			keypadarr[1] = '.';
			keypadarr[2] = '0';
			keypadarr[3] = '0';
			keypadarr[4] = '0';
			keypaditerator = -1;
		}
		else{
			while(temp != 0 && keypaditerator >= 0){
				keypadarr[keypaditerator] = (temp % 10) + (int)'0';
				temp = temp / 10;
				keypaditerator--;
				//add decimal
				if(keypaditerator == 1){
					keypadarr[keypaditerator] = '.';
					keypaditerator--;
				}
			}
			//include leading 0
			if(keypaditerator >= 0 && num <= 0.999999){
				keypadarr[keypaditerator] = '0';
				keypaditerator--;
			}
		}
	}
	else{
		int temp = (int)(num * 100);
		//special case for 0V
		if(num <= 0.001){
			keypadarr[0] = '0';
			keypadarr[1] = '0';
			keypadarr[2] = '.';
			keypadarr[3] = '0';
			keypadarr[4] = '0';
			keypaditerator = -1;
		}
		else{
			while(temp != 0 && keypaditerator >= 0){
				keypadarr[keypaditerator] = (temp % 10) + (int)'0';
				temp = temp / 10;
				keypaditerator--;
				//add decimal
				if(keypaditerator == 2){
					keypadarr[keypaditerator] = '.';
					keypaditerator--;
				}
			}
			//include leading 0
			if(keypaditerator >= 0 && num <= 9.999999){
				keypadarr[keypaditerator] = '0';
				keypaditerator--;
			}
			//include second leading 0
			if(keypaditerator >= 0 && num <= 0.999999){
				keypadarr[keypaditerator] = '0';
				keypaditerator--;
			}
		}
	}
}

/*
 * The following four recursive functions are meant to mostly replace the above function to improve performance and accuracy
 * The above function is still needed once we enter encoder mode but it should only be needed once
 */
void inc_arr_v(int8_t pos){
	//We do not allow incrementing past 12.00
	if( !(pos == 0 && keypadarr[0] == '1' && keypadarr[1] >= '0' && keypadarr[3] >= '0' && keypadarr[4] >= '0') &&
		!(pos == 0 && keypadarr[0] == '0' && keypadarr[1] > '2' && keypadarr[3] >= '0' && keypadarr[4] >= '0') &&
		!(pos == 0 && keypadarr[0] == '0' && keypadarr[1] >= '2' && (keypadarr[3] > '0' || keypadarr[4] > '0')) &&
		!(pos == 1 && keypadarr[0] == '1' && keypadarr[1] == '2' && keypadarr[3] == '0' && keypadarr[4] == '0') &&
		!(pos == 1 && keypadarr[0] == '1' && keypadarr[1] == '1' && (keypadarr[3] > '0' || keypadarr[4] > '0')) &&
		!(pos == 3 && keypadarr[0] == '1' && keypadarr[1] == '2' && keypadarr[3] == '0' && keypadarr[4] >= '0') &&
		!(pos == 3 && keypadarr[0] == '1' && keypadarr[1] == '1' && keypadarr[3] == '9' && keypadarr[4] > '0') &&
		!(pos == 4 && keypadarr[0] == '1' && keypadarr[1] == '2' && keypadarr[3] == '0' && keypadarr[4] == '0') ){
		//Position must be positive and we do not want to increment the '.' char located in position 2
		if(pos >= 0 && pos != 2){
			//Increment
			if(keypadarr[pos] < '9'){
				keypadarr[pos]++;
			}
			//Recursively increment
			else{
				keypadarr[pos] = '0';
				inc_arr_v(pos-1);
			}
		}
		//Skip decimal place
		else if(pos == 2){
			inc_arr_v(pos-1);
		}
	}
	//If an attempt to pass 12V is made we can just set 12V
	else{
		keypadarr[0] = '1';
		keypadarr[1] = '2';
		keypadarr[2] = '.';
		keypadarr[3] = '0';
		keypadarr[4] = '0';
	}
	//update keypad iterator
	for(int i = 0; i < keypadlength; i++){
		if(keypadarr[i] != 'z'){
			keypaditerator = i - 1;
			break;
		}
	}
}

void inc_arr_a(int8_t pos){
	//We do not allow incrementing past 0.800
	if( !(pos == 0) &&//just don't even increment the first digit since our max limit is < 1
		!(pos == 2 && keypadarr[0] == '0' && keypadarr[2] == '8' && keypadarr[3] == '0' && keypadarr[4] == '0') &&
		!(pos == 2 && keypadarr[0] == '0' && keypadarr[2] == '7' && (keypadarr[3] > '0' || keypadarr[4] > '0')) &&
		!(pos == 3 && keypadarr[0] == '0' && keypadarr[2] == '8' && keypadarr[3] == '0' && keypadarr[4] >= '0') &&
		!(pos == 3 && keypadarr[0] == '0' && keypadarr[2] == '7' && keypadarr[3] == '9' && keypadarr[4] > '0') &&
		!(pos == 4 && keypadarr[0] == '0' && keypadarr[2] == '8' && keypadarr[3] == '0' && keypadarr[4] == '0') ){
		//Position must be positive and we do not want to increment the '.' char located in position 1
		if(pos >= 0 && pos != 1){
			//Increment
			if(keypadarr[pos] < '9'){
				keypadarr[pos]++;
			}
			//Recursively increment
			else{
				keypadarr[pos] = '0';
				inc_arr_a(pos-1);
			}
		}
		//Skip decimal place
		else if(pos == 1){
			inc_arr_a(pos-1);
		}
	}
	//If an attempt to pass 0.8A is made we can just set 0.8A
	else{
		keypadarr[0] = '0';
		keypadarr[1] = '.';
		keypadarr[2] = '8';
		keypadarr[3] = '0';
		keypadarr[4] = '0';
	}
	//update keypad iterator
	for(int i = 0; i < keypadlength; i++){
		if(keypadarr[i] != 'z'){
			keypaditerator = i - 1;
			break;
		}
	}
}

void dec_arr_v(int8_t pos){
	//We do not allow decrementing past 00.00
	if( !(pos == 0 && keypadarr[0] == '0') &&
		!(pos == 1 && keypadarr[0] == '0' && keypadarr[1] == '0') &&
		!(pos == 3 && keypadarr[0] == '0' && keypadarr[1] == '0' && keypadarr[3] == '0') &&
		!(pos == 4 && keypadarr[0] == '0' && keypadarr[1] == '0' && keypadarr[3] == '0' && keypadarr[4] == '0') ){
		//Position must be positive and we do not want to increment the '.' char located in position 2
		if(pos <= 4 && pos != 2){
			//Increment
			if(keypadarr[pos] > '0'){
				keypadarr[pos]--;
			}
			//Recursively increment
			else{
				keypadarr[pos] = '9';
				dec_arr_v(pos-1);
			}
		}
		//Skip decimal place
		else if(pos == 2){
			dec_arr_v(pos-1);
		}
	}
	//If an attempt to pass 0V is made we can just set 0V
	else{
		keypadarr[0] = '0';
		keypadarr[1] = '0';
		keypadarr[2] = '.';
		keypadarr[3] = '0';
		keypadarr[4] = '0';
	}
	//update keypad iterator
	for(int i = 0; i < keypadlength; i++){
		if(keypadarr[i] != 'z'){
			keypaditerator = i - 1;
			break;
		}
	}
}

void dec_arr_a(int8_t pos){
	//We do not allow decrementing past 0.000
	if( !(pos == 0 && keypadarr[0] == '0') &&
		!(pos == 1 && keypadarr[0] == '0' && keypadarr[2] == '0') &&
		!(pos == 3 && keypadarr[0] == '0' && keypadarr[2] == '0' && keypadarr[3] == '0') &&
		!(pos == 4 && keypadarr[0] == '0' && keypadarr[2] == '0' && keypadarr[3] == '0' && keypadarr[4] == '0') ){
		//Position must be positive and we do not want to increment the '.' char located in position 2
		if(pos <= 4 && pos != 1){
			//Increment
			if(keypadarr[pos] > '0'){
				keypadarr[pos]--;
			}
			//Recursively increment
			else{
				keypadarr[pos] = '9';
				dec_arr_a(pos-1);
			}
		}
		//Skip decimal place
		else if(pos == 1){
			dec_arr_a(pos-1);
		}
	}
	//If an attempt to pass 0A is made we can just set 0A
	else{
		keypadarr[0] = '0';
		keypadarr[1] = '.';
		keypadarr[2] = '0';
		keypadarr[3] = '0';
		keypadarr[4] = '0';
	}
	//update keypad iterator
	for(int i = 0; i < keypadlength; i++){
		if(keypadarr[i] != 'z'){
			keypaditerator = i - 1;
			break;
		}
	}
}

void keypad_sm(char num){
	//A=V1;B=A1;C=V2;D=A2;
	if(kpenum == WAIT){
		//While in wait we only listen to letters for channel number and type
		if(num == 'A'){
			kpenum = V1;
			clear_keypad();
		}
		else if(num == 'B'){
			kpenum = A1;
			clear_keypad();
		}
		else if(num == 'C'){
			kpenum = V2;
			clear_keypad();
		}
		else if(num == 'D'){
			kpenum = A2;
			clear_keypad();
		}
		else if(num == '*'){
			//Disable channel output for second mcu if second MCU claims to be active
			if(chstat_aux_rx){
				chstat_aux_tx = 0;
			}
			//Enable channel output for second mcu if second MCU claims to be inactive
			else{
				chstat_aux_tx = 1;
			}
		}
		else if(num == '/'){
			//Disable channel output if active
			if(chstat_main){
				chstat_main = 0;
			}
			//Enable channel output if inactive
			else{
				chstat_main = 1;
			}
		}
	}
	else if(kpenum == V1){
		//Stop listening to keypad in encoder mode
		if(encmode){
			if(num == 'A'){
				uint8_t test = check_keypad(0);
				if(test){
					//Only update the value if valid
					volt_set_aux = translate_keypad();
				}
				kpenum = WAIT;
				clear_keypad();
				encmode = 0;
				encpos = 0;
			}
			else if(num == 'B'){
				kpenum = WAIT;
				clear_keypad();
				encmode = 0;
				encpos = 0;
			}
			else if(num == 'C'){
				kpenum = WAIT;
				clear_keypad();
				encmode = 0;
				encpos = 0;
			}
			else if(num == 'D'){
				kpenum = WAIT;
				clear_keypad();
				encmode = 0;
				encpos = 0;
			}
			else if(num == '.'){
				//If a keypad entry key is pressed forget what we were doing and go back to keypad entry
				clear_keypad();
				encmode = 0;
				encpos = 0;
			}
			else if(num >= '0' && num <= '9'){
				//If a keypad entry key is pressed forget what we were doing and go back to keypad entry
				clear_keypad();
				encmode = 0;
				encpos = 0;
			}
			else if(num == '#'){
				//If a keypad entry key is pressed forget what we were doing and go back to keypad entry
				clear_keypad();
				encmode = 0;
				encpos = 0;
			}
			else if(num == '+'){
				uint8_t test = check_keypad(0);
				if(test){
					//Only update the value if valid
					volt_set_aux = translate_keypad();
				}
				kpenum = WAIT;
				clear_keypad();
				encmode = 0;
				encpos = 0;
			}
			else if(num == '-'){
				kpenum = WAIT;
				clear_keypad();
				encmode = 0;
				encpos = 0;
			}
			else if(num == 's'){
				//skip decimal
				if(encpos == 1){
					encpos++;
					encpos++;
				}
				else if(encpos < 4){
					encpos++;
				}
				else{
					encpos = 0;
				}
			}
			else if(num == '['){
				dec_arr_v(encpos);
			}
			else if(num == ']'){
				inc_arr_v(encpos);
			}
		}
		//Default keypad sm
		else{
			//While in channel listen to numbers/decimal and letters for ENTER
			if(num == 'A'){
				uint8_t test = check_keypad(0);
				if(test){
					//Only update the value if valid
					volt_set_aux = translate_keypad();
				}
				kpenum = WAIT;
				clear_keypad();
			}
			else if(num == 'B'){
				kpenum = WAIT;
				clear_keypad();
			}
			else if(num == 'C'){
				kpenum = WAIT;
				clear_keypad();
			}
			else if(num == 'D'){
				kpenum = WAIT;
				clear_keypad();
			}
			else if(num == '.'){
				//It "should" be impossible to pass in a bad decimal and we can ignore bad calls
				update_keypad(num);
			}
			else if(num >= '0' && num <= '9'){
				//It "should" be impossible to overfill the array but we could do additional checks here for valid numbers
				update_keypad(num);
			}
			else if(num == '#'){
				update_keypad('z');
			}
			else if(num == '+'){
				uint8_t test = check_keypad(0);
				if(test){
					//Only update the value if valid
					volt_set_aux = translate_keypad();
				}
				kpenum = WAIT;
				clear_keypad();
			}
			else if(num == '-'){
				kpenum = WAIT;
				clear_keypad();
			}
			else if(num == 's'){
				fill_keypad(0, volt_set_aux);
				encmode = 1;
				encpos = 0;
			}
			else if(num == '['){
				fill_keypad(0, volt_set_aux);
				encmode = 1;
				encpos = 0;
			}
			else if(num == ']'){
				fill_keypad(0, volt_set_aux);
				encmode = 1;
				encpos = 0;
			}
		}
	}
	else if(kpenum == A1){
		if(encmode){
			if(num == 'A'){
				kpenum = WAIT;
				clear_keypad();
				encmode = 0;
				encpos = 0;
			}
			else if(num == 'B'){
				uint8_t test = check_keypad(0);
				if(test){
					//Only update the value if valid
					amp_set_aux = translate_keypad();
				}
				kpenum = WAIT;
				clear_keypad();
				encmode = 0;
				encpos = 0;
			}
			else if(num == 'C'){
				kpenum = WAIT;
				clear_keypad();
				encmode = 0;
				encpos = 0;
			}
			else if(num == 'D'){
				kpenum = WAIT;
				clear_keypad();
				encmode = 0;
				encpos = 0;
			}
			else if(num == '.'){
				//If a keypad entry key is pressed forget what we were doing and go back to keypad entry
				clear_keypad();
				encmode = 0;
				encpos = 0;
			}
			else if(num >= '0' && num <= '9'){
				//If a keypad entry key is pressed forget what we were doing and go back to keypad entry
				clear_keypad();
				encmode = 0;
				encpos = 0;
			}
			else if(num == '#'){
				//If a keypad entry key is pressed forget what we were doing and go back to keypad entry
				clear_keypad();
				encmode = 0;
				encpos = 0;
			}
			else if(num == '+'){
				uint8_t test = check_keypad(0);
				if(test){
					//Only update the value if valid
					amp_set_aux = translate_keypad();
				}
				kpenum = WAIT;
				clear_keypad();
				encmode = 0;
				encpos = 0;
			}
			else if(num == '-'){
				kpenum = WAIT;
				clear_keypad();
				encmode = 0;
				encpos = 0;
			}
			else if(num == 's'){
				//skip decimal
				if(encpos == 0){
					encpos++;
					encpos++;
				}
				else if(encpos < 4){
					encpos++;
				}
				else{
					encpos = 0;
				}
			}
			else if(num == '['){
				dec_arr_a(encpos);
			}
			else if(num == ']'){
				inc_arr_a(encpos);
			}
		}
		//Default keypad sm
		else{
			//While in channel listen to numbers/decimal and letters for ENTER
			if(num == 'A'){
				kpenum = WAIT;
				clear_keypad();
			}
			else if(num == 'B'){
				uint8_t test = check_keypad(1);
				if(test){
					//Only update the value if valid
					amp_set_aux = translate_keypad();
				}
				kpenum = WAIT;
				clear_keypad();
			}
			else if(num == 'C'){
				kpenum = WAIT;
				clear_keypad();
			}
			else if(num == 'D'){
				kpenum = WAIT;
				clear_keypad();
			}
			else if(num == '.'){
				//It "should" be impossible to pass in a bad decimal and we can ignore bad calls
				update_keypad(num);
			}
			else if(num >= '0' && num <= '9'){
				//It "should" be impossible to overfill the array but we could do additional checks here for valid numbers
				update_keypad(num);
			}
			else if(num == '#'){
				update_keypad('z');
			}
			else if(num == '+'){
				uint8_t test = check_keypad(1);
				if(test){
					//Only update the value if valid
					amp_set_aux = translate_keypad();
				}
				kpenum = WAIT;
				clear_keypad();
			}
			else if(num == '-'){
				kpenum = WAIT;
				clear_keypad();
			}
			else if(num == 's'){
				fill_keypad(1, amp_set_aux);
				encmode = 1;
				encpos = 0;
			}
			else if(num == '['){
				fill_keypad(1, amp_set_aux);
				encmode = 1;
				encpos = 0;
			}
			else if(num == ']'){
				fill_keypad(1, amp_set_aux);
				encmode = 1;
				encpos = 0;
			}
		}
	}
	else if(kpenum == V2){
		//Stop listening to keypad in encoder mode
		if(encmode){
			if(num == 'A'){
				kpenum = WAIT;
				clear_keypad();
				encmode = 0;
				encpos = 0;
			}
			else if(num == 'B'){
				kpenum = WAIT;
				clear_keypad();
				encmode = 0;
				encpos = 0;
			}
			else if(num == 'C'){
				uint8_t test = check_keypad(0);
				if(test){
					//Only update the value if valid
					volt_set_main_old = volt_set_main;
					volt_set_main = translate_keypad();
				}
				kpenum = WAIT;
				clear_keypad();
				encmode = 0;
				encpos = 0;
			}
			else if(num == 'D'){
				kpenum = WAIT;
				clear_keypad();
				encmode = 0;
				encpos = 0;
			}
			else if(num == '.'){
				//If a keypad entry key is pressed forget what we were doing and go back to keypad entry
				clear_keypad();
				encmode = 0;
				encpos = 0;
			}
			else if(num >= '0' && num <= '9'){
				//If a keypad entry key is pressed forget what we were doing and go back to keypad entry
				clear_keypad();
				encmode = 0;
				encpos = 0;
			}
			else if(num == '#'){
				//If a keypad entry key is pressed forget what we were doing and go back to keypad entry
				clear_keypad();
				encmode = 0;
				encpos = 0;
			}
			else if(num == '+'){
				uint8_t test = check_keypad(0);
				if(test){
					//Only update the value if valid
					volt_set_main_old = volt_set_main;
					volt_set_main = translate_keypad();
				}
				kpenum = WAIT;
				clear_keypad();
				encmode = 0;
				encpos = 0;
			}
			else if(num == '-'){
				kpenum = WAIT;
				clear_keypad();
				encmode = 0;
				encpos = 0;
			}
			else if(num == 's'){
				//skip decimal
				if(encpos == 1){
					encpos++;
					encpos++;
				}
				else if(encpos < 4){
					encpos++;
				}
				else{
					encpos = 0;
				}
			}
			else if(num == '['){
				dec_arr_v(encpos);
			}
			else if(num == ']'){
				inc_arr_v(encpos);
			}
		}
		//Default keypad sm
		else{
			//While in channel listen to numbers/decimal and letters for ENTER
			if(num == 'A'){
				kpenum = WAIT;
				clear_keypad();
			}
			else if(num == 'B'){
				kpenum = WAIT;
				clear_keypad();
			}
			else if(num == 'C'){
				uint8_t test = check_keypad(0);
				if(test){
					//Only update the value if valid
					volt_set_main_old = volt_set_main;
					volt_set_main = translate_keypad();
				}
				kpenum = WAIT;
				clear_keypad();
			}
			else if(num == 'D'){
				kpenum = WAIT;
				clear_keypad();
			}
			else if(num == '.'){
				//It "should" be impossible to pass in a bad decimal and we can ignore bad calls
				update_keypad(num);
			}
			else if(num >= '0' && num <= '9'){
				//It "should" be impossible to overfill the array but we could do additional checks here for valid numbers
				update_keypad(num);
			}
			else if(num == '#'){
				update_keypad('z');
			}
			else if(num == '+'){
				uint8_t test = check_keypad(0);
				if(test){
					//Only update the value if valid
					volt_set_main_old = volt_set_main;
					volt_set_main = translate_keypad();
				}
				kpenum = WAIT;
				clear_keypad();
			}
			else if(num == '-'){
				kpenum = WAIT;
				clear_keypad();
			}
			else if(num == 's'){
				fill_keypad(0, volt_set_main);
				encmode = 1;
				encpos = 0;
			}
			else if(num == '['){
				fill_keypad(0, volt_set_main);
				encmode = 1;
				encpos = 0;
			}
			else if(num == ']'){
				fill_keypad(0, volt_set_main);
				encmode = 1;
				encpos = 0;
			}
		}
	}
	else if(kpenum == A2){
		if(encmode){
			if(num == 'A'){
				kpenum = WAIT;
				clear_keypad();
				encmode = 0;
				encpos = 0;
			}
			else if(num == 'B'){
				kpenum = WAIT;
				clear_keypad();
				encmode = 0;
				encpos = 0;
			}
			else if(num == 'C'){
				kpenum = WAIT;
				clear_keypad();
				encmode = 0;
				encpos = 0;
			}
			else if(num == 'D'){
				uint8_t test = check_keypad(0);
				if(test){
					//Only update the value if valid
					amp_set_main_old = amp_set_main;
					amp_set_main = translate_keypad();
					update_ADC_watchdog(amp_set_main);
				}
				kpenum = WAIT;
				clear_keypad();
				encmode = 0;
				encpos = 0;
			}
			else if(num == '.'){
				//If a keypad entry key is pressed forget what we were doing and go back to keypad entry
				clear_keypad();
				encmode = 0;
				encpos = 0;
			}
			else if(num >= '0' && num <= '9'){
				//If a keypad entry key is pressed forget what we were doing and go back to keypad entry
				clear_keypad();
				encmode = 0;
				encpos = 0;
			}
			else if(num == '#'){
				//If a keypad entry key is pressed forget what we were doing and go back to keypad entry
				clear_keypad();
				encmode = 0;
				encpos = 0;
			}
			else if(num == '+'){
				uint8_t test = check_keypad(0);
				if(test){
					//Only update the value if valid
					amp_set_main_old = amp_set_main;
					amp_set_main = translate_keypad();
					update_ADC_watchdog(amp_set_main);
				}
				kpenum = WAIT;
				clear_keypad();
				encmode = 0;
				encpos = 0;
			}
			else if(num == '-'){
				kpenum = WAIT;
				clear_keypad();
				encmode = 0;
				encpos = 0;
			}
			else if(num == 's'){
				//skip decimal
				if(encpos == 0){
					encpos++;
					encpos++;
				}
				else if(encpos < 4){
					encpos++;
				}
				else{
					encpos = 0;
				}
			}
			else if(num == '['){
				dec_arr_a(encpos);
			}
			else if(num == ']'){
				inc_arr_a(encpos);
			}
		}
		//Default keypad sm
		else{
			//While in channel listen to numbers/decimal and letters for ENTER
			if(num == 'A'){
				kpenum = WAIT;
				clear_keypad();
			}
			else if(num == 'B'){
				kpenum = WAIT;
				clear_keypad();
			}
			else if(num == 'C'){
				kpenum = WAIT;
				clear_keypad();
			}
			else if(num == 'D'){
				uint8_t test = check_keypad(1);
				if(test){
					//Only update the value if valid
					amp_set_main_old = amp_set_main;
					amp_set_main = translate_keypad();
					update_ADC_watchdog(amp_set_main);
				}
				kpenum = WAIT;
				clear_keypad();
			}
			else if(num == '.'){
				//It "should" be impossible to pass in a bad decimal and we can ignore bad calls
				update_keypad(num);
			}
			else if(num >= '0' && num <= '9'){
				//It "should" be impossible to overfill the array but we could do additional checks here for valid numbers
				update_keypad(num);
			}
			else if(num == '#'){
				update_keypad('z');
			}
			else if(num == '+'){
				uint8_t test = check_keypad(1);
				if(test){
					//Only update the value if valid
					amp_set_main_old = amp_set_main;
					amp_set_main = translate_keypad();
					update_ADC_watchdog(amp_set_main);
				}
				kpenum = WAIT;
				clear_keypad();
			}
			else if(num == '-'){
				kpenum = WAIT;
				clear_keypad();
			}
			else if(num == 's'){
				fill_keypad(1, amp_set_main);
				encmode = 1;
				encpos = 0;
			}
			else if(num == '['){
				fill_keypad(1, amp_set_main);
				encmode = 1;
				encpos = 0;
			}
			else if(num == ']'){
				fill_keypad(1, amp_set_main);
				encmode = 1;
				encpos = 0;
			}
		}
	}
}

void row_input(void){
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
	HAL_NVIC_SetPriority(Row_1_EXTI_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(Row_1_EXTI_IRQn);
}

void column_input(void){
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	//Disable interrupts
	//HAL_NVIC_SetPriority(Row_1_EXTI_IRQn, 0, 0);
	HAL_NVIC_DisableIRQ(Row_1_EXTI_IRQn);

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
/* Keypad Section End --------------------------------------------------------*/

/* Interrupt Callback Section Begin ------------------------------------------*/
void USB_Interrupt_Callback(void){
	// Checking USBbuffer
	int8_t tempiter = headiter;
	tempiter++;
	if(tempiter >= CIRCSIZE){
	  tempiter = 0;
	}
	//Buffer is not full
	if(tempiter != tailiter){
		memcpy(notacircbuff[headiter], usbbuffer, 64);  // copy the data to the buffer
		headiter++;
		if(headiter >= CIRCSIZE){
			headiter = 0;
		}
	}
	//Ignore commands when buffer full
	memset (usbbuffer, '\0', 64); // clear the buffer
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	//Row 1
	if( (GPIO_Pin == Row_1_Pin) || (GPIO_Pin == Row_2_Pin) || (GPIO_Pin == Row_3_Pin) || (GPIO_Pin == Row_4_Pin) || (GPIO_Pin == Row_5_Pin)){
		//Falling edge
		if(HAL_GPIO_ReadPin(Row_1_GPIO_Port, GPIO_Pin) == 0){
			//Make sure we didn't interrupt on falling edge already
			if(kpedge != 0){
				kpedge = 0;
				//Save rowpin
				rowpin = GPIO_Pin;
				//start debounce
				HAL_TIM_Base_Start_IT(&htim2);
			}
		}
		//Rising edge
		else{
			//Make sure we didn't interrupt on rising edge already
			if(kpedge != 1){
				kpedge = 1;
			}
		}
	}
	else if(GPIO_Pin == Rot_SW_Pin){
		//Falling edge
		if(HAL_GPIO_ReadPin(Rot_SW_GPIO_Port, Rot_SW_Pin) == 0){
			//Make sure we didn't interrupt on falling edge already
			if(swedge != 0){
				swedge = 0;
				//start debounce
				HAL_NVIC_DisableIRQ(Rot_SW_EXTI_IRQn);
				HAL_TIM_Base_Start_IT(&htim9);
			}
		}
		//Rising edge
		else{
			//Make sure we didn't interrupt on rising edge already
			if(swedge != 1){
				swedge = 1;
			}
		}
	}
	else if(GPIO_Pin == Rot_CLK_Pin){
		if (rotenum == NOTURN) {
			HAL_NVIC_DisableIRQ(Rot_CLK_EXTI_IRQn);
			rotenum = (HAL_GPIO_ReadPin(Rot_CLK_GPIO_Port, Rot_CLK_Pin) == HAL_GPIO_ReadPin(Rot_DT_GPIO_Port, Rot_DT_Pin)) ? CWTURN : CCWTURN;
			HAL_TIM_Base_Start_IT(&htim10);
		}
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim2){
		//Disable timer now that we're in its interrupt
		HAL_TIM_Base_Stop_IT(&htim2);
		column_input();
		//Detect which column is held low (pressed)
		for(int i = 0; i < 4; i++){
			if(HAL_GPIO_ReadPin(col_ports[i], col_pins[i]) == 0){
				//Find rowpin value based on array of pins declared globally
				for(int j = 0; j < 5; j++){
					if(rowpin == row_pins[j]){
						//Call keypad state machine based on rowpin and colpin array declared globally
						keypad_sm(keypad_labels[j][i]);
						rowpin = -1;
						break;
					}
				}
				break;
			}
		}
		row_input();
	}
	else if(htim == &htim3){
		//Disable timer now that we're in its interrupt
		HAL_TIM_Base_Stop_IT(&htim3);
		//Update Display
		lcd_psu_update();
		//Start timer again
		HAL_TIM_Base_Start_IT(&htim3);
	}
	else if(htim == &htim4){
		//Disable timer now that we're in its interrupt
		HAL_TIM_Base_Stop_IT(&htim4);
		if(!startmessage){
			startmessage = 1;
			HAL_TIM_Base_Start_IT(&htim4);
		}
		else{
			//Start display
			HAL_TIM_Base_Start_IT(&htim3);
		}
	}
	else if(htim == &htim9){
		//Disable timer now that we're in its interrupt
		HAL_TIM_Base_Stop_IT(&htim9);
		//haha keypad_sm go brrrr
		keypad_sm('s');//s for switch
		HAL_NVIC_SetPriority(Rot_SW_EXTI_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(Rot_SW_EXTI_IRQn);
	}
	else if(htim == &htim10){
		//Disable timer now that we're in its interrupt
		HAL_TIM_Base_Stop_IT(&htim10);
		if(rotenum == CWTURN){
			//haha keypad_sm go brrrr
			keypad_sm(']');//right bracket for CW
		}
		else if(rotenum == CCWTURN){
			//haha keypad_sm go brrrr
			keypad_sm('[');//left bracket for CCW
		}
		rotenum = NOTURN;
		HAL_NVIC_SetPriority(Rot_CLK_EXTI_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(Rot_CLK_EXTI_IRQn);
	}
	else if(htim == &htim11){
		//Disable timer now that we're in its interrupt
		HAL_TIM_Base_Stop_IT(&htim11);
		if(chstat_main == 0){
			//Since we have a delay in updating the LED for channel 1 from UART, lets introduce a dumb fake delay on channel 2
			if(!timercounter){
				HAL_GPIO_WritePin(Status_LED_2_GPIO_Port, Status_LED_2_Pin, GPIO_PIN_RESET);
			}
			timercounter++;
			if(timercounter >= 7){
				timercounter = 0;
			}
			blink = 0;
		}
		else if(chstat_main == 1){
			if(!timercounter){
				HAL_GPIO_WritePin(Status_LED_2_GPIO_Port, Status_LED_2_Pin, GPIO_PIN_SET);
			}
			timercounter++;
			if(timercounter >= 7){
				timercounter = 0;
			}
			blink = 0;
		}
		else if(chstat_main == 2){
			if(blink){
				HAL_GPIO_TogglePin(Status_LED_2_GPIO_Port, Status_LED_2_Pin);
			}
			timercounter++;
			if(timercounter >= 7){
				timercounter = 0;
				blink = 1;
			}
		}

		if(chstat_aux_rx == 0){
			HAL_GPIO_WritePin(Status_LED_1_GPIO_Port, Status_LED_1_Pin, GPIO_PIN_RESET);
		}
		else if(chstat_aux_rx == 1){
			HAL_GPIO_WritePin(Status_LED_1_GPIO_Port, Status_LED_1_Pin, GPIO_PIN_SET);
		}
		else if(chstat_aux_rx == 2){
			HAL_GPIO_TogglePin(Status_LED_1_GPIO_Port, Status_LED_1_Pin);
		}
		HAL_TIM_Base_Start_IT(&htim11);
	}
}

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
	if(chstat_main == 1){
		HAL_GPIO_WritePin(Channel_Shutdown_GPIO_Port, Channel_Shutdown_Pin, GPIO_PIN_SET);
		chstat_main = 2;
	}
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

	//CDC_Transmit_FS(rxbuffer,64);
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

	//CDC_Transmit_FS(rxbuffercpy,32);

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

		lin_num_aux = tempv2;
		cur_num_aux = tempa2;
		chstat_aux_rx = rxbuffercpy[18]-48;

		//strcat((char*)rxbuffercpy, "\n");
		//CDC_Transmit_FS(rxbuffercpy,32);

		//uint8_t tstbuf[64];
		//snprintf((char*)tstbuf, 64, "%f, %f", tempv2, tempa2);
		//CDC_Transmit_FS(tstbuf,64);
	}

	memset (rxbuffer, '\0', 64);  // clear the buffer
	HAL_UART_Receive_DMA (&huart1, rxbuffer, 64);  // Receive 64 Bytes of data
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart){
	HAL_UART_Receive_DMA (&huart1, rxbuffer, 64); //Try again!
}

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
	snprintf((char*)txbuffer_cpy, 32, "*STRT,%05.2f,%5.3f,%d,FNSH!", volt_set_aux, amp_set_aux, chstat_aux_tx);
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
