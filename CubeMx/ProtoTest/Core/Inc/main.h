/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Channel_Shutdown_Pin GPIO_PIN_6
#define Channel_Shutdown_GPIO_Port GPIOA
#define Status_LED_Pin GPIO_PIN_5
#define Status_LED_GPIO_Port GPIOC
#define Rot_1_CLK_Pin GPIO_PIN_0
#define Rot_1_CLK_GPIO_Port GPIOB
#define Rot_1_CLK_EXTI_IRQn EXTI0_IRQn
#define Rot_1_DT_Pin GPIO_PIN_1
#define Rot_1_DT_GPIO_Port GPIOB
#define Rot_1_SW_Pin GPIO_PIN_2
#define Rot_1_SW_GPIO_Port GPIOB
#define Row_1_Pin GPIO_PIN_10
#define Row_1_GPIO_Port GPIOB
#define Row_1_EXTI_IRQn EXTI15_10_IRQn
#define Row_2_Pin GPIO_PIN_11
#define Row_2_GPIO_Port GPIOB
#define Row_2_EXTI_IRQn EXTI15_10_IRQn
#define Row_3_Pin GPIO_PIN_12
#define Row_3_GPIO_Port GPIOB
#define Row_3_EXTI_IRQn EXTI15_10_IRQn
#define Row_4_Pin GPIO_PIN_13
#define Row_4_GPIO_Port GPIOB
#define Row_4_EXTI_IRQn EXTI15_10_IRQn
#define Row_5_Pin GPIO_PIN_14
#define Row_5_GPIO_Port GPIOB
#define Row_5_EXTI_IRQn EXTI15_10_IRQn
#define Row_6_Pin GPIO_PIN_15
#define Row_6_GPIO_Port GPIOB
#define Row_6_EXTI_IRQn EXTI15_10_IRQn
#define Col_1_Pin GPIO_PIN_6
#define Col_1_GPIO_Port GPIOC
#define Col_2_Pin GPIO_PIN_7
#define Col_2_GPIO_Port GPIOC
#define Col_3_Pin GPIO_PIN_8
#define Col_3_GPIO_Port GPIOC
#define Col_4_Pin GPIO_PIN_9
#define Col_4_GPIO_Port GPIOC
#define Rot_2_CLK_Pin GPIO_PIN_3
#define Rot_2_CLK_GPIO_Port GPIOB
#define Rot_2_DT_Pin GPIO_PIN_4
#define Rot_2_DT_GPIO_Port GPIOB
#define ROT_2_SW_Pin GPIO_PIN_5
#define ROT_2_SW_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
