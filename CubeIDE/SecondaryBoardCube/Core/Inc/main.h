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
#define Unused_Pin_1_Pin GPIO_PIN_13
#define Unused_Pin_1_GPIO_Port GPIOC
#define Unused_Pin_2_Pin GPIO_PIN_14
#define Unused_Pin_2_GPIO_Port GPIOC
#define Unused_Pin_3_Pin GPIO_PIN_15
#define Unused_Pin_3_GPIO_Port GPIOC
#define Channel_Shutdown_Pin GPIO_PIN_6
#define Channel_Shutdown_GPIO_Port GPIOA
#define Unused_Pin_4_Pin GPIO_PIN_7
#define Unused_Pin_4_GPIO_Port GPIOA
#define Unused_Pin_5_Pin GPIO_PIN_0
#define Unused_Pin_5_GPIO_Port GPIOB
#define Unused_Pin_6_Pin GPIO_PIN_1
#define Unused_Pin_6_GPIO_Port GPIOB
#define Unused_Pin_7_Pin GPIO_PIN_2
#define Unused_Pin_7_GPIO_Port GPIOB
#define Unused_Pin_8_Pin GPIO_PIN_10
#define Unused_Pin_8_GPIO_Port GPIOB
#define Unused_Pin_9_Pin GPIO_PIN_11
#define Unused_Pin_9_GPIO_Port GPIOB
#define Unused_Pin_10_Pin GPIO_PIN_12
#define Unused_Pin_10_GPIO_Port GPIOB
#define Unused_Pin_11_Pin GPIO_PIN_13
#define Unused_Pin_11_GPIO_Port GPIOB
#define Unused_Pin_12_Pin GPIO_PIN_14
#define Unused_Pin_12_GPIO_Port GPIOB
#define Unused_Pin_13_Pin GPIO_PIN_15
#define Unused_Pin_13_GPIO_Port GPIOB
#define Unused_Pin_14_Pin GPIO_PIN_8
#define Unused_Pin_14_GPIO_Port GPIOA
#define Unused_Pin_15_Pin GPIO_PIN_11
#define Unused_Pin_15_GPIO_Port GPIOA
#define Unused_Pin_16_Pin GPIO_PIN_12
#define Unused_Pin_16_GPIO_Port GPIOA
#define Unused_Pin_17_Pin GPIO_PIN_15
#define Unused_Pin_17_GPIO_Port GPIOA
#define Unused_Pin_18_Pin GPIO_PIN_3
#define Unused_Pin_18_GPIO_Port GPIOB
#define Unused_Pin_19_Pin GPIO_PIN_4
#define Unused_Pin_19_GPIO_Port GPIOB
#define Unused_Pin_20_Pin GPIO_PIN_5
#define Unused_Pin_20_GPIO_Port GPIOB
#define Unused_Pin_21_Pin GPIO_PIN_6
#define Unused_Pin_21_GPIO_Port GPIOB
#define Unused_Pin_22_Pin GPIO_PIN_7
#define Unused_Pin_22_GPIO_Port GPIOB
#define Unused_Pin_23_Pin GPIO_PIN_8
#define Unused_Pin_23_GPIO_Port GPIOB
#define Unused_Pin_24_Pin GPIO_PIN_9
#define Unused_Pin_24_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
