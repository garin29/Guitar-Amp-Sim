/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32h7xx_hal.h"

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
#define BTN_1_Pin GPIO_PIN_4
#define BTN_1_GPIO_Port GPIOE
#define BTN_1_EXTI_IRQn EXTI4_IRQn
#define BTN_2_Pin GPIO_PIN_5
#define BTN_2_GPIO_Port GPIOE
#define BTN_2_EXTI_IRQn EXTI9_5_IRQn
#define BTN_3_Pin GPIO_PIN_0
#define BTN_3_GPIO_Port GPIOF
#define BTN_3_EXTI_IRQn EXTI0_IRQn
#define SPI2_CS_Pin GPIO_PIN_12
#define SPI2_CS_GPIO_Port GPIOB
#define CODEC_NRST_Pin GPIO_PIN_11
#define CODEC_NRST_GPIO_Port GPIOD
#define OUT_SIGN_Pin GPIO_PIN_6
#define OUT_SIGN_GPIO_Port GPIOG
#define YELLOW_LED_Pin GPIO_PIN_2
#define YELLOW_LED_GPIO_Port GPIOD
#define RED_LED_Pin GPIO_PIN_4
#define RED_LED_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
