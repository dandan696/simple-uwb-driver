/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32f1xx_hal.h"
 #include "stdio.h"  //??print ?????main.h?

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
#define DW_RESET_Pin GPIO_PIN_0
#define DW_RESET_GPIO_Port GPIOA
#define DW_NSS_Pin GPIO_PIN_4
#define DW_NSS_GPIO_Port GPIOA
#define DW_SCK_Pin GPIO_PIN_5
#define DW_SCK_GPIO_Port GPIOA
#define DW_MISO_Pin GPIO_PIN_6
#define DW_MISO_GPIO_Port GPIOA
#define DW_MOSI_Pin GPIO_PIN_7
#define DW_MOSI_GPIO_Port GPIOA
#define PA_PWR_EN_Pin GPIO_PIN_0
#define PA_PWR_EN_GPIO_Port GPIOB
#define SW8_Pin GPIO_PIN_1
#define SW8_GPIO_Port GPIOB
#define SW7_Pin GPIO_PIN_2
#define SW7_GPIO_Port GPIOB
#define SW6_Pin GPIO_PIN_10
#define SW6_GPIO_Port GPIOB
#define SW5_Pin GPIO_PIN_11
#define SW5_GPIO_Port GPIOB
#define SW4_Pin GPIO_PIN_12
#define SW4_GPIO_Port GPIOB
#define SW3_Pin GPIO_PIN_13
#define SW3_GPIO_Port GPIOB
#define SW2_Pin GPIO_PIN_14
#define SW2_GPIO_Port GPIOB
#define SW1_Pin GPIO_PIN_15
#define SW1_GPIO_Port GPIOB
#define EXPR_Pin GPIO_PIN_8
#define EXPR_GPIO_Port GPIOA
#define J_TMS_Pin GPIO_PIN_13
#define J_TMS_GPIO_Port GPIOA
#define J_TCK_Pin GPIO_PIN_14
#define J_TCK_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_15
#define LED1_GPIO_Port GPIOA
#define LED3_Pin GPIO_PIN_3
#define LED3_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_4
#define LED2_GPIO_Port GPIOB
#define DW_IRQn_Pin GPIO_PIN_5
#define DW_IRQn_GPIO_Port GPIOB
#define DW_IRQn_EXTI_IRQn EXTI9_5_IRQn
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
