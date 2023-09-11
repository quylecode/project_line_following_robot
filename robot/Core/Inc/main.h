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
#include "stm32f1xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define START_Pin GPIO_PIN_13
#define START_GPIO_Port GPIOC
#define DT_Pin GPIO_PIN_14
#define DT_GPIO_Port GPIOC
#define SCK_Pin GPIO_PIN_15
#define SCK_GPIO_Port GPIOC
#define ENA2_Pin GPIO_PIN_6
#define ENA2_GPIO_Port GPIOA
#define ENB2_Pin GPIO_PIN_7
#define ENB2_GPIO_Port GPIOA
#define STOP_Pin GPIO_PIN_1
#define STOP_GPIO_Port GPIOB
#define INA1_Pin GPIO_PIN_12
#define INA1_GPIO_Port GPIOB
#define INA2_Pin GPIO_PIN_13
#define INA2_GPIO_Port GPIOB
#define INB1_Pin GPIO_PIN_14
#define INB1_GPIO_Port GPIOB
#define INB2_Pin GPIO_PIN_15
#define INB2_GPIO_Port GPIOB
#define PWMA_Pin GPIO_PIN_8
#define PWMA_GPIO_Port GPIOA
#define PWMB_Pin GPIO_PIN_9
#define PWMB_GPIO_Port GPIOA
#define STNDBY_Pin GPIO_PIN_10
#define STNDBY_GPIO_Port GPIOA
#define ENA1_Pin GPIO_PIN_6
#define ENA1_GPIO_Port GPIOB
#define ENB1_Pin GPIO_PIN_7
#define ENB1_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
