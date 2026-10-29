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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

#define BW_N 3 // butter worth filter order

#define CDC_RX_SIZE 32
#define CDC_TX_SIZE 24

/// Butter worth + Running average filter combination.
typedef struct {
	float eps_y[BW_N];
	float eps_x[BW_N + 1];
} FilterParams;

/// PI control parameters.
typedef struct {
	float unicorn; // cumulative error
	float eps_set_point; // command set rpm point
	uint32_t last_encoder;

	TIM_HandleTypeDef* enc_tim; // encoder read pin
	uint32_t pwm_channel;          // speed control pin

	uint16_t dir_pin; // direction control pin
	GPIO_TypeDef* dir_port;

	FilterParams filter;
} PIController;

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
#define MOT2_Dir_Pin GPIO_PIN_7
#define MOT2_Dir_GPIO_Port GPIOA
#define MOT1_Dir_Pin GPIO_PIN_0
#define MOT1_Dir_GPIO_Port GPIOB
#define MOT2_Speed_Pin GPIO_PIN_8
#define MOT2_Speed_GPIO_Port GPIOA
#define MOT1_Speed_Pin GPIO_PIN_9
#define MOT1_Speed_GPIO_Port GPIOA
#define MOT1_EncA_Pin GPIO_PIN_15
#define MOT1_EncA_GPIO_Port GPIOA
#define MOT1_EncB_Pin GPIO_PIN_3
#define MOT1_EncB_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
