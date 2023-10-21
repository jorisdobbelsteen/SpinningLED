/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32f3xx_hal.h"

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
#define STATUS_LED_RED_CHANNEL TIM_CHANNEL_2
#define RotationCounterTimer htim15
#define StatusLedTimer htim3
#define MotorTimer htim17
#define MOTOR_CHANNEL TIM_CHANNEL_1
#define STATUS_LED_GREEN_CHANNEL TIM_CHANNEL_4
#define RotationCaptureTimer htim2
#define STATUS_LED_YELLOW_CHANNEL TIM_CHANNEL_3
#define ROTATION_CAPTURE_HALF_CHANNEL TIM_CHANNEL_2
#define ROTATION_CAPTURE_TYPE uint32_t
#define ROTATION_CAPTURE_FULL_CHANNEL TIM_CHANNEL_1
#define ButtonTimer htim6
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOF
#define VCP_TX_Pin GPIO_PIN_2
#define VCP_TX_GPIO_Port GPIOA
#define RED_LED_Pin GPIO_PIN_4
#define RED_LED_GPIO_Port GPIOA
#define STOP_BTN_Pin GPIO_PIN_5
#define STOP_BTN_GPIO_Port GPIOA
#define START_BTN_Pin GPIO_PIN_6
#define START_BTN_GPIO_Port GPIOA
#define YELLOW_LED_Pin GPIO_PIN_0
#define YELLOW_LED_GPIO_Port GPIOB
#define GREEN_LED_Pin GPIO_PIN_1
#define GREEN_LED_GPIO_Port GPIOB
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define VCP_RX_Pin GPIO_PIN_15
#define VCP_RX_GPIO_Port GPIOA
#define LD3_Pin GPIO_PIN_3
#define LD3_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
