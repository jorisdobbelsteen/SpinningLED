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
#include "stm32f4xx_hal.h"
#include "stm32f4xx_ll_dma.h"
#include "stm32f4xx_ll_spi.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_cortex.h"
#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_system.h"
#include "stm32f4xx_ll_utils.h"
#include "stm32f4xx_ll_pwr.h"
#include "stm32f4xx_ll_gpio.h"

#include "stm32f4xx_ll_exti.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
#define FOUR_CHANNEL 1
#define BRIGHTNESS_DEFAULT 0x80
#define BRIGHTNESS_NODETECT 0x08
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
#define hLEDS1_TXDMA_STREAM LL_DMA_STREAM_1
#define hLEDS4 SPI3
#define hLEDS3_TXDMA_STREAM LL_DMA_STREAM_4
#define hLEDS3 SPI2
#define hLEDS4_TXDMA_STREAM LL_DMA_STREAM_5
#define hLEDS2 SPI1
#define hLEDS1 SPI4
#define TIM_ERROR_CHANNEL_LED TIM_CHANNEL_4
#define hLEDS4_TXDMA DMA1
#define hLEDS2_TXDMA_STREAM LL_DMA_STREAM_2
#define hLEDS2_TXDMA DMA2
#define TIM_ROTATION_CHANNEL_LED TIM_CHANNEL_2
#define hTIM_PIXEL htim2
#define hLEDS3_TXDMA DMA1
#define hTIM_ERROR htim3
#define TIM_ROTATION_CHANNEL_DETECT TIM_CHANNEL_1
#define hLEDS1_TXDMA DMA2
#define hTIM_ROTATION htim1
#define ROTATION_LED_PULSE_MS 5
#define LEDS_X 400
#define LEDS_Y 57
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define MODEBTN_Pin GPIO_PIN_8
#define MODEBTN_GPIO_Port GPIOC
#define ERROR_LED_Pin GPIO_PIN_9
#define ERROR_LED_GPIO_Port GPIOC
#define ROTATION_DETECT_Pin GPIO_PIN_8
#define ROTATION_DETECT_GPIO_Port GPIOA
#define ROTATION_LED_Pin GPIO_PIN_9
#define ROTATION_LED_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define TXDMA(peripheral) peripheral##_TXDMA , peripheral##_TXDMA_STREAM
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
