/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#define LED5_Pin GPIO_PIN_0
#define LED5_GPIO_Port GPIOC
#define LED6_Pin GPIO_PIN_1
#define LED6_GPIO_Port GPIOC
#define LED9_Pin GPIO_PIN_2
#define LED9_GPIO_Port GPIOC
#define LED10_Pin GPIO_PIN_3
#define LED10_GPIO_Port GPIOC
#define SENSOR_LED_1_Pin GPIO_PIN_0
#define SENSOR_LED_1_GPIO_Port GPIOA
#define SENSOR_LED_2_Pin GPIO_PIN_1
#define SENSOR_LED_2_GPIO_Port GPIOA
#define ST_TX_Pin GPIO_PIN_2
#define ST_TX_GPIO_Port GPIOA
#define V_BATT_Pin GPIO_PIN_3
#define V_BATT_GPIO_Port GPIOA
#define LSM6DSR_CS_Pin GPIO_PIN_4
#define LSM6DSR_CS_GPIO_Port GPIOA
#define GY_SCLK_Pin GPIO_PIN_5
#define GY_SCLK_GPIO_Port GPIOA
#define GY_SDO_Pin GPIO_PIN_6
#define GY_SDO_GPIO_Port GPIOA
#define GY_SD1_Pin GPIO_PIN_7
#define GY_SD1_GPIO_Port GPIOA
#define SENSOR_4_Pin GPIO_PIN_4
#define SENSOR_4_GPIO_Port GPIOC
#define SENSOR_3_Pin GPIO_PIN_5
#define SENSOR_3_GPIO_Port GPIOC
#define SENSOR_2_Pin GPIO_PIN_0
#define SENSOR_2_GPIO_Port GPIOB
#define SENSOR_1_Pin GPIO_PIN_1
#define SENSOR_1_GPIO_Port GPIOB
#define SPEAKER_PWM_Pin GPIO_PIN_2
#define SPEAKER_PWM_GPIO_Port GPIOB
#define TOF_SCL_Pin GPIO_PIN_10
#define TOF_SCL_GPIO_Port GPIOB
#define GPIO1_TOF_Pin GPIO_PIN_12
#define GPIO1_TOF_GPIO_Port GPIOB
#define LED8_Pin GPIO_PIN_14
#define LED8_GPIO_Port GPIOB
#define LED7_Pin GPIO_PIN_15
#define LED7_GPIO_Port GPIOB
#define FUN_PWM_Pin GPIO_PIN_6
#define FUN_PWM_GPIO_Port GPIOC
#define MOTOR_ENABLE_R_Pin GPIO_PIN_8
#define MOTOR_ENABLE_R_GPIO_Port GPIOC
#define MOTOR_ENABLE_L_Pin GPIO_PIN_9
#define MOTOR_ENABLE_L_GPIO_Port GPIOC
#define STBY_Pin GPIO_PIN_12
#define STBY_GPIO_Port GPIOA
#define CSN_EN2_Pin GPIO_PIN_15
#define CSN_EN2_GPIO_Port GPIOA
#define EN_CLK_Pin GPIO_PIN_10
#define EN_CLK_GPIO_Port GPIOC
#define EN_MISO_Pin GPIO_PIN_11
#define EN_MISO_GPIO_Port GPIOC
#define EN_MOSI_Pin GPIO_PIN_12
#define EN_MOSI_GPIO_Port GPIOC
#define CSN_EN1_Pin GPIO_PIN_2
#define CSN_EN1_GPIO_Port GPIOD
#define TOF_SDA_Pin GPIO_PIN_3
#define TOF_SDA_GPIO_Port GPIOB
#define TOF_XSHOUT_Pin GPIO_PIN_4
#define TOF_XSHOUT_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_5
#define LED1_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_6
#define LED2_GPIO_Port GPIOB
#define LED3_Pin GPIO_PIN_7
#define LED3_GPIO_Port GPIOB
#define LED4_Pin GPIO_PIN_8
#define LED4_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
