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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BATT_MEASURE_Pin GPIO_PIN_3
#define BATT_MEASURE_GPIO_Port GPIOC
#define ARM1_Pin GPIO_PIN_1
#define ARM1_GPIO_Port GPIOA
#define PYRO1_Pin GPIO_PIN_1
#define PYRO1_GPIO_Port GPIOB
#define CONT1_Pin GPIO_PIN_2
#define CONT1_GPIO_Port GPIOB
#define PYRO2_Pin GPIO_PIN_11
#define PYRO2_GPIO_Port GPIOF
#define CONT2_Pin GPIO_PIN_12
#define CONT2_GPIO_Port GPIOF
#define PYRO3_Pin GPIO_PIN_13
#define PYRO3_GPIO_Port GPIOF
#define CONT3_Pin GPIO_PIN_14
#define CONT3_GPIO_Port GPIOF
#define PYRO4_Pin GPIO_PIN_15
#define PYRO4_GPIO_Port GPIOF
#define CONT4_Pin GPIO_PIN_0
#define CONT4_GPIO_Port GPIOG
#define PYRO5_Pin GPIO_PIN_1
#define PYRO5_GPIO_Port GPIOG
#define CONT5_Pin GPIO_PIN_7
#define CONT5_GPIO_Port GPIOE
#define PYRO6_Pin GPIO_PIN_8
#define PYRO6_GPIO_Port GPIOE
#define CONT6_Pin GPIO_PIN_9
#define CONT6_GPIO_Port GPIOE
#define PYRO7_Pin GPIO_PIN_10
#define PYRO7_GPIO_Port GPIOE
#define CONT7_Pin GPIO_PIN_11
#define CONT7_GPIO_Port GPIOE
#define PYRO8_Pin GPIO_PIN_12
#define PYRO8_GPIO_Port GPIOE
#define CONT8_Pin GPIO_PIN_13
#define CONT8_GPIO_Port GPIOE
#define Servo_ARM_CHECK_Pin GPIO_PIN_4
#define Servo_ARM_CHECK_GPIO_Port GPIOG
#define NUM_LEDS_0 5
#define NUM_LEDS_1 5
#define NUM_LEDS_2 2
#define NUM_LEDS_3 2
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
