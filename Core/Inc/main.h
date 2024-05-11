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
#include "stm32f7xx_hal.h"

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
#define SWITCH_2_Pin GPIO_PIN_0
#define SWITCH_2_GPIO_Port GPIOC
#define SWITCH_3_Pin GPIO_PIN_1
#define SWITCH_3_GPIO_Port GPIOC
#define TIM2_CH1_ENCODER_2A_Pin GPIO_PIN_0
#define TIM2_CH1_ENCODER_2A_GPIO_Port GPIOA
#define TIM2_CH2_ENCODER_2B_Pin GPIO_PIN_1
#define TIM2_CH2_ENCODER_2B_GPIO_Port GPIOA
#define TIM5_CH3_MOTOR5_PWM_Pin GPIO_PIN_2
#define TIM5_CH3_MOTOR5_PWM_GPIO_Port GPIOA
#define TIM5_CH4_MOTOR6_PWM_Pin GPIO_PIN_3
#define TIM5_CH4_MOTOR6_PWM_GPIO_Port GPIOA
#define SWITCH_4_Pin GPIO_PIN_4
#define SWITCH_4_GPIO_Port GPIOA
#define TIM3_CH1_MOTOR4_PWM_Pin GPIO_PIN_6
#define TIM3_CH1_MOTOR4_PWM_GPIO_Port GPIOA
#define TIM3_CH2_MOTOR_PWM_Pin GPIO_PIN_7
#define TIM3_CH2_MOTOR_PWM_GPIO_Port GPIOA
#define MOTOR_2B_Pin GPIO_PIN_11
#define MOTOR_2B_GPIO_Port GPIOB
#define MOTOR_2A_Pin GPIO_PIN_12
#define MOTOR_2A_GPIO_Port GPIOB
#define TIM12_CH1_MOTOR7_PWM_Pin GPIO_PIN_14
#define TIM12_CH1_MOTOR7_PWM_GPIO_Port GPIOB
#define TIM12_CH2_MOTOR8_PWM_Pin GPIO_PIN_15
#define TIM12_CH2_MOTOR8_PWM_GPIO_Port GPIOB
#define TIM4_CH1_ENCODER_4A_Pin GPIO_PIN_12
#define TIM4_CH1_ENCODER_4A_GPIO_Port GPIOD
#define TIM4_CH2_ENCODER_4B_Pin GPIO_PIN_13
#define TIM4_CH2_ENCODER_4B_GPIO_Port GPIOD
#define IR_Pin GPIO_PIN_2
#define IR_GPIO_Port GPIOG
#define TIM8_CH1_ENCODER_3A_Pin GPIO_PIN_6
#define TIM8_CH1_ENCODER_3A_GPIO_Port GPIOC
#define TIM8_CH2_ENCODER_3B_Pin GPIO_PIN_7
#define TIM8_CH2_ENCODER_3B_GPIO_Port GPIOC
#define TIM3_CH3_MOTOR2_PWM_Pin GPIO_PIN_8
#define TIM3_CH3_MOTOR2_PWM_GPIO_Port GPIOC
#define TIM3_CH4_MOTOR1_PWM_Pin GPIO_PIN_9
#define TIM3_CH4_MOTOR1_PWM_GPIO_Port GPIOC
#define TIM1_CH1_ENCODER_1A_Pin GPIO_PIN_8
#define TIM1_CH1_ENCODER_1A_GPIO_Port GPIOA
#define TIM1_CH2_ENCODER_1B_Pin GPIO_PIN_9
#define TIM1_CH2_ENCODER_1B_GPIO_Port GPIOA
#define SPI1_CS_Pin GPIO_PIN_10
#define SPI1_CS_GPIO_Port GPIOA
#define MOTOR_1_B_Pin GPIO_PIN_11
#define MOTOR_1_B_GPIO_Port GPIOA
#define MOTOR_1_A_Pin GPIO_PIN_12
#define MOTOR_1_A_GPIO_Port GPIOA
#define S2_Pin GPIO_PIN_15
#define S2_GPIO_Port GPIOA
#define SWITCH_6_Pin GPIO_PIN_11
#define SWITCH_6_GPIO_Port GPIOC
#define SWITCH_5_Pin GPIO_PIN_2
#define SWITCH_5_GPIO_Port GPIOD
#define SWITCH_1_Pin GPIO_PIN_3
#define SWITCH_1_GPIO_Port GPIOD
#define S3_Pin GPIO_PIN_4
#define S3_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
