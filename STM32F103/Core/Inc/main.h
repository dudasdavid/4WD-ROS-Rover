/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#define PC13_LED_Pin GPIO_PIN_13
#define PC13_LED_GPIO_Port GPIOC
#define TIM2_CH1_RL_Pin GPIO_PIN_0
#define TIM2_CH1_RL_GPIO_Port GPIOA
#define TIM2_CH2_RR_Pin GPIO_PIN_1
#define TIM2_CH2_RR_GPIO_Port GPIOA
#define TIM3_CH1_STEERING_Pin GPIO_PIN_6
#define TIM3_CH1_STEERING_GPIO_Port GPIOA
#define IMU_LRDY_Pin GPIO_PIN_1
#define IMU_LRDY_GPIO_Port GPIOB
#define TIM2_CH3_FL_Pin GPIO_PIN_10
#define TIM2_CH3_FL_GPIO_Port GPIOB
#define TIM2_CH4_FR_Pin GPIO_PIN_11
#define TIM2_CH4_FR_GPIO_Port GPIOB
#define IMU_LIN2_Pin GPIO_PIN_12
#define IMU_LIN2_GPIO_Port GPIOB
#define IMU_LIN1_Pin GPIO_PIN_13
#define IMU_LIN1_GPIO_Port GPIOB
#define IMU_GRDY_Pin GPIO_PIN_14
#define IMU_GRDY_GPIO_Port GPIOB
#define IMU_GINT_Pin GPIO_PIN_15
#define IMU_GINT_GPIO_Port GPIOB
#define TIM1_CH1_DC_PLUS_Pin GPIO_PIN_8
#define TIM1_CH1_DC_PLUS_GPIO_Port GPIOA
#define TIM1_CH2_DC_MINUS_Pin GPIO_PIN_9
#define TIM1_CH2_DC_MINUS_GPIO_Port GPIOA
#define ENA_SERVO_Pin GPIO_PIN_10
#define ENA_SERVO_GPIO_Port GPIOA
#define MOT_ENC_Pin GPIO_PIN_6
#define MOT_ENC_GPIO_Port GPIOB
#define MOT_ENC_EXTI_IRQn EXTI9_5_IRQn
#define IMU_SCL_Pin GPIO_PIN_8
#define IMU_SCL_GPIO_Port GPIOB
#define IMU_SDA_Pin GPIO_PIN_9
#define IMU_SDA_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
