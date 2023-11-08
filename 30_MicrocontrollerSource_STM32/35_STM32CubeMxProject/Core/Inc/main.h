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
#include "stm32h7xx_hal.h"

#include "stm32h7xx_ll_tim.h"
#include "stm32h7xx_ll_bus.h"
#include "stm32h7xx_ll_cortex.h"
#include "stm32h7xx_ll_rcc.h"
#include "stm32h7xx_ll_system.h"
#include "stm32h7xx_ll_utils.h"
#include "stm32h7xx_ll_pwr.h"
#include "stm32h7xx_ll_gpio.h"
#include "stm32h7xx_ll_dma.h"

#include "stm32h7xx_ll_exti.h"

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
#define TIM15_CH1_MotPWM1_1_Pin GPIO_PIN_5
#define TIM15_CH1_MotPWM1_1_GPIO_Port GPIOE
#define TIM15_CH2_MotPWM1_2_Pin GPIO_PIN_6
#define TIM15_CH2_MotPWM1_2_GPIO_Port GPIOE
#define LD1_Pin GPIO_PIN_1
#define LD1_GPIO_Port GPIOC
#define LD2_Pin GPIO_PIN_2
#define LD2_GPIO_Port GPIOC
#define SenD1_Pin GPIO_PIN_12
#define SenD1_GPIO_Port GPIOE
#define SenD2_Pin GPIO_PIN_13
#define SenD2_GPIO_Port GPIOE
#define VL53_INT_Pin GPIO_PIN_14
#define VL53_INT_GPIO_Port GPIOE
#define VL53_INT_EXTI_IRQn EXTI15_10_IRQn
#define VL53_XSHUT_Pin GPIO_PIN_15
#define VL53_XSHUT_GPIO_Port GPIOE
#define I2C2_SCL_VL53_Pin GPIO_PIN_10
#define I2C2_SCL_VL53_GPIO_Port GPIOB
#define I2C2_SDA_VL53_Pin GPIO_PIN_11
#define I2C2_SDA_VL53_GPIO_Port GPIOB
#define IR_PIN_INPUT_Pin GPIO_PIN_12
#define IR_PIN_INPUT_GPIO_Port GPIOB
#define IR_PIN_INPUT_EXTI_IRQn EXTI15_10_IRQn
#define TIM12_CH1_MotPWM2_2_Pin GPIO_PIN_14
#define TIM12_CH1_MotPWM2_2_GPIO_Port GPIOB
#define TIM12_CH2_MotPWM2_1_Pin GPIO_PIN_15
#define TIM12_CH2_MotPWM2_1_GPIO_Port GPIOB
#define USER_BUTTON_Pin GPIO_PIN_12
#define USER_BUTTON_GPIO_Port GPIOA
#define LDD1_Pin GPIO_PIN_15
#define LDD1_GPIO_Port GPIOA
#define BLU_ABN_MEM_Pin GPIO_PIN_3
#define BLU_ABN_MEM_GPIO_Port GPIOD
#define BLU_RESET_Pin GPIO_PIN_4
#define BLU_RESET_GPIO_Port GPIOD
#define EEPROM_WC_Pin GPIO_PIN_0
#define EEPROM_WC_GPIO_Port GPIOE
#define MPU6050_INT_Pin GPIO_PIN_1
#define MPU6050_INT_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
