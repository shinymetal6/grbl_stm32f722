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
#include "stm32f7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32_hal_port.h"

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
#define USER_Btn_Pin GPIO_PIN_13
#define USER_Btn_GPIO_Port GPIOC
#define USER_Btn_EXTI_IRQn EXTI15_10_IRQn
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOH
#define SPINDLE_PWM_BIT_Pin GPIO_PIN_6
#define SPINDLE_PWM_BIT_GPIO_Port GPIOA
#define SPINDLE_ENABLE_BIT_Pin GPIO_PIN_4
#define SPINDLE_ENABLE_BIT_GPIO_Port GPIOC
#define SPINDLE_DIRECTION_BIT_Pin GPIO_PIN_5
#define SPINDLE_DIRECTION_BIT_GPIO_Port GPIOC
#define LD1_Pin GPIO_PIN_0
#define LD1_GPIO_Port GPIOB
#define COOLANT_FLOOD_BIT_Pin GPIO_PIN_7
#define COOLANT_FLOOD_BIT_GPIO_Port GPIOE
#define COOLANT_MIST_BIT_Pin GPIO_PIN_8
#define COOLANT_MIST_BIT_GPIO_Port GPIOE
#define X_STEP_BIT_Pin GPIO_PIN_9
#define X_STEP_BIT_GPIO_Port GPIOE
#define Y_STEP_BIT_Pin GPIO_PIN_10
#define Y_STEP_BIT_GPIO_Port GPIOE
#define Z_STEP_BIT_Pin GPIO_PIN_11
#define Z_STEP_BIT_GPIO_Port GPIOE
#define A_STEP_BIT_Pin GPIO_PIN_12
#define A_STEP_BIT_GPIO_Port GPIOE
#define B_STEP_BIT_Pin GPIO_PIN_13
#define B_STEP_BIT_GPIO_Port GPIOE
#define C_STEP_BIT_Pin GPIO_PIN_14
#define C_STEP_BIT_GPIO_Port GPIOE
#define STEPPERS_DISABLE_BIT_Pin GPIO_PIN_15
#define STEPPERS_DISABLE_BIT_GPIO_Port GPIOE
#define CONTROL_RESET_BIT_Pin GPIO_PIN_10
#define CONTROL_RESET_BIT_GPIO_Port GPIOB
#define CONTROL_FEED_HOLD_BIT_Pin GPIO_PIN_11
#define CONTROL_FEED_HOLD_BIT_GPIO_Port GPIOB
#define CONTROL_CYCLE_START_BIT_Pin GPIO_PIN_12
#define CONTROL_CYCLE_START_BIT_GPIO_Port GPIOB
#define CONTROL_SAFETY_DOOR_BIT_Pin GPIO_PIN_13
#define CONTROL_SAFETY_DOOR_BIT_GPIO_Port GPIOB
#define LD3_Pin GPIO_PIN_14
#define LD3_GPIO_Port GPIOB
#define PROBE_BIT_Pin GPIO_PIN_15
#define PROBE_BIT_GPIO_Port GPIOB
#define STLK_RX_Pin GPIO_PIN_8
#define STLK_RX_GPIO_Port GPIOD
#define STLK_TX_Pin GPIO_PIN_9
#define STLK_TX_GPIO_Port GPIOD
#define X_DIRECTION_BIT_Pin GPIO_PIN_10
#define X_DIRECTION_BIT_GPIO_Port GPIOD
#define Y_DIRECTION_BIT_Pin GPIO_PIN_11
#define Y_DIRECTION_BIT_GPIO_Port GPIOD
#define Z_DIRECTION_BIT_Pin GPIO_PIN_12
#define Z_DIRECTION_BIT_GPIO_Port GPIOD
#define A_DIRECTION_BIT_Pin GPIO_PIN_13
#define A_DIRECTION_BIT_GPIO_Port GPIOD
#define B_DIRECTION_BIT_Pin GPIO_PIN_14
#define B_DIRECTION_BIT_GPIO_Port GPIOD
#define C_DIRECTION_BIT_Pin GPIO_PIN_15
#define C_DIRECTION_BIT_GPIO_Port GPIOD
#define USB_PowerSwitchOn_Pin GPIO_PIN_6
#define USB_PowerSwitchOn_GPIO_Port GPIOG
#define USB_OverCurrent_Pin GPIO_PIN_7
#define USB_OverCurrent_GPIO_Port GPIOG
#define USB_SOF_Pin GPIO_PIN_8
#define USB_SOF_GPIO_Port GPIOA
#define USB_VBUS_Pin GPIO_PIN_9
#define USB_VBUS_GPIO_Port GPIOA
#define USB_ID_Pin GPIO_PIN_10
#define USB_ID_GPIO_Port GPIOA
#define USB_DM_Pin GPIO_PIN_11
#define USB_DM_GPIO_Port GPIOA
#define USB_DP_Pin GPIO_PIN_12
#define USB_DP_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define X_LIMIT_BIT_Pin GPIO_PIN_0
#define X_LIMIT_BIT_GPIO_Port GPIOD
#define X_LIMIT_BIT_EXTI_IRQn EXTI0_IRQn
#define Y_LIMIT_BIT_Pin GPIO_PIN_1
#define Y_LIMIT_BIT_GPIO_Port GPIOD
#define Y_LIMIT_BIT_EXTI_IRQn EXTI1_IRQn
#define Z_LIMIT_BIT_Pin GPIO_PIN_2
#define Z_LIMIT_BIT_GPIO_Port GPIOD
#define Z_LIMIT_BIT_EXTI_IRQn EXTI2_IRQn
#define A_LIMIT_BIT_Pin GPIO_PIN_3
#define A_LIMIT_BIT_GPIO_Port GPIOD
#define A_LIMIT_BIT_EXTI_IRQn EXTI3_IRQn
#define B_LIMIT_BIT_Pin GPIO_PIN_4
#define B_LIMIT_BIT_GPIO_Port GPIOD
#define B_LIMIT_BIT_EXTI_IRQn EXTI4_IRQn
#define C_LIMIT_BIT_Pin GPIO_PIN_5
#define C_LIMIT_BIT_GPIO_Port GPIOD
#define C_LIMIT_BIT_EXTI_IRQn EXTI9_5_IRQn
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define LD2_Pin GPIO_PIN_7
#define LD2_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
extern	TIM_HandleTypeDef htim6;
extern	TIM_HandleTypeDef htim7;
extern	TIM_HandleTypeDef htim13;
extern	UART_HandleTypeDef huart3;

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
