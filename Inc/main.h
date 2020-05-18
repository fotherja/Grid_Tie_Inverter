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
#define V_Supply_Sense_Pin GPIO_PIN_2
#define V_Supply_Sense_GPIO_Port GPIOC
#define PhB_I_Sense_Pin GPIO_PIN_3
#define PhB_I_Sense_GPIO_Port GPIOC
#define PhA_I_Sense_Pin GPIO_PIN_1
#define PhA_I_Sense_GPIO_Port GPIOA
#define NSS_Pin GPIO_PIN_2
#define NSS_GPIO_Port GPIOA
#define Line_V_Sense_Pin GPIO_PIN_3
#define Line_V_Sense_GPIO_Port GPIOA
#define DAC_Out_Pin GPIO_PIN_4
#define DAC_Out_GPIO_Port GPIOA
#define DRV_Enable_Pin GPIO_PIN_1
#define DRV_Enable_GPIO_Port GPIOB
#define Ph_A_PWM_Pin GPIO_PIN_9
#define Ph_A_PWM_GPIO_Port GPIOE
#define Ph_B_PWM_Pin GPIO_PIN_11
#define Ph_B_PWM_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
