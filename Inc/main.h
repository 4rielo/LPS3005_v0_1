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
#include "stm32f0xx_hal.h"

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
#define Bt_M3_Pin GPIO_PIN_13
#define Bt_M3_GPIO_Port GPIOC
#define Bt_M2_Pin GPIO_PIN_14
#define Bt_M2_GPIO_Port GPIOC
#define Bt_M1_Pin GPIO_PIN_15
#define Bt_M1_GPIO_Port GPIOC
#define Bt_Lock_Pin GPIO_PIN_0
#define Bt_Lock_GPIO_Port GPIOF
#define Temp_ADC_Pin GPIO_PIN_0
#define Temp_ADC_GPIO_Port GPIOA
#define V_Fb2_ADC_Pin GPIO_PIN_1
#define V_Fb2_ADC_GPIO_Port GPIOA
#define I_Set_Pin GPIO_PIN_4
#define I_Set_GPIO_Port GPIOA
#define V_Set_Pin GPIO_PIN_5
#define V_Set_GPIO_Port GPIOA
#define I_Fb_ADC_Pin GPIO_PIN_6
#define I_Fb_ADC_GPIO_Port GPIOA
#define I_Set_ADC_Pin GPIO_PIN_7
#define I_Set_ADC_GPIO_Port GPIOA
#define V_Set_ADC_Pin GPIO_PIN_0
#define V_Set_ADC_GPIO_Port GPIOB
#define V_Fb_ADC_Pin GPIO_PIN_1
#define V_Fb_ADC_GPIO_Port GPIOB
#define RelayOut_Pin GPIO_PIN_2
#define RelayOut_GPIO_Port GPIOB
#define Buzzer_Pin GPIO_PIN_11
#define Buzzer_GPIO_Port GPIOB
#define RelayB_Pin GPIO_PIN_12
#define RelayB_GPIO_Port GPIOB
#define RelayA_Pin GPIO_PIN_13
#define RelayA_GPIO_Port GPIOB
#define Fan_Pin GPIO_PIN_14
#define Fan_GPIO_Port GPIOB
#define Refresh_Pin GPIO_PIN_15
#define Refresh_GPIO_Port GPIOB
#define Rt_B_Pin GPIO_PIN_8
#define Rt_B_GPIO_Port GPIOA
#define Rt_A_Pin GPIO_PIN_9
#define Rt_A_GPIO_Port GPIOA
#define Rt_Sw_Pin GPIO_PIN_10
#define Rt_Sw_GPIO_Port GPIOA
#define Bt_CC_Pin GPIO_PIN_15
#define Bt_CC_GPIO_Port GPIOA
#define Bt_CC_EXTI_IRQn EXTI4_15_IRQn
#define DIM_Digits2_Pin GPIO_PIN_4
#define DIM_Digits2_GPIO_Port GPIOB
#define Bt_CV_Pin GPIO_PIN_6
#define Bt_CV_GPIO_Port GPIOB
#define Bt_CV_EXTI_IRQn EXTI4_15_IRQn
#define Bt_OutputOn_Pin GPIO_PIN_7
#define Bt_OutputOn_GPIO_Port GPIOB
#define Bt_OutputOn_EXTI_IRQn EXTI4_15_IRQn
#define DIM_LEDs_Pin GPIO_PIN_8
#define DIM_LEDs_GPIO_Port GPIOB
#define DIM_Digits_Pin GPIO_PIN_9
#define DIM_Digits_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
