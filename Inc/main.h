/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define GPIN4_Pin GPIO_PIN_2
#define GPIN4_GPIO_Port GPIOE
#define GPIN5_Pin GPIO_PIN_3
#define GPIN5_GPIO_Port GPIOE
#define GPIN6_Pin GPIO_PIN_4
#define GPIN6_GPIO_Port GPIOE
#define GPIN7_Pin GPIO_PIN_5
#define GPIN7_GPIO_Port GPIOE
#define GPIN8_Pin GPIO_PIN_6
#define GPIN8_GPIO_Port GPIOE
#define LCD_BL_Pin GPIO_PIN_0
#define LCD_BL_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_14
#define LED1_GPIO_Port GPIOB
#define LED0_Pin GPIO_PIN_15
#define LED0_GPIO_Port GPIOB
#define GPOUT3_Pin GPIO_PIN_3
#define GPOUT3_GPIO_Port GPIOG
#define GPIN3_Pin GPIO_PIN_6
#define GPIN3_GPIO_Port GPIOG
#define GPIN2_Pin GPIO_PIN_7
#define GPIN2_GPIO_Port GPIOG
#define GPIN1_Pin GPIO_PIN_6
#define GPIN1_GPIO_Port GPIOC
#define GPOUT1_Pin GPIO_PIN_7
#define GPOUT1_GPIO_Port GPIOC
#define GPOUT2_Pin GPIO_PIN_8
#define GPOUT2_GPIO_Port GPIOC
#define GPOUT4_Pin GPIO_PIN_10
#define GPOUT4_GPIO_Port GPIOC
#define GPOUT5_Pin GPIO_PIN_11
#define GPOUT5_GPIO_Port GPIOC
#define GPOUT6_Pin GPIO_PIN_12
#define GPOUT6_GPIO_Port GPIOC
#define RS485RE_Pin GPIO_PIN_2
#define RS485RE_GPIO_Port GPIOD
#define ETH_RESET_Pin GPIO_PIN_6
#define ETH_RESET_GPIO_Port GPIOD
#define GPOUT7_Pin GPIO_PIN_12
#define GPOUT7_GPIO_Port GPIOG
#define GPOUT8_Pin GPIO_PIN_13
#define GPOUT8_GPIO_Port GPIOG
#define CD4052A_Pin GPIO_PIN_8
#define CD4052A_GPIO_Port GPIOB
#define CD4052B_Pin GPIO_PIN_9
#define CD4052B_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
