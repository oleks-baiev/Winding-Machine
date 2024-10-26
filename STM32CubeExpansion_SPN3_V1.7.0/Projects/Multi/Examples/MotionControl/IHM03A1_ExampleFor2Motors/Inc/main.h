/**
  ******************************************************************************
  * @file    Multi/Examples/MotionControl/IHM03A1_ExampleFor2Motors/Inc/main.h 
  * @author  IPC Rennes
  * @version V1.6.0
  * @date    June 4th, 2018
  * @brief   Header for main.c module
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "x_nucleo_ihmxx.h"
#include "powerstep01.h"
#ifdef USE_STM32F4XX_NUCLEO
#include "x_nucleo_ihm03a1_stm32f4xx.h"
#endif
#ifdef USE_STM32F3XX_NUCLEO
#include "x_nucleo_ihm03a1_stm32f3xx.h"
#endif
#ifdef USE_STM32F0XX_NUCLEO
#include "x_nucleo_ihm03a1_stm32f0xx.h"
#endif
#ifdef USE_STM32L0XX_NUCLEO
#include "x_nucleo_ihm03a1_stm32l0xx.h"
#endif
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void SystemClock_Config(void);
void MyErrorHandler(uint16_t error);

#define ILI9341_CS_Pin GPIO_PIN_0
#define ILI9341_CS_GPIO_Port GPIOA
#define ILI9341_DC_Pin GPIO_PIN_0
#define ILI9341_DC_GPIO_Port GPIOB
#define ILI9341_RES_Pin GPIO_PIN_1
#define ILI9341_RES_GPIO_Port GPIOB
#define ILI9341_LED_Pin GPIO_PIN_8
#define ILI9341_LED_GPIO_Port GPIOB

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
