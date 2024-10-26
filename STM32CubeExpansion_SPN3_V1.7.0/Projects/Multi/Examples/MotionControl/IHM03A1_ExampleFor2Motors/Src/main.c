/**
  ******************************************************************************
  * @file    Multi/Examples/MotionControl/IHM03A1_ExampleFor2Motors/Src/main.c 
  * @author  IPC Rennes
  * @version V1.6.0
  * @date    June 4th, 2018
  * @brief   This example shows how to use 2 Powerstep01 devices
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "ili9341.h"
#include "fonts.h"
#include "testimg.h"
#include <stdio.h>
#include <stdlib.h>
    
/** @addtogroup IHM03A1_Example_for_2_motor_devices
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
 SPI_HandleTypeDef  hspi3;
 DMA_HandleTypeDef hdma_spi3_tx;
 ADC_HandleTypeDef hadc1;
 TIM_HandleTypeDef htim2;

 static volatile uint16_t gLastError;
 volatile uint8_t txComplete = 1;
 volatile uint8_t newAdcValIsReady = 0;
 uint32_t adcVal;
 uint32_t adcValOld = 0;
 const uint32_t adcDelta = 3;

/* Private function prototypes -----------------------------------------------*/
static void MyBusyInterruptHandler(void);
static void MyFlagInterruptHandler(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI3_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
/* Private functions ---------------------------------------------------------*/



void init() {
	HAL_GPIO_WritePin(ILI9341_LED_GPIO_Port, ILI9341_LED_Pin, GPIO_PIN_SET);
    ILI9341_Unselect();
    ILI9341_Init();
}

void loop() {

//    // Check border
//    ILI9341_FillScreen(ILI9341_BLACK);
//
//    for(int x = 0; x < ILI9341_WIDTH; x++) {
//        ILI9341_DrawPixel(x, 0, ILI9341_RED);
//        ILI9341_DrawPixel(x, ILI9341_HEIGHT-1, ILI9341_RED);
//    }
//
//    for(int y = 0; y < ILI9341_HEIGHT; y++) {
//        ILI9341_DrawPixel(0, y, ILI9341_RED);
//        ILI9341_DrawPixel(ILI9341_WIDTH-1, y, ILI9341_RED);
//    }
//
//    HAL_Delay(1000);

    // Check font
//ILI9341_FillScreen(ILI9341_BLACK);
//ILI9341_WriteString(0, 0,"Font_7x10, HELLO", Font_7x10, ILI9341_RED,ILI9341_BLACK);
//ILI9341_WriteString(0,3*10,"Font_11x18,HELLO",Font_11x18,ILI9341_GREEN,ILI9341_BLACK);
//ILI9341_WriteString(0,3*20,"Font_16x26,HELLO",Font_16x26,ILI9341_BLUE,ILI9341_BLACK);

ILI9341_DrawImage(0,0,128,160,
(const uint16_t*)a91364c40cf394d1b08c28871b056b87);

int i = 0;
while(1)
{
	char str[5];
//	sprintf(str, "%5d", i);
//	ILI9341_WriteString(0, 0, str, Font_7x10, ILI9341_WHITE, ILI9341_BLACK);
//	HAL_Delay(1000);
//	i++;
	if(newAdcValIsReady)
		{
			newAdcValIsReady = 0;
			if(abs(adcVal - adcValOld) > adcDelta)
			{
//			sprintf(str, "%5d", adcVal);
//			ILI9341_WriteString(0, 12, str, Font_7x10, ILI9341_WHITE, ILI9341_BLACK);
				updateSpeed(adcVal);
				displaySpeed(adcVal);
			}
		}
}

    HAL_Delay(1000);
    ILI9341_InvertColors(TRUE);
    HAL_Delay(1000);
    ILI9341_InvertColors(FALSE);

    HAL_Delay(5000);

    // Check colors
    ILI9341_FillScreen(ILI9341_WHITE);
    ILI9341_WriteString(0, 0, "WHITE", Font_11x18, ILI9341_BLACK, ILI9341_WHITE);
    HAL_Delay(500);

    ILI9341_FillScreen(ILI9341_BLUE);
    ILI9341_WriteString(0, 0, "BLUE", Font_11x18, ILI9341_BLACK, ILI9341_BLUE);
    HAL_Delay(500);

    ILI9341_FillScreen(ILI9341_RED);
    ILI9341_WriteString(0, 0, "RED", Font_11x18, ILI9341_BLACK, ILI9341_RED);
    HAL_Delay(500);

    ILI9341_FillScreen(ILI9341_GREEN);
    ILI9341_WriteString(0, 0, "GREEN", Font_11x18, ILI9341_BLACK, ILI9341_GREEN);
    HAL_Delay(500);

    ILI9341_FillScreen(ILI9341_CYAN);
    ILI9341_WriteString(0, 0, "CYAN", Font_11x18, ILI9341_BLACK, ILI9341_CYAN);
    HAL_Delay(500);

    ILI9341_FillScreen(ILI9341_MAGENTA);
    ILI9341_WriteString(0, 0, "MAGENTA", Font_11x18, ILI9341_BLACK, ILI9341_MAGENTA);
    HAL_Delay(500);

    ILI9341_FillScreen(ILI9341_YELLOW);
    ILI9341_WriteString(0, 0, "YELLOW", Font_11x18, ILI9341_BLACK, ILI9341_YELLOW);
    HAL_Delay(500);

    ILI9341_FillScreen(ILI9341_BLACK);
    ILI9341_WriteString(0, 0, "BLACK", Font_11x18, ILI9341_WHITE, ILI9341_BLACK);
    HAL_Delay(500);


HAL_Delay(3000);

}



/**
  * @brief  Main program
  * @param  None
  * @retval None
  */

union powerstep01_Init_u initDeviceParameters =
{
  /* common parameters */
  .cm.cp.cmVmSelection = POWERSTEP01_CM_VM_CURRENT, // enum powerstep01_CmVm_t
  582, // Acceleration rate in step/s2, range 14.55 to 59590 steps/s^2
  582, // Deceleration rate in step/s2, range 14.55 to 59590 steps/s^2
  488, // Maximum speed in step/s, range 15.25 to 15610 steps/s
  0, // Minimum speed in step/s, range 0 to 976.3 steps/s
  POWERSTEP01_LSPD_OPT_OFF, // Low speed optimization bit, enum powerstep01_LspdOpt_t
  244.16, // Full step speed in step/s, range 7.63 to 15625 steps/s
  POWERSTEP01_BOOST_MODE_OFF, // Boost of the amplitude square wave, enum powerstep01_BoostMode_t
  281.25, // Overcurrent threshold settings via enum powerstep01_OcdTh_t
  STEP_MODE_1_16, // Step mode settings via enum motorStepMode_t
  POWERSTEP01_SYNC_SEL_DISABLED, // Synch. Mode settings via enum powerstep01_SyncSel_t
  (POWERSTEP01_ALARM_EN_OVERCURRENT|
   POWERSTEP01_ALARM_EN_THERMAL_SHUTDOWN|
   POWERSTEP01_ALARM_EN_THERMAL_WARNING|
   POWERSTEP01_ALARM_EN_UVLO|
   POWERSTEP01_ALARM_EN_STALL_DETECTION|
   POWERSTEP01_ALARM_EN_SW_TURN_ON|
   POWERSTEP01_ALARM_EN_WRONG_NPERF_CMD), // Alarm settings via bitmap enum powerstep01_AlarmEn_t
  POWERSTEP01_IGATE_64mA, // Gate sink/source current via enum powerstep01_Igate_t
  POWERSTEP01_TBOOST_0ns, // Duration of the overboost phase during gate turn-off via enum powerstep01_Tboost_t
  POWERSTEP01_TCC_500ns, // Controlled current time via enum powerstep01_Tcc_t
  POWERSTEP01_WD_EN_DISABLE, // External clock watchdog, enum powerstep01_WdEn_t
  POWERSTEP01_TBLANK_375ns, // Duration of the blanking time via enum powerstep01_TBlank_t
  POWERSTEP01_TDT_125ns, // Duration of the dead time via enum powerstep01_Tdt_t
  /* current mode parameters */
  32.812 * 4, // Hold torque in mV, range from 7.8mV to 1000 mV
  32.812 * 4, // Running torque in mV, range from 7.8mV to 1000 mV
  32.812 * 4, // Acceleration torque in mV, range from 7.8mV to 1000 mV
  32.812 * 4, // Deceleration torque in mV, range from 7.8mV to 1000 mV
  POWERSTEP01_TOFF_FAST_8us, //Maximum fast decay time , enum powerstep01_ToffFast_t
  POWERSTEP01_FAST_STEP_12us, //Maximum fall step time , enum powerstep01_FastStep_t
  3.0, // Minimum on-time in us, range 0.5us to 64us
  21.0, // Minimum off-time in us, range 0.5us to 64us
  POWERSTEP01_CONFIG_INT_16MHZ_OSCOUT_2MHZ, // Clock setting , enum powerstep01_ConfigOscMgmt_t
  POWERSTEP01_CONFIG_SW_HARD_STOP, // External switch hard stop interrupt mode, enum powerstep01_ConfigSwMode_t
  POWERSTEP01_CONFIG_TQ_REG_TVAL_USED, // External torque regulation enabling , enum powerstep01_ConfigEnTqReg_t
  POWERSTEP01_CONFIG_VS_COMP_DISABLE, // Motor Supply Voltage Compensation enabling , enum powerstep01_ConfigEnVscomp_t
  POWERSTEP01_CONFIG_OC_SD_DISABLE, // Over current shutwdown enabling, enum powerstep01_ConfigOcSd_t
  POWERSTEP01_CONFIG_UVLOVAL_LOW, // UVLO Threshold via powerstep01_ConfigUvLoVal_t
  POWERSTEP01_CONFIG_VCCVAL_15V, // VCC Val, enum powerstep01_ConfigVccVal_t
  POWERSTEP01_CONFIG_TSW_048us, // Switching period, enum powerstep01_ConfigTsw_t
  POWERSTEP01_CONFIG_PRED_DISABLE, // Predictive current enabling , enum powerstep01_ConfigPredEn_t
};

int main(void)
{
  int32_t pos;
  uint32_t myMaxSpeed;
  uint32_t myMinSpeed;
  uint16_t myAcceleration;
  uint16_t myDeceleration;
  uint32_t readData;

  /* STM32F4xx HAL library initialization */
  HAL_Init();
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI3_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  //HAL_SPI_MspInit(&hspi3);
  

  /* Configure the system clock */
  SystemClock_Config();




  init();
  HAL_TIM_Base_Start(&htim2);
  HAL_ADC_Start_IT(&hadc1);






    
//----- Init of the Powerstep01 library 
  /* Set the Powerstep01 library to use 2 devices */
  BSP_MotorControl_SetNbDevices(BSP_MOTOR_CONTROL_BOARD_ID_POWERSTEP01, 2);
  /* When BSP_MotorControl_Init is called with NULL pointer,                  */
  /* the Powerstep01 registers are set with the predefined values from file   */
  /* powerstep01_target_config.h, otherwise the registers are set using the   */
  /* powerstep01_Init_u relevant union of structure values.                   */
  /* The first call to BSP_MotorControl_Init initializes the first device     */
  /* whose Id is 0.                                                           */
  /* The nth call to BSP_MotorControl_Init initializes the nth device         */
  /* whose Id is n-1.                                                         */
  BSP_MotorControl_Init(BSP_MOTOR_CONTROL_BOARD_ID_POWERSTEP01, NULL);
  BSP_MotorControl_Init(BSP_MOTOR_CONTROL_BOARD_ID_POWERSTEP01, &initDeviceParameters);
  
  /* Attach the function MyFlagInterruptHandler (defined below) to the flag interrupt */
  BSP_MotorControl_AttachFlagInterrupt(MyFlagInterruptHandler);

  /* Attach the function MyBusyInterruptHandler (defined below) to the busy interrupt */
  BSP_MotorControl_AttachBusyInterrupt(MyBusyInterruptHandler);
  
  /* Attach the function Error_Handler (defined below) to the error Handler*/
  BSP_MotorControl_AttachErrorHandler(MyErrorHandler);

  /* Set Current position to be the home position for device 0 */
  BSP_MotorControl_CmdResetPos(0);

  /* Set Current position to be the home position for device 1 */
  BSP_MotorControl_CmdResetPos(1);

  while(1)
	{
		loop();
	}

  //Powerstep01_SelectStepMode()
  while(1)
  {
  	BSP_MotorControl_CmdRun(1, BACKWARD, 976.3 / 2);
  	BSP_MotorControl_CmdRun(1, BACKWARD, 976.3);
  	BSP_MotorControl_CmdRun(1, BACKWARD, 976.3 * 2);
  	BSP_MotorControl_CmdRun(1, BACKWARD, 976.3 * 4);
  	BSP_MotorControl_CmdRun(1, BACKWARD, 976.3 * 8);
  	BSP_MotorControl_CmdRun(1, BACKWARD, 976.3 * 160);
  	//HAL_Delay(1000);
  }

  /* Request device 0 to Goto position 3200 */
  BSP_MotorControl_GoTo(0,3200);  

  /* Wait for device 0 ends moving */  
  BSP_MotorControl_WaitWhileActive(0);

  /* Get current position of device 0*/
  pos = BSP_MotorControl_GetPosition(0);

  /* If the read position of device 0 is 3200 */
  /* Request device 1 to go to the same position */
  if (pos == 3200)
  {
    BSP_MotorControl_GoTo(1,pos); 

    /* Wait for  device 1 ends moving */  
    BSP_MotorControl_WaitWhileActive(1);
  }
  
  /* Wait for 2 seconds */
  HAL_Delay(2000);  

  /* Get current position of device 1*/
  pos = BSP_MotorControl_GetPosition(1);
  
  /* Set current position of device 1 to be its mark position*/
  BSP_MotorControl_SetMark(1, pos); 
  
 /* Request device 1 to Goto position -3200 */
  BSP_MotorControl_GoTo(1,-3200);  

  /* Wait for device 1 ends moving */  
  BSP_MotorControl_WaitWhileActive(1);

  /* Get current position of device 1*/
  pos = BSP_MotorControl_GetPosition(1);
  
  /* If the read position of device 1 is -3200 */
  /* Request device 1 to go to the same position */
  if (pos == -3200)
  {
    BSP_MotorControl_GoTo(0,pos); 

    /* Wait for  device 1 ends moving */  
    BSP_MotorControl_WaitWhileActive(0);
  }
  
  /* Wait for 2 seconds */
  HAL_Delay(2000);  

  /* Get current position of device 0*/
  pos = BSP_MotorControl_GetPosition(0);
  
  /* Set current position of device 0 to be its mark position*/
  BSP_MotorControl_SetMark(0, pos); 
  
  /* Request both devices to go home */
  BSP_MotorControl_QueueCommands(0,POWERSTEP01_GO_HOME,0);
  BSP_MotorControl_QueueCommands(1,POWERSTEP01_GO_HOME,0);
  BSP_MotorControl_SendQueuedCommands();

  /* Wait for both devices end moving */
  BSP_MotorControl_WaitForAllDevicesNotBusy();

  /* Wait for 2 seconds */
  HAL_Delay(2000);  
  
  /* Request both devices to go to their mark position */
  BSP_MotorControl_QueueCommands(0,POWERSTEP01_GO_MARK,0);
  BSP_MotorControl_QueueCommands(1,POWERSTEP01_GO_MARK,0);
  BSP_MotorControl_SendQueuedCommands();

  /* Wait for both devices end moving */  
  BSP_MotorControl_WaitForAllDevicesNotBusy();

  /* Wait for 2 seconds */
  HAL_Delay(2000);  

  /* Request device 0 to run in FORWARD direction at 400 steps/s*/
  BSP_MotorControl_CmdRun(0, FORWARD, 0x068DB);

  /* Wait for device 0 reaches its max speed */
  do
  {
    readData = BSP_MotorControl_CmdGetParam(0, POWERSTEP01_SPEED);
  }while (readData != 0x068DB);

  /* Request device 1 to run in backward direction at 200 steps/s*/
  BSP_MotorControl_CmdRun(1, FORWARD, 0x0346D);

  /* Wait for device 1 reaches its max speed */
    do
  {
    readData = BSP_MotorControl_CmdGetParam(1, POWERSTEP01_SPEED);
  }while (readData != 0x0346D);

  /* Wait for 5 seconds */
  HAL_Delay(5000);

  /* Request device 0 to make a soft stop */
  BSP_MotorControl_CmdSoftStop(0);

  /* Wait for device 0 ends moving */  
  BSP_MotorControl_WaitWhileActive(0);

  /* Request device 1 to make a hard stop */
  BSP_MotorControl_HardStop(1);

 /* Wait for device 1 ends moving */  
  BSP_MotorControl_WaitWhileActive(1);
  
  /* Wait for 2 seconds */
  HAL_Delay(2000);  

  /* Request both devices to go home */
  BSP_MotorControl_QueueCommands(0,POWERSTEP01_GO_HOME,0);
  BSP_MotorControl_QueueCommands(1,POWERSTEP01_GO_HOME,0);
  BSP_MotorControl_SendQueuedCommands();
  
  /* Wait for both devices end moving */  
  BSP_MotorControl_WaitForAllDevicesNotBusy();

  /* Wait for 2 seconds */
  HAL_Delay(2000);  

  /* Get acceleration, deceleration and MinSpeed of device 0*/
  myMaxSpeed= BSP_MotorControl_CmdGetParam(0, POWERSTEP01_MAX_SPEED);
  myAcceleration = BSP_MotorControl_CmdGetParam(0, POWERSTEP01_ACC);
  myDeceleration = BSP_MotorControl_CmdGetParam(0, POWERSTEP01_DEC);
  myMinSpeed  = BSP_MotorControl_CmdGetParam(0, POWERSTEP01_MIN_SPEED); 
  
  /* Select 1/16 microstepping mode for device 0 */
  BSP_MotorControl_SelectStepMode(0, STEP_MODE_1_16);
  
  /* Select full step mode for device 1 */
  BSP_MotorControl_SelectStepMode(1, STEP_MODE_FULL);
  
  /* Set speed and acceleration of device 1 */
  /* Do not scale with microstepping */
  BSP_MotorControl_CmdSetParam(1, POWERSTEP01_ACC, myAcceleration);
  BSP_MotorControl_CmdSetParam(1, POWERSTEP01_DEC, myDeceleration);
  BSP_MotorControl_CmdSetParam(1, POWERSTEP01_MIN_SPEED, myMinSpeed);
  BSP_MotorControl_CmdSetParam(1, POWERSTEP01_MAX_SPEED, myMaxSpeed);
  
  /* Queue request for device 0 to go to position 6400 */
  BSP_MotorControl_QueueCommands(0,POWERSTEP01_GO_TO,6400);  

  /* Queue request for device 1 to go to the equivalent reverse position */
  /* in full step mode -6400 /16 = -400 */
  BSP_MotorControl_QueueCommands(1,POWERSTEP01_GO_TO,-400);
  
  /* Send the Go To requests */
  BSP_MotorControl_SendQueuedCommands();
  
  /* Wait for both devices end moving */  
  BSP_MotorControl_WaitForAllDevicesNotBusy();

  /* Wait for 2 seconds */
  HAL_Delay(2000);

  /* Get current position of device 0*/
  pos = BSP_MotorControl_GetPosition(0);
  
  /* Set current position of device 0 to be its mark position*/
  BSP_MotorControl_SetMark(0, pos); 
  
  /* Get current position of device 1*/
  pos = BSP_MotorControl_GetPosition(1);
  
  /* Set current position of device 1 to be its mark position*/
  BSP_MotorControl_SetMark(1, pos); 
  
  /* Halve speed and acceleration of device 0 */
  BSP_MotorControl_CmdSetParam(1, POWERSTEP01_ACC, myAcceleration>>1);
  BSP_MotorControl_CmdSetParam(1, POWERSTEP01_DEC, myDeceleration>>1);
  BSP_MotorControl_CmdSetParam(1, POWERSTEP01_MIN_SPEED, myMinSpeed>>1);
  BSP_MotorControl_CmdSetParam(1, POWERSTEP01_MAX_SPEED, myMaxSpeed>>1);

  /* Wait for 5 seconds before entering the loop */
  HAL_Delay(5000);
 
  /* Infinite loop */
  while(1)
  {
    /* Device 0 is using  1/16 microstepping mode */
    /* Device 1 is using full step mode  */
    /* Request both devices to go to their mark position */
    BSP_MotorControl_QueueCommands(0,POWERSTEP01_GO_MARK,0);
    BSP_MotorControl_QueueCommands(1,POWERSTEP01_GO_MARK,0);
    BSP_MotorControl_SendQueuedCommands();

    /* Wait for both devices end moving */
    BSP_MotorControl_WaitForAllDevicesNotBusy();

    /* Request both devices to go home */
    BSP_MotorControl_QueueCommands(0,POWERSTEP01_GO_HOME,0);
    BSP_MotorControl_QueueCommands(1,POWERSTEP01_GO_HOME,0);
    BSP_MotorControl_SendQueuedCommands();

    /* Wait for both devices end moving */
    BSP_MotorControl_WaitForAllDevicesNotBusy();
  }

}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ILI9341_CS_GPIO_Port, ILI9341_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, ILI9341_DC_Pin|ILI9341_RES_Pin|ILI9341_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : ILI9341_CS_Pin */
  GPIO_InitStruct.Pin = ILI9341_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(ILI9341_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ILI9341_DC_Pin ILI9341_RES_Pin ILI9341_LED_Pin */
  GPIO_InitStruct.Pin = ILI9341_DC_Pin|ILI9341_RES_Pin|ILI9341_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

}

static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_10B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 2;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 420000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

void displaySpeed(uint32_t newSpeed) //put 10bit adc value
{
	const uint32_t minSpeed = 0;
	const uint32_t maxSpeed = 1023;
	const uint32_t minPedalValue = 270;
	const uint32_t maxPedalValue = 800;
	if(newSpeed < 270)
	{
		newSpeed = 270;
	}
	newSpeed -= minPedalValue;
	uint32_t pos = (128 * newSpeed) / (maxPedalValue - minPedalValue);

	ILI9341_FillRectangle(0, 1, pos, 5, ILI9341_RED);
	ILI9341_FillRectangle(pos, 1, ILI9341_WIDTH, 5, ILI9341_BLACK);
}

void updateSpeed(uint32_t newSpeed)
{
	const uint32_t minSpeed = 0;
		const uint32_t maxSpeed = 1023;
		const uint32_t minPedalValue = 270;
		const uint32_t maxPedalValue = 800;
		if(newSpeed < 270)
		{
			newSpeed = 270;
		}
		newSpeed -= minPedalValue;
		if(newSpeed < 30)
		{
			BSP_MotorControl_HardStop(1);
		}
		else
		{
			BSP_MotorControl_CmdRun(1, BACKWARD, newSpeed * 100);
		}

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}


/**
  * @brief  This function is the User handler for the flag interrupt
  * @param  None
  * @retval None
  */
void MyFlagInterruptHandler(void)
{
  /* Get the value of the status register via the command GET_STATUS */
  uint16_t statusRegister = BSP_MotorControl_CmdGetStatus(0);

  /* Check HIZ flag: if set, power brigdes are disabled */
  if ((statusRegister & POWERSTEP01_STATUS_HIZ) == POWERSTEP01_STATUS_HIZ)
  {
    // HIZ state
  }

  /* Check BUSY flag: if not set, a command is under execution */
  if ((statusRegister & POWERSTEP01_STATUS_BUSY) == 0)
  {
    // BUSY
  }

  /* Check SW_F flag: if not set, the SW input is opened */
  if ((statusRegister & POWERSTEP01_STATUS_SW_F ) == 0)
  {
     // SW OPEN
  }
  else
  {
    // SW CLOSED   
  }  
  /* Check SW_EN bit */
  if ((statusRegister & POWERSTEP01_STATUS_SW_EVN) == POWERSTEP01_STATUS_SW_EVN)
  {
    // switch turn_on event
  }  
  /* Check direction bit */
  if ((statusRegister & POWERSTEP01_STATUS_DIR) == 0)
  {
    // BACKWARD
  }
  else  
  {
    // FORWARD 
  }
  if ((statusRegister & POWERSTEP01_STATUS_MOT_STATUS) == POWERSTEP01_STATUS_MOT_STATUS_STOPPED )
  {
       // MOTOR STOPPED
  }
  else  if ((statusRegister & POWERSTEP01_STATUS_MOT_STATUS) == POWERSTEP01_STATUS_MOT_STATUS_ACCELERATION )
  {
           // MOTOR ACCELERATION
  }  
  else  if ((statusRegister & POWERSTEP01_STATUS_MOT_STATUS) == POWERSTEP01_STATUS_MOT_STATUS_DECELERATION )
  {
           // MOTOR DECELERATION
  }
  else  if ((statusRegister & POWERSTEP01_STATUS_MOT_STATUS) == POWERSTEP01_STATUS_MOT_STATUS_CONST_SPD )
  {
       // MOTOR RUNNING AT CONSTANT SPEED
  }  

  /* Check Command Error flag: if set, the command received by SPI can't be performed */
  /* This often occures when a command is sent to the Powerstep01 */
  /* while it is in HIZ state or it the sent command does not exist*/
  if ((statusRegister & POWERSTEP01_STATUS_CMD_ERROR) == POWERSTEP01_STATUS_CMD_ERROR)
  {
       // Command Error
  }  

  /* Check Step mode clock flag: if set, the device is working in step clock mode */
  if ((statusRegister & POWERSTEP01_STATUS_STCK_MOD) == POWERSTEP01_STATUS_STCK_MOD)
  {
     //Step clock mode enabled
  }  

  /* Check UVLO flag: if not set, there is an undervoltage lock-out */
  if ((statusRegister & POWERSTEP01_STATUS_UVLO) == 0)
  {
     //undervoltage lock-out 
  }  

  /* Check UVLO ADC flag: if not set, there is an ADC undervoltage lock-out */
  if ((statusRegister & POWERSTEP01_STATUS_UVLO_ADC) == 0)
  {
     //ADC undervoltage lock-out 
  } 
  
  /* Check thermal STATUS flags: if  set, the thermal status is not normal */
  if ((statusRegister & POWERSTEP01_STATUS_TH_STATUS) != 0)
  {
    //thermal status: 1: Warning, 2: Bridge shutdown, 3: Device shutdown
  }    

  /* Check OCD  flag: if not set, there is an overcurrent detection */
  if ((statusRegister & POWERSTEP01_STATUS_OCD) == 0)
  {
    //overcurrent detection 
  }      

  /* Check STALL_A flag: if not set, there is a Stall condition on bridge A */
  if ((statusRegister & POWERSTEP01_STATUS_STALL_A) == 0)
  {
    //overcurrent detection 
  }    

  /* Check STALL_B flag: if not set, there is a Stall condition on bridge B */
  if ((statusRegister & POWERSTEP01_STATUS_STALL_B) == 0)
  {
    //overcurrent detection 
  }      

}

/**
  * @brief  This function is the User handler for the busy interrupt
  * @param  None
  * @retval None
  */
void MyBusyInterruptHandler(void)
{

   if (BSP_MotorControl_CheckBusyHw())
   {
      /* Busy pin is low, so at list one Powerstep01 chip is busy */
     /* To be customized (for example Switch on a LED) */
   }
   else
   {
     /* To be customized (for example Switch off a LED) */
   }
};


/**
  * @brief  This function is executed in case of error occurrence.
  * @param[in] error Number of the error
  * @retval None
  */
void MyErrorHandler(uint16_t error)
{
  /* Backup error number */
  gLastError = error;

  /* Infinite loop */
  while(1)
  {
  }
}

/* USER CODE BEGIN 4 */
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef * hspi)
{
    txComplete = 1;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    adcVal = HAL_ADC_GetValue(&hadc1); // Read & Update The ADC Result
    newAdcValIsReady = 1;
    //HAL_GPIO_TogglePin(ADC_CHECK_GPIO_Port, ADC_CHECK_Pin); // Toggle Interrupt Rate Indicator Pin
}


#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
