/**
  ******************************************************************************
  * @file    Multi/Examples/MotionControl/IHM03A1_ExampleFor1Motor/Src/main.c 
  * @author  IPC Rennes
  * @version V1.6.0
  * @date    June 4th, 2018
  * @brief   This example shows how to use 1 Powerstep01 device
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
    
/** @addtogroup IHM03A1_Example_for_RegisterHandling
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Uncomment the define for the desired mode and comment the undesired one    */
#define CURRENT_MODE //to use initialization parameters for current mode
//#define VOLTAGE_MODE //to use initialization parameters for voltage mode

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
 static volatile uint16_t gLastError;

#ifdef CURRENT_MODE
/* Initialization parameters for current mode */
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
  328.12, // Hold torque in mV, range from 7.8mV to 1000 mV
  328.12, // Running torque in mV, range from 7.8mV to 1000 mV 
  328.12, // Acceleration torque in mV, range from 7.8mV to 1000 mV
  328.12, // Deceleration torque in mV, range from 7.8mV to 1000 mV
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
#endif //CURRENT_MODE

#ifdef VOLTAGE_MODE
/* Initialization parameters for voltage mode */
union powerstep01_Init_u initDeviceParameters =
{
  /* common parameters */
  .vm.cp.cmVmSelection = POWERSTEP01_CM_VM_VOLTAGE, // enum powerstep01_CmVm_t
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
  /* voltage mode parameters */
  16.02, // Hold duty cycle (torque) in %, range 0 to 99.6%
  16.02, // Run duty cycle (torque) in %, range 0 to 99.6%
  16.02, // Acceleration duty cycle (torque) in %, range 0 to 99.6%
  16.02, // Deceleration duty cycle (torque) in %, range 0 to 99.6%
  61.512, // Intersect speed settings for BEMF compensation in steps/s, range 0 to 3906 steps/s
  0.03815, // BEMF start slope settings for BEMF compensation in % step/s, range 0 to 0.4% s/step
  0.06256, // BEMF final acc slope settings for BEMF compensation in % step/s, range 0 to 0.4% s/step
  0.06256, // BEMF final dec slope settings for BEMF compensation in % step/s, range 0 to 0.4% s/step
  1, // Thermal compensation param, range 1 to 1.46875
  531.25, // Stall threshold settings in mV, range 31.25mV to 1000mV
  POWERSTEP01_CONFIG_INT_16MHZ_OSCOUT_2MHZ, // Clock setting , enum powerstep01_ConfigOscMgmt_t
  POWERSTEP01_CONFIG_SW_HARD_STOP, // External switch hard stop interrupt mode, enum powerstep01_ConfigSwMode_t
  POWERSTEP01_CONFIG_VS_COMP_DISABLE, // Motor Supply Voltage Compensation enabling , enum powerstep01_ConfigEnVscomp_t
  POWERSTEP01_CONFIG_OC_SD_DISABLE, // Over current shutwdown enabling, enum powerstep01_ConfigOcSd_t
  POWERSTEP01_CONFIG_UVLOVAL_LOW, // UVLO Threshold via powerstep01_ConfigUvLoVal_t
  POWERSTEP01_CONFIG_VCCVAL_15V, // VCC Val, enum powerstep01_ConfigVccVal_t
  POWERSTEP01_CONFIG_PWM_DIV_2, // PWM Frequency Integer division, enum powerstep01_ConfigFPwmInt_t
  POWERSTEP01_CONFIG_PWM_MUL_1, // PWM Frequency Integer Multiplier, enum powerstep01_ConfigFPwmDec_t
};
#endif //VOLTAGE_MODE

/* Private function prototypes -----------------------------------------------*/
static void MyBusyInterruptHandler(void);
static void MyFlagInterruptHandler(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  int32_t pos;
  uint32_t readData;

  /* STM32F4xx HAL library initialization */
  HAL_Init();
  
  /* Configure the system clock */
  SystemClock_Config();
    
//----- Init of the Powerstep01 library 
  /* Set the Powerstep01 library to use 1 device */
  BSP_MotorControl_SetNbDevices(BSP_MOTOR_CONTROL_BOARD_ID_POWERSTEP01, 1);
  /* When BSP_MotorControl_Init is called with NULL pointer,                  */
  /* the Powerstep01 registers are set with the predefined values from file   */
  /* powerstep01_target_config.h, otherwise the registers are set using the   */
  /* powerstep01_Init_u relevant union of structure values.                   */
  /* The first call to BSP_MotorControl_Init initializes the first device     */
  /* whose Id is 0.                                                           */
  /* The nth call to BSP_MotorControl_Init initializes the nth device         */
  /* whose Id is n-1.                                                         */
  /* Uncomment the call to BSP_MotorControl_Init below to initialize the      */
  /* device with the union declared in the the main.c file and comment the    */
  /* subsequent call having the NULL pointer                                  */
  //BSP_MotorControl_Init(BSP_MOTOR_CONTROL_BOARD_ID_POWERSTEP01, &initDeviceParameters);
  BSP_MotorControl_Init(BSP_MOTOR_CONTROL_BOARD_ID_POWERSTEP01, NULL);

  /* Attach the function MyFlagInterruptHandler (defined below) to the flag interrupt */
  BSP_MotorControl_AttachFlagInterrupt(MyFlagInterruptHandler);

  /* Attach the function MyBusyInterruptHandler (defined below) to the busy interrupt */
  BSP_MotorControl_AttachBusyInterrupt(MyBusyInterruptHandler);
  
  /* Attach the function Error_Handler (defined below) to the error Handler*/
  BSP_MotorControl_AttachErrorHandler(MyErrorHandler);

//----- Move of 16000 steps in the FW direction

  /* Move device 0 of 16000 steps in the FORWARD direction*/
  BSP_MotorControl_Move(0, FORWARD, 16000);

  /* Wait for the motor of device 0 ends moving */
  BSP_MotorControl_WaitWhileActive(0);

  /* Wait for 2 seconds */
  HAL_Delay(2000);  
  
//----- Move of 16000 steps in the BW direction

  /* Move device 0 of 16000 steps in the BACKWARD direction*/
  BSP_MotorControl_Move(0, BACKWARD, 16000);

  /* Wait for the motor of device 0 ends moving */
  BSP_MotorControl_WaitWhileActive(0);

   /* Set the current position of device 0 to be the Home position */
  BSP_MotorControl_CmdResetPos(0);
  
  /* Wait for 2 seconds */
  HAL_Delay(2000);

//----- Go to position -6400
  /* Request device 0 to go to position -6400 */
  BSP_MotorControl_GoTo(0,-6400);  
  
  /* Wait for the motor ends moving */
  BSP_MotorControl_WaitWhileActive(0);

  /* Get current position of device 0*/
  pos = BSP_MotorControl_GetPosition(0);

  /* Mark the current position */
  BSP_MotorControl_SetMark(0, pos);

  /* Wait for 2 seconds */
  HAL_Delay(2000);
  
//----- Go Home
  /* Request device 0 to go to Home */
  BSP_MotorControl_GoHome(0);  
  BSP_MotorControl_WaitWhileActive(0);

  /* Get current position of device 0 */
  pos = BSP_MotorControl_GetPosition(0);
  
  /* Wait for 2 seconds */
  HAL_Delay(2000);

//----- Go to position 6400

  /* Request device 0 to go to position 6400 */
  BSP_MotorControl_GoTo(0,6400);  
  
  /* Wait for the motor of device 0 ends moving */
  BSP_MotorControl_WaitWhileActive(0);

  /* Get current position of device 0*/
  pos = BSP_MotorControl_GetPosition(0);

  /* Wait for 2 seconds */
  HAL_Delay(2000);
  
//----- Go Mark which was set previously after go to -6400

  /* Request device 0 to go to Mark position */
  BSP_MotorControl_GoMark(0);  
  
  /* Wait for the motor of device 0 ends moving */
  BSP_MotorControl_WaitWhileActive(0);

  /* Get current position of device 0 */
  pos = BSP_MotorControl_GetPosition(0);

  /* Wait for 2 seconds */
  HAL_Delay(2000);

//----- Run the motor BACKWARD

  /* Request device 0 to run BACKWARD at 26843 steps/tick (400 step/s) */
  BSP_MotorControl_CmdRun(0,BACKWARD,26843);       

//----- Get parameter example   
  /* Wait for device 0 reaches the targeted speed */
  do
  {
    readData = BSP_MotorControl_CmdGetParam(0, POWERSTEP01_SPEED);
  }while (readData != 26843);

//----- Soft stopped required while running

  /* Request a soft stop of device 0 and keep the power bridges enabled */
  BSP_MotorControl_CmdSoftHiZ(0);

  /* Wait for the motor of device 0 ends moving */  
  BSP_MotorControl_WaitWhileActive(0);

  /* Wait for 2 seconds */
  HAL_Delay(2000);

//----- Run stopped by hardstop

  /* Request device 0 to run in FORWARD direction */ 
  /* at 20132 steps/tick (300 step/s) */
  BSP_MotorControl_CmdRun(0,FORWARD,20132);         
  HAL_Delay(5000);
  
  /* Request device 0 to immediatly stop */
  BSP_MotorControl_HardStop(0);
  BSP_MotorControl_WaitWhileActive(0);

  /* Wait for 2 seconds */
  HAL_Delay(2000);

//----- GOTO stopped by soft stop

 /* Request device 0 to go to position 200000  */
  BSP_MotorControl_GoTo(0,200000);  
  HAL_Delay(2000);

  /* Request device 0 to perform a soft stop */
  BSP_MotorControl_CmdSoftStop(0);
  BSP_MotorControl_WaitWhileActive(0);

  /* Wait for 2 seconds */
  HAL_Delay(2000);

//----- Read inexistent register to test MyFlagInterruptHandler

  /* Try to read an inexistent register */
  /* the flag interrupt should be raised */
  /* and the MyFlagInterruptHandler function called */
  BSP_MotorControl_CmdGetParam(0, 0x1F);
  
//----- Put the bridges in high impedance
  
  /* Request disabling of device 0 power bridges */
  BSP_MotorControl_CmdHardHiZ(0);
  HAL_Delay(5000);
  
//----- Step clock mode example

  /* Enable Step Clock Mode of the Powerstep01, enabling power bridges */
  BSP_MotorControl_CmdStepClock(0, FORWARD);
  
  /* Wait for 1 second */
  HAL_Delay(1000);
  
  /* Enable the step clock at 333 Hz */
  BSP_MotorControl_StartStepClock(333);
  
  /* Let the motor runs for 5 second at 333 step/s */
  HAL_Delay(5000);
  
  /* Stop the step clock */
  BSP_MotorControl_StopStepClock();
  
  /* Wait for 2 seconds */
  HAL_Delay(2000);

//----- Set parameter example
  /* Change the maximum speed to 3 * 2^18 step/tick */
  BSP_MotorControl_CmdSetParam(0,POWERSTEP01_MAX_SPEED, 3);
  
  /* Request device 0 to move 3200 microsteps */
  BSP_MotorControl_Move(0,FORWARD,3200);
    
  /* Wait for the motor of device 0 ends moving */
  BSP_MotorControl_WaitWhileActive(0);
  
  /* Wait for 2 seconds */
  HAL_Delay(2000);
  
//----- Change step mode to full step mode
  /* Select full step mode for device 0 */
  BSP_MotorControl_SelectStepMode(0,STEP_MODE_FULL);

  /* Set the device 0 position to POWERSTEP01_MIN_POSITION + 199 */
  BSP_MotorControl_CmdSetParam(0,POWERSTEP01_ABS_POS,(uint32_t)(POWERSTEP01_MIN_POSITION+199));

  /* Request device 0 to go to the POWERSTEP01_MAX_POSITION using the shortest path */
  BSP_MotorControl_GoTo(0,POWERSTEP01_MAX_POSITION);

  /* Wait for the motor of device 0 ends moving */
  BSP_MotorControl_WaitWhileActive(0);

  /* Get current position */
  pos = BSP_MotorControl_GetPosition(0);

  /* Wait for 2 seconds */
  HAL_Delay(2000);
  
  /* Set the device 0 position to POWERSTEP01_MIN_POSITION + 199 */
  BSP_MotorControl_CmdSetParam(0,POWERSTEP01_ABS_POS,(uint32_t)(POWERSTEP01_MIN_POSITION+199));  

//----- GoTo_DIR example
  /* Request device 0 to go to the POWERSTEP01_MAX_POSITION using backward direction */
  BSP_MotorControl_CmdGoToDir(0,BACKWARD,POWERSTEP01_MAX_POSITION);

  /* Wait for the motor of device 0 ends moving */
  BSP_MotorControl_WaitWhileActive(0);

  /* Get current position */
  pos = BSP_MotorControl_GetPosition(0);  
  
  /* Restore device 0 initial max speed */
  BSP_MotorControl_SetAnalogValue(0, POWERSTEP01_MAX_SPEED, POWERSTEP01_CONF_PARAM_MAX_SPEED_DEVICE_0);
  
//----- Restore initial microstepping mode

  /* Reset device 0 to its initial  microstepping mode */
  BSP_MotorControl_SelectStepMode(0,POWERSTEP01_CONF_PARAM_STEP_MODE_DEVICE_0);
  
  /* Wait for 2 seconds */
  HAL_Delay(2000);
 
  /* Infinite loop */
  while(1)
  {
    /* Request device 0 to go position -6400 */
    BSP_MotorControl_GoTo(0, -6400);

    /* Wait for the motor of device 0 ends moving */
    BSP_MotorControl_WaitWhileActive(0);

    /* Request device 0 to go position 6400 */
    BSP_MotorControl_GoTo(0,6400);

    /* Wait for the motor of device 0 ends moving */
    BSP_MotorControl_WaitWhileActive(0);
  }

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

  /* Check Command Error flag: if set, the command received by SPI can't be */
  /* performed. This occurs for instance when a move command is sent to the */
  /* Powerstep01 while it is already running */
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
}


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
