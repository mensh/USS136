/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without
  *modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright
  *notice,
  *      this list of conditions and the following disclaimer in the
  *documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its
  *contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  *ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
  *LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
  *USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "stm32f2xx_hal.h"
#include "cpu_utils.h"
#include "tcc_task.h"
#include "can_task.h"
/* USER CODE BEGIN Includes */
#define F_INT_LOW                                                              \
  HAL_GPIO_WritePin(F_INT_M_GPIO_Port, F_INT_M_Pin, GPIO_PIN_RESET);
#define F_INT_HIGHT                                                            \
  HAL_GPIO_WritePin(F_INT_M_GPIO_Port, F_INT_M_Pin, GPIO_PIN_SET);

#define EN_READ HAL_GPIO_ReadPin(EN_GPIO_Port, EN_Pin);

uint16_t CPU_load;

/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId defaultTaskHandle;
osThreadId IntegtatorId;
osThreadId TCC1_Task;
osThreadId TCC2_Task;
osThreadId TCC3_Task;
osThreadId TCC4_Task;

osThreadId CAN_TaskId;
osTimerId myTimer01Handle;

/* USER CODE BEGIN Variables */
extern uint32_t Timer_100_ms;
extern uint32_t Timer_1_s;
extern uint32_t Timer_8_s;
/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void StartDefaultTask(void const *argument);
void IntegratorTask(void const *argument);
void CAN_Task(void const *argument);

void Callback01(void const *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

void MX_FREERTOS_Init(void) {

  osTimerDef(myTimer01, Callback01);
  myTimer01Handle = osTimerCreate(osTimer(myTimer01), osTimerPeriodic, NULL);

  osThreadDef(defaultTask, StartDefaultTask, osPriorityBelowNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

	
  osThreadDef(TCC1_Task, Sensor_1_Task, osPriorityBelowNormal, 0, 128);
  TCC1_Task = osThreadCreate(osThread(TCC1_Task), NULL);
	
	
  osThreadDef(TCC2_Task, Sensor_2_Task, osPriorityBelowNormal, 0, 128);
  TCC2_Task = osThreadCreate(osThread(TCC2_Task), NULL);
	
		
  osThreadDef(TCC3_Task, Sensor_3_Task, osPriorityBelowNormal, 0, 128);
  TCC3_Task = osThreadCreate(osThread(TCC3_Task), NULL);
	
		
  osThreadDef(TCC4_Task, Sensor_4_Task, osPriorityBelowNormal, 0, 128);
  TCC4_Task = osThreadCreate(osThread(TCC4_Task), NULL);
	
	
	
  osThreadDef(IntegratorTask, IntegratorTask, osPriorityNormal, 0, 128);
  IntegtatorId = osThreadCreate(osThread(IntegratorTask), NULL);

  osThreadDef(CAN_Task, CAN_Task, osPriorityNormal, 0, 128);
  CAN_TaskId = osThreadCreate(osThread(CAN_Task), NULL);

  osTimerStart(myTimer01Handle, 1);
}

void CAN_Task(void const *argument) {
  while (1) {
    CAN_Transmit();
    osDelay(100);
  }
}

void IntegratorTask(void const *argument) {
  static uint8_t state;
  uint8_t EN_Control;
  while (1) {
    if (state == 0) {
      F_INT_LOW;
      state = 1;
    } else {
      F_INT_HIGHT;
      state = 0;
    }
    EN_Control = EN_READ;
    if (EN_Control == 1) {
    } else {
    }
    osDelay(1);
    CPU_load = osGetCPUUsage();
  }
}

void StartDefaultTask(void const *argument) { 
	Task_TCC();
}

/* Callback01 function */
void Callback01(void const *argument) {
  /* USER CODE BEGIN Callback01 */
  if (Timer_100_ms > 0)
    Timer_100_ms--;
  if (Timer_1_s > 0)
    Timer_1_s--;
  if (Timer_8_s > 0)
    Timer_8_s--;
  /* USER CODE END Callback01 */
}

/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
