/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "INS_task.h"
#include "chassisL_task.h"
#include "chassisR_task.h"
#include "DT7_task.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId INS_TASKHandle;
osThreadId CHASSISR_TASKHandle;
osThreadId CHASSISL_TASKHandle;
osThreadId OBSERVE_TASKHandle;
osThreadId DT7_TASKHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void INS_Task(void const * argument);
void ChassiR_Task(void const * argument);
void ChassisrL_Task(void const * argument);
void OBSERVE_Task(void const * argument);
void DT7_Task(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of INS_TASK */
  osThreadDef(INS_TASK, INS_Task, osPriorityRealtime, 0, 512);
  INS_TASKHandle = osThreadCreate(osThread(INS_TASK), NULL);

  /* definition and creation of CHASSISR_TASK */
  osThreadDef(CHASSISR_TASK, ChassiR_Task, osPriorityAboveNormal, 0, 512);
  CHASSISR_TASKHandle = osThreadCreate(osThread(CHASSISR_TASK), NULL);

  /* definition and creation of CHASSISL_TASK */
  osThreadDef(CHASSISL_TASK, ChassisrL_Task, osPriorityAboveNormal, 0, 512);
  CHASSISL_TASKHandle = osThreadCreate(osThread(CHASSISL_TASK), NULL);

  /* definition and creation of OBSERVE_TASK */
  osThreadDef(OBSERVE_TASK, OBSERVE_Task, osPriorityHigh, 0, 512);
  OBSERVE_TASKHandle = osThreadCreate(osThread(OBSERVE_TASK), NULL);

  /* definition and creation of DT7_TASK */
  osThreadDef(DT7_TASK, DT7_Task, osPriorityAboveNormal, 0, 128);
  DT7_TASKHandle = osThreadCreate(osThread(DT7_TASK), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_INS_Task */
/**
* @brief Function implementing the INS_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_INS_Task */
void INS_Task(void const * argument)
{
  /* USER CODE BEGIN INS_Task */
  /* Infinite loop */
  for(;;)
  {
		INS_task();
    osDelay(1);
  }
  /* USER CODE END INS_Task */
}

/* USER CODE BEGIN Header_ChassiR_Task */
/**
* @brief Function implementing the CHASSISR_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ChassiR_Task */
void ChassiR_Task(void const * argument)
{
  /* USER CODE BEGIN ChassiR_Task */
  /* Infinite loop */
  for(;;)
  {
		chassisR_task();
    osDelay(1);
  }
  /* USER CODE END ChassiR_Task */
}

/* USER CODE BEGIN Header_ChassisrL_Task */
/**
* @brief Function implementing the CHASSISL_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ChassisrL_Task */
void ChassisrL_Task(void const * argument)
{
  /* USER CODE BEGIN ChassisrL_Task */
  /* Infinite loop */
  for(;;)
  {
		chassisL_task();
    osDelay(1);
  }
  /* USER CODE END ChassisrL_Task */
}

/* USER CODE BEGIN Header_OBSERVE_Task */
/**
* @brief Function implementing the OBSERVE_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_OBSERVE_Task */
void OBSERVE_Task(void const * argument)
{
  /* USER CODE BEGIN OBSERVE_Task */
  /* Infinite loop */
  for(;;)
  {
		//Observe_task();
    osDelay(1);
  }
  /* USER CODE END OBSERVE_Task */
}

/* USER CODE BEGIN Header_DT7_Task */
/**
* @brief Function implementing the DT7_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_DT7_Task */
void DT7_Task(void const * argument)
{
  /* USER CODE BEGIN DT7_Task */
  /* Infinite loop */
  for(;;)
  {
		DT7_task();
    osDelay(1);
  }
  /* USER CODE END DT7_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
