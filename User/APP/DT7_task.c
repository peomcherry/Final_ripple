/**
  *********************************************************************
  * @file      DT7_task.c/h
  * @brief     该任务是读取并处理ps2手柄传来的遥控数据，
	*            将遥控数据转化为期望的速度、期望的转角、期望的腿长等
  * @note       
  * @history
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  *********************************************************************
  */
/* Includes ------------------------------------------------------------------*/
	
#include "DT7_task.h"
#include "cmsis_os.h"
//#include "usart.h"


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "BSP_DWT.h"
#include "usart.h"
#include "BMI088driver.h"
#include "ist8310driver.h"
#include "bsp_sbus.h"
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */


/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
extern rc_info_t rc;

/* USER CODE END PM */


/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */


void DT7_task(void)
{
	/* USER CODE BEGIN INIT */
	 /* USER CODE BEGIN Init */
		//printf("DT7_task init_begin\r\n");

		
		
	
  /* USER CODE END Init */
		dbus_uart_init();

  /* USER CODE END INIT */
	
	printf("DT7_task init_ok\r\n");
	while(1)
	{
		//printf("CH1=%d    CH2=%d    CH3=%d    CH4=%d\r\n",rc.ch1,rc.ch2,rc.ch3,rc.ch4);

		
		
		osDelay(10);
	}
	
}

/**BMI088初始化测试函数
		
//		float gyro[3], accel[3], temp;

//		BMI088_read(gyro, accel, &temp);
//		printf("gyro  =    %f    %f    %f\r\n",gyro[0],gyro[1],gyro[2]);
//		printf("accel =    %f    %f    %f\r\n",accel[0],accel[1],accel[2]);
//		printf("temp = %f\n",temp);
**/
