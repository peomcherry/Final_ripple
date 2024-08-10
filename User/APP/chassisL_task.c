/* Includes ------------------------------------------------------------------*/
#include "chassisL_task.h"
#include "cmsis_os.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "chassisR_task.h"
#include "usart.h"
#include "BSP_DWT.h"
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */


/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

extern chassis_t chassis_move;	//µ×ÅÌ½á¹¹Ìå

/* USER CODE END PM */


/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */


void chassisL_task(void)
{
	/* USER CODE BEGIN INIT */
	DWT_Delay(1);
	printf("MI_Init_start\r\n");
	init_cybergear(&chassis_move.wheel_motor[0].para , CAN_MI_M1_ID,MI_Motion_mode);
	init_cybergear(&chassis_move.wheel_motor[1].para , CAN_MI_M2_ID,MI_Motion_mode);
	printf("MI_Init_over\r\n");
  /* USER CODE END INIT */
	
	while(1)
	{
		
		//printf("MI_Speed_start\r\n");
		
		MI_Motor_mit_ctrl(&hcan1,CAN_MI_M1_ID, 0, 10.0 ,0, MI_1_POS_KD, 0);
		MI_Motor_mit_ctrl(&hcan1,CAN_MI_M2_ID, 0, 10.0 ,0, MI_2_POS_KD, 0);
//		motor_controlmode(&chassis_move.wheel_motor[0].para,0,0,5.0,0,0.5);
//		motor_controlmode(&chassis_move.wheel_motor[1].para,0,0,5.0,0,0.5);
		
		
		//printf("\r\nSerial Output Message by DMA\r\n");
		//DWT_Delay(0.5f);
		
		osDelay(1);
	}
	
}
