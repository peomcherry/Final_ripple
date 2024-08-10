
/* Includes ------------------------------------------------------------------*/
#include "chassisR_task.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usart.h"
#include "BSP_DWT.h"
#include "DM8009_drv.h"
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */


/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
									
chassis_t chassis_move;
/* USER CODE END PM */


/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */


void chassisR_task(void)
{
	
	
	/* USER CODE BEGIN INIT */
	CAN1_Config();
	DWT_Delay(1);
	printf("chassisR_task_start_init\r\n");
	for(int i = 0 ; i < 10 ; i++)
	{
		enable_motor_mode(&hcan1,DM8009_1_CAN_ID,MIT_MODE);
	}
	for(int i = 0; i < 10 ; i++)
	{
		enable_motor_mode(&hcan1,DM8009_2_CAN_ID,MIT_MODE);
	}
	for(int i = 0; i < 10 ; i++)
	{
		enable_motor_mode(&hcan1,DM8009_3_CAN_ID,MIT_MODE);
	}
	for(int i = 0; i < 10 ; i++)
	{
		enable_motor_mode(&hcan1,DM8009_4_CAN_ID,MIT_MODE);
	}
	
	printf("chassisR_task_init_over\r\n");
	
  /* USER CODE END INIT */
	
	while(1)
	{
		
		//printf("Joint_Motor_pos_control_start\r\n");
		Joint_Motor_pos_control(-70);
//				DM_Motor_mit_ctrl(&hcan1,DM8009_1_CAN_ID, degreesToRadians(-40), 0,DM8009_1_POS_KP, DM8009_1_POS_KD, 0);	
//				DM_Motor_mit_ctrl(&hcan1,DM8009_2_CAN_ID, degreesToRadians(-70), 0,DM8009_2_POS_KP, DM8009_2_POS_KD, 0);
//				DM_Motor_mit_ctrl(&hcan1,DM8009_3_CAN_ID, degreesToRadians(70), 0,DM8009_3_POS_KP, DM8009_3_POS_KD, 0);		
//				DM_Motor_mit_ctrl(&hcan1,DM8009_4_CAN_ID, degreesToRadians(-70), 0,DM8009_4_POS_KP, DM8009_4_POS_KD, 0);	
		
		osDelay(1);
	}
	
}









