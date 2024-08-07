/* Includes ------------------------------------------------------------------*/
#include "chassisL_task.h"
#include "cmsis_os.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usart.h"
#include "BSP_DWT.h"
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */


/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */


/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */


void chassisL_task(void)
{
	/* USER CODE BEGIN INIT */

  /* USER CODE END INIT */
	
	while(1)
	{
		
		//printf("\r\nSerial Output Message by DMA\r\n");
		//DWT_Delay(0.5f);
		osDelay(1);
	}
	
}
