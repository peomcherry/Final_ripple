#include "bsp_can.h"
#include "can.h"
#include "DM8009_drv.h"
#include "CyberGear_drv.h"
#include "string.h"
#include "chassisR_task.h"

CAN_RxHeaderTypeDef RxHeader1;
uint8_t g_Can1RxData[64];

CAN_RxHeaderTypeDef RxHeader2;	
uint8_t g_Can2RxData[64];



void CAN1_Config(void)			//CAN1滤波配置
{
  CAN_FilterTypeDef sFilterConfig;
  /* Configure Rx filter */	
	//CAN1	配置使能
	sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterIdHigh = 0x0000; 
  sFilterConfig.FilterIdLow = 0x0000; 
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterBank = 0;
	sFilterConfig.SlaveStartFilterBank = 0;
	
//		HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);
//    HAL_CAN_Start(&hcan1);
//    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	
	
//    sFilterConfig.SlaveStartFilterBank = 14;
//    sFilterConfig.FilterBank = 14;
//	
//	

		
		//三件套
		if(HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
		{
		Error_Handler();
		}
	 if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
   {
        Error_Handler();
   }
	if (HAL_CAN_Start(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
		
/* 全局过滤设置 */
/* 接收到消息ID与标准ID过滤不匹配，不接受 */
/* 接收到消息ID与扩展ID过滤不匹配，不接受 */
/* 过滤标准ID远程帧 */ 
/* 过滤扩展ID远程帧 */ 
//  if (HAL_CAN_ConfigGlobalFilter(&hcan1, CAN_REJECT, CAN_REJECT, CAN_FILTER_REMOTE, CAN_FILTER_REMOTE) != HAL_OK)
//  {
//    Error_Handler();
//  }


  /* Start the FDCAN module */

}

void CAN2_Config(void)			//CAN2滤波配置
{
  CAN_FilterTypeDef sFilterConfig;
  /* Configure Rx filter */
	sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterIdHigh = 0x0000; 
  sFilterConfig.FilterIdLow = 0x0000; 
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
	sFilterConfig.SlaveStartFilterBank = 14;
  sFilterConfig.FilterBank = 14;
  if (HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }
	
	HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig);
  HAL_CAN_Start(&hcan2);
  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);

  /* Configure global filter:
     Filter all remote frames with STD and EXT ID
     Reject non matching frames with STD ID and EXT ID */


  /* Activate Rx FIFO 0 new message notification on both FDCAN instances */


  if (HAL_CAN_Start(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
}

uint8_t canx_send_data(CAN_HandleTypeDef *hcan, uint16_t id, uint8_t *data, uint32_t len , uint16_t ExternID_Flag)	//发送函数
{
	CAN_TxHeaderTypeDef TxHeader;

uint32_t send_mail_0_box;
uint32_t send_mail_1_box;
uint32_t send_mail_2_box;

	
	TxHeader.StdId = id;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.DLC = len;       
	
	
	
	if( ExternID_Flag ==  STD_CAN_ID )
	{
		TxHeader.IDE = CAN_ID_STD;
		TxHeader.ExtId=0;
	}
	else if( ExternID_Flag == EXT_CAN_ID )
	{
		TxHeader.IDE = CAN_ID_EXT;
	}
	
		if(HAL_CAN_AddTxMessage(hcan, &TxHeader, data, &send_mail_0_box) != HAL_OK) 
	{
		printf("ERROR:MAILBOX0\r\n");
		if(HAL_CAN_AddTxMessage(hcan, &TxHeader, data, &send_mail_1_box) != HAL_OK)
		{
			printf("ERROR:MAILBOX1\r\n");
			//HAL_CAN_AddTxMessage(hcan, &TxHeader, data, (uint32_t*)CAN_TX_MAILBOX2);
			if(HAL_CAN_AddTxMessage(hcan, &TxHeader, data, &send_mail_2_box)!= HAL_OK)
				printf("ERROR\r\n");
    }
   }
	/*找到空的发送邮箱，把数据发送出去*/
//	if(HAL_CAN_AddTxMessage(hcan, &TxHeader, data, (uint32_t*)CAN_TX_MAILBOX0) != HAL_OK) 
//	{
//		printf("ERROR:MAILBOX0\r\n");
//		if(HAL_CAN_AddTxMessage(hcan, &TxHeader, data, (uint32_t*)CAN_TX_MAILBOX1) != HAL_OK)
//		{
//			printf("ERROR:MAILBOX1\r\n");
//			//HAL_CAN_AddTxMessage(hcan, &TxHeader, data, (uint32_t*)CAN_TX_MAILBOX2);
//			if(HAL_CAN_AddTxMessage(hcan, &TxHeader, data, (uint32_t*)CAN_TX_MAILBOX2)!= HAL_OK)
//				printf("ERROR\r\n");
//    }
//   }
	
	 
	
//	TxHeader.Identifier = id;                 // CAN ID
//  TxHeader.IdType =  FDCAN_STANDARD_ID ;        
//  TxHeader.TxFrameType = FDCAN_DATA_FRAME;  
//	
//  if(len<=8)	
//	{
//	  TxHeader.DLC = len<<16;     // 发送长度：8byte
//	}
//	else  if(len==12)	
//	{
//	   TxHeader.DLC =FDCAN_DLC_BYTES_12;
//	}
//	else  if(len==16)	
//	{
//	  TxHeader.DLC =FDCAN_DLC_BYTES_16;
//	
//	}
//  else  if(len==20)
//	{
//		TxHeader.DLC =FDCAN_DLC_BYTES_20;
//	}		
//	else  if(len==24)	
//	{
//	 TxHeader.DLC =FDCAN_DLC_BYTES_24;	
//	}else  if(len==48)
//	{
//	 TxHeader.DLC =FDCAN_DLC_BYTES_48;
//	}else  if(len==64)
//   {
//		 TxHeader.DLC =FDCAN_DLC_BYTES_64;
//	 }
											
//	TxHeader.ErrorStateIndicator =  FDCAN_ESI_ACTIVE;
//  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;//比特率切换关闭，不适用于经典CAN
//  TxHeader.FDFormat =  FDCAN_CLASSIC_CAN;            // CANFD
//  TxHeader.TxEventFifoControl =  FDCAN_NO_TX_EVENTS;  
//  TxHeader.MessageMarker = 0;//消息标记

   // 发送CAN指令
//  if(HAL_FDCAN_AddMessageToTxFifoQ(hcan, &TxHeader, data) != HAL_OK)
//  {
//        // 发送失败处理
//      Error_Handler();      
//  }
//	 HAL_FDCAN_AddMessageToTxFifoQ(hcan, &TxHeader, data);
	 return 0;
}



extern chassis_t chassis_move;	//底盘结构体

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)	//CAN_FIFO0的接收函数（CAN1用FIFO0）
{
	 if(hcan->Instance == CAN1)
    {
        /* 假设g_Can1RxData和RxHeader1已经定义且大小合适 */
        CAN_RxHeaderTypeDef RxHeader1;
        uint8_t g_Can1RxData[8]; // 根据实际数据长度调整数组大小
//E7 === CAN5 M4
//					CAN7  M6
			
        /* 清空接收数组 */
        memset(g_Can1RxData, 0, sizeof(g_Can1RxData));

        /* 从RX FIFO0获取消息 */
        if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader1, g_Can1RxData) == HAL_OK)
        {
					if(RxHeader1.IDE == CAN_ID_STD)
					{
                  /* 根据ID处理消息 */
                  switch(RxHeader1.StdId)
                  {
                      case DM8009_1_MASTER_ID:dm8009_data(&chassis_move.joint_motor[0] ,g_Can1RxData, RxHeader1.DLC ); break;
                      case DM8009_2_MASTER_ID:dm8009_data(&chassis_move.joint_motor[1] ,g_Can1RxData, RxHeader1.DLC ); break;      
                      case DM8009_3_MASTER_ID:dm8009_data(&chassis_move.joint_motor[2] ,g_Can1RxData, RxHeader1.DLC ); break;
                      case DM8009_4_MASTER_ID:dm8009_data(&chassis_move.joint_motor[3] ,g_Can1RxData, RxHeader1.DLC ); break;
                      default: break;
                  }
					}
					else if(RxHeader1.IDE == CAN_ID_EXT)
					{
						                  /* 根据ID处理消息 */
                  switch(((RxHeader1.ExtId&0xFFFF)>>8))
                  {
                      case CAN_MI_M1_ID:{
												if(RxHeader1.ExtId >> 24 != 0){
														CyberGear_data(&chassis_move.wheel_motor[0],g_Can1RxData, RxHeader1.DLC , RxHeader1.ExtId);
												}else{
														chassis_move.wheel_motor[0].para.MCU_ID = g_Can1RxData[0];
												}
												break;
											}
																		
                      case CAN_MI_M2_ID:{
												if(RxHeader1.ExtId >> 24 != 0){
														CyberGear_data(&chassis_move.wheel_motor[1],g_Can1RxData, RxHeader1.DLC , RxHeader1.ExtId);
												}else{
														chassis_move.wheel_motor[0].para.MCU_ID = g_Can1RxData[1];
												}
                          break;
											}

                      default:
                          break;
                  }
					}
        }
    }
		else if(hcan->Instance == CAN2)
		{
			
		}
    
    /* 重新激活接收FIFO0的消息待定中断 */
    HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}





/**********************CAN_FIFO0的Callback_QwQ如果以后用得到的话*********************************************************

void HAL_FDCAN_RxFifo0Callback(CAN_HandleTypeDef *hcan, uint32_t RxFifo0ITs)
{ 
  if((RxFifo0ITs & CAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
  {
    if(hfdcan->Instance == CAN1)
    {
      // Retrieve Rx messages from RX FIFO0 
			memset(g_Can1RxData, 0, sizeof(g_Can1RxData));	//接收前先清空数组	
      HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader1, g_Can1RxData);
			
			switch(RxHeader1.Identifier)
			{
        case 3 :dm4310_fbdata(&chassis_move.joint_motor[0], g_Can1RxData,RxHeader1.DataLength);break;
        case 4 :dm4310_fbdata(&chassis_move.joint_motor[1], g_Can1RxData,RxHeader1.DataLength);break;	         	
				case 0 :dm6215_fbdata(&chassis_move.wheel_motor[0], g_Can1RxData,RxHeader1.DataLength);break;
				default: break;
			}			
	  }
  }
}

void HAL_FDCAN_RxFifo1Callback(CAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
  if((RxFifo1ITs & CAN_IT_RX_FIFO1_NEW_MESSAGE) != RESET)
  {
    if(hfdcan->Instance == CAN2)
    {
      // Retrieve Rx messages from RX FIFO0 
			memset(g_Can2RxData, 0, sizeof(g_Can2RxData));
      HAL_FDCAN_GetRxMessage(hfdcan, CAN_RX_FIFO1, &RxHeader2, g_Can2RxData);
			switch(RxHeader2.Identifier)
			{
        case 3 :dm4310_fbdata(&chassis_move.joint_motor[2], g_Can2RxData,RxHeader2.DataLength);break;
        case 4 :dm4310_fbdata(&chassis_move.joint_motor[3], g_Can2RxData,RxHeader2.DataLength);break;	         	
				case 0 :dm6215_fbdata(&chassis_move.wheel_motor[1], g_Can2RxData,RxHeader2.DataLength);break;
				default: break;
			}	
    }
  }
}

********************************************************************************************************
*/

