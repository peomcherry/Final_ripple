/**
  ****************************(C)HXC_ROBOCON****************************
  * @file       CyberGear_drv.c/h
  * @brief      С�׵��������
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V0.0.0     7-8-2024       Peomcherry        test
  *
  @verbatim
  =========================================================================
  =========================================================================
  @endverbatim
  ****************************(C)HXC_ROBOCON****************************
  **/
#include "main.h"
#include "can.h"
#include "CyberGear_drv.h"
#include "DM8009_drv.h"

CAN_RxHeaderTypeDef rxMsg;//���ͽ��սṹ��
CAN_TxHeaderTypeDef txMsg;//�������ýṹ��
uint8_t rx_data[8];       //��������
uint32_t Motor_Can_ID;    //�������ݵ��ID
uint8_t byte[4];          //ת����ʱ����
//uint32_t send_mail_box = {0};//NONE

    
//#define can_txd() HAL_CAN_AddTxMessage(&hcan1, &txMsg, tx_data, &send_mail_box)//CAN���ͺ궨��

//MI_Motor mi_motor[2];//Ԥ�ȶ����ĸ�С�׵��


//void wheel_motor_speed_control(float speed){
//
//	motor_controlmode(&mi_motor[0], 0, 0.f, -speed, 0.f , 0.5f);
//	motor_controlmode(&mi_motor[1], 0, 0.f, speed, 0.f , 0.5f);
//
//}


/**
 * @Func		my_can_filter_init
 * @Brief       CAN1��CAN2�˲�������
 * @param       _hcan
 * @Data        2022��12��16��
 */
void __can_filter_init_recv_all(CAN_HandleTypeDef *_hcan)
{

    // can1 &can2 use same filter config
    CAN_FilterTypeDef CAN_FilterType = {0};
    /*
    static CanTxMsgTypeDef		Tx1Message;
    static CanRxMsgTypeDef 		Rx1Message;
    */
    CAN_FilterType.FilterBank = 0;
    CAN_FilterType.FilterIdHigh = 0x0000;
    CAN_FilterType.FilterIdLow = 0x0000;
    CAN_FilterType.FilterMaskIdHigh = 0x0000; //(((uint32_t)0x1313<<3)&0xFFFF0000)>>16;
    CAN_FilterType.FilterMaskIdLow = 0x0000;  //(((uint32_t)0x1313<<3)|CAN_ID_EXT|CAN_RTR_DATA)&0xFFFF;
    CAN_FilterType.FilterFIFOAssignment = CAN_RX_FIFO0;
    CAN_FilterType.FilterMode = CAN_FILTERMODE_IDMASK;
    CAN_FilterType.FilterScale = CAN_FILTERSCALE_32BIT;
    CAN_FilterType.FilterActivation = ENABLE;
    CAN_FilterType.SlaveStartFilterBank = 14;
    if (HAL_CAN_ConfigFilter(_hcan, &CAN_FilterType) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_CAN_ActivateNotification(_hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_CAN_Start(_hcan) != HAL_OK)
    {
        Error_Handler();
    }
}
/**
  * @brief          ������ת4�ֽں���
  * @param[in]      f:������
  * @retval         4�ֽ�����
  * @description  : IEEE 754 Э��
  */
static uint8_t* Float_to_Byte(float f)
{
	unsigned long longdata = 0;
	longdata = *(unsigned long*)&f;       
	byte[0] = (longdata & 0xFF000000) >> 24;
	byte[1] = (longdata & 0x00FF0000) >> 16;
	byte[2] = (longdata & 0x0000FF00) >> 8;
	byte[3] = (longdata & 0x000000FF);
	return byte;
}

///**
//  * @brief          С�׵������16λ����ת����
//  * @param[in]      x:16λ����
//  * @param[in]      x_min:��Ӧ��������
//  * @param[in]      x_max:��Ӧ��������
//  * @param[in]      bits:����λ��
//  * @retval         ���ظ���ֵ
//  */
//static float uint16_to_float(uint16_t x,float x_min,float x_max,int bits)
//{
//    uint32_t span = (1 << bits) - 1;
//    float offset = x_max - x_min;
//    return offset * x / span + x_min;
//}

/**
  * @brief          С�׵�����͸���ת16λ����
  * @param[in]      x:����
  * @param[in]      x_min:��Ӧ��������
  * @param[in]      x_max:��Ӧ��������
  * @param[in]      bits:����λ��
  * @retval         ���ظ���ֵ
  */
static int float_to_uint(float x, float x_min, float x_max, int bits)
{
  float span = x_max - x_min;
  float offset = x_min;
  if(x > x_max) x=x_max;
  else if(x < x_min) x= x_min;
  return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}

/**
  * @brief          д��������
  * @param[in]      Motor:��Ӧ���Ƶ���ṹ��
  * @param[in]      Index:д�������Ӧ��ַ
  * @param[in]      Value:д�����ֵ
  * @param[in]      Value_type:д�������������
  * @retval         none
  */
static void Set_Motor_Parameter(MI_Motor *Motor,uint16_t Index,float Value,char Value_type){
	uint8_t tx_data[8];
	txMsg.ExtId = Communication_Type_SetSingleParameter<<24|Master_CAN_ID<<8|Motor->CAN_ID;
	tx_data[0]=Index;
	tx_data[1]=Index>>8;
	tx_data[2]=0x00;
	tx_data[3]=0x00;
	if(Value_type == 'f'){
		Float_to_Byte(Value);
		tx_data[4]=byte[3];
		tx_data[5]=byte[2];
		tx_data[6]=byte[1];
		tx_data[7]=byte[0];		
	}
	else if(Value_type == 's'){
		tx_data[4]=(uint8_t)Value;
		tx_data[5]=0x00;
		tx_data[6]=0x00;
		tx_data[7]=0x00;				
	}
	  	//Ѱ�����䷢������
		if(HAL_CAN_AddTxMessage(&hcan1, &txMsg, tx_data, (uint32_t*)CAN_TX_MAILBOX0) != HAL_OK) 
		{
			if(HAL_CAN_AddTxMessage(&hcan1, &txMsg, tx_data, (uint32_t*)CAN_TX_MAILBOX1) != HAL_OK)
			{
				HAL_CAN_AddTxMessage(&hcan1, &txMsg, tx_data, (uint32_t*)CAN_TX_MAILBOX2);
       }
		} 
//	can_txd();	
}

///**
//  * @brief          ��ȡ����ظ�֡��չID�еĵ��CANID
//  * @param[in]      CAN_ID_Frame:����ظ�֡�е���չCANID   
//  * @retval         ���CANID
//  */
//static uint32_t Get_Motor_ID(uint32_t CAN_ID_Frame)
//{
//	return (CAN_ID_Frame&0xFFFF)>>8;
//}

///**
//  * @brief          ����ظ�֡���ݴ�����
//  * @param[in]      Motor:��Ӧ���Ƶ���ṹ��   
//  * @param[in]      DataFrame:����֡
//  * @param[in]      IDFrame:��չID֡
//  * @retval         None
//  */
//static void Motor_Data_Handler(MI_Motor *Motor,uint8_t DataFrame[8],uint32_t IDFrame)
//{	
//		Motor->Angle=uint16_to_float(DataFrame[0]<<8|DataFrame[1],MIN_P,MAX_P,16);
//		Motor->Speed=uint16_to_float(DataFrame[2]<<8|DataFrame[3],V_MIN,V_MAX,16);			
//		Motor->Torque=uint16_to_float(DataFrame[4]<<8|DataFrame[5],T_MIN,T_MAX,16);				
//		Motor->Temp=(DataFrame[6]<<8|DataFrame[7])*Temp_Gain;
//		Motor->error_code=(IDFrame&0x1F0000)>>16;	
//}

/**
  * @brief          С�׵��ID���
  * @param[in]      id:  ���Ƶ��CAN_ID������Ĭ��0x7F��
  * @retval         none
  */
void chack_cybergear(uint8_t ID)
{
    uint8_t tx_data[8] = {0};
    txMsg.ExtId = Communication_Type_GetID<<24|Master_CAN_ID<<8|ID;
  	//Ѱ�����䷢������
		if(HAL_CAN_AddTxMessage(&hcan1, &txMsg, tx_data, (uint32_t*)CAN_TX_MAILBOX0) != HAL_OK) 
		{
			if(HAL_CAN_AddTxMessage(&hcan1, &txMsg, tx_data, (uint32_t*)CAN_TX_MAILBOX1) != HAL_OK)
			{
				HAL_CAN_AddTxMessage(&hcan1, &txMsg, tx_data, (uint32_t*)CAN_TX_MAILBOX2);
       }
		} 
//		can_txd();
}

/**
  * @brief          ʹ��С�׵��
  * @param[in]      Motor:��Ӧ���Ƶ���ṹ��   
  * @retval         none
  */
void start_cybergear(MI_Motor *Motor)
{
    uint8_t tx_data[8] = {0}; 
    txMsg.ExtId = Communication_Type_MotorEnable<<24|Master_CAN_ID<<8|Motor->CAN_ID;
  	//Ѱ�����䷢������
		if(HAL_CAN_AddTxMessage(&hcan1, &txMsg, tx_data, (uint32_t*)CAN_TX_MAILBOX0) != HAL_OK) 
		{
			if(HAL_CAN_AddTxMessage(&hcan1, &txMsg, tx_data, (uint32_t*)CAN_TX_MAILBOX1) != HAL_OK)
			{
				HAL_CAN_AddTxMessage(&hcan1, &txMsg, tx_data, (uint32_t*)CAN_TX_MAILBOX2);
       }
		} 
//		can_txd();
}

/**
  * @brief          ֹͣ���
  * @param[in]      Motor:��Ӧ���Ƶ���ṹ��   
  * @param[in]      clear_error:�������λ��0 ����� 1�����
  * @retval         None
  */
void stop_cybergear(MI_Motor *Motor,uint8_t clear_error)
{
	uint8_t tx_data[8]={0};
	tx_data[0]=clear_error;//�������λ����
	txMsg.ExtId = Communication_Type_MotorStop<<24|Master_CAN_ID<<8|Motor->CAN_ID;
  	//Ѱ�����䷢������
		if(HAL_CAN_AddTxMessage(&hcan1, &txMsg, tx_data, (uint32_t*)CAN_TX_MAILBOX0) != HAL_OK) 
		{
			if(HAL_CAN_AddTxMessage(&hcan1, &txMsg, tx_data, (uint32_t*)CAN_TX_MAILBOX1) != HAL_OK)
			{
				HAL_CAN_AddTxMessage(&hcan1, &txMsg, tx_data, (uint32_t*)CAN_TX_MAILBOX2);
       }
		} 
//	can_txd();
}

/**
  * @brief          ���õ��ģʽ(����ֹͣʱ������)
  * @param[in]      Motor:  ����ṹ��
  * @param[in]      Mode:   �������ģʽ��1.�˶�ģʽMotion_mode 2. λ��ģʽPosition_mode 3. �ٶ�ģʽSpeed_mode 4. ����ģʽCurrent_mode��
  * @retval         none
  */
void set_mode_cybergear(MI_Motor *Motor,uint8_t Mode)
{	
	Set_Motor_Parameter(Motor,Run_mode,Mode,'s');
}

/**
  * @brief          ��������ģʽ�����õ���
  * @param[in]      Motor:  ����ṹ��
  * @param[in]      Current:��������
  * @retval         none
  */
void set_current_cybergear(MI_Motor *Motor,float Current)
{
	Set_Motor_Parameter(Motor,Iq_Ref,Current,'f');
}

/**
  * @brief          ���õ�����
  * @param[in]      Motor:  ����ṹ��
  * @retval         none
  */
void set_zeropos_cybergear(MI_Motor *Motor)
{
	uint8_t tx_data[8]={0};
	txMsg.ExtId = Communication_Type_SetPosZero<<24|Master_CAN_ID<<8|Motor->CAN_ID;
  	//Ѱ�����䷢������
		if(HAL_CAN_AddTxMessage(&hcan1, &txMsg, tx_data, (uint32_t*)CAN_TX_MAILBOX0) != HAL_OK) 
		{
			if(HAL_CAN_AddTxMessage(&hcan1, &txMsg, tx_data, (uint32_t*)CAN_TX_MAILBOX1) != HAL_OK)
			{
				HAL_CAN_AddTxMessage(&hcan1, &txMsg, tx_data, (uint32_t*)CAN_TX_MAILBOX2);
       }
		} 
//	can_txd();		
}

/**
  * @brief          ���õ��CANID
  * @param[in]      Motor:  ����ṹ��
  * @param[in]      Motor:  ������ID
  * @retval         none
  */
void set_CANID_cybergear(MI_Motor *Motor,uint8_t CAN_ID)
{
	uint8_t tx_data[8]={0};
	txMsg.ExtId = Communication_Type_CanID<<24|CAN_ID<<16|Master_CAN_ID<<8|Motor->CAN_ID;
    Motor->CAN_ID = CAN_ID;//���µ�ID�������ṹ��
  	//Ѱ�����䷢������
		if(HAL_CAN_AddTxMessage(&hcan1, &txMsg, tx_data, (uint32_t*)CAN_TX_MAILBOX0) != HAL_OK) 
		{
			if(HAL_CAN_AddTxMessage(&hcan1, &txMsg, tx_data, (uint32_t*)CAN_TX_MAILBOX1) != HAL_OK)
			{
				HAL_CAN_AddTxMessage(&hcan1, &txMsg, tx_data, (uint32_t*)CAN_TX_MAILBOX2);
       }
		}     
//	can_txd();	
}
/**
  * @brief          С�׵����ʼ��
  * @param[in]      Motor:  ����ṹ��
  * @param[in]      Can_Id: С�׵��ID(Ĭ��0x7F)
  * @param[in]      Motor_Num: ������
  * @param[in]      mode: �������ģʽ��0.�˶�ģʽMotion_mode 1. λ��ģʽPosition_mode 2. �ٶ�ģʽSpeed_mode 3. ����ģʽCurrent_mode��
  * @retval         none
  */
void init_cybergear(MI_Motor *Motor,uint8_t Can_Id, uint8_t mode)
{
    txMsg.StdId = 0;            //����CAN���ͣ���׼֡���� 
    txMsg.ExtId = 0;            //����CAN���ͣ���չ֡����     
    txMsg.IDE = CAN_ID_EXT;     //����CAN���ͣ���չ֡
    txMsg.RTR = CAN_RTR_DATA;   //����CAN���ͣ�����֡
    txMsg.DLC = 0x08;           //����CAN���ͣ����ݳ���
    
	Motor->CAN_ID=Can_Id;       //ID���� 
	set_mode_cybergear(Motor,mode);//���õ��ģʽ
	start_cybergear(Motor);        //ʹ�ܵ��
}

/**
  * @brief          С���˿�ģʽָ��
  * @param[in]      Motor:  Ŀ�����ṹ��
  * @param[in]      torque: ��������[-12,12] N*M
  * @param[in]      MechPosition: λ������[-12.5,12.5] rad
  * @param[in]      speed: �ٶ�����[-30,30] rpm
  * @param[in]      kp: ������������
  * @param[in]      kd: ΢�ֲ�������
  * @retval         none
  */
void motor_controlmode(MI_Motor *Motor,float torque, float MechPosition, float speed, float kp, float kd)
{   
    uint8_t tx_data[8];//�������ݳ�ʼ��
    //װ�������
    tx_data[0]=float_to_uint(MechPosition,P_MIN,P_MAX,16)>>8;  
    tx_data[1]=float_to_uint(MechPosition,P_MIN,P_MAX,16);  
    tx_data[2]=float_to_uint(speed,V_MIN,V_MAX,16)>>8;  
    tx_data[3]=float_to_uint(speed,V_MIN,V_MAX,16);  
    tx_data[4]=float_to_uint(kp,KP_MIN,KP_MAX,16)>>8;  
    tx_data[5]=float_to_uint(kp,KP_MIN,KP_MAX,16);  
    tx_data[6]=float_to_uint(kd,KD_MIN,KD_MAX,16)>>8;  
    tx_data[7]=float_to_uint(kd,KD_MIN,KD_MAX,16); 
    
    txMsg.ExtId = Communication_Type_MotionControl<<24|float_to_uint(torque,XIAOMI_T_MIN,XIAOMI_T_MAX,16)<<8|Motor->CAN_ID;//װ����չ֡����
  	//Ѱ�����䷢������
		if(HAL_CAN_AddTxMessage(&hcan1, &txMsg, tx_data, (uint32_t*)CAN_TX_MAILBOX0) != HAL_OK) 
		{
			if(HAL_CAN_AddTxMessage(&hcan1, &txMsg, tx_data, (uint32_t*)CAN_TX_MAILBOX1) != HAL_OK)
			{
				HAL_CAN_AddTxMessage(&hcan1, &txMsg, tx_data, (uint32_t*)CAN_TX_MAILBOX2);
       }
		}  
//	can_txd();
}

/**
  * @brief          (��д)С���˿�ģʽָ��
  * @param[in]      Motor:  Ŀ�����ṹ��
  * @param[in]      torque: ��������[-12,12] N*M
  * @param[in]      MechPosition: λ������[-12.5,12.5] rad
  * @param[in]      speed: �ٶ�����[-30,30] rpm	//CAN_MI_M1_ID
  * @param[in]      kp: ������������
  * @param[in]      kd: ΢�ֲ�������
  * @retval         none
  */
void MI_Motor_mit_ctrl(hcan_t* hcan, uint16_t motor_id, float pos, float vel,float kp, float kd, float torq)
{
	    uint8_t tx_data[8];//�������ݳ�ʼ��
    //װ�������
    tx_data[0]=float_to_uint(pos,P_MIN,P_MAX,16)>>8;  
    tx_data[1]=float_to_uint(pos,P_MIN,P_MAX,16);  
    tx_data[2]=float_to_uint(vel,V_MIN,V_MAX,16)>>8;  
    tx_data[3]=float_to_uint(vel,V_MIN,V_MAX,16);  
    tx_data[4]=float_to_uint(kp,KP_MIN,KP_MAX,16)>>8;  
    tx_data[5]=float_to_uint(kp,KP_MIN,KP_MAX,16);  
    tx_data[6]=float_to_uint(kd,KD_MIN,KD_MAX,16)>>8;  
    tx_data[7]=float_to_uint(kd,KD_MIN,KD_MAX,16); 
    txMsg.ExtId = Communication_Type_MotionControl<<24|float_to_uint(torq,XIAOMI_T_MIN,XIAOMI_T_MAX,16)<<8|motor_id;//װ����չ֡����
  	//Ѱ�����䷢������
		if(HAL_CAN_AddTxMessage(&hcan1, &txMsg, tx_data, (uint32_t*)CAN_TX_MAILBOX0) != HAL_OK) 
		{
			if(HAL_CAN_AddTxMessage(&hcan1, &txMsg, tx_data, (uint32_t*)CAN_TX_MAILBOX1) != HAL_OK)
			{
				HAL_CAN_AddTxMessage(&hcan1, &txMsg, tx_data, (uint32_t*)CAN_TX_MAILBOX2);
       }
		} 
	
}



void CyberGear_data(Wheel_Motor_t *motor, uint8_t *rx_data , uint32_t data_len ,uint32_t ExtId)
{

						motor->para.Angle=uint_to_float(rx_data[0]<<8|rx_data[1],MIN_P,MAX_P,16);
						motor->para.Speed=uint_to_float(rx_data[2]<<8|rx_data[3],V_MIN,V_MAX,16);			
						motor->para.Torque=uint_to_float(rx_data[4]<<8|rx_data[5],XIAOMI_T_MIN,XIAOMI_T_MAX,16);				
						motor->para.Temp=(rx_data[6]<<8|rx_data[7])*Temp_Gain;
						motor->para.error_code=(ExtId&0x1F0000)>>16;
						//detect_hook(CHASSIS_MOTOR1_TOE);
					
	
}






///*****************************�ص����� ����ӻش���Ϣ ��ת������*****************************/
///**
//  * @brief          hal��CAN�ص�����,���յ������
//  * @param[in]      hcan:CAN���ָ��
//  * @retval         none
//  */
//uint8_t printf_flag;
//void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
//{
//	
//   HAL_GPIO_TogglePin(LED_G_GPIO_Port,LED_G_Pin);              //LED��˸ָʾ
//   HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxMsg, rx_data);//��������
//	Motor_Can_ID=Get_Motor_ID(rxMsg.ExtId);//���Ȼ�ȡ�ش����ID��Ϣ  
//    switch(Motor_Can_ID)                   //����ӦID�����Ϣ��ȡ����Ӧ�ṹ��
//    {
//        case 0X7F:
//					printf("0x7F_M_ID:%d\r\n",rxMsg.ExtId>>24);
//					
//            if(rxMsg.ExtId>>24 != 0){
//							printf_flag=1;//����Ƿ�Ϊ�㲥ģʽ
//              Motor_Data_Handler(&mi_motor[0],rx_data,rxMsg.ExtId);
//						}
//            else 
//              mi_motor[0].MCU_ID = rx_data[0];
//            break; 
//				case 0X7E:
//					printf("0x7E_M_ID:%d\r\n",rxMsg.ExtId>>24);
//            if(rxMsg.ExtId>>24 != 0){
//							printf_flag=1;//����Ƿ�Ϊ�㲥ģʽ
//              Motor_Data_Handler(&mi_motor[1],rx_data,rxMsg.ExtId);
//						}
//            else 
//              mi_motor[1].MCU_ID = rx_data[0];
//            break; 
//        default:
//            break;		
//    }
//}
