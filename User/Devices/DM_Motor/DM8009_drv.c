#include "DM8009_drv.h"


#include "can.h"
#include "arm_math.h"


#define M_PI 3.1415926f

///////////////////////////�û������Ĵ��/////////////////////////////////////////////////////


char Selection=0;

uint8_t Data_Enable[8]={0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};		//���ʹ������
uint8_t Data_Failure[8]={0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};		//���ʧ������
uint8_t Data_Save_zero[8]={0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};	//��������������

// ���彫�Ƕ�ת��Ϊ���ȵĺ���
float degreesToRadians(float degrees) {
    return degrees * M_PI / 180.0f;
}
// ���彫����ת��Ϊ�Ƕȵĺ�����ʹ�� float ����
float radiansToDegrees(float radians) {
    return radians * 180.0f / M_PI;
}


///////////////////////////���ﶼ�ǽ���ת������/////////////////////////////////////////////////////

float Hex_To_Float(uint32_t *Byte,int num)//ʮ�����Ƶ�������
{
	return *((float*)Byte);
}

uint32_t FloatTohex(float HEX)//��������ʮ������ת��
{
	return *( uint32_t *)&HEX;
}

/**
************************************************************************
* @brief:      	float_to_uint: ������ת��Ϊ�޷�����������
* @param[in]:   x_float:	��ת���ĸ�����
* @param[in]:   x_min:		��Χ��Сֵ
* @param[in]:   x_max:		��Χ���ֵ
* @param[in]:   bits: 		Ŀ���޷���������λ��
* @retval:     	�޷����������
* @details:    	�������ĸ����� x ��ָ����Χ [x_min, x_max] �ڽ�������ӳ�䣬ӳ����Ϊһ��ָ��λ�����޷�������
************************************************************************
**/
int float_to_uint(float x_float, float x_min, float x_max, int bits)
{
	/* Converts a float to an unsigned int, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return (int) ((x_float-offset)*((float)((1<<bits)-1))/span);
}
/**
************************************************************************
* @brief:      	uint_to_float: �޷�������ת��Ϊ����������
* @param[in]:   x_int: ��ת�����޷�������
* @param[in]:   x_min: ��Χ��Сֵ
* @param[in]:   x_max: ��Χ���ֵ
* @param[in]:   bits:  �޷���������λ��
* @retval:     	���������
* @details:    	���������޷������� x_int ��ָ����Χ [x_min, x_max] �ڽ�������ӳ�䣬ӳ����Ϊһ��������
************************************************************************
**/
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
	/* converts unsigned int to float, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

void joint_motor_init(Joint_Motor_t *motor,uint16_t id,uint16_t mode)
{
  motor->mode=mode;
  motor->para.id=id;
}

//void wheel_motor_init(Wheel_Motor_t *motor,uint16_t id,uint16_t mode)
//{
//  motor->mode=mode;
//  motor->para.id=id;
//}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
/********************************************************************************************************
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////���ﶼ�ǵ����Ϣ��ȡ������������//////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
**********************************************************************************************************/
//////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
************************************************************************
* @brief:      	dm8009_data: ��ȡDM8009����������ݺ���
* @param[in]:   motor:    ָ��motor_t�ṹ��ָ�룬������������Ϣ�ͷ�������
* @param[in]:   rx_data:  ָ������������ݵ�����ָ��
* @param[in]:   data_len: ���ݳ���
* @retval:     	void
* @details:    	�ӽ��յ�����������ȡDM4310����ķ�����Ϣ���������ID��
*               ״̬��λ�á��ٶȡ�Ť������¶Ȳ������Ĵ������ݵ�
************************************************************************
**/
void dm8009_data(Joint_Motor_t *motor, uint8_t *rx_data,uint32_t data_len)
{ 
	if(data_len == ((uint32_t)0x00080000U))
	{//���ص�������8���ֽ�
//	  motor->para.id = (rx_data[0])&0x0F;
//	  motor->para.state = (rx_data[0])>>4;
		
	  motor->para.p_int=(rx_data[1]<<8)|rx_data[2];
	  motor->para.v_int=(rx_data[3]<<4)|(rx_data[4]>>4);
	  motor->para.t_int=((rx_data[4]&0xF)<<8)|rx_data[5];
	  motor->para.pos = uint_to_float(motor->para.p_int, P_MIN, P_MAX, 16); // (-12.5,12.5)
	  motor->para.vel = uint_to_float(motor->para.v_int, V_MIN, V_MAX, 12); // (-30.0,30.0)
	  motor->para.tor = uint_to_float(motor->para.t_int, DM_T_MIN, DM_T_MAX, 12);  // (-10.0,10.0)

//	  motor->para.Tmos = (float)(rx_data[6]);
//	  motor->para.Tcoil = (float)(rx_data[7]);
	}
}


/**
************************************************************************
* @brief:      	dm4310_fbdata: ��ȡDM4310����������ݺ���
* @param[in]:   motor:    ָ��motor_t�ṹ��ָ�룬������������Ϣ�ͷ�������
* @param[in]:   rx_data:  ָ������������ݵ�����ָ��
* @param[in]:   data_len: ���ݳ���
* @retval:     	void
* @details:    	�ӽ��յ�����������ȡDM4310����ķ�����Ϣ���������ID��
*               ״̬��λ�á��ٶȡ�Ť������¶Ȳ������Ĵ������ݵ�
************************************************************************
**/
void dm4310_fbdata(Joint_Motor_t *motor, uint8_t *rx_data,uint32_t data_len)
{ 
	if(data_len==((uint32_t)0x00080000U))
	{//���ص�������8���ֽ�
	  motor->para.id = (rx_data[0])&0x0F;
	  motor->para.state = (rx_data[0])>>4;
	  motor->para.p_int=(rx_data[1]<<8)|rx_data[2];
	  motor->para.v_int=(rx_data[3]<<4)|(rx_data[4]>>4);
	  motor->para.t_int=((rx_data[4]&0xF)<<8)|rx_data[5];
	  motor->para.pos = uint_to_float(motor->para.p_int, P_MIN, P_MAX, 16); // (-12.5,12.5)
	  motor->para.vel = uint_to_float(motor->para.v_int, V_MIN, V_MAX, 12); // (-30.0,30.0)
	  motor->para.tor = uint_to_float(motor->para.t_int, DM_T_MIN, DM_T_MAX, 12);  // (-10.0,10.0)
	  motor->para.Tmos = (float)(rx_data[6]);
	  motor->para.Tcoil = (float)(rx_data[7]);
	}
}



/*****************************DM6215�����ú��������ʹ�����н����ӡ*******************************
void dm6215_fbdata(Wheel_Motor_t *motor, uint8_t *rx_data,uint32_t data_len)
{ 
	if(data_len==((uint32_t)0x00080000U))
	{//���ص�������8���ֽ�
	  motor->para.id = (rx_data[0])&0x0F;
	  motor->para.state = (rx_data[0])>>4;
	  motor->para.p_int=(rx_data[1]<<8)|rx_data[2];
	  motor->para.v_int=(rx_data[3]<<4)|(rx_data[4]>>4);
	  motor->para.t_int=((rx_data[4]&0xF)<<8)|rx_data[5];
	  motor->para.pos = uint_to_float(motor->para.p_int, P_MIN2, P_MAX2, 16); // (-12.0,12.0)
	  motor->para.vel = uint_to_float(motor->para.v_int, V_MIN2, V_MAX2, 12); // (-30.0,30.0)
	  motor->para.tor = uint_to_float(motor->para.t_int, T_MIN2, T_MAX2, 12);  // (-18.0,18.0)
	  motor->para.Tmos = (float)(rx_data[6]);
	  motor->para.Tcoil = (float)(rx_data[7]);
	}
}
*********************************************************************************/




/////////////////////////////////////////////////////////////////////////////////////////////////////////
/********************************************************************************************************
////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////�����Ƕ����е������ʹ�ܡ�ʧ�ܡ�����Ĳ���///////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////
**********************************************************************************************************/
//////////////////////////////////////////////////////////////////////////////////////////////////////////



void enable_motor_mode(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id)
{
	uint8_t data[8];
	uint16_t id = motor_id + mode_id;
	
	data[0] = 0xFF;
	data[1] = 0xFF;
	data[2] = 0xFF;
	data[3] = 0xFF;
	data[4] = 0xFF;
	data[5] = 0xFF;
	data[6] = 0xFF;
	data[7] = 0xFC;
	
	canx_send_data(hcan, id, data, 8 , STD_CAN_ID);
}

void save_motor_zero(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id)
{
	uint8_t data[8];
	uint16_t id = motor_id + mode_id;
	
	data[0] = 0xFF;
	data[1] = 0xFF;
	data[2] = 0xFF;
	data[3] = 0xFF;
	data[4] = 0xFF;
	data[5] = 0xFF;
	data[6] = 0xFF;
	data[7] = 0xFE;
	
	canx_send_data(hcan, id, data, 8, STD_CAN_ID);
}


/**
************************************************************************
* @brief:      	disable_motor_mode: ���õ��ģʽ����
* @param[in]:   hcan:     ָ��CAN_HandleTypeDef�ṹ��ָ��
* @param[in]:   motor_id: ���ID��ָ��Ŀ����
* @param[in]:   mode_id:  ģʽID��ָ��Ҫ���õ�ģʽ
* @retval:     	void
* @details:    	ͨ��CAN�������ض�������ͽ����ض�ģʽ������
************************************************************************
**/
void disable_motor_mode(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id)
{
	uint8_t data[8];
	uint16_t id = motor_id + mode_id;
	
	data[0] = 0xFF;
	data[1] = 0xFF;
	data[2] = 0xFF;
	data[3] = 0xFF;
	data[4] = 0xFF;
	data[5] = 0xFF;
	data[6] = 0xFF;
	data[7] = 0xFD;
	
	canx_send_data(hcan, id, data, 8, STD_CAN_ID);
}

/**
************************************************************************
* @brief:      	mit_ctrl: MITģʽ�µĵ�����ƺ���
* @param[in]:   hcan:			ָ��CAN_HandleTypeDef�ṹ��ָ�룬����ָ��CAN����
* @param[in]:   motor_id:	���ID��ָ��Ŀ����
* @param[in]:   pos:			λ�ø���ֵ
* @param[in]:   vel:			�ٶȸ���ֵ
* @param[in]:   kp:				λ�ñ���ϵ��
* @param[in]:   kd:				λ��΢��ϵ��
* @param[in]:   torq:			ת�ظ���ֵ
* @retval:     	void
* @details:    	ͨ��CAN������������MITģʽ�µĿ���֡��
************************************************************************
**/


/////////////////////////////////////////////////////////////////////////////////////////////////////////
/********************************************************************************************************
///////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////������MIT��λ�á��ٶ����ֿ���ģʽ����������//////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////
**********************************************************************************************************/
//////////////////////////////////////////////////////////////////////////////////////////////////////////





void DM_Motor_mit_ctrl(hcan_t* hcan, uint16_t motor_id, float pos, float vel,float kp, float kd, float torq)
{
	uint8_t data[8];
	uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
	uint16_t id = motor_id + MIT_MODE;

	pos_tmp = float_to_uint(pos,  P_MIN,  P_MAX,  16);
	vel_tmp = float_to_uint(vel,  V_MIN,  V_MAX,  12);
	kp_tmp  = float_to_uint(kp,   KP_MIN, KP_MAX, 12);
	kd_tmp  = float_to_uint(kd,   KD_MIN, KD_MAX, 12);
	tor_tmp = float_to_uint(torq, DM_T_MIN,  DM_T_MAX,  12);

	data[0] = (pos_tmp >> 8);
	data[1] = pos_tmp;
	data[2] = (vel_tmp >> 4);
	data[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
	data[4] = kp_tmp;
	data[5] = (kd_tmp >> 4);
	data[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
	data[7] = tor_tmp;
	
	canx_send_data(hcan, id, data, 8, STD_CAN_ID);
}
/**
************************************************************************
* @brief:      	pos_speed_ctrl: λ���ٶȿ��ƺ���
* @param[in]:   hcan:			ָ��CAN_HandleTypeDef�ṹ��ָ�룬����ָ��CAN����
* @param[in]:   motor_id:	���ID��ָ��Ŀ����
* @param[in]:   vel:			�ٶȸ���ֵ
* @retval:     	void
* @details:    	ͨ��CAN������������λ���ٶȿ�������
************************************************************************
**/
void DM_Motor_pos_speed_ctrl(hcan_t* hcan,uint16_t motor_id, float pos, float vel)
{
	uint16_t id;
	uint8_t *pbuf, *vbuf;
	uint8_t data[8];
	
	id = motor_id + POS_MODE;
	pbuf=(uint8_t*)&pos;
	vbuf=(uint8_t*)&vel;
	
	data[0] = *pbuf;
	data[1] = *(pbuf+1);
	data[2] = *(pbuf+2);
	data[3] = *(pbuf+3);

	data[4] = *vbuf;
	data[5] = *(vbuf+1);
	data[6] = *(vbuf+2);
	data[7] = *(vbuf+3);
	
	canx_send_data(hcan, id, data, 8, STD_CAN_ID);
}
/**
************************************************************************
* @brief:      	speed_ctrl: �ٶȿ��ƺ���
* @param[in]:   hcan: 		ָ��CAN_HandleTypeDef�ṹ��ָ�룬����ָ��CAN����
* @param[in]:   motor_id: ���ID��ָ��Ŀ����
* @param[in]:   vel: 			�ٶȸ���ֵ
* @retval:     	void
* @details:    	ͨ��CAN�������������ٶȿ�������
************************************************************************
**/
void DM_Motor_speed_ctrl(hcan_t* hcan,uint16_t motor_id, float vel)
{
	uint16_t id;
	uint8_t *vbuf;
	uint8_t data[4];
	
	id = motor_id + SPEED_MODE;
	vbuf=(uint8_t*)&vel;
	
	data[0] = *vbuf;
	data[1] = *(vbuf+1);
	data[2] = *(vbuf+2);
	data[3] = *(vbuf+3);
	
	canx_send_data(hcan, id, data, 4, STD_CAN_ID);
}



void DM_Motor_mit_ctrl2(hcan_t* hcan, uint16_t motor_id, float pos, float vel,float kp, float kd, float torq)
{
	uint8_t data[8];
	uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
	uint16_t id = motor_id + MIT_MODE;

	pos_tmp = float_to_uint(pos,  P_MIN2,  P_MAX2,  16);
	vel_tmp = float_to_uint(vel,  V_MIN2,  V_MAX2,  12);
	kp_tmp  = float_to_uint(kp,   KP_MIN2, KP_MAX2, 12);
	kd_tmp  = float_to_uint(kd,   KD_MIN2, KD_MAX2, 12);
	tor_tmp = float_to_uint(torq, T_MIN2,  T_MAX2,  12);

	data[0] = (pos_tmp >> 8);
	data[1] = pos_tmp;
	data[2] = (vel_tmp >> 4);
	data[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
	data[4] = kp_tmp;
	data[5] = (kd_tmp >> 4);
	data[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
	data[7] = tor_tmp;
	
	canx_send_data(hcan, id, data, 8 , STD_CAN_ID);
}

















/////////////////////////////////////////////////////////////////////////////////////////////////////////
/********************************************************************************************************
///////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////������һЩ��������ĺ���//////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////
**********************************************************************************************************/
//////////////////////////////////////////////////////////////////////////////////////////////////////////


void Joint_Motor_pos_control(float pos){
		#if Motar_mode==0
		switch(Selection)
		{
			case 0:
			{
				DM_Motor_mit_ctrl(&hcan1,DM8009_1_CAN_ID, degreesToRadians(-pos), 0,DM8009_1_POS_KP, DM8009_1_POS_KD, 0);	  //IMT����ģʽ��IDΪMOTOR_1_CAN_ID��������ٶȿ��Ʒ���
				Selection++;
				break;
			}
			case 1:
			{
				DM_Motor_mit_ctrl(&hcan1,DM8009_2_CAN_ID, degreesToRadians(-10), 0,DM8009_2_POS_KP, DM8009_2_POS_KD, 0);			//IMT����ģʽ��IDΪMOTOR_2_CAN_ID��������ٶȿ��Ʒ���
				Selection++;
				break;
			}
			case 2:
			{
				DM_Motor_mit_ctrl(&hcan1,DM8009_3_CAN_ID, degreesToRadians(-pos), 0,DM8009_3_POS_KP, DM8009_3_POS_KD, 0);			//IMT����ģʽ��IDΪMOTOR_3_CAN_ID��������ٶȿ��Ʒ���
				Selection++;
				break;
			}
			case 3:
			{
				DM_Motor_mit_ctrl(&hcan1,DM8009_4_CAN_ID, degreesToRadians(pos), 0,DM8009_4_POS_KP, DM8009_4_POS_KD, 0);			//IMT����ģʽ��IDΪMOTOR_4_CAN_ID��������ٶȿ��Ʒ���
				Selection=0;
				break;
			}
		}
	#elif Motar_mode==1
		switch(Selection)
		{
			case 0:
			{
				PosSpeed_CtrlMotor(&hcan1,0X102, 10, 5);	  //λ���ٶȿ���ģʽ��IDΪ0X01��������ٶȿ��Ʒ���
				Selection++;
				break;
			}
			case 1:
			{
				PosSpeed_CtrlMotor(&hcan1,0X103,  10, 5);			//λ���ٶȿ���ģʽ��IDΪ0X03��������ٶȿ��Ʒ���
				Selection++;
				break;
			}
			case 2:
			{
				PosSpeed_CtrlMotor(&hcan1,0X104,  10, 5);			//λ���ٶȿ���ģʽ��IDΪ0x05��������ٶȿ��Ʒ���
				Selection=0;
				break;
			}
		}
	#elif Motar_mode==2
		switch(Selection)
		{
			case 0:
			{
				Speed_CtrlMotor(&hcan1,0X202, 5);	  //�ٶȿ���ģʽ��IDΪ0X01��������ٶȿ��Ʒ���
				Selection++;
				break;
			}
			case 1:
			{
				Speed_CtrlMotor(&hcan1,0X203, 5);			//�ٶȿ���ģʽ��IDΪ0X03��������ٶȿ��Ʒ���
				Selection++;
				break;
			}
			case 2:
			{
				Speed_CtrlMotor(&hcan1,0X204, 5);			//�ٶȿ���ģʽ��IDΪ0x05��������ٶȿ��Ʒ���
				Selection=0;
				break;
			}
		}
	#endif

}

