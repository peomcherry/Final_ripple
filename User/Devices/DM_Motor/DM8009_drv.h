#ifndef __DM8009_DRV_H__
#define __DM8009_DRV_H__
#include "main.h"
#include "can.h"
#include "bsp_can.h"


#define Motar_mode 0	//设置模式为何种模式，为0为IMT模式，为1为位置速度模式，为2为速度模式

#define MIT_MODE 			0x000
#define POS_MODE			0x100
#define SPEED_MODE		0x200

#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -30.0f
#define V_MAX 30.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f

#define DM_T_MIN -10.0f
#define DM_T_MAX 10.0f

#define P_MIN2 -12.0f
#define P_MAX2 12.0f
#define V_MIN2 -45.0f
#define V_MAX2 45.0f
#define KP_MIN2 0.0f
#define KP_MAX2 500.0f
#define KD_MIN2 0.0f
#define KD_MAX2 5.0f
#define T_MIN2 -18.0f
#define T_MAX2 18.0f


#define STD_CAN_ID 0
#define EXT_CAN_ID 1

#define DM8009_1_MASTER_ID	0x08			//DM电机1_MASTER_ID接收模式下的ID
#define DM8009_1_CAN_ID 		0x01      //DM电机1_CAN_ID	,MIT模式下控制帧
#define DM8009_2_MASTER_ID	0x02			//DM电机2_MASTER_ID接收模式下的ID
#define DM8009_2_CAN_ID 		0x03      //DM电机2_CAN_ID
#define DM8009_3_MASTER_ID	0x04			//DM电机3_MASTER_ID接收模式下的ID
#define DM8009_3_CAN_ID 		0x05      //DM电机3_CAN_ID
#define DM8009_4_MASTER_ID	0x06			//DM电机4_MASTER_ID接收模式下的ID
#define DM8009_4_CAN_ID 		0x07      //DM电机4_CAN_ID


#define DM8009_1_POS_KP 20      //电机1位置KP
#define DM8009_1_POS_KD 1      //电机1位置KD
#define DM8009_2_POS_KP 20      //电机1位置KP
#define DM8009_2_POS_KD 1      //电机1位置KD
#define DM8009_3_POS_KP 20      //电机1位置KP
#define DM8009_3_POS_KD 1      //电机1位置KD
#define DM8009_4_POS_KP 20      //电机1位置KP
#define DM8009_4_POS_KD 1      //电机1位置KD


typedef struct 
{
	uint16_t id;
	uint16_t state;
	int p_int;
	int v_int;
	int t_int;
	int kp_int;
	int kd_int;
	float pos;
	float vel;
	float tor;
	float Kp;
	float Kd;
	float Tmos;
	float Tcoil;
}motor_fbpara_t;


typedef struct
{
	uint16_t mode;
	motor_fbpara_t para;
}Joint_Motor_t ;

//这个结构体先注释掉
//typedef struct
//{
//	uint16_t mode;
//	float wheel_T;//轮毂电机的输出扭矩，单位为N
//	
//	motor_fbpara_t para;	//此结构体为达妙电机专用
//}Wheel_Motor_t ;		


extern void dm8009_data(Joint_Motor_t *motor, uint8_t *rx_data,uint32_t data_len);			
extern void dm4310_fbdata(Joint_Motor_t *motor, uint8_t *rx_data,uint32_t data_len);		
//extern void dm6215_fbdata(Wheel_Motor_t *motor, uint8_t *rx_data,uint32_t data_len);	

extern void enable_motor_mode(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id);
extern void disable_motor_mode(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id);

//关节电机
extern void DM_Motor_mit_ctrl(hcan_t* hcan, uint16_t motor_id, float pos, float vel,float kp, float kd, float torq);
extern void DM_Motor_pos_speed_ctrl(hcan_t* hcan,uint16_t motor_id, float pos, float vel);
extern void DM_Motor_speed_ctrl(hcan_t* hcan,uint16_t motor_id, float _vel);

//轮毂电机MIT模式发送命令
extern void DM_Motor_mit_ctrl2(hcan_t* hcan, uint16_t motor_id, float pos, float vel,float kp, float kd, float torq);

extern void joint_motor_init(Joint_Motor_t *motor,uint16_t id,uint16_t mode);
//extern void wheel_motor_init(Wheel_Motor_t *motor,uint16_t id,uint16_t mode);
	
extern float Hex_To_Float(uint32_t *Byte,int num);//十六进制到浮点数
extern uint32_t FloatTohex(float HEX);//浮点数到十六进制转换

extern float uint_to_float(int x_int, float x_min, float x_max, int bits);
extern int float_to_uint(float x_float, float x_min, float x_max, int bits);


extern void save_motor_zero(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id);

void Joint_Motor_pos_control(float pos);



#endif /* __DM4310_DRV_H__ */



