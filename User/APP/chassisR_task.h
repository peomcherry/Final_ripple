#ifndef __CHASSISR_TASK_H
#define __CHASSISR_TASK_H

#include "main.h"
#include "DM8009_drv.h"
#include "CyberGear_drv.h"
#include "VMC_calc.h"
#include "pid.h"
#include "INS_task.h"

#define ROLL_PID_KP 140.0f
#define ROLL_PID_KI 0.0f 
#define ROLL_PID_KD 10.0f
#define ROLL_PID_MAX_OUT  100.0f
#define ROLL_PID_MAX_IOUT 0.0f

#define TP_PID_KP 20.0f
#define TP_PID_KI 0.0f 
#define TP_PID_KD 1.0f
#define TP_PID_MAX_OUT  2.0f
#define TP_PID_MAX_IOUT 0.0f

#define TURN_PID_KP 4.0f
#define TURN_PID_KI 0.0f 
#define TURN_PID_KD 0.4f
#define TURN_PID_MAX_OUT  1.0f//轮毂电机的额定扭矩
#define TURN_PID_MAX_IOUT 0.0f//






typedef struct
{
  Joint_Motor_t joint_motor[4];	//这个结构体的本质还是DM的电机，为了兼容稍微进行修改
  Wheel_Motor_t wheel_motor[2];
	//MI_Motor wheel_motor[2];			//这次权衡了一下选择改主函数，因为大量的赋值操作都是初始化和对扭矩/ID的控制
	
	float v_set;//期望速度，单位是m/s
	float x_set;//期望位置，单位是m
  float v_ramp_set;
	
	float turn_set;//期望yaw轴弧度
	float roll_set;	//期望roll轴弧度
	float roll_x;
	float phi_set;
	float theta_set;
	
	float leg_set;//期望腿长，单位是m
	float last_leg_set;

	float v_filter;//滤波后的车体速度，单位是m/s
	float x_filter;//滤波后的车体位置，单位是m
	
	float myPithR;
	float myPithGyroR;
	float myPithL;
	float myPithGyroL;
	float roll;
	float total_yaw;
	float theta_err;//两腿夹角误差
		
	float turn_T;//yaw轴补偿
	float roll_f0;//roll轴补偿
		
	float leg_tp;//防劈叉补偿
	
	uint8_t start_flag;//启动标志

	uint8_t jump_flag;//右腿跳跃标志
	uint8_t jump_flag2;//左腿跳跃标志
	
	uint8_t prejump_flag;//预跳跃标志
	uint8_t recover_flag;//一种情况下的倒地自起标志
	
} chassis_t;


extern void ChassisR_init(chassis_t *chassis,vmc_leg_t *vmc,pid_type_def *legr);
extern void ChassisR_task(void);
extern void Pensation_init(pid_type_def *roll,pid_type_def *Tp,pid_type_def *turn);
extern void mySaturate(float *in,float min,float max);
extern void chassisR_feedback_update(chassis_t *chassis,vmc_leg_t *vmc,INS_t *ins);
extern void chassisR_control_loop(chassis_t *chassis,vmc_leg_t *vmcr,INS_t *ins,float *LQR_K,pid_type_def *leg);

extern void chassisR_task(void);

#endif
