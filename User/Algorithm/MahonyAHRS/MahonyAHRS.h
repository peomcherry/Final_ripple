//=====================================================================================================
// MahonyAHRS.h
//=====================================================================================================
//
// Madgwick's implementation of Mayhony's AHRS algorithm.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author			Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=====================================================================================================
#ifndef MahonyAHRS_h
#define MahonyAHRS_h

//----------------------------------------------------------------------------------------------------
// Variable declaration

extern volatile float twoKp;			// 2 * proportional gain (Kp)
extern volatile float twoKi;			// 2 * integral gain (Ki)
extern volatile float q0, q1, q2, q3;	// quaternion of sensor frame relative to auxiliary frame

//---------------------------------------------------------------------------------------------------
// Function declarations

void MahonyAHRSupdate(float q[4], float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void MahonyAHRSupdateIMU(float q[4], float gx, float gy, float gz, float ax, float ay, float az);


//====================================================================================================
// THIS IS MY TURN QwQ!!
//====================================================================================================


#include "math.h"
#include "stdlib.h"
#include "stm32f4xx.h"
#include "arm_math.h"

/*************************************
完成时间：2023年09月02日 
功能介绍：实现mahony姿态角解算算法的模块封装
知乎账号：龙胆也
B站账号：华南小虎队
***************************************/

#define DEG2RAD 0.0174533f
#define RAD2DEG 57.295671f

typedef struct Axis3f_t
{
  float x;
  float y;
  float z;
}Axis3f;


// 定义 MAHONY_FILTER_t 结构体，用于封装 Mahony 滤波器的数据和函数
struct MAHONY_FILTER_t
{
    // 输入参数
    float Kp, Ki;          // 比例和积分增益
    float dt;              // 采样时间间隔
    Axis3f  gyro, acc;     // 陀螺仪和加速度计数据

    // 过程参数
    float exInt, eyInt, ezInt;                // 积分误差累计
    float q0, q1, q2, q3;            // 四元数
    float rMat[3][3];               // 旋转矩阵

    // 输出参数
    float pitch, roll, yaw;         // 姿态角：俯仰角，滚转角，偏航角

    // 函数指针
    void (*mahony_init)(struct MAHONY_FILTER_t *mahony_filter, float Kp, float Ki, float dt);
    void (*mahony_input)(struct MAHONY_FILTER_t *mahony_filter, Axis3f gyro, Axis3f acc);
    void (*mahony_update)(struct MAHONY_FILTER_t *mahony_filter);
    void (*mahony_output)(struct MAHONY_FILTER_t *mahony_filter);
    void (*RotationMatrix_update)(struct MAHONY_FILTER_t *mahony_filter);
};

// 函数声明
void mahony_init(struct MAHONY_FILTER_t *mahony_filter, float Kp, float Ki, float dt);          // 初始化函数
void mahony_input(struct MAHONY_FILTER_t *mahony_filter, Axis3f gyro, Axis3f acc);              // 输入数据函数
void mahony_update(struct MAHONY_FILTER_t *mahony_filter);                                      // 更新滤波器函数
void mahony_output(struct MAHONY_FILTER_t *mahony_filter);                                      // 输出姿态角函数
void RotationMatrix_update(struct MAHONY_FILTER_t *mahony_filter);                              // 更新旋转矩阵函数





void AHRS_init(float quat[4], float accel[3], float mag[3]);
void AHRS_update(float quat[4], float time, float gyro[3], float accel[3], float mag[3]);









#endif
//=====================================================================================================
// End of file
//=====================================================================================================
