//=====================================================================================================
// MahonyAHRS.c
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

//---------------------------------------------------------------------------------------------------
// Header files

#include "MahonyAHRS.h"
#include <math.h>

//---------------------------------------------------------------------------------------------------
// Definitions

#define sampleFreq	1000.0f			// sample frequency in Hz
#define twoKpDef	(100.0f * 0.5f)	// 2 * proportional gain
#define twoKiDef	(100.0f * 0.0f)	// 2 * integral gain

//---------------------------------------------------------------------------------------------------
// Variable definitions

volatile float twoKp = twoKpDef;				 							// 2 * proportional gain (Kp)
volatile float twoKi = twoKiDef;											// 2 * integral gain (Ki)
//volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;					// quaternion of sensor frame relative to auxiliary frame
volatile float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;	// integral error terms scaled by Ki

//---------------------------------------------------------------------------------------------------
// Function declarations

float invSqrt(float x);

//====================================================================================================
// Functions

//---------------------------------------------------------------------------------------------------
// AHRS algorithm update
/**
  * @brief          用于更新四元数姿态估计
  * @param[in]     	 q[4]：四元数数组，表示当前姿态
											gx, gy, gz：陀螺仪测量的角速度，单位为弧度每秒
											ax, ay, az：加速度计测量的加速度，单位为g（重力加速度）
											mx, my, mz：磁力计测量的磁场强度，单位为高斯
  * @retval       	  none
  */
void MahonyAHRSupdate(float q[4], float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
	float recipNorm;																									//用于存储归一化因子
  float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;  //四元数的乘积项
	float hx, hy, bx, bz;																							//磁场方向的辅助变量
	float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;								//半角速度和半角加速度
	float halfex, halfey, halfez;																			//误差项的一半
	float qa, qb, qc;																										//四元数的临时变量

	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
		//若磁力计测量无效（所有值为0），则使用IMU蒜贩更新姿态
	if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
		MahonyAHRSupdateIMU(q, gx, gy, gz, ax, ay, az);
		return;
	}

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
		//如果加速度测量有值，则计算反馈
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
			//归一化加速度计测量值
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;     

		// Normalise magnetometer measurement
			//归一化磁力计测量值
		recipNorm = invSqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;   

        // Auxiliary variables to avoid repeated arithmetic
					//计算四元数乘积的辅助变量，避免重复计算
        q0q0 = q[0] * q[0];
        q0q1 = q[0] * q[1];
        q0q2 = q[0] * q[2];
        q0q3 = q[0] * q[3];
        q1q1 = q[1] * q[1];
        q1q2 = q[1] * q[2];
        q1q3 = q[1] * q[3];
        q2q2 = q[2] * q[2];
        q2q3 = q[2] * q[3];
        q3q3 = q[3] * q[3];   
					//其他四元数乘积项的计算

        // Reference direction of Earth's magnetic field
					//计算地磁场的参考方向
        hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
        hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
        bx = sqrt(hx * hx + hy * hy);
        bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

		// Estimated direction of gravity and magnetic field
				//计算估计的中立和磁场方向
		halfvx = q1q3 - q0q2;
		halfvy = q0q1 + q2q3;
		halfvz = q0q0 - 0.5f + q3q3;
        halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
        halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
        halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);  
	
		// Error is sum of cross product between estimated direction and measured direction of field vectors
			//计算误差项，即估计方向与测量方向的叉乘之和
		halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
		halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
		halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

		// Compute and apply integral feedback if enabled
		// 如果启用了积分反馈，则计算并应用积分反馈
		if(twoKi > 0.0f) {
			integralFBx += twoKi * halfex * (1.0f / sampleFreq);	// integral error scaled by Ki
			integralFBy += twoKi * halfey * (1.0f / sampleFreq);
			integralFBz += twoKi * halfez * (1.0f / sampleFreq);
			gx += integralFBx;	// apply integral feedback
			gy += integralFBy;
			gz += integralFBz;
		}
		else {// 防止积分饱和
			
			integralFBx = 0.0f;	// prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// Apply proportional feedback
		// 应用比例反馈
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}
	
	// Integrate rate of change of quaternion
	// 积分四元数变化率
	gx *= (0.5f * (1.0f / sampleFreq));		// pre-multiply common factors
	gy *= (0.5f * (1.0f / sampleFreq));
	gz *= (0.5f * (1.0f / sampleFreq));
	// 更新四元数
	qa = q[0];
	qb = q[1];
	qc = q[2];
	q[0] += (-qb * gx - qc * gy - q[3] * gz);
	q[1] += (qa * gx + qc * gz - q[3] * gy);
	q[2] += (qa * gy - qb * gz + q[3] * gx);
	q[3] += (qa * gz + qb * gy - qc * gx); 
	
	// Normalise quaternion
	// 归一化四元数
	recipNorm = invSqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
	q[0] *= recipNorm;
	q[1] *= recipNorm;
	q[2] *= recipNorm;
	q[3] *= recipNorm;
}

//---------------------------------------------------------------------------------------------------
// IMU algorithm update

void MahonyAHRSupdateIMU(float q[4], float gx, float gy, float gz, float ax, float ay, float az) {
	float recipNorm;
	float halfvx, halfvy, halfvz;
	float halfex, halfey, halfez;
	float qa, qb, qc;

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;        

		// Estimated direction of gravity and vector perpendicular to magnetic flux
		halfvx = q[1] * q[3] - q[0] * q[2];
		halfvy = q[0] * q[1] + q[2] * q[3];
		halfvz = q[0] * q[0] - 0.5f + q[3] * q[3];
	
		// Error is sum of cross product between estimated and measured direction of gravity
		halfex = (ay * halfvz - az * halfvy);
		halfey = (az * halfvx - ax * halfvz);
		halfez = (ax * halfvy - ay * halfvx);

		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			integralFBx += twoKi * halfex * (1.0f / sampleFreq);	// integral error scaled by Ki
			integralFBy += twoKi * halfey * (1.0f / sampleFreq);
			integralFBz += twoKi * halfez * (1.0f / sampleFreq);
			gx += integralFBx;	// apply integral feedback
			gy += integralFBy;
			gz += integralFBz;
		}
		else {
			integralFBx = 0.0f;	// prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// Apply proportional feedback
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}
	
	// Integrate rate of change of quaternion
	gx *= (0.5f * (1.0f / sampleFreq));		// pre-multiply common factors
	gy *= (0.5f * (1.0f / sampleFreq));
	gz *= (0.5f * (1.0f / sampleFreq));
	qa = q[0];
	qb = q[1];
	qc = q[2];
	q[0] += (-qb * gx - qc * gy - q[3] * gz);
	q[1] += (qa * gx + qc * gz - q[3] * gy);
	q[2] += (qa * gy - qb * gz + q[3] * gx);
	q[3] += (qa * gz + qb * gy - qc * gx); 
	
	// Normalise quaternion
	recipNorm = invSqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
	q[0] *= recipNorm;
	q[1] *= recipNorm;
	q[2] *= recipNorm;
	q[3] *= recipNorm;
}

//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

//====================================================================================================
// END OF CODE
//====================================================================================================

//====================================================================================================
// THIS IS MY TURN QwQ!!
//====================================================================================================



void AHRS_init(float quat[4], float accel[3], float mag[3])
{
    quat[0] = 1.0f;
    quat[1] = 0.0f;
    quat[2] = 0.0f;
    quat[3] = 0.0f;

}

void AHRS_update(float quat[4], float time, float gyro[3], float accel[3], float mag[3])
{
    MahonyAHRSupdate(quat, gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2], mag[0], mag[1], mag[2]);
}





struct MAHONY_FILTER_t mahony_filter;

/*计算旋转矩阵*///大地坐标系 R 转换到机体坐标系 b 的坐标转换矩阵
void RotationMatrix_update(struct MAHONY_FILTER_t *mahony_filter)
{
    float q1q1 = mahony_filter->q1 * mahony_filter->q1;
    float q2q2 = mahony_filter->q2 * mahony_filter->q2;
    float q3q3 = mahony_filter->q3 * mahony_filter->q3;

    float q0q1 = mahony_filter->q0 * mahony_filter->q1;
    float q0q2 = mahony_filter->q0 * mahony_filter->q2;
    float q0q3 = mahony_filter->q0 * mahony_filter->q3;
    float q1q2 = mahony_filter->q1 * mahony_filter->q2;
    float q1q3 = mahony_filter->q1 * mahony_filter->q3;
    float q2q3 = mahony_filter->q2 * mahony_filter->q3;

    mahony_filter->rMat[0][0] = 1.0f - 2.0f * q2q2 - 2.0f * q3q3;
    mahony_filter->rMat[0][1] = 2.0f * (q1q2  -q0q3);
    mahony_filter->rMat[0][2] = 2.0f * (q1q3 +q0q2);

    mahony_filter->rMat[1][0] = 2.0f * (q1q2 +q0q3);
    mahony_filter->rMat[1][1] = 1.0f - 2.0f * q1q1 - 2.0f * q3q3;
    mahony_filter->rMat[1][2] = 2.0f * (q2q3  -q0q1);

    mahony_filter->rMat[2][0] = 2.0f * (q1q3  -q0q2);
    mahony_filter->rMat[2][1] = 2.0f * (q2q3  +q0q1);
    mahony_filter->rMat[2][2] = 1.0f - 2.0f * q1q1 - 2.0f * q2q2;
}

// 将陀螺仪和加速度计的输入值赋予滤波器结构体
void mahony_input(struct MAHONY_FILTER_t *mahony_filter,Axis3f gyro,Axis3f acc)
{
    mahony_filter->gyro = gyro;
    mahony_filter->acc = acc;
}

// 使用Mahony算法更新滤波器状态
void mahony_update(struct MAHONY_FILTER_t *mahony_filter)
{
    float normalise;
    float ex,ey,ez;
    
    /*角速度，度转弧度*/
//    mahony_filter->gyro.x *= DEG2RAD;        /* 度转弧度 */
//    mahony_filter->gyro.y *= DEG2RAD;
//    mahony_filter->gyro.z *= DEG2RAD;;
       
    /*单位化加速计测量值*/
    normalise =sqrt(mahony_filter->acc.x * mahony_filter->acc.x 
										+mahony_filter->acc.y * mahony_filter->acc.y 
										+mahony_filter->acc.z * mahony_filter->acc.z);
	
    mahony_filter->acc.x =mahony_filter->acc.x/ normalise;
	mahony_filter->acc.y =mahony_filter->acc.y/normalise;   
    mahony_filter->acc.z =mahony_filter->acc.z/normalise;

    /*加速计读取的方向与重力加速计方向的差值，用向量叉乘计算*/
	//vx=mahony_filter->rMat[2][0]=2.0f * (q1q3 + -q0q2)
	//vy=mahony_filter->rMat[2][1]=2.0f * (q2q3 - -q0q1)
	//vz=mahony_filter->rMat[2][2]=1.0f - 2.0f * q1q1 - 2.0f * q2q2
    ex = (mahony_filter->acc.y * mahony_filter->rMat[2][2] - mahony_filter->acc.z * mahony_filter->rMat[2][1]);
    ey = (mahony_filter->acc.z * mahony_filter->rMat[2][0] - mahony_filter->acc.x * mahony_filter->rMat[2][2]);
    ez = (mahony_filter->acc.x * mahony_filter->rMat[2][1] - mahony_filter->acc.y * mahony_filter->rMat[2][0]);
    
    /*误差累计，与积分常数相乘*/
    mahony_filter->exInt += mahony_filter->Ki * ex * mahony_filter->dt ;  
    mahony_filter->eyInt += mahony_filter->Ki * ey * mahony_filter->dt ;
    mahony_filter->ezInt += mahony_filter->Ki * ez * mahony_filter->dt ;
    
    /*用叉积误差来做PI修正陀螺零偏，即抵消陀螺读数中的偏移量*/
    mahony_filter->gyro.x += mahony_filter->Kp * ex + mahony_filter->exInt;
    mahony_filter->gyro.y += mahony_filter->Kp * ey + mahony_filter->eyInt;
    mahony_filter->gyro.z += mahony_filter->Kp * ez + mahony_filter->ezInt;
    
    /* 一阶近似算法，四元数运动学方程的离散化形式和积分 */
    float q0Last = mahony_filter->q0;
    float q1Last = mahony_filter->q1;
    float q2Last = mahony_filter->q2;
    float q3Last = mahony_filter->q3;
    float halfT = mahony_filter->dt * 0.5f;
    mahony_filter->q0 += (-q1Last * mahony_filter->gyro.x - q2Last * mahony_filter->gyro.y - q3Last * mahony_filter->gyro.z) * halfT;
    mahony_filter->q1 += ( q0Last * mahony_filter->gyro.x + q2Last * mahony_filter->gyro.z - q3Last * mahony_filter->gyro.y) * halfT;
    mahony_filter->q2 += ( q0Last * mahony_filter->gyro.y - q1Last * mahony_filter->gyro.z + q3Last * mahony_filter->gyro.x) * halfT;
    mahony_filter->q3 += ( q0Last * mahony_filter->gyro.z + q1Last * mahony_filter->gyro.y - q2Last * mahony_filter->gyro.x) * halfT;
    
    /*单位化四元数*/
    normalise = sqrt(mahony_filter->q0 * mahony_filter->q0 
										+mahony_filter->q1 * mahony_filter->q1 
										+mahony_filter->q2 * mahony_filter->q2 
										+mahony_filter->q3 * mahony_filter->q3);
												
    mahony_filter->q0 = mahony_filter->q0/normalise;
    mahony_filter->q1 = mahony_filter->q1/normalise;
    mahony_filter->q2 = mahony_filter->q2/normalise;
    mahony_filter->q3 = mahony_filter->q3/normalise;
    
    /*计算旋转矩阵*/
    mahony_filter->RotationMatrix_update(mahony_filter);
}

// 从旋转矩阵中提取姿态角
void mahony_output(struct MAHONY_FILTER_t *mahony_filter)
{
    /*计算roll pitch yaw 欧拉角*/
    mahony_filter->pitch = -asinf(mahony_filter->rMat[2][0]); 
    mahony_filter->roll = atan2f(mahony_filter->rMat[2][1], mahony_filter->rMat[2][2]) ;
    mahony_filter->yaw = atan2f(mahony_filter->rMat[1][0], mahony_filter->rMat[0][0]) ;
}

// 初始化Mahony滤波器的参数和函数指针
void mahony_init(struct MAHONY_FILTER_t *mahony_filter,float Kp,float Ki,float dt)
{
    mahony_filter->Kp = Kp;
    mahony_filter->Ki = Ki;
    mahony_filter->dt = dt;
    mahony_filter->q0 = 1;
    mahony_filter->mahony_input = mahony_input;
    mahony_filter->mahony_update = mahony_update;
    mahony_filter->mahony_output = mahony_output;    
    mahony_filter->RotationMatrix_update = RotationMatrix_update;
}







