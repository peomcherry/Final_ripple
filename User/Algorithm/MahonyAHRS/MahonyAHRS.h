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
���ʱ�䣺2023��09��02�� 
���ܽ��ܣ�ʵ��mahony��̬�ǽ����㷨��ģ���װ
֪���˺ţ�����Ҳ
Bվ�˺ţ�����С����
***************************************/

#define DEG2RAD 0.0174533f
#define RAD2DEG 57.295671f

typedef struct Axis3f_t
{
  float x;
  float y;
  float z;
}Axis3f;


// ���� MAHONY_FILTER_t �ṹ�壬���ڷ�װ Mahony �˲��������ݺͺ���
struct MAHONY_FILTER_t
{
    // �������
    float Kp, Ki;          // �����ͻ�������
    float dt;              // ����ʱ����
    Axis3f  gyro, acc;     // �����Ǻͼ��ٶȼ�����

    // ���̲���
    float exInt, eyInt, ezInt;                // ��������ۼ�
    float q0, q1, q2, q3;            // ��Ԫ��
    float rMat[3][3];               // ��ת����

    // �������
    float pitch, roll, yaw;         // ��̬�ǣ������ǣ���ת�ǣ�ƫ����

    // ����ָ��
    void (*mahony_init)(struct MAHONY_FILTER_t *mahony_filter, float Kp, float Ki, float dt);
    void (*mahony_input)(struct MAHONY_FILTER_t *mahony_filter, Axis3f gyro, Axis3f acc);
    void (*mahony_update)(struct MAHONY_FILTER_t *mahony_filter);
    void (*mahony_output)(struct MAHONY_FILTER_t *mahony_filter);
    void (*RotationMatrix_update)(struct MAHONY_FILTER_t *mahony_filter);
};

// ��������
void mahony_init(struct MAHONY_FILTER_t *mahony_filter, float Kp, float Ki, float dt);          // ��ʼ������
void mahony_input(struct MAHONY_FILTER_t *mahony_filter, Axis3f gyro, Axis3f acc);              // �������ݺ���
void mahony_update(struct MAHONY_FILTER_t *mahony_filter);                                      // �����˲�������
void mahony_output(struct MAHONY_FILTER_t *mahony_filter);                                      // �����̬�Ǻ���
void RotationMatrix_update(struct MAHONY_FILTER_t *mahony_filter);                              // ������ת������





void AHRS_init(float quat[4], float accel[3], float mag[3]);
void AHRS_update(float quat[4], float time, float gyro[3], float accel[3], float mag[3]);









#endif
//=====================================================================================================
// End of file
//=====================================================================================================
