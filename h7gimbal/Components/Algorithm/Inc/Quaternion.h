#ifndef QUATERNION_H
#define QUATERNION_H

#include "stdint.h"
#include "stdbool.h"
#include "Kalman_Filter.h"

/* ==== 文件说明 ================================================================
 * Quaternion.h —— 四元数 + 扩展卡尔曼滤波（EKF）姿态解算头文件
 * ---------------------------------------------------------------------------
 * 作用：用四元数表示云台姿态（避免欧拉角的"万向节死锁"），并用扩展卡尔曼滤波
 *       （EKF）把陀螺仪（积分预测）和加速度计（重力方向修正）融合，
 *       估计出当前姿态四元数 + 陀螺零偏，最后换算成欧拉角输出。
 *
 * 核心结构 Quaternion_Info_Typedef：
 *   quat[4]       当前姿态四元数（q0 标量 + q1,q2,q3 矢量）
 *   Deviate[3]    陀螺零偏估计（用来补偿陀螺漂移）
 *   EulerAngle[3] 输出的欧拉角（弧度）
 *   QuaternionEKF 内嵌的卡尔曼滤波器句柄（复用 Kalman_Filter 框架）
 *
 * 被谁调用：云台姿态解算任务，每个控制周期调用一次 QuaternionEKF_Update。
 * 新手提示：四元数是"用 4 个数表示 3D 旋转"，比欧拉角多了冗余但避免了死锁。
 * ========================================================================== */

typedef struct
{
  bool Init;   /* 是否已初始化标志 */
	
	float quat[4];   /* 当前姿态四元数 [q0,q1,q2,q3]（q0 是标量，表示旋转角的一半） */
  float Deviate[3];   /* 陀螺零偏估计，用于补偿陀螺漂移 */
	
	float Q1,Q2,R;   /* Q1=四元数过程噪声，Q2=陀螺零偏过程噪声，R=测量噪声 */
	
  float *QuaternionEKF_A_Data;  /*!< pointer to the data of state transition matrix A 矩阵(状态转移)数据指针 */
  float *QuaternionEKF_P_Data;  /*!< pointer to the data of posteriori covariance matrix P 矩阵(误差协方差)数据指针 */
  KalmanFilter_Info_TypeDef QuaternionEKF;  /*!< Extended Kalman Filter 内嵌的扩展卡尔曼滤波器句柄 */

  float Accel[3];      /*!< the data of accel measure 加速度计读数（m/s²） */
  float Gyro[3];       /*!< the data of gyro measure 陀螺角速度（补偿零偏后） */
  float AccelInvNorm;       /*!< the inverse of accel norm 加速度模长的倒数（1/|a|） */
  float GyroInvNorm;        /*!< the inverse of gyro norm 陀螺模长的倒数（1/|w|） */
  float Half_Gyro_dt[3];      /*!< half gyro dt 半角增量 = 0.5·gyro·dt */
  float EulerAngle[3];      /*!< Euler angles in radians: 欧拉角(弧度) [yaw,pitch,roll] */

}Quaternion_Info_Typedef;


/* 全局唯一的姿态解算句柄，其他模块通过它读取当前姿态/欧拉角。 */
extern Quaternion_Info_Typedef Quaternion_Info;

/* 初始化 EKF：Process_Noise1=四元数过程噪声Q1，Process_Noise2=零偏过程噪声Q2，
 * Measure_Noise=测量噪声R；A_Data/P_Data 是外部提供的 A、P 矩阵初值内存。 */
extern void QuaternionEKF_Init(Quaternion_Info_Typedef *Quat,float Process_Noise1,float Process_Noise2,float Measure_Noise,float *QuaternionEKF_A_Data,float *QuaternionEKF_P_Data);

/* 执行一次姿态解算：Gyro[3]=陀螺角速度，Accel[3]=加速度计读数，dt=周期(秒)。
 * 调用后可通过 Quat->quat / Quat->EulerAngle 读取姿态。 */
extern void QuaternionEKF_Update(Quaternion_Info_Typedef *Quat,float Gyro[3],float Accel[3],float dt);


#endif