/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : INS_Task.h
  * @brief          : INS task —— 惯性导航(IMU 姿态解算)任务头文件
  * @author         : Yan Yuanbin
  * @date           : 2023/04/27
  * @version        : v1.0
  ******************************************************************************
  * @attention      : None
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef INS_TASK_H
#define INS_TASK_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"

/* Exported types ------------------------------------------------------------*/
/**
 * @brief typedef structure that contains the information for the INS.
 *        INS(惯性导航系统)对外输出的姿态信息结构体。
 *        由 INS_Task 每个控制周期解算一次，供云台控制任务、视觉等模块读取。
 */
typedef struct
{
	float Pitch_Angle;   /*!< 俯仰角(度)，范围约 -90°~+90° */
	float Yaw_Angle;     /*!< 偏航角(度)，范围 -180°~+180°，转到 ±180° 处会回绕跳变 */
	float Yaw_TolAngle;  /*!< 偏航累计总角度(度)，累加了回绕圈数、可超出 ±180° 不回绕，用于连续旋转控制 */
	float Roll_Angle;    /*!< 横滚角(度) */

  float Pitch_Gyro;    /*!< 俯仰轴角速度(度/秒) */
  float Yaw_Gyro;      /*!< 偏航轴角速度(度/秒) */
  float Roll_Gyro;     /*!< 横滚轴角速度(度/秒) */

  float Angle[3];      /*!< 欧拉角(弧度)，顺序 [yaw, pitch, roll]，直接拷贝自 EKF 输出 */
	float Gyro[3];	    /*!< 三轴角速度(弧度/秒)，直接拷贝自 BMI088 原始读数 */
	float Accel[3];	    /*!< 三轴加速度(单位 g)，已做二阶低通滤波 */

	float Last_Yaw_Angle;   /*!< 上一周期的偏航角，用于检测 ±180° 回绕跳变 */
	int16_t YawRoundCount;  /*!< 偏航角累计回绕圈数(整数圈)，+1 表示正向多转了一圈 */

}INS_Info_Typedef;

/* Externs---------------------------------------------------------*/
extern INS_Info_Typedef INS_Info;

#endif //INS_TASK_H
