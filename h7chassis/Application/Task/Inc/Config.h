/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : Config.c
  * @brief          : Configuare the Robot Functions 
  * @author         : Yan Yuanbin
  * @date           : 2023/05/21
  * @version        : v1.0
  ******************************************************************************
  * @attention      : To be perfected
  ******************************************************************************
  */
/* USER CODE END Header */

/* ============================================================================
 * 文件说明：机器人全局配置 (Config.h) —— 全工程共用的常量与工具宏
 * ----------------------------------------------------------------------------
 * 作用：集中定义全局通用的「常量」和「限幅宏」，供各模块 include 使用，避免
 *        魔法数字散落各处。主要内容：
 *          1. VAL_LIMIT 限幅宏：把任意变量夹在 [min,max] 区间内
 *          2. 物理/数学常量：重力加速度、角度↔弧度换算、欧拉数等
 *          3. 视觉/弹道常量：自瞄解算用（本底盘工程未用到，属云台工程遗留）
 *          4. IMU(BMI088) 数据下标：标定 pitch/yaw/roll 在数组中的位置
 *          5. 遥控器数据来源选择
 *
 * 调用关系：纯头文件（只有宏，无函数/变量），被 Motor.h、INS_Task.c 等 include。
 *
 * ===== 调车关注 =====
 *  - GravityAccel：当地重力加速度 (m/s²)，IMU 姿态解算用。姿态漂了先核对它。
 *  - IMU_*_INDEX_* 系列下标：要和 INS_Task.c 里 QuaternionEKF 输出的欧拉角/
 *    陀螺仪顺序一一对应，改错会导致 pitch/yaw/roll 全部错位，慎改。
 *  - 视觉/弹道相关宏（Bullet_Coefficient 等）在本底盘工程基本是死代码，
 *    从云台工程复制而来，可忽略。
 * ============================================================================
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "stdbool.h"
#include "stdlib.h"
#include "string.h"
#include "math.h"

/* General physics and mathematics constants ---------------------------------*/
/* 通用物理/数学常量区：下面 VAL_LIMIT 是限幅宏，其余是重力加速度、角度弧度换算等。 */

/**
 * @brief the value of local gravity acceleration
 * @note  注意：这个英文注释描述的是 GravityAccel（在下方），却放到了 VAL_LIMIT
 *        上面，是原代码遗留的注释错位，不影响编译。
 */
 /* 限幅宏：把变量 x 夹在 [min,max] 区间内，越界就截断到边界。
  *  用于给角度/电流/功率等物理量兜底（x 会被求值多次，别传带副作用的表达式）。 */
 #define VAL_LIMIT(x,min,max)  do{ \
                                    if ((x) > (max)) {(x) = (max);} \
                                    else if ((x) < (min)) {(x) = (min);} \
                                }while(0U)
 
#define GravityAccel  9.718f    /* 当地重力加速度 (m/s²)，IMU 姿态解算用 */

#define Angle_to_rad  0.01745329f    /* 角度→弧度换算系数 (π/180) */

#define Rad_to_angle  57.2957732f    /* 弧度→角度换算系数 (180/π) */

/**
  * @brief Euler's Number
  * @note  自然常数 e
  */
#define Euler_Number 2.718281828459045f

/**
 * @brief radian system rotation degrees system , 180.f/PI
 * @note  弧度→角度换算系数（与上面的 Rad_to_angle 等价）
 */
#define RadiansToDegrees 57.295779513f

/**
 * @brief degrees system rotation radian system , PI/180.f
 * @note  角度→弧度换算系数（与上面的 Angle_to_rad 等价）
 */
#define DegreesToRadians 0.01745329251f

/* Vision reslove constants -------------------------------------------------*/
/* 视觉自瞄/弹道解算相关常量 —— 这些是从云台工程带过来的遗留代码，
 * 底盘工程没有视觉和发射机构，一般用不到，了解即可。 */

/**
 * @brief  Decision Marking mode
 *         0: select the minimum yaw armor
 *         1: select the minimum distance armor
 */
#define Yaw_Distance_Decision  0

/**
 * @brief ballistic coefficient
 * @note  弹道阻力系数，用于自瞄解算时把子弹飞行时间换算成抬枪量。
 *        17mm: 0.038
 *        42mm: 0.019
 */
#define Bullet_Coefficient  0.038f

/**
 * @brief the half width of little armor
 * @note  小装甲板半宽，单位 m（自瞄打击点补偿用）
 */
#define LittleArmor_HalfWidth   0.07f

/**
 * @brief the half width of Large armor
 * @note  大装甲板半宽，单位 m（自瞄打击点补偿用）
 */
#define LargeArmor_HalfWidth   0.1175f

/* IMU reslove constants ---------------------------------------------------*/
/* IMU(BMI088) 数据下标：QuaternionEKF 输出的欧拉角/陀螺仪/加速度是长度为 3 的数组，
 * 下面这些宏就是标定 pitch/yaw/roll 分别放在数组第几个位置。注意角度和陀螺仪的
 * pitch/yaw 下标是「交叉」的（角度 pitch=2 yaw=0，陀螺仪 pitch=0 yaw=2），
 * 这是按 BMI088 三轴安装方向定的，改安装方向时这里要一起改。 */
/**
 * @brief the flag of bmi088 Calibration
 * @note  BMI088 是否做上电标定：0=不标定 1=标定
 *        0: DISABLE
 *        1: ENABLE
 */
#define IMU_Calibration_ENABLE  0U

/**
 * @brief the index of pitch angle update
 * @note  欧拉角数组中「俯仰 pitch」的下标
 */
#define IMU_ANGLE_INDEX_PITCH  2U
/**
 * @brief the index of yaw angle update
 * @note  欧拉角数组中「偏航 yaw」的下标
 */
#define IMU_ANGLE_INDEX_YAW   0U
/**
 * @brief the index of roll angle update
 * @note  欧拉角数组中「横滚 roll」的下标
 */
#define IMU_ANGLE_INDEX_ROLL   1U

/**
 * @brief the index of pitch gyro update
 * @note  陀螺仪角速度数组中「俯仰 pitch」的下标
 */
#define IMU_GYRO_INDEX_PITCH  0U
/**
 * @brief the index of yaw gyro update
 * @note  陀螺仪角速度数组中「偏航 yaw」的下标
 */
#define IMU_GYRO_INDEX_YAW   2U
/**
 * @brief the index of roll gyro update
 * @note  陀螺仪角速度数组中「横滚 roll」的下标
 */
#define IMU_GYRO_INDEX_ROLL   1U

/**
 * @brief the index of pitch accel update
 * @note  加速度数组中「俯仰 pitch」的下标
 */
#define IMU_ACCEL_INDEX_PITCH  0U
/**
 * @brief the index of yaw accel update
 * @note  加速度数组中「偏航 yaw」的下标
 */
#define IMU_ACCEL_INDEX_YAW   2U
/**
 * @brief the index of roll accel update
 * @note  加速度数组中「横滚 roll」的下标
 */
#define IMU_ACCEL_INDEX_ROLL   1U

/* Remote reslove constants -----------------------------------------------*/
/* 遥控器数据来源选择：本工程遥控器(DR16)走 CAN 收发，所以这里取 0。 */
/**
 * @brief the flag of remote control receive frame data
 * @note  0: CAN
 *        1: USART
 */
#define REMOTE_FRAME_USART_CAN   0U

#endif //ROBOT_CONFIG_H


