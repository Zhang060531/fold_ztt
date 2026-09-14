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

/**
 * =====================================================================
 * 文件说明：全局配置 (Config.h)
 * ---------------------------------------------------------------------
 * 存放整车通用的常量、单位换算、宏开关，被几乎所有模块 include。
 * 主要分四块：
 *  1. 数学/物理常量：重力加速度、角度↔弧度换算、限幅宏 VAL_LIMIT
 *  2. 视觉/弹道常量：自瞄选板策略、弹道系数、装甲板半宽（云台自瞄用到）
 *  3. IMU 姿态解算常量：俯仰/偏航/横滚 的角度、陀螺仪、加速度数组下标
 *  4. 遥控器接收方式开关：走 CAN 还是 USART
 *
 * ===== 调车注意 =====
 *  - Bullet_Coefficient 弹道系数决定子弹下坠补偿：17mm 弹=0.038，
 *    42mm 弹=0.019，换弹种要改这里。
 *  - IMU_ANGLE_INDEX_* / IMU_GYRO_INDEX_* / IMU_ACCEL_INDEX_* 这几个
 *    下标必须和姿态解算模块(Quaternion/INS_Task)里算出来的数组顺序一致，
 *    对不上云台角度就会乱。
 * =====================================================================
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

/**
 * @brief the value of local gravity acceleration
 */
 
// 限幅宏：把变量 x 限制在 [min, max] 区间内（超出就截断）
// 用 do{...}while(0) 包裹，保证宏展开后是一个完整语句，用在 if/else 里也安全
 #define VAL_LIMIT(x,min,max)  do{ \
                                    if ((x) > (max)) {(x) = (max);} \
                                    else if ((x) < (min)) {(x) = (min);} \
                                }while(0U)

// 本地重力加速度（单位 m/s²，与海拔/纬度有关，此处取青岛附近值）
#define GravityAccel  9.79136f

// 角度 → 弧度 换算系数 (π/180)
#define Angle_to_rad  0.01745329f

// 弧度 → 角度 换算系数 (180/π)
#define Rad_to_angle  57.2957732f

/**
  * @brief Euler's Number
  */
#define Euler_Number 2.718281828459045f

/**
 * @brief radian system rotation degrees system , 180.f/PI
 */
#define RadiansToDegrees 57.295779513f

/**
 * @brief degrees system rotation radian system , PI/180.f
 */
#define DegreesToRadians 0.01745329251f

/* Vision reslove constants -------------------------------------------------*/

/**
 * @brief  Decision Marking mode
 *         0: select the minimum yaw armor
 *         1: select the minimum distance armor
 */
#define Yaw_Distance_Decision  0

/**
 * @brief 弹道系数（子弹下坠补偿用）
 * @note  17mm 子弹: 0.038
 *        42mm 子弹: 0.019
 *        换弹种必须改这里，否则打不准
 */
#define Bullet_Coefficient  0.038f

/**
 * @brief 小装甲板半宽（米），自瞄解算偏移用
 */
#define LittleArmor_HalfWidth   0.07f

/**
 * @brief 大装甲板半宽（米），自瞄解算偏移用
 */
#define LargeArmor_HalfWidth   0.1175f

/* IMU reslove constants ---------------------------------------------------*/
/**
 * @brief the flag of bmi088 Calibration
 *        0: DISABLE
 *        1: ENABLE
 */
#define IMU_Calibration_ENABLE 1U

/**
 * @brief the index of pitch angle update
 */
#define IMU_ANGLE_INDEX_PITCH  2U
/**
 * @brief the index of yaw angle update
 */
#define IMU_ANGLE_INDEX_YAW   0U
/**
 * @brief the index of roll angle update
 */
#define IMU_ANGLE_INDEX_ROLL   1U

/**
 * @brief the index of pitch gyro update
 */
#define IMU_GYRO_INDEX_PITCH  0U
/**
 * @brief the index of yaw gyro update
 */
#define IMU_GYRO_INDEX_YAW   2U
/**
 * @brief the index of roll gyro update
 */
#define IMU_GYRO_INDEX_ROLL   1U

/**
 * @brief the index of pitch accel update
 */
#define IMU_ACCEL_INDEX_PITCH  0U
/**
 * @brief the index of yaw accel update
 */
#define IMU_ACCEL_INDEX_YAW   2U
/**
 * @brief the index of roll accel update
 */
#define IMU_ACCEL_INDEX_ROLL   1U

/* Remote reslove constants -----------------------------------------------*/
/**
 * @brief the flag of remote control receive frame data
 * @note  0: CAN
 *        1: USART
 */
#define REMOTE_FRAME_USART_CAN   0U

#endif //ROBOT_CONFIG_H


