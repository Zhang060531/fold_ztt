/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bsp_gpio.h
  * @brief          : The header file of bsp_can.h 
  * @author         : GrassFan Wang
  * @date           : 2025/01/22
  * @version        : v1.0
  ******************************************************************************
  * @attention      : None
  ******************************************************************************
  */
/* USER CODE END Header */

/* ================================================================================ */
/* 文件说明：BSP 层 GPIO（通用输入输出）驱动 —— 头文件                               */
/* -------------------------------------------------------------------------------- */
/* 【GPIO 是干什么的】                                                               */
/*   GPIO 就是芯片引脚，可配置成输入/输出，输出可置高(3.3V)/置低(0V)。                */
/*   本文件主要管理 BMI088 惯性传感器（加速度计+陀螺仪）的两根片选(CS)引脚电平。       */
/*                                                                                  */
/* 【CS 片选引脚的作用】                                                             */
/*   BMI088 的加速度计和陀螺仪共用同一条 SPI 总线，靠 CS 引脚区分「现在跟谁说话」。    */
/*   CS 拉低 = 选中该芯片（SPI 通信生效）；CS 拉高 = 取消选中（忽略总线上的数据）。    */
/*   所以通信前先调用 XXX_NS_L() 拉低，通信完调用 XXX_NS_H() 拉高。                   */
/*                                                                                  */
/* 【关键函数】                                                                     */
/*   - BSP_GPIO_Init()         GPIO 初始化（当前为预留空函数）                       */
/*   - BMI088_ACCEL_NS_L/H()   拉低/拉高加速度计片选                                */
/*   - BMI088_GYRO_NS_L/H()    拉低/拉高陀螺仪片选                                  */
/*                                                                                  */
/* 【被谁调用】                                                                     */
/*   - BSP_GPIO_Init()：MCU_Init()                                                   */
/*   - 片选函数：BMI088 驱动（bmi088.c）在每次 SPI 读写前后调用                      */
/* ================================================================================ */

#ifndef BSP_GPIO_H
#define BSP_GPIO_H

#ifdef __cplusplus
extern "C" {
#endif

/* Externs ------------------------------------------------------------------*/

/**
 * @brief  初始化 GPIO（当前为空，仅预留）
 * @param  无
 * @retval 无
 */
extern void BSP_GPIO_Init(void);

/**
 * @brief  拉低加速度计(BMI088 ACCEL)片选 CS 引脚，选中加速度计
 * @param  无
 * @retval 无
 */
extern void BMI088_ACCEL_NS_L(void);

/**
 * @brief  拉高加速度计(BMI088 ACCEL)片选 CS 引脚，取消选中
 * @param  无
 * @retval 无
 */
extern void BMI088_ACCEL_NS_H(void);

/**
 * @brief  拉低陀螺仪(BMI088 GYRO)片选 CS 引脚，选中陀螺仪
 * @param  无
 * @retval 无
 */
extern void BMI088_GYRO_NS_L(void);

/**
 * @brief  拉高陀螺仪(BMI088 GYRO)片选 CS 引脚，取消选中
 * @param  无
 * @retval 无
 */
extern void BMI088_GYRO_NS_H(void);

#endif