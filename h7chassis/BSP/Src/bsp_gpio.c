/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bsp_gpio.c
  * @brief          : bsp gpio functions 
  * @author         : GrassFan Wan
  * @date           : 2025/1/22
  * @version        : v1.0
  ******************************************************************************
  * @attention      : None
  ******************************************************************************
  */
/* USER CODE END Header */

/* ================================================================================ */
/* 文件说明：BSP 层 GPIO（通用输入输出）驱动 —— 实现文件                             */
/* -------------------------------------------------------------------------------- */
/* 【作用】                                                                         */
/*   管理 BMI088 惯性传感器（加速度计 + 陀螺仪）的片选(CS)引脚。                      */
/*   BMI088 的加速度计和陀螺仪挂在同一条 SPI 总线上，通信时用 CS 引脚选中其中一个。    */
/*                                                                                  */
/* 【CS 片选原理】                                                                   */
/*   CS 是「低电平有效」：拉低表示选中该芯片，此时它才响应 SPI 总线上的数据；          */
/*   拉高表示取消选中。因此一次完整的 SPI 通信流程是：拉低 CS -> 收发数据 -> 拉高 CS。  */
/*                                                                                  */
/* 【关键函数】                                                                     */
/*   - BSP_GPIO_Init()        预留空函数                                            */
/*   - BMI088_ACCEL_NS_L/H()  加速度计片选拉低/拉高                                  */
/*   - BMI088_GYRO_NS_L/H()   陀螺仪片选拉低/拉高                                    */
/* ================================================================================ */

/* Includes ------------------------------------------------------------------*/
#include "bsp_gpio.h"
#include "main.h"

/**
  * @brief  GPIO 初始化（当前为空函数，仅预留）
  * @param  无
  * @retval 无
  * @note   若后续需要配置 GPIO 输出/输入，可在此处用 HAL_GPIO_WritePin 等 API 实现
  */
void BSP_GPIO_Init(void){

//	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14,1);
//  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,1);

}

/**
  * @brief 拉低加速度计片选 CS（选中加速度计）
  * @note GPIO_x: GPIOC
  * @note GPIO_PIN_x: GPIO_PIN_0
  * @note CS 低电平有效：GPIO_PIN_RESET 表示输出低电平，即选中芯片
  */
void BMI088_ACCEL_NS_L(void)
{
    HAL_GPIO_WritePin(ACCEL_CS_GPIO_Port,ACCEL_CS_Pin,GPIO_PIN_RESET);
}

/**
  * @brief 拉高加速度计片选 CS（取消选中加速度计）
  * @note GPIO_x: GPIOC
  * @note GPIO_PIN_x: GPIO_PIN_0
  * @note GPIO_PIN_SET 表示输出高电平，取消选中
  */
void BMI088_ACCEL_NS_H(void)
{
    HAL_GPIO_WritePin(ACCEL_CS_GPIO_Port,ACCEL_CS_Pin,GPIO_PIN_SET);
}

/**
  * @brief 拉低陀螺仪片选 CS（选中陀螺仪）
  * @note GPIO_x: GPIOC
  * @note GPIO_PIN_x: GPIO_PIN_3
  * @note CS 低电平有效：GPIO_PIN_RESET 表示输出低电平，即选中芯片
  */
void BMI088_GYRO_NS_L(void)
{
    HAL_GPIO_WritePin(GYRO_CS_GPIO_Port,GYRO_CS_Pin,GPIO_PIN_RESET);
}

/**
  * @brief 拉高陀螺仪片选 CS（取消选中陀螺仪）
  * @note GPIO_x: GPIOC
  * @note GPIO_PIN_x: GPIO_PIN_3
  * @note GPIO_PIN_SET 表示输出高电平，取消选中
  */
void BMI088_GYRO_NS_H(void)
{
    HAL_GPIO_WritePin(GYRO_CS_GPIO_Port,GYRO_CS_Pin,GPIO_PIN_SET);
}