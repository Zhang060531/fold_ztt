/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bsp_adc.h
  * @brief          : The header file of bsp_adc.h
  * @author         : GrassFan Wang
  * @date           : 2025/01/22
  * @version        : v1.0
  ******************************************************************************
  * @attention      : Pay attention to config the clock source of the advanced TIM
  ******************************************************************************
  */
/* USER CODE END Header */

/* ================================================================================ */
/* 文件说明：BSP 层 ADC（模数转换器）驱动 —— 头文件                                 */
/* -------------------------------------------------------------------------------- */
/* 【ADC 是干什么的】                                                               */
/*   模拟电压是连续量（比如电池电压 24.0V、23.9V……），MCU 只能处理数字量。            */
/*   ADC（Analog-to-Digital Converter）负责把模拟电压按比例转成一个数字。            */
/*   在 3.3V 参考电压、16 位分辨率下：3.3V -> 65535，0V -> 0，中间线性对应。         */
/*                                                                                  */
/* 【本工程用 ADC 采什么】                                                          */
/*   ADC1 通过 DMA 连续采样 2 个通道，结果写入 ADC_Voltage_Val[2]。                  */
/*   目前 USER_ADC_Voltage_Update() 只用第 0 个通道换算电池电压（经 11 倍分压电阻）。 */
/*                                                                                  */
/* 【初始化流程】                                                                   */
/*   1. BSP_ADC_Init() 由 MCU_Init()（bsp_mcu.c）统一调用；                         */
/*   2. 先做 ADC 偏移校准（H7 上电必须校准一次，否则精度差）；                       */
/*   3. 再用 HAL_ADC_Start_DMA() 以 DMA 方式连续采样，采样值自动写入数组。           */
/*                                                                                  */
/* 【关键函数】                                                                     */
/*   - BSP_ADC_Init()             初始化 ADC 并启动 DMA 连续采样                    */
/*   - USER_ADC_Voltage_Update()  把原始采样值换算成实际电压（V）返回               */
/*                                                                                  */
/* 【被谁调用】                                                                     */
/*   - BSP_ADC_Init()：MCU_Init()                                                   */
/*   - USER_ADC_Voltage_Update()：电量/电压监控相关任务                              */
/* ================================================================================ */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BSP_ADC_H
#define BSP_ADC_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "adc.h"

/* Externs ---------------------------------------------*/

/**
 * @brief  初始化 ADC1：先做偏移校准，再启动 DMA 连续采样
 * @param  无
 * @retval 无
 * @note   由 MCU_Init() 调用；采样结果自动写入 ADC_Voltage_Val[2]
 */
void BSP_ADC_Init(void);

/**
 * @brief  读取当前电池电压（把原始采样值换算成实际电压）
 * @param  无
 * @retval 实际电压值，单位 V（float）
 * @note   换算公式：电压 = 原始值 * 3.3 / 65535 * 11
 *         其中 3.3 为参考电压、65535 为 16 位满量程、11 为分压电阻倍率
 */
float USER_ADC_Voltage_Update(void);
#endif //BSP_PWM_H