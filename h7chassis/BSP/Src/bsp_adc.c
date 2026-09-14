/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bsp_adc.c
  * @brief          : bsp adc functions 
  * @author         : GrassFan Wang
  * @date           : 2025/01/22
  * @version        : v1.0
  ******************************************************************************
  * @attention      : Pay attention to enable the adc
  ******************************************************************************
  */
/* USER CODE END Header */

/* ================================================================================ */
/* 文件说明：BSP 层 ADC（模数转换器）驱动 —— 实现文件                               */
/* -------------------------------------------------------------------------------- */
/* 【ADC 是干什么的】                                                               */
/*   ADC 把模拟电压（连续量）转换成数字量（离散量），MCU 才能读取和处理。            */
/*   本项目 ADC1 通过 DMA 连续采样 2 个通道，结果写入 ADC_Voltage_Val[2]，           */
/*   目前主要用来测量电池电压（经 11 倍分压电阻后送到 ADC 引脚）。                    */
/*                                                                                  */
/* 【为什么要校准】                                                                 */
/*   STM32H7 的 ADC 内部存在零点偏移，上电后必须先调用 HAL_ADCEx_Calibration_Start()  */
/*   做一次偏移校准，否则采样值会有固定误差。                                        */
/*                                                                                  */
/* 【为什么用 DMA】                                                                 */
/*   不用 DMA 的话每次采样都要 CPU 手动去读寄存器；用 DMA 后 ADC 会把结果自动搬进     */
/*   数组，CPU 完全不用管，省下的时间可以跑别的任务。                                */
/*                                                                                  */
/* 【关键函数】                                                                     */
/*   - BSP_ADC_Init()             校准 + 启动 DMA 连续采样                          */
/*   - USER_ADC_Voltage_Update()  原始值 -> 实际电压换算                            */
/* ================================================================================ */

/* Includes ------------------------------------------------------------------*/
#include "bsp_adc.h"
#include "adc.h"

/**
 * @brief ADC 采样结果数组
 * @note  大小 2，对应 ADC 采样的 2 个通道
 * @note  __attribute__((section(".AXI_SRAM"))) 表示把数组放到 AXI SRAM 段：
 *        STM32H7 有多种 RAM，AXI SRAM 可被 DMA 直接访问、速度快，适合做 DMA 缓冲区。
 */
__attribute__((section (".AXI_SRAM"))) uint16_t ADC_Voltage_Val[2];
 
/**
  * @brief  初始化 ADC1：先做偏移校准，再启动 DMA 连续采样
  * @param  无
  * @retval 无
  * @note   1. HAL_ADCEx_Calibration_Start()：ADC 偏移校准，H7 上电必须校准一次
  *            参数 ADC_CALIB_OFFSET 表示只校零点偏移；ADC_SINGLE_ENDED 表示单端输入
  *         2. HAL_ADC_Start_DMA()：启动 DMA 连续采样，结果自动写入 ADC_Voltage_Val
  */
void BSP_ADC_Init(void){

 	HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ADC_Voltage_Val,2);

}
//------------------------------------------------------------------------------

/**
  * @brief  读取并换算当前电池电压
  * @param  无
  * @retval 实际电压值，单位 V（float）
  * @note   换算公式：Voltage = 原始值 * 3.3 / 65535 * 11
  *         - 3.3f  ：ADC 参考电压 3.3V
  *         - 65535 ：16 位 ADC 满量程（2^16 - 1），即 3.3V 对应的数字量
  *         - 11.0f ：电池电压经电阻分压缩小 11 倍后才接到 ADC，这里乘回还原
  */
float USER_ADC_Voltage_Update(void){

  float Voltage;
	Voltage = (ADC_Voltage_Val[0]*3.3f/65535)*11.0f;
	return Voltage;
   
}   
//------------------------------------------------------------------------------













