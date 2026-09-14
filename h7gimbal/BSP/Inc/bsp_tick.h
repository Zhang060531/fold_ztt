/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bsp_tick.h
  * @brief          : The header file of bsptick.h
  * @author         : GrassFan Wang
  * @date           : 2025/01/22
  * @version        : v1.0
  ******************************************************************************
  * @attention      : use the TIM2 as the HAL TimeBase
  ******************************************************************************
  */
/* USER CODE END Header */

/* ================================================================================ */
/* 文件说明：BSP 层 系统滴答延时驱动 —— 头文件                                       */
/* -------------------------------------------------------------------------------- */
/* 【作用】                                                                         */
/*   提供阻塞式延时 Delay_us / Delay_ms。                                            */
/*                                                                                  */
/* 【为什么有两套时基】                                                              */
/*   本工程跑 FreeRTOS，SysTick 被 RTOS 拿去当任务调度心跳；                           */
/*   而 HAL 库的 HAL_GetTick() 时基被切换到了 TIM2（见 CubeMX 配置）。                 */
/*   于是系统里同时存在两套时钟：SysTick(给RTOS) 和 TIM2(给HAL)。                      */
/*   微秒延时通过读 TIM2->CNT 得到微秒级精度。                                        */
/*                                                                                  */
/* 【注意】                                                                         */
/*   这两个延时函数是阻塞的：延时期间 CPU 一直空转等待，不会让出 CPU 给其他任务。      */
/*   只适合极短的延时（比如等待传感器稳定、时序要求严格的场合），不要用它长时间等待。  */
/* ================================================================================ */

#ifndef BSP_TICK_H
#define BSP_TICK_H

#ifdef __cplusplus
extern "C" {
#endif


#include "stdint.h"



/**
 * @brief  微秒级阻塞延时
 * @param  us 延时的微秒数
 * @retval 无
 */
extern void Delay_us(uint32_t us);

/**
 * @brief  毫秒级阻塞延时
 * @param  ms 延时的毫秒数
 * @retval 无
 */
extern void Delay_ms(uint32_t ms);
	
	
#endif