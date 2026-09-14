/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bsp_tick.c
  * @brief          : HAL delay functions 
  * @author         : Yan Yuanbin
  * @date           : 2025/01/22
  * @version        : v1.0
  ******************************************************************************
  * @attention      : use the TIM2 as the HAL TimeBase
  ******************************************************************************
  */
/* USER CODE END Header */

/* ================================================================================ */
/* 文件说明：BSP 层 系统滴答延时驱动 —— 实现文件                                     */
/* -------------------------------------------------------------------------------- */
/* 【时间基准怎么来的】                                                              */
/*   CubeMX 建议把 HAL 的时基从 SysTick 换到别的定时器（本项目用 TIM2），               */
/*   因为跑 FreeRTOS 后 SysTick 要留给 RTOS 当心跳。于是系统里有两套时钟：             */
/*     - SysTick ：给 RTOS 用（启动任务调度后 SysTick->VAL 由 RTOS 更新）             */
/*     - TIM2    ：给 HAL 用（HAL_GetTick 的时基），TIM2 配置成 1MHz、1us 周期        */
/*                                                                                  */
/* 【HaL_Tick() 的原理】                                                             */
/*   把「毫秒(HAL_GetTick)」和「微秒(TIM2->CNT)」拼在一起，得到一个微秒级时间戳：     */
/*     HaL_Tick = ms*1000 + us                                                        */
/*   这样既能覆盖长时间，又有微秒级分辨率。                                           */
/*                                                                                  */
/* 【注意】                                                                         */
/*   Delay_us / Delay_ms 都是阻塞式延时：等待期间 CPU 空转，不会触发任务切换。          */
/*   只适合极短延时，长时间等待请用 RTOS 的 vTaskDelay 等。                            */
/* ================================================================================ */

/* Include------------------------------------------------------------------*/
#include "bsp_tick.h"
#include "stm32h7xx.h"

/**
  * @brief  获取当前微秒级时间戳（把 HAL 的毫秒和 TIM2 的微秒拼起来）
  * @param  无
  * @retval 当前时间戳（单位：微秒）
  * @note   CubeMX 建议把 HAL 时基从 SysTick 换到别的定时器，于是系统里有两套时基：
  *         1. SysTick 给 RTOS 用（启动任务调度后 SysTick->VAL 由 RTOS 更新）
  *         2. HAL Tick 给 HAL 库用，本项目用 TIM2（TIM2->CNT 可提供微秒级计数）
  *         注意：Delay_us / Delay_ms 都是阻塞式延时，不会触发任务调度
  */
static uint32_t HaL_Tick(void)
{
    uint32_t HaL_Tick = 0;
    
	  register uint32_t ms = 0, us= 0;

    ms = HAL_GetTick();      /* 取 HAL 毫秒时基（来自 TIM2 溢出中断累计） */
    // use the TIM2 as the HAL TimeBase
    // Freq:1MHz => 1Tick = 1us
    // Period:1ms
    // 这里用 TIM2 作为 HAL 时基：TIM2 配成 1MHz，即 1 个计数 = 1us，周期 1ms

    us = TIM2->CNT;          /* 读 TIM2 当前计数值，即「当前这一毫秒内的微秒数」 */

    HaL_Tick = ms*1000 + us; /* 毫秒换算成微秒 + 微秒部分 = 完整微秒时间戳 */

    return HaL_Tick;
}

/**
  * @brief  微秒级阻塞延时
  * @param  us 延时的微秒数
  * @retval 无
  * @note   记录起始时间戳，然后空转等待时间差 >= us。阻塞式，期间不调度其他任务。
  */
void Delay_us(uint32_t us)
{
    uint32_t Now = HaL_Tick();      /* 记录起始时间戳（微秒） */

    while((HaL_Tick() - Now) < us); /* 空转直到经过了 us 微秒 */
}

/**
  * @brief  毫秒级阻塞延时
  * @param  ms 延时的毫秒数
  * @retval 无
  * @note   直接基于 HAL_GetTick()（TIM2 毫秒时基），阻塞式。
  */
void Delay_ms(uint32_t ms)
{
    uint32_t Now = HAL_GetTick();      /* 记录起始毫秒时间戳 */

    while((HAL_GetTick() - Now) < ms); /* 空转直到经过了 ms 毫秒 */
}