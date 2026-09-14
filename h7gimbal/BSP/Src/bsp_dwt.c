/**
  ******************************************************************************
  * @file           : bsp_dwt.c
  * @brief          : bsp DWT functions 
  * @author         : GrassFan Wang
  * @date           : 2025/02/17
  * @version        : v1.0
  ******************************************************************************
  * @attention      : none
  ******************************************************************************
  */
/* USER CODE END Header */

/* ================================================================================ */
/* 文件说明：BSP 层 DWT（数据观察与跟踪单元）驱动 —— 实现文件                        */
/* -------------------------------------------------------------------------------- */
/* 【DWT 是什么】                                                                   */
/*   DWT 是 Cortex-M 内核自带的调试/观测单元，内部有一个 32 位周期计数器 CYCCNT，      */
/*   每个 CPU 时钟周期自动 +1。用它数周期就能精确得到经过的时间，实现微秒级延时。      */
/*                                                                                  */
/* 【为什么用 DWT 做延时而不是普通定时器】                                            */
/*   1. 不占用额外的定时器外设（TIM 资源留给 PWM 等）；                               */
/*   2. 不依赖中断，纯硬件计数，读取快、开销小；                                      */
/*   3. 精度高到 1 个 CPU 周期（例如 480MHz 下约 2ns）。                              */
/*                                                                                  */
/* 【初始化流程】                                                                   */
/*   BSP_DWT_Init(主频MHz)：使能 DWT 跟踪 -> 清零 CYCCNT -> 使能 CYCCNT 计数 ->      */
/*   记录 CPU 主频（并换算成每 1ms / 1us 对应的周期数，供后续延时计算）。             */
/*                                                                                  */
/* 【被谁调用】                                                                     */
/*   BSP_DWT_Init()：系统启动时调用一次（由 main 或 MCU_Init 流程）。                */
/* ================================================================================ */

/* Include ---------------------------------------------------------*/
#include "bsp_dwt.h"
#include "stm32h723xx.h"

DWT_Time_Typedef SysTime;   /* 系统时间（秒/毫秒/微秒），供上层读取 */
uint64_t CYCCNT64;          /* 64 位周期计数（用于记录更长时间，避免 32 位溢出） */

/* Private variables ---------------------------------------------------------*/
static uint32_t CPU_FREQ_Hz;      /* CPU 主频，单位 Hz（例如 480MHz -> 480000000） */
static uint32_t CPU_FREQ_Hz_ms;   /* 1 毫秒对应的周期数（= 主频 / 1000） */
static uint32_t CPU_FREQ_Hz_us;   /* 1 微秒对应的周期数（= 主频 / 1000000） */
static uint32_t CYCCNT_RountCount;/* CYCCNT 溢出次数（32 位计数器会回绕） */
static uint32_t CYCCNT_LAST;      /* 上一次读取的 CYCCNT 值，用于计算差值 */

/**
 * @brief  初始化 DWT 周期计数器，使能 CYCCNT 并记录 CPU 主频
 * @param  CPU_Freq_mHz  CPU 主频，单位 MHz（变量名写作 mHz，实际含义是 MHz）
 * @retval 无
 * @note   三步：使能 DWT 跟踪 -> 清零并启动 CYCCNT -> 换算并保存主频参数
 */
void BSP_DWT_Init(uint32_t CPU_Freq_mHz){

     /* Initializes the DWT peripheral */
     /* 使能内核调试/跟踪控制位 TRCENA：
        必须先打开这个总开关，DWT 的计数器等调试功能才能使用 */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    /* clear the DWT CYCCNT register */
    /* 把周期计数器 CYCCNT 清零，从 0 开始数 */
    DWT->CYCCNT = (uint32_t)0u;

    /* Enable Cortex-M DWT CYCCNT register */
    /* 使能 CYCCNT 计数：之后每个 CPU 时钟周期 CYCCNT 自动 +1 */
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    /* update the clock frequence */
    /* 保存并换算 CPU 主频：
       CPU_FREQ_Hz    = 主频(Hz)，例如 480MHz -> 480000000
       CPU_FREQ_Hz_ms = 每 1ms 的周期数 = 主频/1000
       CPU_FREQ_Hz_us = 每 1us 的周期数 = 主频/1000000
       后续延时就用「目标周期数 = 时长 * 对应周期数」来计算 */
    CPU_FREQ_Hz = CPU_Freq_mHz * 1000000;
    CPU_FREQ_Hz_ms = CPU_FREQ_Hz / 1000;
    CPU_FREQ_Hz_us = CPU_FREQ_Hz / 1000000;
    CYCCNT_RountCount = 0;

}




