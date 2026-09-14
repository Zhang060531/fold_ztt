/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bsp_pwm.h
  * @brief          : The header file of bsp_pwm.h
  * @author         : GrassFan Wang
  * @date           : 2025/01/22
  * @version        : v1.0
  ******************************************************************************
  * @attention      : Pay attention to config the clock source of the advanced TIM
  ******************************************************************************
  */
/* USER CODE END Header */

/* ================================================================================ */
/* 文件说明：BSP 层 PWM（脉宽调制）驱动 —— 头文件                                    */
/* -------------------------------------------------------------------------------- */
/* 【PWM 是干什么的】                                                               */
/*   PWM 是一串固定频率的方波，靠「高电平占整个周期的比例（占空比）」来控制输出。      */
/*   占空比越大，等效输出的平均功率越大。本工程用 PWM 驱动 BMI088 的加热片，           */
/*   通过调节占空比给传感器加热，让惯性传感器工作在稳定温度，减小温漂。                */
/*                                                                                  */
/* 【频率和占空比怎么定】                                                            */
/*   频率   = 定时器时钟 / (预分频 PSC + 1) / (自动重装值 ARR + 1)                  */
/*   占空比 = CCR / (ARR + 1)（CCR 是比较值，决定高电平时间）                        */
/*   这些参数在 CubeMX 里配置（见 tim.c 的 htim3 初始化），CCR 在运行时实时修改。      */
/*                                                                                  */
/* 【关键函数】                                                                     */
/*   - BSP_PWM_Init()                启动 TIM3 通道 4 的 PWM 输出                   */
/*   - Heat_Power_Control(Compare)   设置加热功率（改 CCR 即改占空比）               */
/*   - USER_Tim_SetCompare()         通用：给指定定时器通道设置 CCR 比较值           */
/* ================================================================================ */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BSP_PWM_H
#define BSP_PWM_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "tim.h"

/* Exported functions prototypes ---------------------------------------------*/
/**
  * @brief  启动 PWM 输出（TIM3 通道 4）
  * @param  无
  * @retval 无
  */
extern void BSP_PWM_Init(void);

/**
  * @brief  设置 BMI088 加热片的加热功率
  * @param  Compare 比较值 CCR，决定占空比（越大加热越强）
  * @retval 无
  * @note   本质是修改 TIM3 通道 4 的 CCR 寄存器来改变 PWM 占空比
  */
extern void Heat_Power_Control(uint16_t Compare);

/**
  * @brief  通用函数：给指定定时器的指定通道设置 CCR 比较值（即改 PWM 占空比）
  * @param  htim     定时器句柄
  * @param  Channel  通道号（TIM_CHANNEL_1~4）
  * @param  Compare  比较值 CCR，决定占空比
  * @retval 无
  */
extern void USER_Tim_SetCompare(TIM_HandleTypeDef *htim,uint32_t Channel,uint16_t Compare);
#endif //BSP_PWM_H