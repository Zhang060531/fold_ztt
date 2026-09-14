/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bsp_pwm.c
  * @brief          : bsp pwm functions 
  * @author         : Yan Yuanbin
  * @date           : 2023/04/27
  * @version        : v1.0
  ******************************************************************************
  * @attention      : Pay attention to config the clock source of the advanced TIM
  ******************************************************************************
  */
/* USER CODE END Header */

/* ================================================================================ */
/* 文件说明：BSP 层 PWM（脉宽调制）驱动 —— 实现文件                                  */
/* -------------------------------------------------------------------------------- */
/* 【PWM 原理（新手必读）】                                                          */
/*   PWM = 脉宽调制，是一串固定频率的方波。占空比 = 高电平时间 / 周期时间。            */
/*   占空比 100% 时输出一直高（全功率），0% 时一直低（关断），50% 时半功率。            */
/*   对于加热片这种「只看平均功率」的负载，PWM 相当于一个可调电压源。                  */
/*                                                                                  */
/* 【占空比怎么算】                                                                  */
/*   TIM 定时器有一个自动重装寄存器 ARR 和若干个比较寄存器 CCR。                       */
/*   计数器从 0 数到 ARR 循环，当计数值 < CCR 时输出高、>= CCR 时输出低。              */
/*   所以占空比 = CCR / (ARR + 1)。改 CCR 就改占空比，也就是本文件做的事。            */
/*                                                                                  */
/* 【本工程用途】                                                                   */
/*   TIM3 通道 4 输出 PWM 驱动 BMI088 的加热片（Heat_Power），                          */
/*   通过 Heat_Power_Control() 调节占空比给传感器恒温，减小温漂。                     */
/* ================================================================================ */

/* Includes ------------------------------------------------------------------*/
#include "bsp_pwm.h"
#include "tim.h"

/**
  * @brief  启动 TIM3 通道 4 的 PWM 输出
  * @param  无
  * @retval 无
  * @note   HAL_TIM_PWM_Start() 使能对应定时器通道的 PWM 波形输出。
  *         调用后引脚才开始输出方波；调用前设置 CCR 是不起作用的。
  */
void BSP_PWM_Init(void)
{
	//Heat_Power_Tim Start（启动加热片 PWM 定时器）
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
}
//------------------------------------------------------------------------------

/**
  * @brief  设置 BMI088 加热片的加热功率（改 PWM 占空比）
  * @param  Compare 比较值 CCR：越大占空比越大、加热越强
  * @retval 无
  * @note   实际就是修改 TIM3 通道 4 的 CCR 寄存器
  */
void Heat_Power_Control(uint16_t Compare)
{
    USER_Tim_SetCompare(&htim3,TIM_CHANNEL_4,Compare);
}
//------------------------------------------------------------------------------

/**
  * @brief  通用函数：给指定定时器的指定通道设置 CCR 比较值（即改 PWM 占空比）
  * @note   HAL 库的 __HAL_TIM_SET_COMPARE 是宏，这里改为直接写 CCR 寄存器，
  *         通过 switch 选择要写入哪个通道的比较寄存器。
  * @param  htim TIM PWM handle
  * @param  Channel TIM Channels to be configured
  *          This parameter can be one of the following values:
  *            @arg TIM_CHANNEL_1: TIM Channel 1 selected
  *            @arg TIM_CHANNEL_2: TIM Channel 2 selected
  *            @arg TIM_CHANNEL_3: TIM Channel 3 selected
  *            @arg TIM_CHANNEL_4: TIM Channel 4 selected
  * @param  compare specifies the Capture Compare register new value.
  * @retval None
  */
void USER_Tim_SetCompare(TIM_HandleTypeDef *htim,uint32_t Channel,uint16_t Compare)
{
  switch (Channel)
  {
    case TIM_CHANNEL_1:
      htim->Instance->CCR1 = Compare;
    break;

    case TIM_CHANNEL_2:
      htim->Instance->CCR2 = Compare;
    break;

    case TIM_CHANNEL_3:
      htim->Instance->CCR3 = Compare;
    break;

    case TIM_CHANNEL_4:
      htim->Instance->CCR4 = Compare;
    break;

    default:break;
  }
}