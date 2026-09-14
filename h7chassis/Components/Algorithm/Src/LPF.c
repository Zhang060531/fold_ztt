/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : LPF.c
  * @brief          : lowpass filter 
  * @author         : GrassFan Wang
  * @date           : 2025/12/28
  * @version        : v1.0
  ******************************************************************************
  * @attention      : To be perfected
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "LPF.h"

/* ==== 文件说明 ================================================================
 * LPF.c —— 一阶/二阶低通滤波器实现
 * ---------------------------------------------------------------------------
 * 作用：实现低通滤波，让高频毛刺衰减、保留低频有用信号。
 *   一阶: y(k) = alpha·y(k-1) + (1-alpha)·x(k)
 *   二阶: y(k) = a0·y(k-1) + a1·y(k-2) + a2·x(k)
 * 被谁调用：云台控制环里对速度/电流等反馈量做平滑。
 * ========================================================================== */

/* sign：符号函数，返回 +1 / 0 / -1。常用于提取"方向"（例如速度方向判断）。
 * 实现技巧：(input>0) 为真时是 1，为假是 0，两者相减即得 -1/0/+1。 */
float sign(float input){
   return (input>0.0f) - (input<0.0f);

}

/**
  * @brief 初始化一阶低通滤波器.
  * @param LPF: 一阶低通滤波器结构体.
  * @param Alpha: 滤波器系数.
  * @param Frame_Period: 采样周期.
  * @retval 无.
  */
void LowPassFilter1p_Init(LowPassFilter1p_Info_TypeDef *LPF,float Alpha)
{
  LPF->Alpha = Alpha;
  LPF->Input = 0;
  LPF->Output = 0;
}

/**
  * @brief 更新一阶低通滤波器数据.
  * @param Input: 当前输入.
  * @retval 滤波后的值.
  */
float LowPassFilter1p_Update(LowPassFilter1p_Info_TypeDef *LPF,float Input)
{
  LPF->Input = Input;

  if(LPF->Initialized == false)
  {
    LPF->Output = LPF->Input;//第一次进入更新函数 输出等于这次输入
    LPF->Initialized = true;
  }

	/*滤波器系数 = Alhpa 0< Alhpa <1 当滤波系数越小，滤波曲线越平稳，但是滞后性更大。
                                   当滤波系数越大，滤波曲线越接近实际值，滞后性小，但是滤波曲线更抖 */
	
	//滤波值 = Alhpa * 上一次输出 + （1 - Alhpa)*这次输入	
  LPF->Output = LPF->Alpha * LPF->Output +(1.f - LPF->Alpha) * LPF->Input;
	
  return LPF->Output;
}

/**
  * @brief 初始化二阶低通滤波器.
  * @param Alpha[3]: 滤波器系数[3].
  * @retval 无.
  */
void LowPassFilter2p_Init(LowPassFilter2p_Info_TypeDef *LPF,float Alpha[3])
{
  memcpy(LPF->Alpha,Alpha,sizeof(LPF->Alpha));
  LPF->Input = 0;
  memset(LPF->Output,0,sizeof(LPF->Output));
}

/**
  * @brief 更新二阶低通滤波器数据.
  * @param Input: 当前输入.
  * @retval 滤波后的值.
  */
float LowPassFilter2p_Update(LowPassFilter2p_Info_TypeDef *LPF,float Input)
{
	LPF->Input = Input;
  
  if(LPF->Initialized == false)
  {
    LPF->Output[0] = LPF->Input;
    LPF->Output[1] = LPF->Input;
    LPF->Output[2] = LPF->Input;
    LPF->Initialized = true;
  }
  
	LPF->Output[0] = LPF->Output[1];
	LPF->Output[1] = LPF->Output[2];
  /* 二阶递推：把过去输出后移一格，再用 a0·y(k-1) + a1·y(k-2) + a2·x(k) 算新输出 */
  LPF->Output[2] = LPF->Alpha[0] * LPF->Output[1] + LPF->Alpha[1] * LPF->Output[0] + LPF->Alpha[2] * LPF->Input;

	return LPF->Output[2];
}
//------------------------------------------------------------------------------

