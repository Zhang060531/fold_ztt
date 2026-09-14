/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : lowpass_filter.c
  * @brief          : lowpass filter 
  * @author         : Yan Yuanbin
  * @date           : 2023/04/27
  * @version        : v1.0
  ******************************************************************************
  * @attention      : To be perfected
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef LOWPASS_FILTER_H
#define LOWPASS_FILTER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "config.h"

/* ==== 文件说明 ================================================================
 * LPF.h —— 一阶/二阶低通滤波器头文件
 * ---------------------------------------------------------------------------
 * 作用：低通滤波——让"低频信号"通过、衰减"高频毛刺"。传感器数据（陀螺/电流等）
 *       常带高频噪声，滤波后更平滑，适合喂给控制环。
 *
 * 两种滤波器：
 *   一阶 LPF : y(k) = alpha·y(k-1) + (1-alpha)·x(k)
 *      alpha 越接近 1 越平滑但滞后越大；alpha 越接近 0 越跟手但越抖。
 *   二阶 LPF : y(k) = a0·y(k-1) + a1·y(k-2) + a2·x(k)
 *      用了过去两次输出，滚降更陡、滤高频更狠，相位滞后也更明显。
 *
 * 在整车里的用途：云台速度/电流反馈、编码器速度等信号的平滑滤波。
 * ========================================================================== */

/* Exported types ------------------------------------------------------------*/
/**
 * @brief  一阶低通滤波器信息的结构体
 */
typedef struct
{
	bool Initialized;       //初始化标志 0:未初始化 1：初始化
    float Input;          //当前输入
    float Output;         //输出
    float Alpha;          //滤波系数（0~1，越大越平滑、滞后越大；越小越跟手、越抖）
}LowPassFilter1p_Info_TypeDef;

/**
 * @brief 二阶低通滤波器信息的结构体.
 */

typedef struct 
{
    bool Initialized;  //初始化标志 0:未初始化 1：初始化
    float Input;       //当前输入
    float Output[3];   //输出 现在和过去两次的输出
    float Alpha[3];    //二阶滤波器系数 a0,a1,a2（分别对应 y(k-1)、y(k-2)、x(k) 的权重）

}LowPassFilter2p_Info_TypeDef;

/* Extern Functions Prototypes ---------------------------------------------*/

/**
  * @brief 根据函数中指定的参数初始化一阶低通滤波器.
  * @param LPF 滤波器句柄；@param Alpha 滤波系数(0~1，越大越平滑、滞后越大)。
  */
extern void LowPassFilter1p_Init(LowPassFilter1p_Info_TypeDef *LPF,float Alpha);
/**
  * @brief 根据函数中指定的参数更新（计算）一阶低通滤波器
  * @param lpf 滤波器句柄；@param input 本次输入；@retval 滤波后的输出。
  * 公式：y(k) = alpha·y(k-1) + (1-alpha)·x(k)。
  */
extern float LowPassFilter1p_Update(LowPassFilter1p_Info_TypeDef *lpf,float input);
/**
  * @brief 根据函数中指定的参数初始化二阶低通滤波器
  * @param lpf 滤波器句柄；@param alpha[3] 二阶系数 a0,a1,a2。
  */
extern void  LowPassFilter2p_Init(LowPassFilter2p_Info_TypeDef *lpf,float alpha[3]);
/**
  * @brief 根据函数中指定的参数更新（计算）二阶低通滤波器
  * @param lpf 滤波器句柄；@param input 本次输入；@retval 滤波后的输出。
  * 公式：y(k) = a0·y(k-1) + a1·y(k-2) + a2·x(k)。
  */
extern float LowPassFilter2p_Update(LowPassFilter2p_Info_TypeDef *lpf,float input);

/* sign：符号函数，返回 +1 / 0 / -1，常用于提取"方向"。 */
extern float sign(float input);
#endif //LOWPASS_FILTER_H
