/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : ramp.c
  * @brief          : ramp functions 
  * @author         : Yan Yuanbin
  * @date           : 2023/04/27
  * @version        : v1.0
  ******************************************************************************
  * @attention      : To be perfected
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef RAMP_H
#define RAMP_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "stdbool.h"
#include "stdlib.h"
#include "string.h"
#include "math.h"

/* ==== 文件说明 ================================================================
 * Ramp.h —— 斜坡函数 + 滑动平均滤波头文件
 * ---------------------------------------------------------------------------
 * 作用：提供两个常用工具：
 *   1) f_Ramp_Calc —— 斜坡限幅：让输出以固定速度(ramp)逼近目标值，而不是一步到位，
 *      常用于让电机目标速度/电流平滑爬升，避免突变（机械冲击、超调）。
 *   2) 滑动平均(MovingAverage) —— 对最近 length 个采样取平均，滤除高频噪声。
 *
 * 在整车里的用途：云台/底盘的目标值斜坡生成、传感器采样平滑。
 * ========================================================================== */

/* Exported types ------------------------------------------------------------*/
/**
 * @brief typedef structure that contains the information  for the first order lowpass filter.
 */
typedef struct
{
    bool init;             /*!< init flag 初始化标志 */
    float *filter_buff;    /*!< pointer to the floating-point array of filter buff 存放最近 length 个采样的环形缓冲 */
    uint16_t length;       /*!< the length of filter buff 缓冲区长度（即平均的窗口大小） */
    float input;           /*!< input value 当前输入 */
    float sum;             /*!< sum value 累加和 */
    float output;          /*!< output value 滤波输出（平均值） */
}MovingAverage_Info_TypeDef;

/* Exported functions prototypes ---------------------------------------------*/
/**
  * @brief Calculate the floating-point ramp filter.
  * 中文：斜坡限幅。input 当前值、target 目标值、ramp 每周期最大变化量，返回朝 target 逼近一步后的新值。
  */
extern float f_Ramp_Calc(float input,float target,float ramp);
/**
  * @brief Initializes the moving average filter according to the specified parameters in the
  *         MovingAverage_Info_TypeDef.
  * 中文：初始化滑动平均滤波器。MA 句柄，length 窗口长度（会 malloc length 个 float 的缓冲）。
  */
extern void MovingAverage_Init(MovingAverage_Info_TypeDef *MA,uint16_t length);
/**
  * @brief update the floating-point moving average filter.
  * 中文：更新滑动平均。input 本次输入，返回最近 length 个采样的平均值。
  */
extern float MovingAverage_Update(MovingAverage_Info_TypeDef *MA,float input);

#endif //RAMP_H

