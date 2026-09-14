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

/* Includes ------------------------------------------------------------------*/
#include "ramp.h"

/* ==== 文件说明 ================================================================
 * Ramp.c —— 斜坡函数 + 滑动平均滤波实现
 * ---------------------------------------------------------------------------
 * 作用：
 *   1) f_Ramp_Calc    斜坡限幅：每周期最多朝目标移动 ramp，让输出平滑逼近目标。
 *   2) MovingAverage  滑动平均：对最近 length 个采样取平均，滤除高频噪声。
 * 被谁调用：云台/底盘的目标值斜坡生成、传感器采样平滑。
 * ========================================================================== */

/* Private define ------------------------------------------------------------*/
/**
  * @brief Calculate the floating-point ramp filter.
  * @param input: the filter input variables
  * @param target: the input variables target value
  * @param ramp: the ramp slope
  * @retval the filter output
  */
/* f_Ramp_Calc —— 斜坡限幅
 * 作用：让输出每次最多朝目标移动 ramp，剩余不足 ramp 就一次到位。
 * 参数：input 当前值，target 目标值，ramp 每周期最大变化量（斜率）。
 * 返回：朝 target 靠近一步之后的新值。
 * 用途：让目标值平滑爬升，避免电机目标阶跃（阶跃会导致冲击/超调）。 */
float f_Ramp_Calc(float input,float target,float ramp)
{
  float error = target - input;
  float output = input;

	if (error > 0){
        if (error > ramp){output += ramp;}   
        else{output += error;}
    }else{
        if (error < -ramp){output += -ramp;}
        else{output += error;}
    }

    return output;
}
//------------------------------------------------------------------------------

/**
  * @brief Initializes the moving average filter according to the specified parameters in the
  *         MovingAverage_Info_TypeDef.
  * @param MA: pointer to an MovingAverage_Info_TypeDef structure that
  *         contains the information  for the moving average filter.
  * @param length: the length of filter buff
  * @retval none
  */
/* MovingAverage_Init —— 初始化滑动平均
 * 参数：MA 句柄，length 窗口长度。会 malloc 一段 length 个 float 的缓冲并清零。 */
void MovingAverage_Init(MovingAverage_Info_TypeDef *MA,uint16_t length)
{

  MA->length = length;

  MA->filter_buff = malloc(sizeof(float)*MA->length);
  memset(MA->filter_buff,0,sizeof(float)*MA->length);

  if(MA->filter_buff == NULL)
  {
    return ;
  }

  MA->input = 0;
  MA->output = 0;

  MA->init = true;
}
//------------------------------------------------------------------------------


/**
  * @brief Calculate the floating-point moving average filter.
  * @param MA: pointer to an MovingAverage_Info_TypeDef structure that
  *         contains the information  for the moving average filter.
  * @param input: the input variable
  * @retval the filter output
  */
/* MovingAverage_Update —— 更新滑动平均
 * 作用：把新采样放进缓冲、算出最近 length 个采样的平均值返回。
 * 注意：本实现的"移动缓冲"是向上搬运(与常见向下移动方向相反)，且求和只累加
 *       length-1 个元素——作为教学代码请留意这些细节，实际使用按需核对。 */
float MovingAverage_Update(MovingAverage_Info_TypeDef *MA,float input)
{
  if(MA->init != true)
  {
    return 0;
  }

  /* moving the filter buff 把旧数据整体向后挪一格，腾出 buff[0] 放新采样 */
  for(uint16_t i = 0; i < MA->length-1; i++)
  {
      MA->filter_buff[i+1] = MA->filter_buff[i];
  }

  /* update the filter input */
  MA->filter_buff[0] = input;

  /* calculate the average 累加窗口内的采样（注意：只累加了 length-1 个元素） */
  for(uint16_t i = 0; i < MA->length-1; i++)
  {
      MA->sum += MA->filter_buff[i];
  }

  MA->output = (float)(MA->sum / MA->length);
	
	MA->sum = 0;

  return MA->output;
}
//------------------------------------------------------------------------------


