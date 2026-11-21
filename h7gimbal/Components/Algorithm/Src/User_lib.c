#include "User_lib.h"

/**
 * @brief 斜坡函数：让速度从 last_speed 平滑逼近 target_speed
 * @note  每次最多变化 max_step，避免速度突变。
 *        目标速度为 0 时把步长放大 3 倍（刹车更快更果断）。
 *        调车时 max_step 越大响应越快但越容易顿挫。
 */
float ramp_speed(float last_speed, float target_speed, float max_step)
{

    float delta = target_speed - last_speed;


    if (target_speed == 0)
        max_step *= 3;
    if (delta > max_step)
    {
        return last_speed + max_step;
    }
    else if (delta < -max_step)
    {
        return last_speed - max_step;
    }
    else
    {
        return target_speed; 
    }

}

/* Existing functions (unchanged) */
//------------------------------------------------------------------------------

float F_Loop_Constrain(float Input, float Min_Value, float Max_Value)
{
    if (Max_Value < Min_Value)
    {
        return Input;
    }
       
    float len = Max_Value - Min_Value;    

    if (Input > Max_Value)
    {
        do{
            Input -= len;
        }while (Input > Max_Value);
    }
    else if (Input < Min_Value)
    {
        do{
            Input += len;
        }while (Input < Min_Value);
    }
    return Input;
}

void ConstrainLoop(float* output, float input, float min, float max)
{
    if (input > max)
    {
        do
        {
            input = input - (max - min);
        }while(input > max);
    }
    else if(input < min)
    {
        do
        {
            input = input + (max - min);
        }while(input < min);
    }
    
    *output = input;
}

// 无符号整数 → 浮点物理量（MIT 协议解码）
// 把 [0, 2^Bits-1] 的整数按量程 [X_min, X_max] 线性映射回浮点数
float uint_to_float(int X_int, float X_min, float X_max, int Bits)
{
    float span = X_max - X_min;      // 量程跨度
    float offset = X_min;            // 量程下界
    return ((float)X_int)*span/((float)((1<<Bits)-1)) + offset;
}

// 浮点物理量 → 无符号整数（MIT 协议编码，是 uint_to_float 的逆过程）
int float_to_uint(float x, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}

// 分段线性映射：把输入区间 [in_min,in_max]（零点 in_zero）映射到输出区间
// [out_min,out_max]（零点 out_zero），并处理输入过零跳变，结果用循环限幅到输出区间
void SectionTransform(float* output, float input, float out_min, float out_zero, float out_max, float in_min, float in_zero, float in_max)
{
    float temp_in = input - in_zero;
    float temp_out = 0;
    
    if(temp_in > (in_max - in_min) / 2)
    {
        temp_in = temp_in - (in_max - in_min);
    }
    else if(temp_in < -(in_max - in_min) / 2)
    {
        temp_in = temp_in + (in_max - in_min);
    }
    
    temp_out = temp_in * (out_max - out_min) / (in_max - in_min) + out_zero;
    ConstrainLoop(output, temp_out, out_min, out_max);
}

void MinMaxConstrain(float* input, float min, float max)
{
    if(*input > max)
    {
        *input = max;
    }
    else if(*input < min)
    {
        *input = min;
    }
}



/**
  * @brief 防止PITCH、YAW轴绕大圈达到设定值，计算出一个临时的设定值
  * @param 角度设定值
  * @param 角度反馈值
  * * @retval 临时角度设定值
  */
float PreventFullCircleConstrain(float set, float fdb)
{
    if(set - fdb > 180.0f)
        return set - 360.0f; 
    else if(set - fdb < -180.0f)
        return set + 360.0f; 
    else
        return set;
}


float filter_LPF1(float temp,float alpha) 
{
    static float value,last_value;

    value = temp * alpha + (1.0 - alpha) * last_value;
    last_value=value;

    return value;

}


