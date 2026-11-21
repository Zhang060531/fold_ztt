#include "User_lib.h"

//control任务的390行
/**
 * @brief 一阶低通滤波器
 * @note  value = temp*alpha + last*(1-alpha)，alpha 越大越跟手(越接近原始值)，
 *        alpha 越小越平滑(越滤高频)。FdbUpdate 里用 alpha=0.8 滤轮速。
 *        注意：内部用 static 变量保存历史值，只适合滤波单个信号，多信号共用会串。
 * @param temp 本次采样值
 * @param alpha 滤波系数 (0~1)
 * @retval 滤波后的值
 */
float filter(float temp, float alpha)
{
    static float value, last_value;
    value = temp * alpha + (1.0f - alpha) * last_value;
    last_value = value;
    return value;
}


/**
 * @brief 将 Input 环绕约束到 [Min_Value, Max_Value] 区间(用于角度环绕)
 * @param Input     输入值
 * @param Min_Value 区间下界
 * @param Max_Value 区间上界
 * @retval 约束后的值
 * @note  若 Max<Min 则直接返回原值; 否则按区间长度循环加减使其落回区间内。
 */
float F_Loop_Constrain(float Input, float Min_Value, float Max_Value)
{
    if (Max_Value < Min_Value) return Input;
    float len = Max_Value - Min_Value;    
    if (Input > Max_Value) { do { Input -= len; } while (Input > Max_Value); }
    else if (Input < Min_Value) { do { Input += len; } while (Input < Min_Value); }
    return Input;
}

//Motor.c的296行
/**
 * @brief MIT 协议解码: 无符号整数 -> 浮点物理量
 * @param X_int 原始无符号整数
 * @param X_min 物理量下界
 * @param X_max 物理量上界
 * @param Bits  位宽(如 12/16)
 * @retval 映射后的浮点物理量 = X_int * (X_max-X_min) / (2^Bits-1) + X_min
 * @note  例如位置 16 位、量程 [-2PI,2PI]: 中间值 0x8000 对应 0 rad。
 */
float uint_to_float(int X_int, float X_min, float X_max, int Bits)
{
    float span = X_max - X_min; float offset = X_min;
    return ((float)X_int)*span/((float)((1<<Bits)-1)) + offset;
}

/**
 * @brief MIT 协议编码: 浮点物理量 -> 无符号整数
 * @param x     物理量
 * @param x_min 物理量下界
 * @param x_max 物理量上界
 * @param bits  位宽(如 12/16)
 * @retval 映射后的无符号整数 = (x - x_min) * (2^bits-1) / (x_max - x_min)
 * @note  与 uint_to_float 互逆; 超出量程会溢出截断, 调用前应保证 x 在量程内。
 */
int float_to_uint(float x, float x_min, float x_max, int bits)
{
    float span = x_max - x_min; float offset = x_min;
    return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}

//bsp_can里的
/* int16 -> float：正确处理符号位(等价于简单强转) */
float Int16ToFloat(int16_t data)
{
    if (data >= 0)
    {
        return (float)data;
    }
    else
    {
        float temp = (float) - data;
        return -temp;
    }
}

/* int16 -> int32：正确处理符号位 */
int Int16ToInt32(int16_t data)
{
    if (data >= 0)
    {
        return (int)data;
    }
    else
    {
        int temp = (int) - data;
        return -temp;
    }
}