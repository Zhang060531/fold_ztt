/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : PID.h
  * @brief          : PID functions with Advanced Algorithms
  * @date           : 2024/12/29 (Updated)
  * @version        : v2.0
  ******************************************************************************
  * @attention      : Added Feedforward, Integral Separation, Bang-Bang, 
  * Derivative-on-Measure, and Fuzzy PID.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CONTROLLER_PID_H
#define CONTROLLER_PID_H

/* Includes ------------------------------------------------------------------*/
#include "config.h" // 请确保包含标准库，如 <stdint.h>, <math.h>, <string.h> 等
#include "lpf.h"
#include <stdbool.h>
#include <math.h>

/* Exported defines -----------------------------------------------------------*/
/**
 * @brief 限制变量在指定范围内的宏
 */
#define VAL_LIMIT(x,min,max)  do{ \
                                    if ((x) > (max)) {(x) = (max);} \
                                    else if ((x) < (min)) {(x) = (min);} \
                                }while(0U)

/**
 * @brief PID 参数的数量，已扩展以支持高级算法
 */
#ifndef PID_PARAMETER_NUM 
#define PID_PARAMETER_NUM 12                            
#endif

/* Exported types ------------------------------------------------------------*/
/**
 * @brief PID 控制器错误状态枚举
 */
typedef enum
{
    PID_ERROR_NONE = 0x00U,        /*!< 无错误 */
    PID_FAILED_INIT = 0x01U,       /*!< 初始化失败 */
    PID_CALC_NANINF = 0x02U,       /*!< 出现 NaN 或 Infinity */
    PID_Status_NUM,
} PID_Status_e;

/**
 * @brief PID 控制器类型枚举（包含各种进阶和衍生变体）
 */
typedef enum
{
    PID_Type_None = 0x00U,               /*!< 无类型 */
    PID_POSITION = 0x01U,                /*!< 常规位置式 PID */
    PID_VELOCITY = 0x02U,                /*!< 常规增量式 (速度) PID */
    PID_DERIVATIVE_ON_MEASURE = 0x03U,   /*!< 微分先行 PID (防止目标突变产生的微分冲击) */
    PID_INTEGRAL_SEPARATION = 0x04U,     /*!< 积分分离 PID (防止大幅度调整时的积分饱和) */
    PID_BANG_BANG = 0x05U,               /*!< 起停式/变结构 PID (大误差全速，小误差PID) */
    PID_FUZZY = 0x06U,                   /*!< 模糊自适应 PID (根据规则库动态调整参数) */
    PID_TYPE_NUM,
} PID_Type_e;

/**
 * @brief PID 错误处理结构体
 */
typedef struct
{
    uint16_t ErrorCount;    /*!< 错误状态判定计数 */
    PID_Status_e Status;    /*!< 错误状态 */
} PID_ErrorHandler_Typedef;

/**
 * @brief PID 控制器参数结构体
 */
typedef struct
{
    float KP;               // 比例系数 (对于模糊PID作为基准Kp)
    float KI;               // 积分系数 (对于模糊PID作为基准Ki)
    float KD;               // 微分系数 (对于模糊PID作为基准Kd)
    float Alpha;            // 微分一阶低通滤波器系数 (0~1)
    float Deadband;         // 死区：当误差绝对值小于此值，停止调整
    float LimitIntegral;    // 积分限幅
    float LimitOutput;      // 总输出限幅
    
    // ----- 进阶拓展参数 -----
    float KF;               // 前馈系数 (Feedforward Gain, 通常乘以目标值)
    float IntSeparationErr; // 积分分离阈值 (仅PID_INTEGRAL_SEPARATION有效)
    float BangBangErr;      // BangBang控制阈值 (仅PID_BANG_BANG有效)
    float Fuzzy_ErrMax;     // 模糊PID：最大预期误差绝对值 (用于归一化 E)
    float Fuzzy_dErrMax;    // 模糊PID：最大预期误差变化率绝对值 (用于归一化 EC)
} PID_Parameter_Typedef;

/**
 * @brief PID 控制器主信息结构体
 */
typedef struct _PID_TypeDef
{
    PID_Type_e Type;        // PID类型
    
    float Target;           // 目标值
    float Measure;          // 当前实际值
    float Measure_Prev;     // 上一次实际值 (微分先行等算法需用)
    
    float Err[3];           // 误差数组: Err[0]当前误差, Err[1]上次误差, Err[2]上上次误差
    float Integral;         // 误差积分值
    float Pout;             // 比例输出
    float Iout;             // 积分输出
    float Dout;             // 微分输出
    float Fout;             // 前馈输出
    float Output;           // 总输出
    
    // 模糊 PID 动态调节出的实际参数记录（便于调试与监测）
    float Real_KP;          
    float Real_KI;          
    float Real_KD;

    LowPassFilter1p_Info_TypeDef Dout_LPF;  // 微分输出的一阶低通滤波器
    
    PID_Parameter_Typedef Param;            // PID参数结构体
    PID_ErrorHandler_Typedef ERRORHandler;  // PID错误处理结构体

    /**
     * @brief 初始化PID参数的函数指针
     */
    PID_Status_e (*PID_Param_Init)(struct _PID_TypeDef *PID, float *Param);

    /**
     * @brief 清除PID计算过程量的函数指针
     */
    void (*PID_Calc_Clear)(struct _PID_TypeDef *PID);
                
} PID_Info_TypeDef;

/* Exported functions prototypes ---------------------------------------------*/
/**
 * @brief 初始化 PID 控制器
 */
extern void PID_Init(PID_Info_TypeDef *Pid, PID_Type_e type, float para[PID_PARAMETER_NUM]);

/**
 * @brief PID 计算主体函数
 */
extern float PID_Calculate(PID_Info_TypeDef *PID, float Target, float Measure);
extern void PID_Calc_Clear(PID_Info_TypeDef *PID);
extern void PID_ErrorHandle(PID_Info_TypeDef *PID);

#endif //CONTROLLER_PID_H