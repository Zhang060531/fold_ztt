/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : PID.c
  * @brief          : PID functions with Advanced Algorithms
  * @date           : 2024/12/29 (Updated)
  * @version        : v2.0
  ******************************************************************************
  * @attention      : 增加了前馈、积分分离、Bang-Bang、微分先行和模糊PID等功能
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "PID.h"
#include <string.h> // for memset
/* Includes ------------------------------------------------------------------*/

//======================= 模糊 PID 规则库 =======================
// PB=3, PM=2, PS=1, ZO=0, NS=-1, NM=-2, NB=-3
// 以数组下标 0~6 分别对应 NB~PB
// 这是一套经典的 7x7 模糊控制规则库

// Delta Kp 规则库: 误差大时需要大的Kp来减小误差，误差小时减小Kp防止超调
static const float Fuzzy_Rule_dKp[7][7] = {
/* EC: NB, NM, NS, ZO, PS, PM, PB */
/* E: NB */ { 3,  3,  2,  2,  1,  0,  0 },
/* E: NM */ { 3,  3,  2,  1,  1,  0, -1 },
/* E: NS */ { 2,  2,  2,  1,  0, -1, -1 },
/* E: ZO */ { 2,  2,  1,  0, -1, -2, -2 },
/* E: PS */ { 1,  1,  0, -1, -1, -2, -2 },
/* E: PM */ { 1,  0, -1, -2, -2, -2, -3 },
/* E: PB */ { 0,  0, -2, -2, -2, -3, -3 }
};

// Delta Ki 规则库: 误差大时减小Ki防止积分超调，误差小时增大Ki以消除静差
static const float Fuzzy_Rule_dKi[7][7] = {
/* EC: NB, NM, NS, ZO, PS, PM, PB */
/* E: NB */ {-3, -3, -2, -2, -1,  0,  0 },
/* E: NM */ {-3, -3, -2, -1, -1,  0,  0 },
/* E: NS */ {-3, -2, -1, -1,  0,  1,  1 },
/* E: ZO */ {-2, -2, -1,  0,  1,  2,  2 },
/* E: PS */ {-2, -1,  0,  1,  1,  2,  3 },
/* E: PM */ { 0,  0,  1,  1,  2,  3,  3 },
/* E: PB */ { 0,  0,  1,  2,  2,  3,  3 }
};

// Delta Kd 规则库: 抑制误差变化率，误差大时降低Kd响应速度，误差居中时增大Kd抑制超调
static const float Fuzzy_Rule_dKd[7][7] = {
/* EC: NB, NM, NS, ZO, PS, PM, PB */
/* E: NB */ { 1, -1, -3, -3, -3, -2,  1 },
/* E: NM */ { 1, -1, -3, -2, -2, -1,  0 },
/* E: NS */ { 0, -1, -2, -1, -1,  0,  0 },
/* E: ZO */ { 0, -1, -1,  0,  1,  1,  1 },
/* E: PS */ { 0,  0,  0,  1,  1,  2,  2 },
/* E: PM */ { 2,  1,  1,  2,  2,  3,  3 },
/* E: PB */ { 3,  2,  2,  3,  3,  3,  3 }
};
//==============================================================

/**
 * @brief 初始化PID参数
 */
static PID_Status_e PID_Param_Init(PID_Info_TypeDef *PID, float Param[PID_PARAMETER_NUM])
{
    if(PID->Type == PID_Type_None || Param == NULL)
    {
        return PID_FAILED_INIT;
    }
    
    // 初始化基础 PID 参数
    PID->Param.KP = Param[0];
    PID->Param.KI = Param[1];
    PID->Param.KD = Param[2];
    PID->Param.Alpha = Param[3];
    
    if( PID->Param.Alpha > 0.f &&  PID->Param.Alpha < 1.f) 
    {
        LowPassFilter1p_Init(&PID->Dout_LPF, PID->Param.Alpha);
    }
        
    PID->Param.Deadband = Param[4];
    PID->Param.LimitIntegral = Param[5];
    PID->Param.LimitOutput = Param[6];
    
    // 初始化高级拓展参数
    PID->Param.KF = Param[7];               // 前馈参数
    PID->Param.IntSeparationErr = Param[8]; // 积分分离阈值
    PID->Param.BangBangErr = Param[9];      // 起停控制阈值
    PID->Param.Fuzzy_ErrMax = Param[10];    // 模糊控制最大误差绝对值
    PID->Param.Fuzzy_dErrMax = Param[11];   // 模糊控制最大误差差分绝对值

    PID->Real_KP = PID->Param.KP;
    PID->Real_KI = PID->Param.KI;
    PID->Real_KD = PID->Param.KD;

    // 清除PID错误计数
    PID->ERRORHandler.ErrorCount = 0;

    return PID_ERROR_NONE;
}

/**
 * @brief 清除PID计算值，将所有输出赋0.
 */
void PID_Calc_Clear(PID_Info_TypeDef *PID)
{
    memset(PID->Err, 0, sizeof(PID->Err));
    PID->Measure_Prev = 0;
    PID->Integral = 0;
        
    PID->Pout = 0;
    PID->Iout = 0;
    PID->Dout = 0;
    PID->Fout = 0;
    PID->Output = 0;
}

/**
 * @brief 初始化PID控制器.
 */
void PID_Init(PID_Info_TypeDef *PID, PID_Type_e Type, float Param[PID_PARAMETER_NUM])
{
    PID->Type = Type;
    PID->PID_Calc_Clear = PID_Calc_Clear;
    PID->PID_Param_Init = PID_Param_Init;

    PID->PID_Calc_Clear(PID);
    PID->ERRORHandler.Status = PID->PID_Param_Init(PID, Param);
}

/**
 * @brief 判断PID错误状态
 */
void PID_ErrorHandle(PID_Info_TypeDef *PID)
{
    if(isnan(PID->Output) || isinf(PID->Output))
    {
        PID->ERRORHandler.Status = PID_CALC_NANINF;
    }
}

/**
 * @brief PID控制器计算函数 (整合多种高级算法)
 */
float PID_Calculate(PID_Info_TypeDef *PID, float Target, float Measure)
{       
    PID_ErrorHandle(PID);
    if(PID->ERRORHandler.Status != PID_ERROR_NONE)
    {
        PID->PID_Calc_Clear(PID);
        return 0;
    }
  
    // 更新实际值和上次实际值
    PID->Measure_Prev = PID->Measure;
    PID->Target = Target;
    PID->Measure = Measure;

    // 误差数组递推推入：0(当前)，1(上次)，2(上上次)
    PID->Err[2] = PID->Err[1];
    PID->Err[1] = PID->Err[0];
    PID->Err[0] = PID->Target - PID->Measure;

    // 计算前馈分量 (前馈通常基于目标值直接计算)
    PID->Fout = PID->Param.KF * PID->Target;
        
    // 检查死区
    if(fabsf(PID->Err[0]) < PID->Param.Deadband)
    {
        return PID->Output; // 误差在死区范围内，维持上次输出
    }

    //---------------------------------------------------------
    // 算法逻辑分支
    //---------------------------------------------------------
    
    // 1. Bang-Bang (起停式) PID 特殊判断
    if (PID->Type == PID_BANG_BANG)
    {
        if (fabsf(PID->Err[0]) > PID->Param.BangBangErr)
        {
            // 误差超大，直接全速输出
            PID->Output = (PID->Err[0] > 0) ? PID->Param.LimitOutput : -PID->Param.LimitOutput;
            PID->Integral = 0; // 同时清除积分防止切入PID时饱和
            return PID->Output;
        }
        // 否则继续执行后半部分的常规计算逻辑 (按位置式计算)
    }

    // 2. Fuzzy (模糊自适应) PID 动态计算参数
    if (PID->Type == PID_FUZZY && PID->Param.Fuzzy_ErrMax > 0 && PID->Param.Fuzzy_dErrMax > 0)
    {
        float e_norm  = (PID->Err[0] / PID->Param.Fuzzy_ErrMax) * 3.0f;
        float ec_norm = ((PID->Err[0] - PID->Err[1]) / PID->Param.Fuzzy_dErrMax) * 3.0f;
        
        // 四舍五入到下标 0~6 (-3对应0, 0对应3, 3对应6)
        int e_idx  = (int)roundf(e_norm) + 3;
        int ec_idx = (int)roundf(ec_norm) + 3;
        
        // 安全截断
        VAL_LIMIT(e_idx, 0, 6);
        VAL_LIMIT(ec_idx, 0, 6);

        // 从规则表中获取模糊调整量并按比例映射回真实参数 (假设规则最大调节幅度为基准值的约20%)
        PID->Real_KP = PID->Param.KP + PID->Param.KP * 0.2f * (Fuzzy_Rule_dKp[e_idx][ec_idx] / 3.0f);
        PID->Real_KI = PID->Param.KI + PID->Param.KI * 0.2f * (Fuzzy_Rule_dKi[e_idx][ec_idx] / 3.0f);
        PID->Real_KD = PID->Param.KD + PID->Param.KD * 0.2f * (Fuzzy_Rule_dKd[e_idx][ec_idx] / 3.0f);
    }
    else
    {
        PID->Real_KP = PID->Param.KP;
        PID->Real_KI = PID->Param.KI;
        PID->Real_KD = PID->Param.KD;
    }

    //---------------------------------------------------------
    // 执行 PID 主计算逻辑
    //---------------------------------------------------------
    if(PID->Type == PID_VELOCITY)
    {
        /* 增量式 (Velocity) PID 计算 */
        PID->Pout = PID->Real_KP * (PID->Err[0] - PID->Err[1]);
        PID->Iout = PID->Real_KI * PID->Err[0];
        PID->Dout = PID->Real_KD * (PID->Err[0] - 2.f * PID->Err[1] + PID->Err[2]);
        
        if( PID->Param.Alpha > 0.f && PID->Param.Alpha < 1.f ){
            PID->Dout_LPF.Alpha = PID->Param.Alpha;
            PID->Dout = LowPassFilter1p_Update(&PID->Dout_LPF, PID->Dout);
        }
        
        // 增量式直接叠加在原输出上
        PID->Output += PID->Pout + PID->Iout + PID->Dout + PID->Fout;
    }
    else 
    {
        /* 位置式 (Position) 以及其变体计算 */
        
        // 积分分离 PID 处理：误差过大时挂起积分
        if (PID->Type == PID_INTEGRAL_SEPARATION && fabsf(PID->Err[0]) > PID->Param.IntSeparationErr)
        {
            // 积分被挂起，不累加
        }
        else
        {
            if(PID->Real_KI != 0) {
                PID->Integral += PID->Err[0];
            } else {
                PID->Integral = 0;
            }
        }
        VAL_LIMIT(PID->Integral, -PID->Param.LimitIntegral, PID->Param.LimitIntegral);
        
        // 计算 P、I 项
        PID->Pout = PID->Real_KP * PID->Err[0];
        PID->Iout = PID->Real_KI * PID->Integral;
        
        // 微分先行 PID 处理：微分计算不带入目标变化率
        if (PID->Type == PID_DERIVATIVE_ON_MEASURE)
        {
            // Measure_Prev - Measure 代替 Err - Err_prev
            PID->Dout = PID->Real_KD * (PID->Measure_Prev - PID->Measure);
        }
        else
        {
            PID->Dout = PID->Real_KD * (PID->Err[0] - PID->Err[1]);
        }
        
        // 微分项低通滤波
        if( PID->Param.Alpha > 0.f && PID->Param.Alpha < 1.f ){
             PID->Dout_LPF.Alpha = PID->Param.Alpha;
             PID->Dout = LowPassFilter1p_Update(&PID->Dout_LPF, PID->Dout);
        }
        
        // 组合最终输出
        PID->Output = PID->Pout + PID->Iout + PID->Dout + PID->Fout;
    }

    // 最终总输出限幅
    VAL_LIMIT(PID->Output, -PID->Param.LimitOutput, PID->Param.LimitOutput);
    return PID->Output;
}