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

/* =====
 * 文件: PID.c —— 通用 PID 控制器实现
 * 作用: 实现 PID.h 中声明的 PID_Init/PID_Calculate, 内含 7x7 模糊规则库、
 *       参数初始化、计算量清零、错误检测, 以及整合 Bang-Bang/模糊/积分分离/
 *       微分先行/增量式等算法的计算主体。
 * 结构/调用关系:
 *   - PID_Init() -> PID_Calc_Clear() + PID_Param_Init()
 *   - PID_Calculate() 按 Type 分支选择算法, 最终输出经 VAL_LIMIT 限幅
 *   - 仅被上层电机环(位置环/速度环)调用, 不直接操作硬件
 * ===== 调车关注 =====
 *   - 模糊 PID 实际调节幅度为基准值的 ±20%(代码中的 0.2f), 想更激进可改这里。
 *   - 增量式(VELOCITY)会累加 Output, 需配合输出限幅, 停止前调用 PID_Calc_Clear 清零。
 *   - 死区命中时直接返回上次输出, 别让 Deadband 设太大导致小误差被忽略。
 * =====
 */

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
 * @brief 初始化 PID 参数(把外部传入的 12 个参数写入控制器, 并按需初始化微分低通滤波器)
 * @param PID   待初始化的 PID 控制器句柄
 * @param Param 参数数组, 长度 PID_PARAMETER_NUM(12), 顺序为:
 *              [0]KP [1]KI [2]KD [3]Alpha [4]Deadband [5]LimitIntegral
 *              [6]LimitOutput [7]KF [8]IntSeparationErr [9]BangBangErr
 *              [10]Fuzzy_ErrMax [11]Fuzzy_dErrMax
 * @retval PID_ERROR_NONE  初始化成功
 * @retval PID_FAILED_INIT 类型为 None 或参数指针为空, 初始化失败
 * @note  Alpha 在 (0,1) 开区间内才会真正启用微分低通滤波, 否则关闭。
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
 * @brief 清零 PID 全部计算过程量(误差、积分、各分项与总输出)
 * @param PID 控制器句柄
 * @note  在目标突变、模式切换或停止输出前调用, 可防止积分/微分残留造成跳变。
 */
static void PID_Calc_Clear(PID_Info_TypeDef *PID)
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
 * @brief 初始化 PID 控制器(设置类型、绑定函数指针、清零并载入参数)
 * @param PID   控制器句柄
 * @param Type  控制算法类型(位置式/增量式/微分先行/积分分离/Bang-Bang/模糊)
 * @param Param 参数数组, 顺序见 PID_Param_Init 注释
 * @note  先 PID_Calc_Clear 再 PID_Param_Init, 并把结果写入 ERRORHandler.Status。
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
 * @brief 检测 PID 输出是否出现 NaN/Inf, 并置相应错误状态
 * @param PID 控制器句柄
 * @note  仅在输出为非法浮点数时标记 PID_CALC_NANINF, 由 PID_Calculate 检测后清空输出。
 */
static void PID_ErrorHandle(PID_Info_TypeDef *PID)
{
    if(isnan(PID->Output) || isinf(PID->Output))
    {
        PID->ERRORHandler.Status = PID_CALC_NANINF;
    }
}

/**
 * @brief PID 计算主体(整合位置式/增量式/微分先行/积分分离/Bang-Bang/模糊)
 * @param PID     控制器句柄
 * @param Target  目标值(量纲取决于被控对象, 如目标速度 rpm / 目标位置编码器值)
 * @param Measure 实际测量值(与 Target 同量纲)
 * @retval 计算后的控制量(已做输出限幅); 若出现错误则清零并返回 0
 * @note
 *   - 前馈 Fout = KF * Target, 与误差无关, 用于目标突变时的快速响应。
 *   - 位置式: 输出 = KP*E + KI*ΣE + KD*ΔE + 前馈(绝对量)。
 *   - 增量式(VELOCITY): 输出为增量, 会累加到上次 Output 上。
 *   - 微分先行(DERIVATIVE_ON_MEASURE): 用 -(Measure-Measure_prev) 计算微分,
 *     避免目标值阶跃带来的微分冲击。
 *   - Bang-Bang: |误差| 超过阈值时直接输出满幅 LimitOutput, 并清零积分。
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
    
    // 1. Bang-Bang (起停式) PID 特殊判断: 大误差时直接输出满幅(全速逼近),
    //    小误差时才转入常规 PID 平滑收敛, 兼顾响应速度与稳态精度。
    if (PID->Type == PID_BANG_BANG)
    {
        if (fabsf(PID->Err[0]) > PID->Param.BangBangErr)
        {
            // 误差超大，直接全速输出(符号跟随误差方向)
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

        // 从规则表取模糊调整量(规则值∈[-3,3])除以 3 归一化到 [-1,1],
        // 再乘 0.2f 得到 ±20% 的调节幅度, 叠加到基准参数上得到本次实际使用的 Kp/Ki/Kd。
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
        /* 增量式 (Velocity) PID 计算 —— 下面三项是"增量", 叠加到上次输出上 */
        PID->Pout = PID->Real_KP * (PID->Err[0] - PID->Err[1]);              /* 比例增量: KP*(Ek - Ek-1) */
        PID->Iout = PID->Real_KI * PID->Err[0];                              /* 积分增量: KI*Ek */
        PID->Dout = PID->Real_KD * (PID->Err[0] - 2.f * PID->Err[1] + PID->Err[2]); /* 微分增量: KD*(Ek - 2Ek-1 + Ek-2) */
        
        if( PID->Param.Alpha > 0.f && PID->Param.Alpha < 1.f ){
            PID->Dout_LPF.Alpha = PID->Param.Alpha;
            PID->Dout = LowPassFilter1p_Update(&PID->Dout_LPF, PID->Dout);
        }
        
        // 增量式: 输出增量叠加到上次输出上(增量式 PID 天生带"记忆", 必须配合输出限幅)
        PID->Output += PID->Pout + PID->Iout + PID->Dout + PID->Fout;
    }
    else 
    {
        /* 位置式 (Position) 以及其变体计算 */
        
        // 积分分离 PID 处理：误差过大(离目标远)时挂起积分, 避免大误差阶段积分堆积导致超调;
        // 只有误差进入阈值内才恢复积分累加, 用于消除稳态静差。
        if (PID->Type == PID_INTEGRAL_SEPARATION && fabsf(PID->Err[0]) > PID->Param.IntSeparationErr)
        {
            // 积分被挂起，不累加
        }
        else
        {
            if(PID->Real_KI != 0) {
                PID->Integral += PID->Err[0];   /* 位置式积分: 直接累加误差(未乘控制周期 dt, 调 KI 时注意控制频率) */
            } else {
                PID->Integral = 0;
            }
        }
        VAL_LIMIT(PID->Integral, -PID->Param.LimitIntegral, PID->Param.LimitIntegral); /* 积分限幅, 防止积分饱和(windup) */
        
        // 计算 P、I 项
        PID->Pout = PID->Real_KP * PID->Err[0];
        PID->Iout = PID->Real_KI * PID->Integral;
        
        // 微分先行 PID 处理：微分只对测量值求导, 不带入目标变化率,
        // 可避免目标值阶跃(如遥控器突然打满)时产生巨大的微分冲击。
        if (PID->Type == PID_DERIVATIVE_ON_MEASURE)
        {
            // 用 (Measure_Prev - Measure) 代替 (Err - Err_prev)
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
        
        // 组合最终输出: 位置式输出 = 比例 + 积分 + 微分 + 前馈
        PID->Output = PID->Pout + PID->Iout + PID->Dout + PID->Fout;
    }

    // 最终总输出限幅
    VAL_LIMIT(PID->Output, -PID->Param.LimitOutput, PID->Param.LimitOutput);
    return PID->Output;
}