/* =====
 * 文件: Power_Control.h —— 底盘功率控制头文件(超级电容功率环 + RLS 参数辨识)
 * 作用: 依据裁判系统功率上限与超级电容电压, 动态分配/限制舵向电机与动力电机的
 *       电流指令, 保证总功率不超限, 并利用 RLS(递推最小二乘)在线辨识动力电机的
 *       摩擦/铜损系数, 提高功率预测精度。
 * 结构/调用关系:
 *   - PowerControl_t: 控制器句柄(含动力电机 RLS 与参数缓存), 全局实例 g_power_ctrl。
 *   - PowerControl_Init() 初始化参数 -> PowerControl_Process() 每周期执行:
 *       能量环算额外功率 -> 舵向电机预测+限流 -> 动力电机预测+RLS 辨识+限流。
 *   - 依赖 Chassis_Info_Typedef(Control_Task.h) 读取电机反馈并写回电流指令。
 * ===== 调车关注 =====
 *   - 功率模型: P = k0*I*w + k1*|w| + k2*I^2 + k3/4 (单位 W)
 *       k0: 机电转换(转矩系数*减速比), k1: 摩擦系数, k2: 铜损, k3: 静态损耗。
 *   - 舵向系数为固定值; 动力 k1/k2 由 RLS 在线辨识, 初值见 DRIVE_K1/DRIVE_K2。
 *   - 能量环: extra_power = ENERGY_KP * (电容电压 - CAP_TARGET_VOLTAGE),
 *     电压高于目标则"放电"允许超功率, 低于目标则"充电"压缩功率。
 * =====
 */

#ifndef POWER_CONTROL_H
#define POWER_CONTROL_H

#include "stdint.h"
#include "Control_Task.h" 

// ================= RLS 配置 =================
#define RLS_LAMBDA          0.999f  // 遗忘因子(越接近1, 旧数据遗忘越慢, 估计越平滑)
#define RLS_DELTA           100.0f  // 协方差矩阵初始值(越大初始收敛越快, 也越容易抖)
#define RLS_UPDATE_MIN_PWR  5.0f    // 最小更新功率(W): 真实功率低于该值时不更新 RLS, 避免低速噪声

// ================= 动力电机参数 (M3508) =================
// 需根据实际标定，这里参考 HKUST 麦轮参数微调
// k0 = 转矩系数(Nm/A) * 减速比 / 电调电流单位(16384/20A), 用于机械功率 P = k0*I*w (单位 W)
#define DRIVE_K0            0.00035f
#define DRIVE_K1            0.0001f  // 初始摩擦系数(乘 |w|, 单位 W/(rad/s)), 后续由 RLS 在线修正
#define DRIVE_K2            0.0005f  // 初始铜损系数(乘 I^2, 单位 W/A^2), 后续由 RLS 在线修正
#define DRIVE_K3            5.0f     // 动力组静态损耗(W, 4 个电机分摊, 单次按 k3/4 计)

// ================= 舵向电机参数 (GM6020/LK6010) =================
// 舵向电机功率模型系数(固定值, 不做在线辨识)
#define STEER_K0            0.00074f  // 机电转换系数: P = k0*I*w (W)
#define STEER_K1            0.005f    // 摩擦系数(乘 |w|)
#define STEER_K2            0.0129f   // 铜损系数(乘 I^2)
#define STEER_K3            5.0f      // 静态损耗(W, 按 k3/4 分摊到单电机)

// ================= 分配策略 =================
#define STEER_POWER_RATIO   0.80f    // 舵电机最大允许占用总功率的 80%

/**
 * @brief 二维递推最小二乘(RLS)辨识器, 用于在线估计动力电机的 k1(摩擦)、k2(铜损)
 * @note  回归模型: 功率残差 ≈ k1*|w| + k2*I^2, 通过功率误差不断修正 theta。
 */
typedef struct {
    float theta[2];     // 参数估计值 [k1, k2], 单位见 DRIVE_K1/DRIVE_K2
    float P[2][2];      // 协方差矩阵(初始为 RLS_DELTA, 越大收敛越快但越抖)
    float lambda;       // 遗忘因子(0~1, 越接近 1 记忆越长, 越接近 0 对新数据越敏感)
} RLS_t;

// 功率控制器句柄(全局单例 g_power_ctrl)
typedef struct {
    RLS_t rls_drive;    // 动力电机 RLS 辨识器(只辨识 k1/k2)

    // 参数缓存(Init 时从宏常量载入, 之后供 Process 使用)
    float k0_drive, k3_drive;                       // 动力电机 k0/k3
    float k0_steer, k1_steer, k2_steer, k3_steer;   // 舵向电机 k0~k3(固定)

    // 状态观测 (调试用, 可配合 VoFa/上位机观察功率分配)
    float steer_power_cost;     // 舵向电机实际消耗功率估计(W)
    float drive_power_limit;    // 分配给动力电机的功率上限(W)
} PowerControl_t;

extern PowerControl_t g_power_ctrl;

/**
 * @brief 初始化功率控制器(载入动力/舵向电机模型系数, 初始化动力电机 RLS)
 * @param pc 控制器句柄
 */
void PowerControl_Init(PowerControl_t *pc);

/**
 * @brief 核心处理函数: 能量环 + 舵向/动力电机功率预测与限流
 * @param pc        控制器句柄
 * @param chassis   底盘数据(读取电机反馈, 写回 torque_current/current_set 限流结果)
 * @param max_power 裁判系统给的最大底盘功率限制(W)
 * @note  流程: 1)能量环算 extra_power 得到 final_power_limit;
 *             2)舵向电机按 STEER_POWER_RATIO 配额预测并限流;
 *             3)扣除舵机消耗与静态损耗得到动力配额;
 *             4)动力电机预测、RLS 在线辨识、限流。
 */
void PowerControl_Process(PowerControl_t *pc, Chassis_Info_Typedef *chassis, float max_power);

#endif