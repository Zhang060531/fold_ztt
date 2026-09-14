/* =====
 * 文件: Power_Control.c —— 底盘功率控制实现(能量环 + RLS + 限流)
 * 作用: 实现 PowerControl_Init / PowerControl_Process, 以及内部辅助函数
 *       Solve_Quadratic_Limit(反解限流)、RLS_Update(递推最小二乘)。
 * 结构/调用关系:
 *   - 由 Control_Task 每周期调用 PowerControl_Process(&g_power_ctrl, &chassis, 裁判功率上限)。
 *   - 内部顺序: 能量环 -> 舵向预测/限流 -> 动力配额 -> 动力预测/RLS/限流。
 * ===== 调车关注 =====
 *   - 能量环: ENERGY_KP 越大, 电容电压波动引起的功率增减越激进(爆发越猛/回充越快);
 *     CAP_TARGET_VOLTAGE 是"保持电压", 高于它放电、低于它充电。
 *   - MIN_RUNNING_POWER 是最低保底功率, 防止限流过死导致底盘推不动。
 *   - RLS 只在真实功率 > RLS_UPDATE_MIN_PWR 且 Σ|w| 足够大时更新, 避免低速噪声污染辨识。
 * =====
 */

#include "Power_Control.h"
#include "math.h"

PowerControl_t g_power_ctrl;   /* 全局功率控制器实例(供 Control_Task 每周期调用) */

// ================= 能量环参数 =================
// 假设电容满电 22.5V，最低 3V
// 设定一个保持电压，比如 13V。
// 高于 13V 时，P项增加功率限制（放电）；低于 13V 时，P项减小功率限制（充电）
#define CAP_TARGET_VOLTAGE  13.0f   // 电容保持电压(V): 高于它放电、低于它充电
#define ENERGY_KP           15.0f   // 能量环比例系数(W/V), 需调试。(22V-13V)*15 = 135W 爆发
#define MIN_RUNNING_POWER   25.0f   // 最低运行功率(W), 防止限制太死导致无法移动

// ================= 内部辅助函数 =================

/**
 * @brief 反解限流: 给定功率上限, 求解二次方程 A*I^2 + B*I + C = 0 得到允许的最大电流
 * @param a,b,c      二次方程系数(功率模型 P = a*I^2 + b*I + c 移项后 = 0)
 * @param raw_current 原始电流指令(当前版本未使用, 保留参数)
 * @retval 限流后的电流值
 * @note
 *   - 方程无实根(判别式<0)说明该转速下无论多大电流都会超功率, 取抛物线顶点电流(最小损耗点)。
 *   - 两个根中取绝对值较小者, 通常对应较小的电流热损耗。
 *   注意: 疑似bug —— 参数 raw_current 未被使用, 若想按"原始电流方向"取根可在此处改进。
 */
static float Solve_Quadratic_Limit(float a, float b, float c, float raw_current) {
    if (fabsf(a) < 1e-6f) return 0.0f; // 避免除零

    float delta = b * b - 4.0f * a * c;

    if (delta < 0) {
        // 无解: 当前转速下无论给多大电流都会超功率, 取抛物线顶点 -b/(2a) 作为最小损耗点
        return -b / (2.0f * a);
    }

    float sqrt_delta = sqrtf(delta);
    float x1 = (-b + sqrt_delta) / (2.0f * a);
    float x2 = (-b - sqrt_delta) / (2.0f * a);

    float abs_x1 = fabsf(x1);
    float abs_x2 = fabsf(x2);

    // 简单策略：取绝对值较小的解(两个根一正一负或同号时, 绝对值小者电流热损耗更小)
    return (abs_x1 < abs_x2) ? x1 : x2;
}

/**
 * @brief 二维递推最小二乘(RLS)更新: 用功率残差修正 k1/k2 估计
 * @param rls         RLS 辨识器句柄
 * @param vel_sum     Σ|w| (4 个动力电机转速绝对值之和, rad/s)
 * @param curr_sq_sum ΣI^2 (4 个电机反馈电流平方和, A^2)
 * @param p_error     功率残差 = 真实动力功率 - 由当前 k1/k2 预测的损耗功率 (W)
 * @note  回归模型: p_error ≈ k1*vel_sum + k2*curr_sq_sum,
 *        通过卡尔曼式增益 K 更新 theta, 并用遗忘因子 lambda 老化旧数据。
 */
static void RLS_Update(RLS_t *rls, float vel_sum, float curr_sq_sum, float p_error) {
    float phi[2] = {vel_sum, curr_sq_sum};   /* 回归向量 [Σ|w|, ΣI^2] */

    // 1. 计算卡尔曼增益 K = P*phi / (lambda + phi^T*P*phi)
    float P_phi[2];
    P_phi[0] = rls->P[0][0] * phi[0] + rls->P[0][1] * phi[1];
    P_phi[1] = rls->P[1][0] * phi[0] + rls->P[1][1] * phi[1];
    float div = rls->lambda + (phi[0] * P_phi[0] + phi[1] * P_phi[1]);
    float K[2] = {P_phi[0] / div, P_phi[1] / div};

    // 2. 计算预测误差 e = 真实残差 - 当前模型预测值
    float y_est = phi[0] * rls->theta[0] + phi[1] * rls->theta[1];
    float e = p_error - y_est;
    
    // 3. Update Theta
    rls->theta[0] += K[0] * e;
    rls->theta[1] += K[1] * e;
    
    // 约束参数范围 (物理参数不能为负)
    if (rls->theta[0] < 1e-5f) rls->theta[0] = 1e-5f; // k1
    if (rls->theta[1] < 1e-5f) rls->theta[1] = 1e-5f; // k2

    // 4. 更新协方差矩阵 P = (P - K*phi^T*P) / lambda
    float term00 = K[0] * P_phi[0];
    float term01 = K[0] * P_phi[1];
    float term10 = K[1] * P_phi[0];
    float term11 = K[1] * P_phi[1];
    
    rls->P[0][0] = (rls->P[0][0] - term00) / rls->lambda;
    rls->P[0][1] = (rls->P[0][1] - term01) / rls->lambda;
    rls->P[1][0] = (rls->P[1][0] - term10) / rls->lambda;
    rls->P[1][1] = (rls->P[1][1] - term11) / rls->lambda;
}

// ================= API 实现 =================

/**
 * @brief 初始化功率控制器: 载入电机模型系数并初始化动力电机 RLS
 * @param pc 控制器句柄
 */
void PowerControl_Init(PowerControl_t *pc) {
    // 动力轮 RLS 初始化(遗忘因子、初始参数、协方差矩阵)
    pc->rls_drive.lambda = RLS_LAMBDA;
    pc->rls_drive.theta[0] = DRIVE_K1;
    pc->rls_drive.theta[1] = DRIVE_K2;
    pc->rls_drive.P[0][0] = RLS_DELTA; pc->rls_drive.P[1][1] = RLS_DELTA;
    
    pc->k0_drive = DRIVE_K0;
    pc->k3_drive = DRIVE_K3;

    // 舵电机参数 (固定模型)
    pc->k0_steer = STEER_K0;
    pc->k1_steer = STEER_K1;
    pc->k2_steer = STEER_K2;
    pc->k3_steer = STEER_K3;
}

/**
 * @brief 功率控制核心: 能量环 + 舵向/动力电机功率预测与限流
 * @param pc        控制器句柄
 * @param chassis   底盘数据(读反馈, 写回限流后的电流指令)
 * @param max_power 裁判系统允许的最大底盘功率(W)
 * @note  见 Power_Control.h 中该函数的详细流程说明。
 */
void PowerControl_Process(PowerControl_t *pc, Chassis_Info_Typedef *chassis, float max_power) {
    // 0. 准备数据 & 能量环计算

    // --- 能量环 (Energy Loop) ---
    // 读取超级电容电压(单位 V, bsp_can.c 中已除以 100), 近似反映电容剩余容量
    float cap_voltage = chassis->super_power.capa_voltage;
    
    // 计算额外的功率缓冲 (P控制)
    // 如果电压 > 目标值，extra_power > 0，允许超功率(放电)
    // 如果电压 < 目标值，extra_power < 0，限制功率(充电)
    float extra_power = ENERGY_KP * (cap_voltage - CAP_TARGET_VOLTAGE); /* 额外功率缓冲, 单位 W */
    
    // 计算最终允许的底盘总功率
    float final_power_limit = max_power + extra_power;
    
    // 安全钳位
    if (final_power_limit < MIN_RUNNING_POWER) final_power_limit = MIN_RUNNING_POWER;
    
    // ---------------------------

    float steer_predict_power[4] = {0};
    float steer_cmd_power_sum = 0;
    float steer_avail_power = final_power_limit * STEER_POWER_RATIO; 
    
    float drive_predict_power[4] = {0};
    float drive_cmd_power_sum = 0;
    
    // RLS 数据累计
    float rls_w_sum = 0;
    float rls_i_sq_sum = 0;
    float rls_mech_power_sum = 0;

    // ================= 1. 舵向电机功率预测 =================
    // 模型: P = k0*I*w + k1*|w| + k2*I^2 + k3/4 (单位 W)
    //   k0*I*w 为机械输出功率(负值视为制动/回馈, 不计算), k1*|w| 摩擦, k2*I^2 铜损, k3/4 静态损耗分摊
    for(int i=0; i<4; i++) {
        // 转速单位换算: RPM(转/分) -> rad/s, 系数 = 2*PI/60 ≈ 0.10472
        float w = (float)chassis->steer_motor_fdb[i].speed * 0.10472f;
        float i_set = (float)chassis->steer_motor_set[i].torque_current;
        
        float p_mech = pc->k0_steer * i_set * w;
        if(p_mech < 0) p_mech = 0; 
        
        steer_predict_power[i] = p_mech + 
                                 pc->k1_steer * fabsf(w) + 
                                 pc->k2_steer * i_set * i_set + 
                                 pc->k3_steer / 4.0f;
        
        steer_cmd_power_sum += steer_predict_power[i];
    }

    // ================= 2. 舵向电机限制 =================
    float steer_consume_est = 0; 
    
    if (steer_cmd_power_sum > steer_avail_power) {
        for(int i=0; i<4; i++) {
            float w = (float)chassis->steer_motor_fdb[i].speed * 0.10472f;
            float i_set = (float)chassis->steer_motor_set[i].torque_current;
            
            float p_limit_i = (steer_predict_power[i] / steer_cmd_power_sum) * steer_avail_power;
            
            // 由功率上限 p_limit_i 反解电流: k2*I^2 + (k0*w)*I + (k1*|w| + k3/4 - p_limit_i) = 0
            float A = pc->k2_steer;
            float B = pc->k0_steer * w;
            float C = pc->k1_steer * fabsf(w) + pc->k3_steer / 4.0f - p_limit_i;
            
            float i_limit = Solve_Quadratic_Limit(A, B, C, i_set);
            
            if (fabsf(i_limit) < fabsf(i_set)) {
                chassis->steer_motor_set[i].torque_current = (int16_t)i_limit;
                steer_consume_est += p_limit_i; 
            } else {
                steer_consume_est += steer_predict_power[i];
            }
        }
    } else {
        steer_consume_est = steer_cmd_power_sum;
    }
    pc->steer_power_cost = steer_consume_est;

    // ================= 3. 动力电机功率配额 =================
    // 总允许功率 - 舵机实际消耗 - 动力组静态损耗, 剩余给动力电机
    float drive_avail_power = final_power_limit - steer_consume_est - pc->k3_drive;
    if (drive_avail_power < 5.0f) drive_avail_power = 5.0f; // 最小保底
    pc->drive_power_limit = drive_avail_power;

    // ================= 4. 动力电机预测 =================
    float k0 = pc->k0_drive;
    float k1 = pc->rls_drive.theta[0];
    float k2 = pc->rls_drive.theta[1];
    
    for(int i=0; i<4; i++) {
        // 动力电机转速
        float w = (float)chassis->chassis_motor_fdb[i].Data.Velocity * 0.10472f; 
        float i_set = (float)chassis->chassis_motor_set[i].current_set;
        
        float p_mech = k0 * i_set * w;          /* 机械功率, 负值(制动/回馈)按 0 计 */
        if(p_mech < 0) p_mech = 0;

        drive_predict_power[i] = p_mech + k1 * fabsf(w) + k2 * i_set * i_set; /* 机械 + 摩擦 + 铜损 */
        drive_cmd_power_sum += drive_predict_power[i];
        
        // RLS 采集 (用反馈电流)
        float i_fdb = (float)chassis->chassis_motor_fdb[i].Data.Current;
        float p_mech_fdb = k0 * i_fdb * w; 
        if(p_mech_fdb < 0) p_mech_fdb = 0;
        
        rls_mech_power_sum += p_mech_fdb;
        rls_w_sum += fabsf(w);
        rls_i_sq_sum += (i_fdb * i_fdb);
    }

    // ================= 5. RLS 在线辨识更新 =================
    // 使用超级电容/裁判系统反馈的真实底盘功率(单位 W, bsp_can.c 中已除以 10)
    float real_total_power = chassis->super_power.curren_chassis_power;

    if (real_total_power > RLS_UPDATE_MIN_PWR && rls_w_sum > 1.0f) {
        // 动力真实功率 = 总功率 - 舵机消耗 - 动力静态损耗
        float drive_real_power = real_total_power - steer_consume_est - pc->k3_drive;
        // 功率残差 = 动力真实功率 - 已由 k0 算出的机械功率(即摩擦 + 铜损部分)
        float p_error = drive_real_power - rls_mech_power_sum;
        
        RLS_Update(&pc->rls_drive, rls_w_sum, rls_i_sq_sum, p_error);
    }

    // ================= 6. 动力电机限制 =================
    if (drive_cmd_power_sum > drive_avail_power) {
        for(int i=0; i<4; i++) {
            float w = (float)chassis->chassis_motor_fdb[i].Data.Velocity * 0.10472f;
            float i_set = (float)chassis->chassis_motor_set[i].current_set;
            
            float p_limit_i = (drive_predict_power[i] / drive_cmd_power_sum) * drive_avail_power;
            
            float A = k2;
            float B = k0 * w;
            float C = k1 * fabsf(w) - p_limit_i; 
            
            float i_limit = Solve_Quadratic_Limit(A, B, C, i_set);
            
            if (fabsf(i_limit) < fabsf(i_set)) {
                chassis->chassis_motor_set[i].current_set = (int16_t)i_limit;
            }
        }
    }
}