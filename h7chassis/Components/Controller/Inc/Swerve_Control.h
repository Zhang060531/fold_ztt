/* =====
 * 文件: Swerve_Control.h —— 舵轮(Swerve)底盘运动解算头文件
 * 作用: 声明舵轮底盘逆运动学解算函数 Swerve_Processing, 将底盘目标速度
 *       (Vx/Vy/Vw) 解算为 4 个舵向模块的转向角与驱动轮转速。
 * 结构/调用关系:
 *   - Swerve_Processing(Chassis_Info_Typedef *chassis): 由 Control_Task 周期调用,
 *     读入 chassis_xyz_set_f[3](Vx,Vy,Vw), 输出 steer_motor_set[].position(转向角)
 *     与 chassis_motor_set[].speed_rpm_set(驱动转速)。
 *   - 依赖 STEER_OFFSET[4] 存储每个舵向模块的编码器零点。
 * ===== 调车关注 =====
 *   - 角度单位: 编码器 16384 对应 180°(32768 一圈), UNIT_TO_RAD_COEFF 完成换算。
 *   - STEER_OFFSET[4] 是 4 个舵向电机的机械零点偏移, 装车后需重新标定。
 *   - "最短路径"逻辑会在"直接转向"与"反向转180°+驱动轮反转"之间选代价更小的方案,
 *     避免舵机绕远路, 同时保证驱动轮实际运动方向正确。
 * =====
 */

#ifndef SWERVE_CONTROL_H
#define SWERVE_CONTROL_H

#include "Control_Task.h"
#include "math.h"

// 修复 M_PI 未定义的问题
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// ================= 配置参数 =================
// 角度转换系数: 编码器 16384 个单位 = 180° = PI rad, 即 32768 个单位 = 一圈(2PI)
// UNIT_TO_RAD_COEFF: 编码器单位 -> 弧度; RAD_TO_UNIT_COEFF: 弧度 -> 编码器单位
#define UNIT_TO_RAD_COEFF   (3.14159265f / 16384.0f)
#define RAD_TO_UNIT_COEFF   (16384.0f / 3.14159265f)

// 舵向电机零点偏移数组(单位: 编码器单位), 每个舵向模块安装角不同, 装车后标定;
// 定义在 Swerve_Control.c, 值来自 Control_Task.h 的 CHASSIS_6015_*_Y_ANGLE 宏
extern const float STEER_OFFSET[4];

// ================= 函数声明 =================

/**
 * @brief 舵轮底盘逆运动学解算核心函数
 * @param chassis 底盘数据结构体: 输入 chassis_xyz_set_f[](Vx,Vy,Vw) 与
 *                steer_motor_fdb[](当前舵向角), 输出 steer_motor_set[].position
 *                (目标舵向角) 与 chassis_motor_set[].speed_rpm_set(目标驱动转速)
 * @note  逆解公式(以底盘中心为原点, Vx 前向/Vy 左向/Vw 逆时针为正):
 *        每个轮子的目标速度 = 平动速度 + 旋转速度叉乘轮位;
 *        再经 atan2 得到舵向角, 模长得到驱动轮线速度, 最后换算成 rpm。
 */
void Swerve_Processing(Chassis_Info_Typedef *chassis);

#endif