/* =====
 * 文件: Swerve_Control.c —— 舵轮底盘逆运动学解算实现
 * 作用: 实现 Swerve_Processing, 完成底盘目标速度到 4 个舵向模块
 *       (转向角 + 驱动转速) 的解算, 含最短路径(反转轮子)优化与零速锁定。
 * 结构/调用关系:
 *   - 由 Control_Task 周期调用 Swerve_Processing(&chassis)。
 *   - 内部: 目标速度向量 -> 每个轮子的合成速度向量 -> atan2 求转向角 +
 *     模长求驱动速度 -> 换算 rpm -> 写回电机指令。
 * ===== 调车关注 =====
 *   - 坐标系: Vx 前向(+), Vy 左向(+), Vw 逆时针(+), 单位 m/s 与 rad/s。
 *   - 轮位: R = Radius * 0.707107 为"轮心到中心"的距离(半对角线), 影响旋转分量权重。
 *   - 零速锁定: 目标速度接近 0 时保持上一次转向角, 避免停车时舵机乱抖。
 *   - 若某轮转向方向反了, 优先检查该轮 STEER_OFFSET 或正反转设置。
 * =====
 */

#include "Swerve_Control.h"
#include "math.h"

// 零点偏移数组(单位: 编码器单位), 对应 4 个舵向模块安装时的机械零点
const float STEER_OFFSET[4] = {
    CHASSIS_6015_1_Y_ANGLE,
    CHASSIS_6015_2_Y_ANGLE,
    CHASSIS_6015_3_Y_ANGLE,
    CHASSIS_6015_4_Y_ANGLE
};

// 内部状态记录 (用于零速保持): 记录每个舵向模块上一次的目标转向角(rad)
static float last_steer_target_rad[4] = {0};

/**
 * @brief 将角度规约到 [-PI, PI] 区间(角度归一化, 保证最短转向路径计算正确)
 * @param angle 任意弧度值(可大于 2PI 或小于 -2PI)
 * @retval 规约后的角度, 落在 [-PI, PI]
 * @note  用 while 循环反复加减 2PI, 适用于任意大小输入。
 */
static float Clamp_Rad(float angle) {
    while (angle > (float)M_PI) {
        angle -= 2.0f * (float)M_PI;
    }
    while (angle < -(float)M_PI) {
        angle += 2.0f * (float)M_PI;
    }
    return angle;
}

/**
 * @brief 舵轮底盘逆运动学解算: 目标速度向量 -> 4 轮转向角 + 驱动转速
 * @param chassis 底盘数据(输入 Vx/Vy/Vw 与当前舵向角, 输出转向角与驱动 rpm)
 * @note  逆解 + 最短路径优化 + 零速锁定, 详见 Swerve_Control.h 说明。
 */
void Swerve_Processing(Chassis_Info_Typedef *chassis) {
    // 1. 获取目标速度向量 (单位 m/s 与 rad/s)
    //    CHASSIS_X/Y/Z 分别对应 Vx(前向)/Vy(左向)/Vw(逆时针旋转)
    float Vx = chassis->chassis_xyz_set_f[CHASSIS_X];
    float Vy = chassis->chassis_xyz_set_f[CHASSIS_Y];
    float Vw = chassis->chassis_xyz_set_f[CHASSIS_Z];

    // 2. 轮子速度换算系数 (m/s -> rpm)
    //    M3508_MOTOR_RPM_TO_VECTOR 是"每 rpm 对应的轮线速度(m/s)", 取倒数即得 rpm 换算系数
    float vel_to_rpm = 1.0f / M3508_MOTOR_RPM_TO_VECTOR;

    // 3. 计算四个轮子的目标线速度向量(逆运动学)
    //    Radius 为底盘特征尺寸(Control_Task.h 中 = 2.3961f),
    //    乘 0.707107(=1/√2) 得到轮心到中心的"回转半径" R。
    float R = Radius * 0.707107f;

    // 逆运动学解算: 每个轮子速度 = 平动速度(Vx,Vy) + 旋转分量(±Vw*R)
    // 旋转分量方向由轮子所在象限决定(以 +x 前向/+y 左向的坐标系来看):
    float wheel_vx[4], wheel_vy[4];
    wheel_vx[0] = Vx - Vw * R;   wheel_vy[0] = Vy + Vw * R;   // 轮0: 右前
    wheel_vx[1] = Vx - Vw * R;   wheel_vy[1] = Vy - Vw * R;   // 轮1: 左前
    wheel_vx[2] = Vx + Vw * R;   wheel_vy[2] = Vy - Vw * R;   // 轮2: 左后
    wheel_vx[3] = Vx + Vw * R;   wheel_vy[3] = Vy + Vw * R;   // 轮3: 右后

    for (int i = 0; i < 4; i++) {
        float target_angle_rad = 0.0f;
        float final_rpm = 0.0f;

        // 零速锁定逻辑: 目标速度向量接近 0(停车)时保持上一次转向角并令转速为 0,
        // 避免 atan2(0,0) 产生随机角度导致舵机乱抖。
        if (fabsf(wheel_vx[i]) < 1e-3f && fabsf(wheel_vy[i]) < 1e-3f) {
            target_angle_rad = last_steer_target_rad[i];
            final_rpm = 0.0f;
        } else {
            // 计算原始目标角度 (atan2f 返回 -PI ~ PI, 表示该轮合成速度的方向)
            float raw_target = atan2f(wheel_vy[i], wheel_vx[i]);
            float raw_speed = sqrtf(wheel_vx[i]*wheel_vx[i] + wheel_vy[i]*wheel_vy[i]); /* 合成速度模长(m/s) */

            // === 最短路径优化 (Shortest Path) ===
            // 舵轮可整圈旋转, 目标方向与"反向+驱动轮反转"等价;
            // 比较两种方案所需转向角, 选更小的, 避免舵机绕远路。
            // 1. 获取当前实际角度(编码器单位), 减去零点偏移后转为弧度
            float current_pos_unit = (float)chassis->steer_motor_fdb[i].encoder_position;
            // 转为相对角度(以零点为基准的绝对角度, 单位 rad)
            float current_rad_total = (current_pos_unit - STEER_OFFSET[i]) * UNIT_TO_RAD_COEFF;

            // 方案A: 舵机直接转到目标角, 驱动轮按合成速度正向转
            float err_A = Clamp_Rad(raw_target - current_rad_total);
            float target_A = current_rad_total + err_A;

            // 方案B: 舵机转到"反方向"并让驱动轮反转(相当于转 180°)
            float target_B_raw = raw_target + (float)M_PI;
            float err_B = Clamp_Rad(target_B_raw - current_rad_total);
            float target_B = current_rad_total + err_B;

            // 比较两种方案的转向代价(角位移绝对值), 取小者
            if (fabsf(err_A) < fabsf(err_B)) {
                target_angle_rad = target_A;
                final_rpm = raw_speed * vel_to_rpm;
            } else {
                // 方案B胜出: 舵机转反向, 驱动轮反转(转速取负)保证实际运动方向正确
                target_angle_rad = target_B;
                final_rpm = -raw_speed * vel_to_rpm;
            }
            
            // 更新历史
            last_steer_target_rad[i] = Clamp_Rad(target_angle_rad);
        }

        // ================= 输出写入 =================
        // 1. 目标角度(rad) 转回编码器单位, 并加回零点偏移
        float set_pos_unit = target_angle_rad * RAD_TO_UNIT_COEFF + STEER_OFFSET[i];

        // 2. 写入指令: 舵向电机走位置(编码器单位), 驱动电机走速度(rpm)
        chassis->steer_motor_set[i].position = (int32_t)set_pos_unit;
        chassis->chassis_motor_set[i].speed_rpm_set = (int16_t)final_rpm;
    }
}