

/* ============================================================================
 * 文件说明：底盘控制任务 (Control_Task.c) —— 底盘控制核心
 * ----------------------------------------------------------------------------
 * 这是底盘最核心的文件。FreeRTOS 任务 Control_Task 在这里跑，每 1ms 循环一次
 * （见底部 osDelay(1)），负责：
 *
 *   1. 读反馈：4 个 M3508 轮毂电机、达妙 yaw 电机、拨弹电机、IMU 角度、超电、裁判系统
 *   2. 读设定：从云台 CAN 0x21 帧拿 X/Y/Z 三个速度设定
 *   3. 坡度补偿：根据底盘俯仰/横滚角，给上坡的轮子多分配力
 *   4. 轮速解算：把 X/Y/Z 速度分解成 4 个轮子的目标转速（麦轮公式）
 *   5. 速度环 PID：轮速误差 → 电流指令
 *   6. 功率限幅：拉格朗日优化 + 缓冲能量，保证不超功率
 *
 * ===== 控制结构 =====
 *   单环「速度环」：目标轮速(RPM) → [PID] → 电流指令(±16384)
 *   链条：云台速度设定 → 麦轮解算 → 4 个 speed_rpm_set → 速度环 PID → current_set
 *         → 功率限幅 → 通过 CAN 0x200 帧发给电调
 *
 * ===== 调车关注 =====
 *  - 速度环 PID 参数在 Control_Task.h 的 CHASSIS_MOTOR_PID_* 宏里
 *  - 麦轮解算公式在 Mecanum_Calc()，轮径/减速比/轮距改 Control_Task.h 顶部
 *  - 功率限幅核心在 PowerLimit() 和 Lagrange_Mecanum_Power_Clamp()，
 *    涉及超电策略、缓冲能量、拉格朗日电流分配，是最复杂的部分
 *  - 坡度补偿在 Slope_Compensation_Update()，阈值 7°（角度小就不补偿）
 * ============================================================================
 */
#include "cmsis_os.h"
#include "Control_Task.h"
#include "bsp_uart.h"
#include "Remote_Control.h"
#include "PID.h"
#include "Referee_System.h"
#include "Motor.h"
#include "arm_math.h"
#include "bsp_can.h"
#include "math.h"
#include "INS_Task.h"
#include "ramp.h"


/* ===== 全局变量 =====
 * INS_Info             ：IMU 姿态数据（外部定义，见 INS_Task.c）
 * chassis              ：底盘控制核心结构体（本文件定义，全工程引用）
 * chassis_motor_pid[4] ：4 个轮毂电机共用的速度环 PID
 * set                  ：超级电容功率设定值 (W)，PowerLimit 里算出后发给超电板
 * dbg_predict_wheel_power / dbg_predict_total_power / dbg_referee_real_power：
 *                        调试用功率预测/实测变量（当前代码里未再赋值使用） */
extern INS_Info_Typedef INS_Info;               

Chassis_Info_Typedef chassis;                   
PID_Info_TypeDef chassis_motor_pid[4];          


float set = 0;                                  


float dbg_predict_wheel_power = 0.0f;           
float dbg_predict_total_power = 0.0f;           
float dbg_referee_real_power  = 0.0f;           


/* 函数前向声明（与头文件一致，供本文件内部调用） */
void ChassisInit(void);
void FdbUpdate(Chassis_Info_Typedef* chassis);
void Slope_Compensation_Update(Chassis_Info_Typedef* chassis);
void PowerLimit(Chassis_Info_Typedef* chassis_power_control);
void TimeErrorCalc(Chassis_Info_Typedef* chassis);
void Absolute_Calc(Chassis_Info_Typedef* absolute_speed, float angle);
void Mecanum_Calc(Chassis_Info_Typedef *speed);
void CHASSIS_Single_Loop_Out(void);
float filter(float temp, float alpha);
void Transmit_Shoot_Info(FDCAN_TxFrame_TypeDef *FDCAN_TxFrame, float speed, int16_t heat, int16_t heatlimit, int16_t shooter_barrel_cooling_value);
void SentShootToGimbal(FDCAN_TxFrame_TypeDef *FDCAN_TxFrame, uint8_t *Rxdata);


static void Lagrange_Mecanum_Power_Clamp(Chassis_Info_Typedef *chassis, float P_max,
        float k1_w, float k2_w, float k3_w, float k4_w);


/**
 * @brief 底盘控制任务主函数（FreeRTOS 任务入口）
 * @note  每 1ms 循环一次，按固定顺序执行：
 *          1. Check_All_Motors_Offline：检测电机离线，离线电机电流清零（安全）
 *          2. FdbUpdate：读所有反馈（电机/IMU/云台/裁判系统/超电）
 *          3. Slope_Compensation_Update：坡度补偿系数更新
 *          4. CHASSIS_Single_Loop_Out：麦轮解算 + 速度环 PID，得到电流指令
 *          5. PowerLimit：功率限幅（超电策略 + 拉格朗日电流分配）
 *        顺序很关键：先有反馈，再算补偿，再 PID，最后限功率。
 * @param argument 未使用
 * @retval 无
 */
void Control_Task(void const * argument)
{
    ChassisInit();                       /* 上电初始化：PID 参数、功率系数 */
    for (;;)
    {
        Check_All_Motors_Offline();      /* 第1步：检测电机离线，离线电机电流清零 */

        FdbUpdate(&chassis);             /* 第2步：读取全部反馈 */
/* 下面这段注释掉的代码是「直接用遥控器摇杆控制底盘」的旧写法，
 * 现在底盘速度设定改由云台通过 CAN 0x21 帧下发（见 FdbUpdate 里的 GetChassisXSpeedSet 等）。 */
//        chassis.chassis_xyz_set[CHASSIS_X] = remote_ctrl.rc.ch[2] * 0.014;
//        chassis.chassis_xyz_set[CHASSIS_Y] = remote_ctrl.rc.ch[3] * 0.014;
//        chassis.chassis_xyz_set[CHASSIS_Z] = -remote_ctrl.rc.ch[0] * 0.01;
			//if (remote_ctrl.rc.s[1]==1)
			//	chassis.chassis_xyz_set[CHASSIS_Z] = 10;
        Slope_Compensation_Update(&chassis);   /* 第3步：坡度补偿系数更新 */

        CHASSIS_Single_Loop_Out();             /* 第4步：麦轮解算 + 速度环 PID，得到电流指令 */


				PowerLimit(&chassis);                    /* 第5步：功率限幅（超电策略 + 拉格朗日电流分配） */

        osDelay(1);                            /* 延时 1ms，形成 1kHz 控制周期 */
    }
}


/**
 * @brief 底盘上电初始化：配置速度环 PID 参数和功率模型系数
 * @note  4 个轮毂电机共用同一套 PID 参数（CHASSIS_MOTOR_PID_* 宏，位置式 PID）；
 *        功率模型系数 K1~K4 存在 PowerCtrl[1]（PowerCtrl[0] 未使用）；
 *        坡度补偿系数 K_angle 初始化为 1（即先不补偿）。
 *        最后 osDelay(10) 等待 10ms 让外设稳定。
 * @param 无
 * @retval 无
 */
void ChassisInit(void)
{
    for (int i = 0; i < 4; i++)
    {
        /* 4 个轮毂电机共用的速度环 PID 参数（位置式） */
        chassis_motor_pid[i].Param.KP = CHASSIS_MOTOR_PID_KP;
        chassis_motor_pid[i].Param.KI = CHASSIS_MOTOR_PID_KI;
        chassis_motor_pid[i].Param.KD = CHASSIS_MOTOR_PID_KD;
        chassis_motor_pid[i].Param.LimitIntegral = CHASSIS_MOTOR_PID_MAX_IOUT;
        chassis_motor_pid[i].Param.LimitOutput = CHASSIS_MOTOR_PID_MAX_OUT;
        chassis_motor_pid[i].Type = PID_POSITION;

        chassis.Slope.K_angle[i] = 1.0f;   /* 坡度补偿系数初始化为 1（不补偿） */
    }

    /* 功率模型系数（实测拟合）：P = K1*i² + K2*w² + K3*i*w + K4/4，单位归一化到 W */
    chassis.PowerCtrl[1].K1 = 1.23e-07f;
    chassis.PowerCtrl[1].K2 = 1.453e-07f;
    chassis.PowerCtrl[1].K3 = 1.9968e-6f;
    chassis.PowerCtrl[1].K4 = 16.324f;

    osDelay(10);
}


/**
 * @brief 底盘速度环输出：把 4 个轮子的目标转速经过 PID 算出电流指令
 * @note  调用顺序：先 Absolute_Calc 做坐标系旋转 + 麦轮解算得到 speed_rpm_set，
 *        再对每个轮子做速度环 PID（设定=目标转速 RPM，反馈=实际转速 RPM，
 *        输出=电流 current_set，范围 ±16384）。
 * @param 无
 * @retval 无
 */
void CHASSIS_Single_Loop_Out(void)
{
    /* 第1步：坐标系旋转 + 麦轮解算，得到 4 个轮子的目标转速 (RPM) */
    Absolute_Calc(&chassis, 0);

    /* 第2步：对每个轮子做速度环 PID，输出电流指令 */
    for (int i = 0; i < 4; i++)
    {
        chassis.chassis_motor_set[i].current_set = PID_Calculate(&chassis_motor_pid[i],
            chassis.chassis_motor_set[i].speed_rpm_set,
            chassis.chassis_motor_fdb[i].Data.Velocity);
    }
}

/**
 * @brief   IMU角度变换，根据底盘朝向计算等效Pitch/Roll
 */
/**
 * @brief IMU 角度变换：根据底盘朝向(车头/车尾)计算等效的 Pitch/Roll
 * @note  IMU 装在云台上，云台转 180°(车尾朝前)后，底盘坐标系的前后/左右轴会翻转，
 *        所以这里按 Chassis_Head 把 IMU 的 pitch/roll 重新映射成底盘的 Pitch/Roll。
 *        HEAD：Pitch=-Roll, Roll=Pitch；TAIL(及默认)：Pitch=Roll, Roll=-Pitch。
 * @param 无
 * @retval 无
 */
static void INS_Angle_Calc(void)
{
    Chassis_Info_Typedef *C = &chassis;
    float Pitch = INS_Info.Pitch_Angle;
    float Roll = INS_Info.Roll_Angle;
    switch(C->gimbal_data.Chassis_Head)
    {
        case HEAD:  C->Angle.Pitch = -Roll;
                    C->Angle.Roll =  Pitch;
                    break;
        case TAIL: C->Angle.Pitch = Roll;
                    C->Angle.Roll = -Pitch;
                    break;
        default:
                    C->Angle.Pitch = Roll;
                    C->Angle.Roll = -Pitch;
                    break;
    }
}


/**
 * @brief 读取所有反馈数据（每 1ms 调用一次）
 * @note  依次读取：4 个轮毂电机、拨弹电机、达妙 yaw 电机、超级电容数据、
 *        云台发来的 X/Y/Z 速度设定、裁判系统数据、云台状态数据。
 *        轮毂电机反馈的 Velocity(RPM) 会换算成 m/s 并做一阶低通滤波，存到 Velocity_filter。
 *        最后调用 TimeErrorCalc 计算时间差。
 * @param chassis 底盘结构体指针
 * @retval 无
 */
void FdbUpdate(Chassis_Info_Typedef* chassis)
{
    INS_Angle_Calc();                       /* 先按车头朝向把 IMU 角度换算成底盘等效 Pitch/Roll */
    chassis->time_last = chassis->time_now; /* 保存上次时间戳，用于算时间差 */

    /* 读取 4 个轮毂电机反馈，并把 RPM 换算成 m/s 后做一阶低通滤波 */
    chassis->time_now.motor[M1] = GetChassisMotor1Fdb(&(chassis->chassis_motor_fdb[M1]));
    chassis->chassis_motor_fdb[M1].Data.Velocity_filter = filter(Int16ToFloat(chassis->chassis_motor_fdb[M1].Data.Velocity) * M3508_MOTOR_RPM_TO_VECTOR, 0.8);
    chassis->time_now.motor[M2] = GetChassisMotor2Fdb(&(chassis->chassis_motor_fdb[M2]));
    chassis->chassis_motor_fdb[M2].Data.Velocity_filter = filter(Int16ToFloat(chassis->chassis_motor_fdb[M2].Data.Velocity) * M3508_MOTOR_RPM_TO_VECTOR, 0.8);
    chassis->time_now.motor[M3] = GetChassisMotor3Fdb(&(chassis->chassis_motor_fdb[M3]));
    chassis->chassis_motor_fdb[M3].Data.Velocity_filter = filter(Int16ToFloat(chassis->chassis_motor_fdb[M3].Data.Velocity) * M3508_MOTOR_RPM_TO_VECTOR, 0.8);
    chassis->time_now.motor[M4] = GetChassisMotor4Fdb(&(chassis->chassis_motor_fdb[M4]));
    chassis->chassis_motor_fdb[M4].Data.Velocity_filter = filter(Int16ToFloat(chassis->chassis_motor_fdb[M4].Data.Velocity) * M3508_MOTOR_RPM_TO_VECTOR, 0.8);

    /* 拨弹电机 + 达妙 yaw 电机 + 超级电容数据 */
    GetShootFdb(&(chassis->sht_motor_fdb));
    chassis->time_now.yaw = GetYawMotorFdb(&(chassis->yaw_motor_fdb));
    GetSuperData(&(chassis->super_power));

    /* 从云台 CAN 0x21 帧读 X/Y/Z 速度设定（Y 轴取了反号，匹配车体坐标方向） */
    chassis->time_now.speed_set[CHASSIS_X] = GetChassisXSpeedSet(&(chassis->chassis_xyz_set[CHASSIS_X]));
    chassis->time_now.speed_set[CHASSIS_Y] = -GetChassisYSpeedSet(&(chassis->chassis_xyz_set[CHASSIS_Y]));
    chassis->time_now.speed_set[CHASSIS_Z] = GetChassisZSpeedSet(&(chassis->chassis_xyz_set[CHASSIS_Z]));

    /* 裁判系统数据 + 云台状态数据 */
    Referee_System_Data_Update(&chassis->ref_data);
    GetGimbalData(&chassis->gimbal_data);

    TimeErrorCalc(chassis);                 /* 计算各数据源时间差 */
}


/**
 * @brief 坡度补偿系数更新：根据底盘俯仰/横滚角给 4 个轮子分配不同的力
 * @note  当底盘上坡时（Pitch/Roll 绝对值 > 7°），用 sin/cos 组合成 4 个补偿系数
 *        K_angle[0..3]（对应 4 个轮子），上坡一侧的轮子多分配力，避免上坡打滑/后仰。
 *        角度小于 7° 时不补偿（Cos=1, Sin=0，K_angle=1）。
 *        这些系数会在 Lagrange_Mecanum_Power_Clamp 里乘到权重 W_w 上。
 * @param chassis 底盘结构体指针
 * @retval 无
 */
void Slope_Compensation_Update(Chassis_Info_Typedef* chassis)
{
    // Pitch补偿 (前后坡)
	
	//////////////////////////////看看底盘动是这个原因吗//////////////////////////////////////////
    /* |俯仰角| > 7° 才认为在爬前后坡，否则不补偿（防抖死区） */
    if (fabs(chassis->Angle.Pitch) > 7.0f)
    {
        chassis->Slope.Cos[0] = arm_cos_f32(chassis->Angle.Pitch * PI / 180.0f);
        chassis->Slope.Sin[0] = arm_sin_f32(chassis->Angle.Pitch * PI / 180.0f);
    }
    else
    {
        chassis->Slope.Cos[0] = 1.0f;
        chassis->Slope.Sin[0] = 0.0f;
    }
    // Roll补偿 (左右坡)
    /* |横滚角| > 7° 才认为在爬左右坡 */
    if (fabs(chassis->Angle.Roll) > 7.0f)
    {
        chassis->Slope.Cos[1] = arm_cos_f32(chassis->Angle.Roll * PI / 180.0f);
        chassis->Slope.Sin[1] = arm_sin_f32(chassis->Angle.Roll * PI / 180.0f);
    }
    else
    {
        chassis->Slope.Cos[1] = 1.0f;
        chassis->Slope.Sin[1] = 0.0f;
    }

    /* 把 Pitch/Roll 的 sin/cos 组合成 4 个轮子的补偿系数（2x2 组合，对应 4 个轮位） */
    chassis->Slope.K_angle[2] = (chassis->Slope.Cos[0] + chassis->Slope.Sin[0]) * (chassis->Slope.Cos[1] + chassis->Slope.Sin[1]);
    chassis->Slope.K_angle[3] = (chassis->Slope.Cos[0] + chassis->Slope.Sin[0]) * (chassis->Slope.Cos[1] - chassis->Slope.Sin[1]);
    chassis->Slope.K_angle[0] = (chassis->Slope.Cos[0] - chassis->Slope.Sin[0]) * (chassis->Slope.Cos[1] - chassis->Slope.Sin[1]);
    chassis->Slope.K_angle[1] = (chassis->Slope.Cos[0] - chassis->Slope.Sin[0]) * (chassis->Slope.Cos[1] + chassis->Slope.Sin[1]);
}


/**
 * @brief 计算各电机两次更新之间的时间差（毫秒）
 * @note  time_error = time_now - time_last，用于判断数据是否超时/丢帧。
 *        注意：电机的离线检测实际在 Motor.c 的 Check_All_Motors_Offline 里做。
 * @param chassis 底盘结构体指针
 * @retval 无
 */
void TimeErrorCalc(Chassis_Info_Typedef* chassis)
{
    chassis->time_error.motor[M1] = chassis->time_now.motor[M1] - chassis->time_last.motor[M1];
    chassis->time_error.motor[M2] = chassis->time_now.motor[M2] - chassis->time_last.motor[M2];
    chassis->time_error.motor[M3] = chassis->time_now.motor[M3] - chassis->time_last.motor[M3];
    chassis->time_error.motor[M4] = chassis->time_now.motor[M4] - chassis->time_last.motor[M4];
}


/**
 * @brief 坐标系旋转 + 麦轮解算入口
 * @note  把云台坐标系下的 X/Y/Z 速度设定绕 angle 角旋转到底盘坐标系，
 *        再调用 Mecanum_Calc 分解成 4 个轮子的目标转速。
 *        注意：当前调用时 angle 恒为 0（见 CHASSIS_Single_Loop_Out），
 *        所以 cos=1、sin=0，旋转实际没生效，属于预留功能。
 * @param absolute_speed 底盘结构体指针
 * @param angle 旋转角度 (deg)，当前调用恒为 0
 * @retval 无
 */
void Absolute_Calc(Chassis_Info_Typedef* absolute_speed, float angle)
{
    float angle_hd = angle * PI / 180.0f;   /* 角度转弧度 */
    absolute_speed->chassis_xyz_set_f[CHASSIS_Z] = absolute_speed->chassis_xyz_set[CHASSIS_Z];   /* Z 旋转分量不变 */
    /* X/Y 做平面旋转（绕 Z 轴转 angle 角） */
    absolute_speed->chassis_xyz_set_f[CHASSIS_X] = absolute_speed->chassis_xyz_set[CHASSIS_X] * cos(angle_hd) - absolute_speed->chassis_xyz_set[CHASSIS_Y] * sin(angle_hd);
    absolute_speed->chassis_xyz_set_f[CHASSIS_Y] = absolute_speed->chassis_xyz_set[CHASSIS_X] * sin(angle_hd) + absolute_speed->chassis_xyz_set[CHASSIS_Y] * cos(angle_hd);

    /* 麦轮解算：三轴速度 → 4 个轮子目标转速 */
    Mecanum_Calc(absolute_speed);
}


/**
 * @brief 麦轮速度解算：把 X/Y/Z 三轴速度分解成 4 个轮子的目标转速
 * @note  公式（麦轮全向底盘标准解算）：
 *          wheel0 = ( Vx - Vy - Wz*R) * ratio
 *          wheel1 = (-Vx - Vy - Wz*R) * ratio
 *          wheel2 = (-Vx + Vy - Wz*R) * ratio
 *          wheel3 = ( Vx + Vy - Wz*R) * ratio
 *        ratio = 60/(轮周长*π) * 减速比 * 1000，把 m/s 换算成电机 RPM。
 *        结果写入 chassis_motor_set[i].speed_rpm_set（int16，单位 RPM）。
 * @param speed 底盘结构体指针
 * @retval 无
 */
void Mecanum_Calc(Chassis_Info_Typedef *speed)
{
    /* 换算系数：把 m/s 转成电机 RPM（60/(周长*π) 把 m/s→轮RPM，再乘减速比→电机RPM，乘1000 单位换算） */
    float wheel_rpm_ratio = 60.0f / (WHEEL_PERIMETER * PI) * CHASSIS_DECELE_RATIO * 1000.0f;

    /* 三轴速度（底盘坐标系，X/Y 单位 m/s，Z 单位 rad/s） */
    float Vx = speed->chassis_xyz_set_f[CHASSIS_X];
    float Vy = speed->chassis_xyz_set_f[CHASSIS_Y];
    float Wz = speed->chassis_xyz_set_f[CHASSIS_Z];

    /* 旋转分量折算系数（轮距相关，见 Control_Task.h 的 Radius） */
    float L_factor = Radius;
    float wheel_rpm_target[4];

    /* 麦轮标准解算公式（4 轮布置，轮序 wheel0~wheel3 对应 M1~M4） */
    wheel_rpm_target[0] = (Vx - Vy - Wz * L_factor) * wheel_rpm_ratio;
    wheel_rpm_target[1] = (-Vx - Vy - Wz * L_factor) * wheel_rpm_ratio;

    wheel_rpm_target[2] = (-Vx + Vy - Wz * L_factor) * wheel_rpm_ratio;
    wheel_rpm_target[3] = (Vx + Vy - Wz * L_factor) * wheel_rpm_ratio;

    /* 写入各轮目标转速 (RPM)，作为速度环 PID 的设定值 */
    for (int i = 0; i < 4; i++)
    {
        chassis.chassis_motor_set[i].speed_rpm_set = (int16_t)wheel_rpm_target[i];
    }
}

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
 * @brief 拉格朗日功率优化分配：给 4 个轮子重新分配电流，使总功率不超 P_max
 * @note  每个轮子的功率近似为 P = A_w[i]*α² + B_w[i]*α + C_w[i]，其中：
 *          A_w = k1*i², B_w = k3*w*i, C_w = k2*w² + k4/4（i=期望电流，w=当前转速）
 *        引入拉格朗日乘子 λ，求每个轮子的电流缩放系数 α：
 *          α = (2W - λB) / (2W + 2Aλ)，其中 W = 10*坡度补偿系数。
 *        求解两步走：先扩张 λ 上界找到包含解的范围，再用二分法收敛到目标功率。
 *        最后把电流设定值乘 α，实现「按功率预算重新分配电流」的限幅。
 * @param chassis 底盘结构体指针
 * @param P_max 功率上限 (W)
 * @param k1_w,k2_w,k3_w,k4_w 功率模型系数
 * @retval 无
 */
static void Lagrange_Mecanum_Power_Clamp(Chassis_Info_Typedef *chassis, float P_max,
        float k1_w, float k2_w, float k3_w, float k4_w)
{
    /* A/B/C：功率关于缩放系数 α 的二次多项式系数；W：拉格朗日权重 */
    float A_w[4] = {0}, B_w[4] = {0}, C_w[4] = {0}, W_w[4] = {0};

    
    for (int i = 0; i < 4; i++)
    {
        if (!Chassis_Motor[i].is_offline)
        {
            /* 若电流设定恰好为 0，赋一个 ±1 的极小量，避免后面求 α 时除零 */
            if (chassis->chassis_motor_set[i].current_set == 0)
            {
                chassis->chassis_motor_set[i].current_set = (chassis->chassis_motor_fdb[i].Data.Velocity >= 0) ? 1 : -1;
            }
            float i_w = chassis->chassis_motor_set[i].current_set;
            float w_w = chassis->chassis_motor_fdb[i].Data.Velocity;

            /* 功率二次多项式系数：P(α) = A*α² + B*α + C */
            A_w[i] = k1_w * i_w * i_w;
            B_w[i] = k3_w * w_w * i_w;
            C_w[i] = k2_w * w_w * w_w + k4_w / 4;

            /* 权重：含坡度补偿（上坡轮权重大，分配更多电流） */
            W_w[i] = 10.0f * chassis->Slope.K_angle[i];
        }
    }

    /* 拉格朗日乘子 λ（二分法求）、α 为每个轮子的电流缩放系数（初始 1 不缩放） */
    float lambda = 0.0f;
    float lambda_lower = 0.0f;
    float lambda_upper = LAMBDA_INITIAL_UPPER_BOUND;
    float alpha_w[4] = {1, 1, 1, 1};
    float current_power = 0.0f;

    
    /* 第1步：扩张 λ 上界，直到用该 λ 算出的总功率降到 P_max 以下 */
    for (int iter = 0; iter < LAMBDA_UPPER_BOUND_MAX_ITER; iter++)
    {
        current_power = 0.0f;
        for (int i = 0; i < 4; i++)
        {
            if (!Chassis_Motor[i].is_offline)
            {
                /* 由一阶最优条件求每个轮子的 α（λ 越大 α 越小，功率越低） */
                alpha_w[i] = (2.0f * W_w[i] - lambda_upper * B_w[i]) / (2.0f * W_w[i] + 2.0f * A_w[i] * lambda_upper);
                /* α 限幅：保证缩放后电流不超过 ±16384 */
                float max_alpha = MAX_CMD_CURRENT / fabsf((float)chassis->chassis_motor_set[i].current_set);
                float min_alpha = MIN_CMD_CURRENT / fabsf((float)chassis->chassis_motor_set[i].current_set);
                alpha_w[i] = fminf(fmaxf(alpha_w[i], min_alpha), max_alpha);

                float p_w = A_w[i] * alpha_w[i] * alpha_w[i] + B_w[i] * alpha_w[i] + C_w[i];
                current_power += (p_w > 0) ? p_w : 0;
            }
        }

        if (current_power > P_max)
        {
            if (iter == LAMBDA_UPPER_BOUND_MAX_ITER - 1) goto APPLY_ALPHA;  /* 扩到最大还不满足，直接用当前 α 输出（兜底） */
            lambda_lower = lambda_upper;        /* 还没降到 P_max 以下，抬高下界 */
            lambda_upper *= LAMBDA_UPPER_BOUND_STEP;  /* 上界翻倍继续扩张 */
        }
        else
        {
            break;
        }
    }

    
    /* 第2步：二分法收敛 λ，使总功率逼近 P_max（在容差内） */
    for (int iter = 0; iter < LAMBDA_MAX_ITER; iter++)
    {
        lambda = (lambda_lower + lambda_upper) / 2.0f;   /* 二分中点 */
        current_power = 0.0f;

        for (int i = 0; i < 4; i++)
        {
            if (!Chassis_Motor[i].is_offline)
            {
                alpha_w[i] = (2.0f * W_w[i] - lambda * B_w[i]) / (2.0f * W_w[i] + 2.0f * A_w[i] * lambda);
                float limit_alpha = MAX_CMD_CURRENT / fabsf((float)chassis->chassis_motor_set[i].current_set);
                alpha_w[i] = fminf(fmaxf(alpha_w[i], -limit_alpha), limit_alpha);
                float p_w = A_w[i] * alpha_w[i] * alpha_w[i] + B_w[i] * alpha_w[i] + C_w[i];
                current_power += (p_w > 0) ? p_w : 0;
            }
        }

        if (current_power < P_max && (P_max - current_power) < POWER_TOLERANCE)
        {
            break;                            /* 收敛：功率差小于容差 */
        }
        else if (current_power > P_max)
        {
            lambda_lower = lambda;            /* 功率仍超，抬高 λ 下界 */
        }
        else
        {
            lambda_upper = lambda;            /* 功率不足，降低 λ 上界 */
        }
    }

APPLY_ALPHA:
    /* 把求得的缩放系数 α 应用到电流设定上，完成限幅 */
    for (int i = 0; i < 4; i++)
    {
        if (!Chassis_Motor[i].is_offline)
        {
            chassis->chassis_motor_set[i].current_set = (int16_t)(chassis->chassis_motor_set[i].current_set * alpha_w[i]);
        }
    }
}



/**
 * @brief  核心功率限幅算法 (平滑坡度补偿与拉格朗日优化分配 + 缓冲能量PID)
 */
/* ===== 功率限幅相关的全局变量 =====
 * CAP_Test   ：调试用，手动覆盖超电功率设定（|值|>=1 时直接用它，见 PowerLimit 尾部）
 * Tar_Power  ：目标功率 (W)，>10 时覆盖裁判系统下发的底盘功率限幅
 * Power_KP   ：缓冲能量 PID 的比例系数（超电放电功率 = Power_KP * 功率误差）
 * Power_Boost：超电功率加成 (W)，按超电电压分档计算 */
float CAP_Test = 0;
float Tar_Power = 0;
float Power_KP = 1.6F;
float Power_Boost = 0;
/**
 * @brief 核心功率限幅算法（每 1ms 调用一次）
 * @note  整体分 5 步（见函数内 // --- N --- 注释）：
 *          0. 离线电机电流清零（安全）
 *          1. 加载功率模型系数，并按坡度自适应放大 k1
 *          2. 预测本帧 4 轮期望功率
 *          3. 计算功率上限 = 裁判系统限幅 + 超电加成(按超电电压/被打状态)，算超电放电设定
 *          4. 若预测功率超上限，用拉格朗日法重新分配电流
 *          5. 汇总最终预测功率供调试
 * @param chassis_power_control 底盘结构体指针
 * @retval 无
 */
void PowerLimit(Chassis_Info_Typedef *chassis_power_control)
{
    // --- 0. 硬件状态安全检查 ---
    for (int i = 0; i < 4; i++)
        if (Chassis_Motor[i].is_offline)
            chassis_power_control->chassis_motor_set[i].current_set = 0;
    // --- 1. 物理常数加载与坡度自适应膨胀 ---
    float k1_w = chassis_power_control->PowerCtrl[1].K1;
    float k2_w = chassis_power_control->PowerCtrl[1].K2;
    float k3_w = chassis_power_control->PowerCtrl[1].K3;
    float k4_w = chassis_power_control->PowerCtrl[1].K4;
    float slope_max = fmaxf(fabs(chassis.Angle.Pitch), fabs(chassis.Angle.Roll));
    float inflation_factor_w = 1.0f;
    if (slope_max > 3.0f)
    {
        float factor = (slope_max - 3.0f) / 7.0f;
        if (factor > 1.0f) factor = 1.0f;
        inflation_factor_w += 0.35f * factor;
    }
    k1_w *= inflation_factor_w;
    // --- 2. 预测本帧期望功率 ---
    float expected_total_power = k4_w;
    for (int i = 0; i < 4; i++)
    {
        if (!Chassis_Motor[i].is_offline)
        {
            float w_w = chassis_power_control->chassis_motor_fdb[i].Data.Velocity;
            float i_w = chassis_power_control->chassis_motor_set[i].current_set;
            float p_w = k1_w * i_w * i_w + k2_w * w_w * w_w + k3_w * i_w * w_w;
            if (p_w < 0.0f) p_w = 0.0f;
            expected_total_power += p_w;
        }
    }
    // --- 3. 裁判系统基础限幅与超级电容动态策略 ---
    float ref_limit = chassis_power_control->ref_data.robot_status.chassis_power_limit;
    ref_limit = (ref_limit < 10.0f) ? 75.0f : ref_limit;
    // --- 缓冲能量控制 ---
    float buffer_energy = chassis_power_control->ref_data.power_heat_data.buffer_energy;
    /* 注意: 疑似bug —— 判断条件是 buffer_energy > 20，但减去的是 35，
     * 当缓冲能量在 20~35 之间时 (buffer_energy-35) 为负，反而会大幅削减功率。
     * 直觉上这里应该是 buffer_energy > 35 才对，调功率时留意这一行。 */
    if (buffer_energy > 20.0f)
        ref_limit += 3*(buffer_energy - 35.0f);
    // 超电策略
    float cap_voltage = chassis_power_control->super_power.capa_voltage;
    float safe_margin = 6.0f;
    float chassis_max_power = 0.0f;
    set = 0.0f;
    static float HP_Hurt_Add = 0.f;
    static uint8_t Hurt_Flag = 0;
    static uint32_t Hurt_Time = 0;
   // Gimbal_Info_Typedef *G = &chassis.gimbal_data;
    /* 检测血量突变：2 秒内掉血超过 5 点就临时加 20W 功率补偿（被打加速逃跑） */
    static uint16_t HP[2] = {0,0};
    HP[1] = HP[0];
    HP[0] = Referee_System_Info.robot_status.current_HP;
    if (abs(HP[0] - HP[1]) > 5)
    {
        Hurt_Time = HAL_GetTick();
        Hurt_Flag = 1;
    }
    if (Hurt_Flag == 1)
    {
        if (HAL_GetTick() - Hurt_Time < 2000)
            HP_Hurt_Add = 20.0f;
        else
        {
            HP_Hurt_Add = 0.0f;
            Hurt_Flag = 0;
        }
    }
    else HP_Hurt_Add = 0.0f;
    if (HP[0] == 0) HP_Hurt_Add = 0.0f;
	
//    if (G->CAP_Flag == 1)
//    {
    /* 超电功率加成：电压高加成大，电压低加成小（6V 以下不加成） */
    if (cap_voltage > 9.0f) Power_Boost = 30.0f + HP_Hurt_Add;
    else if (cap_voltage > 6.0f) Power_Boost = (40.0f + HP_Hurt_Add) * (cap_voltage - 6.0f) / 3.0f;
    else Power_Boost = 0.0f;
//    }
//    else
//    {
//        if (cap_voltage > 18.5F) Power_Boost = 20.0f;
//        else if (cap_voltage > 18.0f) Power_Boost = 20.0f * (cap_voltage - 18.0f) / 0.5f;
//        else Power_Boost = 0.0f;
//    }
    ref_limit = (Tar_Power > 10)?Tar_Power:ref_limit;   /* 外部目标功率 >10 时覆盖裁判限幅 */
    SuperPower_Info_Typedef *CAP = &chassis.super_power;
    chassis_max_power = ref_limit + Power_Boost;        /* 最终功率上限 = 裁判限幅 + 超电加成 */
    if (CAP->curren_chassis_power > ref_limit - 5.f)    /* 当前功率逼近限幅时，超电开始放电补功率 */
	{
        set = -Power_Boost - Power_KP*(CAP->tot_power - ref_limit);
		if(CAP->capa_voltage < 6.F)
			set = 0.f;
	}
    else 
        set = ref_limit - CAP->curren_chassis_power;
    if ((chassis_power_control->ref_data.robot_status.chassis_power_limit!= 0 &&
        chassis_power_control->ref_data.robot_status.power_management_chassis_output == 0)
        || chassis_power_control->super_power.tot_power <= 0)
        set = 0.0f;
    static float last_supercap_set = 0.0f;
    set = f_Ramp_Calc(last_supercap_set, set, 48.f);
    last_supercap_set = set;
    set = (fabsf(CAP_Test) < 1.f)?set:CAP_Test;
    Sent_Superpower_Data(&FDCAN1_TxFrame, set);   /* 把超电功率设定发给超电板（CAN 0x210） */
    // --- 4. 拉格朗日全局功率优化分配 ---
    if (expected_total_power > chassis_max_power)   /* 预测功率超限才做拉格朗日分配 */
        Lagrange_Mecanum_Power_Clamp(chassis_power_control, chassis_max_power,
            k1_w, k2_w, k3_w, k4_w);
    // --- 5. 更新 Debug 显示数据 ---
    float final_predict_wheel = 0.0f;
    for (int i = 0; i < 4; i++)
    {
        if (!Chassis_Motor[i].is_offline)
        {
            float w_w = chassis_power_control->chassis_motor_fdb[i].Data.Velocity;
            float i_w = chassis_power_control->chassis_motor_set[i].current_set;
            float p_w = k1_w * i_w * i_w + k2_w * w_w * w_w + k3_w * i_w * w_w;
            if (p_w > 0.0f) final_predict_wheel += p_w;
        }
    }
}

/**
 * @brief 把发射数据打包成 CAN 帧发给云台（ID 0x23，走 FDCAN2）
 * @note  数据格式：speed×100 存 int16（Data[0..1]），heat/heatlimit/cooling_value
 *        各占 16bit（大端：高字节在前）。speed==0 时默认 11.5 作兜底值。
 * @param FDCAN_TxFrame 发送帧结构体指针
 * @param speed 弹速 (m/s)
 * @param heat 枪口热量
 * @param heatlimit 枪口热量上限
 * @param shooter_barrel_cooling_value 枪管冷却值
 * @retval 无
 */
void Transmit_Shoot_Info(FDCAN_TxFrame_TypeDef *FDCAN_TxFrame, float speed, int16_t heat, int16_t heatlimit, int16_t shooter_barrel_cooling_value)
{
    if (speed == 0.0f) speed = 11.5f;

    int16_t float_to_int16 = (int16_t)(speed * 100.0f);
    FDCAN_TxFrame->Header.Identifier = 0x23;

    FDCAN_TxFrame->Data[0] = (uint8_t)(float_to_int16 >> 8);
    FDCAN_TxFrame->Data[1] = (uint8_t)(float_to_int16);
    FDCAN_TxFrame->Data[2] = (uint8_t)(heat >> 8);
    FDCAN_TxFrame->Data[3] = (uint8_t)(heat);

    FDCAN_TxFrame->Data[4] = (uint8_t)(heatlimit >> 8);
    FDCAN_TxFrame->Data[5] = (uint8_t)(heatlimit);
    FDCAN_TxFrame->Data[6] = (uint8_t)(shooter_barrel_cooling_value >> 8);
    FDCAN_TxFrame->Data[7] = (uint8_t)(shooter_barrel_cooling_value);

    USER_FDCAN_AddMessageToTxFifoQ(FDCAN_TxFrame);
}

/**
 * @brief 把 8 字节原始数据透传给云台（CAN ID 0x25）
 * @note  Data 区 8 字节原样拷贝，用于底盘↔云台之间的通用透传。
 * @param FDCAN_TxFrame 发送帧结构体指针
 * @param Rxdata 待透传的 8 字节数据
 * @retval 无
 */
void SentShootToGimbal(FDCAN_TxFrame_TypeDef *FDCAN_TxFrame, uint8_t *Rxdata)
{
    FDCAN_TxFrame->Header.Identifier = 0x25;

    FDCAN_TxFrame->Data[0] = Rxdata[0];
    FDCAN_TxFrame->Data[1] = Rxdata[1];
    FDCAN_TxFrame->Data[2] = Rxdata[2];
    FDCAN_TxFrame->Data[3] = Rxdata[3];
    FDCAN_TxFrame->Data[4] = Rxdata[4];
    FDCAN_TxFrame->Data[5] = Rxdata[5];
    FDCAN_TxFrame->Data[6] = Rxdata[6];
    FDCAN_TxFrame->Data[7] = Rxdata[7];

    USER_FDCAN_AddMessageToTxFifoQ(FDCAN_TxFrame);
}