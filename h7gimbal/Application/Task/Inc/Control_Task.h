#ifndef CONTROL_TASK_H
#define CONTROL_TASK_H
#include "main.h"
#include "kalman_filter.h"
#include "pid.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "vision.h"
#include "bsp_can.h"
#include "remote_control.h"
#include "Motor.h"

/**
 * =====================================================================
 * 文件说明：云台控制任务头文件 (Control_Task.h)
 * ---------------------------------------------------------------------
 * 存放云台控制相关的一切：控制参数、模式枚举、核心结构体 Gimbal_t。
 *
 * ===== 三大块内容 =====
 * 1. PID 参数宏（下面一堆 *_PID_* 开头的 #define）
 *    - SHELL = 外环角度环，CORE = 内环角速度环
 *    - 普通参数(无 VISION) = 手动控制用；VISION_* = 视觉自瞄用
 *    - 调车改 PID 就是改这里，改完重新编译烧录即可
 * 2. 模式枚举（Robot_Behavior_e / Chassis_Behavior_e / Motor_Behavior_e）
 * 3. Gimbal_t 结构体：云台所有状态（反馈/设定/模式/时间戳）都收在 gimbal 里
 *
 * ===== 调车重点关注 =====
 *  - 俯仰/偏航的机械限位角度在 PITCH_ABSOLUTE_MAX/MIN_ANGLE、YAW_*_ANGLE
 *  - 摇杆灵敏度由 CHANNEL_PITCH/YAW_SCALE_FACTOR 决定（越大角度动得越慢）
 *  - 底盘跟随 PID 在 FOLLOW_PID_* 一组
 * =====================================================================
 */
#define CHANNEL_PITCH_SCALE_FACTOR 6000.0f  // 俯仰摇杆→角度系数(越大动得越慢)     //9000
#define CHANNEL_YAW_SCALE_FACTOR 3000.0f    // 偏航摇杆→角度系数                   //5000
#define MOUSE_SCALE_FACTOR 200.0f           // 鼠标→角度系数
#define KEY_SCALE_FACTOR 20.0f              // 键盘→底盘速度系数
#define ROTATE_KEY_SCALE_FACTOR (KEY_SCALE_FACTOR / 2)  // 键盘→旋转速度系数



/* ============【新增】偏航轴 速度前馈（手动控制，提高 yaw 响应） ============
 * 把「摇杆/鼠标拨出的目标角速度」直接前馈到内环角速度设定上，
 * 绕开角度环靠误差慢慢累积的滞后：摇杆一动云台立刻跟着转。
 * 原理：参考角速度 = 本周期角度增量 ÷ 控制周期(1ms)。
 * 调车：yaw_vel_ff_gain 越接近 1 响应越快，但抖舵/超调就越明显，可适当调小。
 *       yaw_vel_ff_gain 是全局变量，Keil/CLion 调试时在 Watch 窗口添加它即可实时修改。 */
#define GIMBAL_CONTROL_PERIOD_S 0.001f   /*【新增】云台控制周期(s)，对应 Control_Task 里 osDelay(1) 的 1ms */
extern volatile float yaw_vel_ff_gain;    /*【新增】速度前馈增益 (0~1)，默认 0.5 */


/* =====================================================================
 * PID 参数宏（调车主要改这里）
 * ---------------------------------------------------------------------
 * SHELL = 外环「角度环」：输入角度误差，输出目标角速度(°/s)
 * CORE  = 内环「角速度环」：输入角速度误差，输出力矩(N·m)
 * KP/KI/KD 是 PID 三参数，MAX_IOUT=积分限幅，MAX_OUT=输出限幅
 * ===================================================================== */

/* ---- 俯仰轴 外环(角度环) —— 手动控制 ---- */
#define PITCH_SHELL_PID_KP 50
#define PITCH_SHELL_PID_KI 0
#define PITCH_SHELL_PID_KD 1
#define PITCH_SHELL_PID_MAX_IOUT 100   // 积分限幅(°/s)
#define PITCH_SHELL_PID_MAX_OUT 400    // 输出限幅(目标角速度上限 °/s)

/* ---- 俯仰轴 内环(角速度环) —— 手动控制 ----
 * 注意 KP 是负数：pitch 电机安装方向与 yaw 相反，需要反号才能负反馈 */
#define PITCH_CORE_PID_KP -0.013
#define PITCH_CORE_PID_KI -0
#define PITCH_CORE_PID_KD 0
#define PITCH_CORE_PID_MAX_IOUT 0.5
#define PITCH_CORE_PID_MAX_OUT 2.5   // 输出限幅(力矩上限 N·m)

/* ---- 偏航轴 外环(角度环) —— 手动控制 ---- */
#define YAW_SHELL_PID_KP 20
#define YAW_SHELL_PID_KI 0
#define YAW_SHELL_PID_KD 0.55
#define YAW_SHELL_PID_MAX_IOUT 100
#define YAW_SHELL_PID_MAX_OUT 400

/* ---- 偏航轴 内环(角速度环) —— 手动控制 ---- */
#define YAW_CORE_PID_KP 0.03
#define YAW_CORE_PID_KI 0
#define YAW_CORE_PID_KD 0
#define YAW_CORE_PID_MAX_IOUT 0
#define YAW_CORE_PID_MAX_OUT 2.5

/* ---- 俯仰轴 外环(角度环) —— 视觉自瞄用 ---- */
#define PITCH_VISION_SHELL_PID_KP 20
#define PITCH_VISION_SHELL_PID_KI 0.39   //0.2
#define PITCH_VISION_SHELL_PID_KD 0
#define PITCH_VISION_SHELL_PID_MAX_IOUT 100
#define PITCH_VISION_SHELL_PID_MAX_OUT 400 

/* ---- 俯仰轴 内环(角速度环) —— 视觉自瞄用(注意 KP 为负) ---- */
#define VISION_PITCH_CORE_PID_KP -0.018  
#define VISION_PITCH_CORE_PID_KI 0
#define VISION_PITCH_CORE_PID_KD 0
#define VISION_PITCH_CORE_PID_MAX_IOUT 2.5
#define VISION_PITCH_CORE_PID_MAX_OUT 2.5

/* ---- 偏航轴 外环(角度环) —— 视觉自瞄用 ---- */
#define YAW_VISION_SHELL_PID_KP 28                       //15
#define YAW_VISION_SHELL_PID_KI 0
#define YAW_VISION_SHELL_PID_KD 0
#define YAW_VISION_SHELL_PID_MAX_IOUT 100
#define YAW_VISION_SHELL_PID_MAX_OUT 400
#define VISION_SHELL_INT_SEP_ERR 2.0f   // 视觉外环积分分离阈值(角度误差超过它就取消积分)

/* ---- 偏航轴 内环(角速度环) —— 视觉自瞄用 ---- */
#define VISION_YAW_CORE_PID_KP 0.03  //0.03
#define VISION_YAW_CORE_PID_KI 0.02   //0
#define VISION_YAW_CORE_PID_KD 0
#define VISION_YAW_CORE_PID_MAX_IOUT 2
#define VISION_YAW_CORE_PID_MAX_OUT 2.5     //YAW输出限幅

/* 视觉速度前馈 */
#define VISION_VEL_FF_GAIN      0.95f   /* 速度前馈增益 (0~1), 1.0=完全前馈 */
#define VISION_VEL_LPF_ALPHA    0.85f   /* 速度估计低通滤波系数, 越大越平滑但滞后 */
#define VISION_VEL_CALC_MS      5      /* 速度计算间隔(ms) */

/* 预测前馈：外推补偿视觉处理+传输+电机响应的总延迟 */
#define VISION_PREDICT_TIME_S   0.04f   /* 预测时间(s)，建议范围 0.03~0.06 */
/* 发给视觉的Pitch角度低通滤波系数 (0~1), 越大越平滑但滞后 */
#define PITCH_TX_LPF_ALPHA      0.50f


/* ---- 底盘跟随云台 PID（云台转过一定角度，底盘跟着转回去，见 ChassisSpeedSet）---- */
#define FOLLOW_PID_KP -0.02
#define FOLLOW_PID_KI 0
#define FOLLOW_PID_KD 0
#define FOLLOW_PID_MAX_IOUT 0.2
#define FOLLOW_PID_MAX_OUT 2.4

#define FOLLOW_PID_KP_PIECE1 0.015//-0.030
#define FOLLOW_PID_KI_PIECE1 0
#define FOLLOW_PID_KD_PIECE1 0
#define FOLLOW_PID_KP_PIECE2 0.05//-0.042
#define FOLLOW_PID_KI_PIECE2 0.001
#define FOLLOW_PID_KD_PIECE2 0
#define FOLLOW_PID_ERR_BOUNDARY 5


#ifndef NULL
#define NULL 0
#endif


#ifndef PI
#define PI 3.14159265358979f
#endif


#ifndef ANGLE_TO_RAD
#define ANGLE_TO_RAD 0.01745329251994329576923690768489f
#endif


#ifndef RAD_TO_ANGLE
#define RAD_TO_ANGLE 57.295779513082320876798154814105f
#endif




/* =====================================================================
 * 云台角度限位（调车/防撞结构 重点关注）
 * ---------------------------------------------------------------------
 * PITCH 俯仰：绝对模式 -3°~+25°（低头-3、抬头+25），相对模式同范围
 * YAW   偏航：±180°（可整圈旋转，靠 ConstrainLoop 做环绕）
 * 这些值就是 addPitchAngle/addYawAngle 里限幅用的上下限。
 * 机械上如果俯仰打到极限，就适当调小这里的 PITCH_* 值。
 * ===================================================================== */
#define ANGLE_MIN -180   // 通用角度下界(偏航环绕用)
#define ANGLE_ZERO 0
#define ANGLE_MAX 180    // 通用角度上界

#define PITCH_ABSOLUTE_MAX_ANGLE 25   // 俯仰绝对模式上限(抬头最大 +25°)
#define PITCH_ABSOLUTE_MIN_ANGLE -3   // 俯仰绝对模式下限(低头最大 -3°)
#define YAW_ABSOLUTE_MAX_ANGLE 180
#define YAW_ABSOLUTE_MIN_ANGLE -180

#define PITCH_VISION_MAX_ANGLE 2      // 视觉自瞄时俯仰上限(不让自瞄时抬头太狠)
#define PITCH_VISION_MIN_ANGLE -25
#define YAW_VISION_MAX_ANGLE 180
#define YAW_VISION_MIN_ANGLE -180

#define YAW_RELATIVE_MAX_ANGLE 	180
#define YAW_RELATIVE_MIN_ANGLE 	-180
#define PITCH_RELATIVE_MAX_ANGLE 25
#define PITCH_RELATIVE_MIN_ANGLE -3



/* 底盘速度三分量下标：X/Y 平移，Z 旋转，对应 chassis_set[3] */
#define CHASSIS_X 0
#define CHASSIS_Y 1
#define CHASSIS_Z 2

#define CHASSIS_X_Y_Z_ID 0x301
#define CHASSIS_UI_YAW_ID 0x299 //0x302


#define REMOTE_CHANNLE_TO_CHASSIS_SPEED 0.015f                                                //0.01	
#define ROTATE_REMOTE_CHANNLE_TO_CHASSIS_SPEED (REMOTE_CHANNLE_TO_CHASSIS_SPEED / 50)



/* 键盘按键位掩码：大疆图传键鼠的按键状态是 16bit 位图，每个键占一位。
 * 判断某键是否按下：value & KEY_PRESSED_OFFSET_X */
#define KEY_PRESSED_OFFSET_W            ((uint16_t)1 << 0)
#define KEY_PRESSED_OFFSET_S            ((uint16_t)1 << 1)
#define KEY_PRESSED_OFFSET_A            ((uint16_t)1 << 2)
#define KEY_PRESSED_OFFSET_D            ((uint16_t)1 << 3)
#define KEY_PRESSED_OFFSET_SHIFT        ((uint16_t)1 << 4)
#define KEY_PRESSED_OFFSET_CTRL         ((uint16_t)1 << 5)
#define KEY_PRESSED_OFFSET_Q            ((uint16_t)1 << 6)
#define KEY_PRESSED_OFFSET_E            ((uint16_t)1 << 7)
#define KEY_PRESSED_OFFSET_R            ((uint16_t)1 << 8)
#define KEY_PRESSED_OFFSET_F            ((uint16_t)1 << 9)
#define KEY_PRESSED_OFFSET_G            ((uint16_t)1 << 10)
#define KEY_PRESSED_OFFSET_Z            ((uint16_t)1 << 11)
#define KEY_PRESSED_OFFSET_X            ((uint16_t)1 << 12)
#define KEY_PRESSED_OFFSET_C            ((uint16_t)1 << 13)
#define KEY_PRESSED_OFFSET_V            ((uint16_t)1 << 14)
#define KEY_PRESSED_OFFSET_B            ((uint16_t)1 << 15) //

/* 车头/车尾标志：云台 yaw 转过 ±90° 时车头车尾对调 */
typedef enum
{
	HEAD,   // 车头朝前
	TAIL    // 车尾朝前（云台相对底盘转了约 180°）
}Head_fdb;

/* 三轴枚举：俯仰/偏航/横滚（本工程只用 PITCH/YAW，ROLL 未用） */
typedef enum
{
	PITCH,  // 俯仰轴
	YAW,    // 偏航轴
	ROLL    // 横滚轴(未用)
}Axis_e;

/* 云台工作模式（robot_mode_set 取值） */
typedef enum
{
	INIT_MODE,        // 上电初始化模式(极短，初始化完成后切走)
	ZERO_FORCE_MODE,  // 零力模式：力矩=0，云台可用手随便掰(安全，默认)
	NORMAL_MODE,      // 普通模式：遥控器/键鼠手动控制云台角度
	VISION_FOLLOW     // 视觉跟随：自瞄模式(由视觉偏移自动瞄准)
}Robot_Behavior_e;

/* 底盘跟随云台的模式（chassis_mode 取值） */
typedef enum
{
	CHASSIS_NO_FOLLOW_GIMBAL_MODE,  // 底盘不跟随：云台独立转，底盘按摇杆方向平移
	CHASSIS_FOLLOW_GIMBAL_MODE,     // 底盘跟随：云台转过一定角度底盘跟着转(小陀螺)
	CHASSIS_ROTATE_MODE,            // 底盘旋转模式：底盘原地旋转
	LIFT_ENABLE_MODE,               // 升降机构使能(本车可能未用)
	LIFT_DISABLE_MODE               // 升降机构失能(本车可能未用)
}Chassis_Behavior_e;


/* 电机角度控制模式（motor_mode 取值） */
typedef enum
{
	RELATIVE_MODE,  // 相对模式：以电机编码器相对角度为反馈(角度不累积)
	ABSOLUTE_MODE,  // 绝对模式：以 IMU 绝对角度为反馈(姿态稳)
	VISION_MODE     // 视觉模式：以视觉偏移为反馈(自瞄)
}Motor_Behavior_e;

/* 初始化进度标志位（init_flag 的各个 bit） */
typedef enum{
	INIT_START_BIT = 0,        // bit0: 开始初始化
	INIT_GYRO_START_BIT,       // bit1: 陀螺仪校准开始
	INIT_GYRO_SUCCESSFUL_BIT,  // bit2: 陀螺仪校准成功
	INIT_SUCCESSFUL_BIT        // bit3: 初始化全部完成
}Init_Flag_Bit_e;

/* 超级电容/超功率模式开关 */
typedef enum{
	SUPER_POWER_CLOSE,  // 关闭超功率
	SUPER_POWER_OPEN    // 开启超功率
}Super_Power_Mode_e;

/* 各数据源时间戳：记录每次更新时的系统 tick，用于计算各数据源是否卡顿/更新间隔。
 * model=主控 tick, rc=遥控器 tick, imu=IMU tick, gimbal[3]=电机 tick, vision=视觉 tick */
typedef struct
{
	uint32_t model;
	uint32_t rc;
	uint32_t imu;
	uint32_t gimbal[3];
	uint32_t vision;
}Gimbal_Time_t;

/* 陀螺仪(角速度)数据：fdb=反馈值(°/s)，set=目标值(外环PID输出)，filt=滤波值 */
typedef struct
{
	float fdb[3];
	float set[3];
	float filt[3];	
}Gyro_t;

/* 加速度计数据：fdb=反馈值，set=目标值(本工程基本不用) */
typedef struct
{
	float fdb[3];
	float set[3];
}Accel_t;

/* 角度数据：
 *   absolute_fdb/set[3]  IMU 绝对角（[0]=yaw [1]=roll [2]=pitch，与 IMU 下标一致）
 *   relative_fdb/set[2]  电机编码器相对角（[0]=pitch [1]=yaw）
 *   graph_*              给上位机画波形用的副本
 *   absolute_filt[3]     绝对角滤波值 */
typedef struct
{
	float absolute_fdb[3];
	float absolute_set[3];
	
	float relative_fdb[2];
	float relative_set[2];
	
	float graph_absolute_fdb;
	float graph_absolute_set;
	float graph_relative_fdb;
	float graph_relative_set;
	
	float absolute_filt[3];
}Angle_t;
/* 云台控制核心结构体：整个云台的状态全收在全局变量 gimbal 里。
 *   motor_set[2]           发给达妙电机的控制量 [0]=pitch [1]=yaw
 *   yaw/pitch_motor_fdb    两个达妙电机的反馈数据
 *   robot_mode_set/last    当前/上一次云台工作模式
 *   chassis_mode/last      当前/上一次底盘跟随模式
 *   motor_mode/last[2]     电机角度模式(相对/绝对/视觉)
 *   time_new/last/err      各数据源时间戳与更新间隔
 *   vision_data            视觉数据(自瞄偏移)
 *   angle / gyro           角度 / 陀螺仪角速度
 *   chassis_set[3]         底盘速度指令 [X]=前后 [Y]=左右 [Z]=旋转
 *   super_power_mode       超功率开关
 *   init_flag              初始化进度标志(见 Init_Flag_Bit_e)
 *   reset_flag             复位标志
 *   right_click            鼠标右键按住标志(自瞄)
 *   shootvison_mode        发射视觉模式标志 */
typedef struct
{
	DM_Motor_Contorl_Info_Typedef motor_set[2];
	
	DM_Motor_Info_Typedef yaw_motor_fdb;
	DM_Motor_Info_Typedef pitch_motor_fdb;

	Robot_Behavior_e robot_mode_set;
	Robot_Behavior_e robot_last_mode;
	  
	Chassis_Behavior_e chassis_mode;
	Chassis_Behavior_e chassis_last_mode;
	
	Motor_Behavior_e motor_mode[2];
	Motor_Behavior_e motor_last_mode[2];
	
	Gimbal_Time_t time_new;
	Gimbal_Time_t time_last;
	Gimbal_Time_t time_err;
	
	Vision_Data_t vision_data;
	Angle_t angle;
	Gyro_t gyro;
	
	float chassis_set[3];
	Super_Power_Mode_e super_power_mode;	
	int8_t init_flag;
	int8_t reset_flag;
	uint8_t right_click;
	uint8_t shootvison_mode ;
}Gimbal_t;

extern Gimbal_t gimbal;
extern xQueueHandle CAN1_gimbal_Queue;
void RobotReset(Gimbal_t* reset);
extern void gimbal_task(void const *pvParameters);
extern uint32_t GetMotorFR1Fdb(DJI_Motor_Info_Typedef * motor);
extern uint32_t GetMotorFR2Fdb(DJI_Motor_Info_Typedef * motor);

extern uint32_t GetPitchMotorFdb(DM_Motor_Info_Typedef* motor);
extern uint32_t GetYawMotorFdb(DM_Motor_Info_Typedef* motor);
extern uint32_t GetMotorPitchRelativeAngle(float* angle);
extern uint32_t GetMotorYawRelativeAngle(float* angle);
extern uint32_t GetShootMotorFdb(DJI_Motor_Info_Typedef* motor);
extern void GetIMUPitchAngleFdb(float* angle);
extern void GetIMUYawAngleFdb(float* angle);
extern void GetIMUPitchGyroFdb(float* gyro);
extern void GetIMUYawGyroFdb(float* gyro);
extern void TransmitChassis_Info(FDCAN_TxFrame_TypeDef *FDCAN_TxFrame);
extern void TransmitGimbal_Info(FDCAN_TxFrame_TypeDef *FDCAN_TxFrame);
#endif
