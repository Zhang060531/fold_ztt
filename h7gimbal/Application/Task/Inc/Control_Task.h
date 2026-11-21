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


#include "User_lib.h"
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
#define CHANNEL_PITCH_SCALE_FACTOR 2000.0f  // 俯仰摇杆→角度系数(越大动得越慢)     //9000
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
#define PITCH_SHELL_PID_KP 60
#define PITCH_SHELL_PID_KI 0
#define PITCH_SHELL_PID_KD 1
#define PITCH_SHELL_PID_MAX_IOUT 100   // 积分限幅(°/s)
#define PITCH_SHELL_PID_MAX_OUT 400    // 输出限幅(目标角速度上限 °/s)

/* ---- 俯仰轴 内环(角速度环) —— 手动控制 ----
 * 注意 KP 是负数：pitch 电机安装方向与 yaw 相反，需要反号才能负反馈 */
#define PITCH_CORE_PID_KP 0.1
#define PITCH_CORE_PID_KI 0
#define PITCH_CORE_PID_KD 0
#define PITCH_CORE_PID_MAX_IOUT 0.5
#define PITCH_CORE_PID_MAX_OUT 5.5   // 输出限幅(力矩上限 N·m)

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
#define PITCH_VISION_SHELL_PID_KI 0.2   //0.2
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


/* =====================================================================
 * GM6020 双环 PID（位置环 + 速度环）参数
 * 外环 位置环：角度误差(°) → 目标转速(rpm)
 * 内环 速度环：转速误差(rpm) → 电压
 * ===================================================================== */
#define GM6020_SHELL_PID_KP  7.0f        // 位置环 P：1°角度误差 -> 7rpm 目标转速
#define GM6020_SHELL_PID_KI  0.02f        // 位置环 I：先给 0，有静差再加
#define GM6020_SHELL_PID_KD  0.0f        // 位置环 D：先给 0
#define GM6020_SHELL_PID_MAX_IOUT  100.0f  // 位置环积分限幅(rpm)
#define GM6020_SHELL_PID_MAX_OUT   100.0f  // 位置环输出限幅(目标转速上限 rpm)

#define GM6020_CORE_PID_KP   50.0f       // 速度环 P：1rpm转速误差 → 60 电压
#define GM6020_CORE_PID_KI   0.0f        // 速度环 I：先给 0
#define GM6020_CORE_PID_KD   0.0f        // 速度环 D：先给 0
#define GM6020_CORE_PID_MAX_IOUT  10000.0f  // 速度环积分限幅(电压)
#define GM6020_CORE_PID_MAX_OUT   10000.0f  // 速度环输出限幅(电压)


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

//大pitch GM6020折叠/展开的目标角度（按照安装时的水平和竖直角度 要改！！）
#define GM6020_EXTEND_ANGLE  56.0f  //展开：大pitch立起来（正常作战角度）
#define GM6020_FOLD_ANGLE 125.0f   //收缩：大pitch趴下果冻
//收缩时小pitch保持的水平角度（也是按实测水平fdb角度填 哦对达秒可以直接设置零点）
#define PITCH_LEVEL_ANGLE 0.0f  //收缩时小pitch保持的目标角度

//大pitch抱闸死区（越小就越准但难锁上 越大越容易锁上但停的糙）
#define GM6020_LOCK_DEADBAND 4.0f  //误差小于它就锁死
#define GM6020_UNLOCK_DEADBAND 7.0f  //误差大于它才松开重动

/* =====================================================================
 * 大 pitch 抱闸器硬件接口
 * ---------------------------------------------------------------------
 * 抱闸器控制脚固定为 PC14，禁止再写成 GPIO_PIN_0：
 *   SET   -> 抱闸线圈通电，机械抱闸释放，此时 GM6020 才能带动大 pitch
 *   RESET -> 抱闸线圈断电，内部弹簧把大 pitch 抱死
 *
 * 使用宏统一控制，避免不同位置直接操作 GPIO 时写错端口或引脚。
 * ===================================================================== */
#define GM6020_BRAKE_POWER_ON()   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET)
#define GM6020_BRAKE_POWER_OFF()  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET)

// 大 pitch 抱闸动作时序。
#define GM6020_BRAKE_RELEASE_WAIT_MS   100U   //抱闸通电后等待机械释放
#define GM6020_BRAKE_SETTLE_TIME_MS    100U   //进入目标死区后连续稳定多久才断抱闸
#define GM6020_BRAKE_ENGAGE_HOLD_MS     50U   //抱闸断电后继续闭环保持，等待机械咬合
#define GM6020_BRAKE_SETTLE_SPEED_RPM   10.0f //到位稳定判定允许的最大转速
#define GM6020_FEEDBACK_TIMEOUT_MS     100U   //GM6020 反馈超时后停止输出并抱闸

///* 大 pitch 重力前馈。
// * 折叠方向认为受重力下拉，因此重力前馈方向与折叠方向相反。
// * 先给较小值测试：如果松闸后有向上冲的趋势，减小 VOLTAGE；
// * 如果松闸后仍然明显下坠，逐步增大 VOLTAGE。
// * 如果方向完全反了，把 SIGN 改成 +1.0f。 */
//#define GM6020_GRAVITY_FF_VOLTAGE       2100.0f
//#define GM6020_GRAVITY_FF_SIGN \
//    ((GM6020_FOLD_ANGLE > GM6020_EXTEND_ANGLE) ? -1.0f : 1.0f)

////大pitch重力前馈(余弦建模)
////重力力矩 = K*cos(a-b)
////a = GM6020_Motor.Data.Angle(当前大pitch角度)
////b = 重臂最大(大pitch水平)时的角度读数 待标定
////K = 水平位刚好悬停所需的前馈电压 待标定
////SIGN = 方向 标定时试出松闸不掉的方向
//#define GM6020_GRAVITY_FF_VOLTAGE    2100.0f   //K  
//#define GM6020_GRAVITY_FF_SIGN       -1.0f     //SIGN

/* 大 pitch 重力前馈 —— 正比 cos(仰角)。
 * 伸展=垂直(仰角90°)→重力≈0；折叠=离水平30°(仰角30°)→重力≈0.87最大。
 * K 直接取“折叠位(30°仰角)刚好悬停的电压”，公式里已按 cos 归一化。 */
/* ==== 调参阶段：下面宏先注释，改用全局变量（Keil Watch 实时改），调好再换回宏 ==== */
//#define GM6020_GRAVITY_FF_VOLTAGE       2600.0f   // K：折叠位悬停电压
//#define GM6020_GRAVITY_FF_SIGN           -1.0f    // 方向 ±1
//#define GM6020_EXTEND_ELEV_DEG           90.0f    // 伸展=垂直，仰角90°
//#define GM6020_FOLD_ELEV_DEG             30.0f    // 折叠离水平约30°
//#define GM6020_FOLD_SPEED_DEG_S          130.0f   // 全速巡航角速度(°/s)
//#define GM6020_BRAKE_DIST_DEG            20.0f    // 最后N°线性减速到0

/* 调参用全局变量（定义在 Control_Task.c），调试时在 Watch 窗口改 */
extern volatile float gm6020_ff_voltage;        // K：折叠位悬停电压
extern volatile float gm6020_ff_sign;           // 方向 ±1
extern volatile float gm6020_extend_elev_deg;   // 伸展仰角(°)
extern volatile float gm6020_fold_elev_deg;     // 折叠仰角(°)
extern volatile float gm6020_speed_extend_deg_s;  // 伸展(立起来)巡航速度(°/s)
extern volatile float gm6020_speed_fold_deg_s;    // 折叠(下去)巡航速度(°/s)
extern volatile float gm6020_brake_dist_extend_deg;  // 伸展(立起来)刹车距离(°)
extern volatile float gm6020_brake_dist_fold_deg;    // 折叠(下去)刹车距离(°)

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

#define PITCH_ABSOLUTE_MAX_ANGLE 35   // 俯仰绝对模式上限(抬头最大 +25°)
#define PITCH_ABSOLUTE_MIN_ANGLE -10   // 俯仰绝对模式下限(低头最大 -3°)
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

#define CHASSIS_X_Y_Z_ID 0x301   //没用
#define CHASSIS_UI_YAW_ID 0x299 //0x302  //没用


#define REMOTE_CHANNLE_TO_CHASSIS_SPEED 0.015f                                                //0.01	
#define ROTATE_REMOTE_CHANNLE_TO_CHASSIS_SPEED (REMOTE_CHANNLE_TO_CHASSIS_SPEED / 50)     //没用


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
	ZERO_FORCE_MODE,  // 零力模式：力矩=0，云台可用手随便掰(安全，默认)
	VISION_FOLLOW,     // 视觉跟随：自瞄模式(由视觉偏移自动瞄准)
	
	EXTEND_MODE,
	FOLD_MODE
	
}Robot_Behavior_e;

/* 底盘跟随云台的模式（chassis_mode 取值） */
typedef enum
{
	CHASSIS_NO_FOLLOW_GIMBAL_MODE,  // 底盘不跟随：云台独立转，底盘按摇杆方向平移
	CHASSIS_FOLLOW_GIMBAL_MODE,     // 底盘跟随：云台转过一定角度底盘跟着转(小陀螺)
	CHASSIS_ROTATE_MODE,            // 底盘旋转模式：底盘原地旋转
}Chassis_Behavior_e;


/* 电机角度控制模式（motor_mode 取值） */
typedef enum
{
	RELATIVE_MODE,  // 相对模式：以电机编码器相对角度为反馈(角度不累积)
	ABSOLUTE_MODE,  // 绝对模式：以 IMU 绝对角度为反馈(姿态稳)
	VISION_MODE     // 视觉模式：以视觉偏移为反馈(自瞄)
}Motor_Behavior_e;


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
 *   absolute_filt[3]     绝对角滤波值 */
typedef struct
{
	float absolute_fdb[3];
	float absolute_set[3];
	
	float relative_fdb[2];
	float relative_set[2];
	
	float absolute_filt[3];
}Angle_t;

/* 大 pitch 抱闸状态：
 * LOCKED    : 抱闸断电，电机输出 0
 * RELEASING : 抱闸通电，保持当前角度，等待机械释放
 * MOVING    : 闭环运行到目标角度
 * SETTLING  : 已进入目标死区，连续等待稳定
 * ENGAGING  : 抱闸已断电，继续短时保持等待机械咬合
 */
typedef enum
{
    GM6020_BRAKE_LOCKED = 0,  //抱闸断电抱死，GM6020 不发力
    GM6020_BRAKE_RELEASING,   //抱闸已通电，等待机械释放，同时保持当前角度
    GM6020_BRAKE_MOVING,      //抱闸已释放，闭环运行到目标角度
    GM6020_BRAKE_SETTLING,    //已进入目标死区，连续等待稳定
    GM6020_BRAKE_ENGAGING     //抱闸已断电，继续短时保持以等待机械咬合
} Gm6020_BrakeState_e;

/* GM6020（大 pitch）控制状态：PID + 目标/抱闸状态机/输出 收在一起 */
typedef struct
{
    float  target_angle;          //折叠/展开最终目标角度，单位：度
    
		float  cmd_angle;             //斜坡后的指令角(°)，PID 真正追的设定，每1ms朝 target_angle 逼近
		
		int16_t voltage;              //GM6020 双环 PID 输出，最终经 CAN 下发，范围约 -30000~30000
    
		Gm6020_BrakeState_e brake_state; //当前抱闸时序状态，见 Gm6020_BrakeState_e
    float release_hold_angle;     //RELEASING 阶段保持的角度，防止松闸瞬间下坠
    int16_t gravity_ff_voltage;   //当前重力前馈电压，方便在 Watch/VoFA 中观察
    uint32_t state_enter_ms;      //进入当前状态时的 FreeRTOS tick，用于计算各阶段等待时间
    uint32_t settle_start_ms;     //连续满足目标死区的起始 tick，用于判断是否已经稳定
} Gm6020_Ctrl_t;

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
 *   reset_flag             底盘复位标志
 *   right_click            鼠标右键按住标志(自瞄)
 *   shootvison_mode        发射视觉模式标志 */
typedef struct
{
	DM_Motor_Contorl_Info_Typedef motor_set[2];
	
	DM_Motor_Info_Typedef yaw_motor_fdb;
	DM_Motor_Info_Typedef pitch_motor_fdb;
	
	Gm6020_Ctrl_t gm6020;   //大 pitch(GM6020) 的控制状态
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

	int8_t reset_flag;
	uint8_t right_click;
	uint8_t shootvison_mode ;   
}Gimbal_t;

extern Gimbal_t gimbal;
extern xQueueHandle CAN1_gimbal_Queue;

/* GM6020 双环 PID 相关全局变量（定义在 Control_Task.c） */
//extern PID_Info_TypeDef gm6020_shell_pid;    // GM6020 外环 位置环 PID
//extern PID_Info_TypeDef gm6020_core_pid;     // GM6020 内环 速度环 PID


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
