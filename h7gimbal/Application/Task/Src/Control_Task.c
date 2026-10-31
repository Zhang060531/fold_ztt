/**
 * =====================================================================
 * 文件说明：云台控制任务 (Control_Task.c) —— 云台控制核心
 * ---------------------------------------------------------------------
 * 这是整个云台最核心的文件。FreeRTOS 任务 Control_Task 在这里跑，
 * 每 1ms 循环一次（见底部 osDelay(1)），负责：
 *
 *  1. 读反馈：电机角度、IMU 角度、陀螺仪角速度、视觉数据
 *  2. 设模式：根据遥控器/键盘/鼠标决定云台工作模式
 *  3. 生成目标角度（俯仰/偏航的设定值）
 *  4. 计算底盘速度指令（通过 CAN 0x21 帧发给底盘）
 *  5. 跑串级 PID，算出力矩发给达妙电机
 *  6. 把云台状态通过 CAN 0x22 帧发出去（给底盘/上位机）
 *
 * ===== 控制结构（串级 PID，最重要）=====
 * pitch(俯仰) 和 yaw(偏航) 每个轴都是「双环串级 PID」：
 *   外环 角度环  (gimbal_shell_pid)：角度误差  → 目标角速度 gyro.set
 *   内环 角速度环(gimbal_core_pid) ：角速度误差 → 力矩 motor_set.Torque
 * 链条：角度设定 → [角度PID] → 角速度设定 → [角速度PID] → 力矩 → 发给达妙电机
 *
 * ===== 三种工作模式（robot_mode_set）=====
 *  - ZERO_FORCE_MODE 零力模式：不控制，力矩=0（上电默认，安全）
 *  - NORMAL_MODE     普通模式：遥控器手动控制云台角度
 *  - VISION_FOLLOW   视觉跟随：自瞄，根据视觉偏移自动瞄准
 * 电机角度本身还有 绝对/相对/视觉 三种模式(motor_mode)。
 *
 * ===== 调车关注 =====
 *  - PID 参数都在 Control_Task.h 顶部的一堆 *_PID_* 宏里，改参数改那里
 *  - 两组 PID（正常 gimbal_* / 视觉 vision_*）都在 GimbalInit() 里初始化
 *  - 给底盘的指令在 ChassisSpeedSet()，云台状态在 TransmitGimbal_Info()
 * =====================================================================
 */
#include "Control_Task.h"
#include "shoot_task.h"
#include "CAN_Task.h"
#include "INS_task.h"
#include "usbd_cdc_if.h"
#include "main.h"
#include "pid.h"
#include "cmsis_os.h"
#include "bsp_can.h"
#include "remote_control.h"
#include "vision.h"
#include "math.h"
#include "stdio.h"
#include "stdbool.h"
#include "Image_Transmission.h"  
#include "CRC.h" 
#include "usbd_cdc_if.h"
#include "LPF.h"

/*********************************************************/
/* ==================== 全局变量定义 ==================== */
/*********************************************************/
Gimbal_t gimbal;                // 云台控制核心结构体，存所有反馈值/设定值/模式
PID_Info_TypeDef gimbal_shell_pid[2];  // 外环「角度环」PID，[0]=pitch [1]=yaw
PID_Info_TypeDef vision_shell_pid[2];  // 视觉自瞄的「角度环」PID
PID_Info_TypeDef gimbal_core_pid[2];   // 内环「角速度环」PID，[0]=pitch [1]=yaw
PID_Info_TypeDef vision_core_pid[2];   // 视觉自瞄的「角速度环」PID
int vision_flag = 0;            // 视觉模式标志（1=开视觉）
PID_Info_TypeDef follow_pid, follow_pid2;  // 底盘跟随云台时用的 PID（算底盘旋转量）

//GM6020 双环 PID：外环位置环 + 内环速度环 
PID_Info_TypeDef gm6020_shell_pid;           // 外环 位置环（角度误差→目标转速rpm）
PID_Info_TypeDef gm6020_core_pid;            // 内环 速度环（转速误差→电压）
volatile float  gm6020_target_angle = 0.0f;  // 目标角度(°)，调试时在 Keil Watch 里改
volatile uint8_t gm6020_enable = 0;          // 使能开关：0=关(电压恒0，安全)，1=开
int16_t gm6020_voltage = 0;                  // 双环算出的电压，CAN_Task 里下发


float key_scale_factor = 2.0;   // 键盘控制底盘/云台时的速度系数
uint8_t switchflag = 0;
bool head_flag = HEAD;          // 云台当前朝向：车头(HEAD) 还是 车尾(TAIL)
extern Image_Transmission_Info_TypeDef Image_Transmission_Info;  // 图传数据（外部定义）
extern VT13_Info_TypeDef VT13_Info;    // VT13 图传遥控器数据（键鼠）
extern Remote_Info_Typedef remote_ctrl;  // 大疆 DR16 遥控器数据（摇杆拨杆）
extern Shoot_t shoot;                  // 发射机构数据（摩擦轮/拨弹轮）
/*********************************************************/
/* ============ 遥控器 → 车 的控制链=========== */
/*********************************************************/
/* 本工程同时接了两套遥控，代码里它们的输入是「相加」的：
 *
 * [大疆 DT7 遥控器] (DR16 接收机, SBUS/USART5) —— 变量 remote_ctrl
 *     rc.ch[0]  右摇杆左右 → 云台 yaw 偏航目标角度
 *     rc.ch[1]  右摇杆上下 → 云台 pitch 俯仰目标角度
 *     rc.ch[2]  左摇杆      → 底盘左右平移 (配合 A/D 键)
 *     rc.ch[3]  左摇杆      → 底盘前后平移 (配合 W/S 键)
 *     rc.ch[4]  拨轮        → 拨弹轮 (发射)
 *     rc.s[0]   左上拨杆    → 云台模式开关 (2/3=普通, 其它=零力)
 *     rc.s[1]   右上拨杆    → 作战模式 (1=自瞄, 3=云底跟随, 2=小陀螺/旋转)
 *     rc.mouse  鼠标        → 瞄准 (pitch/yaw 微调)
 *     rc.key    键盘 WASD   → 底盘平移 + 模式快捷键
 *
 * [图传遥控器 VT13] (图传/USART1 透传) —— 变量 VT13_Info
 *     RC.Channel[0..3] 摇杆4通道      RC.Wheel 滚轮
 *     RC.Switch/Stop/Left/Right/Trigger 开关/停止/左/右/扳机
 *     Key.V 键盘        Mouse.* 鼠标
 *
 *********************************************************/
uint32_t vision_last_valid_time[2] = {0, 0};  // YAW/PITCH 最后一次收到有效视觉数据的时间
#define VISION_DATA_TIMEOUT_MS 500  // 500ms 没收到视觉数据就认为视觉丢失

/* 视觉目标速度估计 (速度前馈用) */
static LowPassFilter1p_Info_TypeDef vision_vel_lpf[2];   // YAW/PITCH 速度低通滤波
static float vision_target_vel[2] = {0};                   // 估算的目标角速度 (°/s)
/* 发给视觉的Pitch角度低通滤波 */
static LowPassFilter1p_Info_TypeDef pitch_tx_lpf;          // 发送Pitch角度滤波
/*********************************************************/

/*【新增】yaw 手动控制的速度前馈：参考角速度(°/s)，YawAngleRefSet 里算好，pidCalc 里前馈到内环 */
static float yaw_ref_vel = 0.0f;
volatile float yaw_vel_ff_gain = 0.9f;   /*【新增】速度前馈增益，调试时在 Watch 窗口修改该值即可实时调参 */

/*【新增】pitch 手动控制的速度前馈：和 yaw 一样，PitchAngleRefSet 里算好，pidCalc 里前馈到内环 */
static float pitch_ref_vel = 0.0f;
volatile float pitch_vel_ff_gain = 0.9f; /*【新增】pitch 速度前馈增益，调试时在 Watch 窗口修改该值即可实时调参 */


/*********************************************************/
/* ==================== 静态函数声明 ==================== */
/*********************************************************/
static void GimbalInit(void);
static void GetDataFdb(Gimbal_t* gimbal_fdb);
static void GimbalModeSet(Gimbal_t* gimbal_mode_set);
static void ChassisModeSet(Gimbal_t* ch_mode_set);
static void PitchAngleRefSet(Gimbal_t* pitch_angle_set);
static void YawAngleRefSet(Gimbal_t* yaw_angle_set);
static void ChassisSpeedSet(Gimbal_t* chassis_speed);
static void PitchPidCalc(Gimbal_t* pitch_pid_calc);
static void YawPidCalc(Gimbal_t* yaw_pid_calc);

static void GM6020PidCalc(void);

static void GimbalModeChangeControlTransit(Gimbal_t* gimbal_mode_change);
static void addPitchAngle(Gimbal_t* pitch, float add);
static void addYawAngle(Gimbal_t* yaw, float add);
static void pidCalc(Gimbal_t* pid_calc, Axis_e axis);
static void TimeErrorCalc(Gimbal_t* gimbal);
float ramp_speed(float last_speed, float target_speed, float max_step);
void TransmitChassis_Info(FDCAN_TxFrame_TypeDef *FDCAN_TxFrame);
void TransmitGimbal_Info(FDCAN_TxFrame_TypeDef *FDCAN_TxFrame);
/*********************************************************/
/**
 * @brief 云台控制任务主函数（FreeRTOS 任务入口）
 * @note  每 1ms 循环一次，按固定顺序执行下面 9 个步骤。
 *        顺序很关键：先读反馈→定模式→算目标角度→算底盘速度→跑 PID。
 */
void Control_Task(void const *pvParameters)
{
    GimbalInit();                       // 1. 上电初始化：PID 参数、默认模式
    while (1)
    {
        GetDataFdb(&gimbal);            // 2. 读反馈：电机/IMU/陀螺仪/视觉数据
			
			 //设定模式相关
        GimbalModeSet(&gimbal);         // 3. 根据遥控器设定云台工作模式
        ChassisModeSet(&gimbal);        // 4. 根据遥控器设定底盘跟随模式
        GimbalModeChangeControlTransit(&gimbal);  // 5. 模式切换时的角度平滑过渡
			
				//大pitch
			  GM6020PidCalc();  							//GM6020双环pid算电压（角度-->电压）
			  //小pitch
        PitchAngleRefSet(&gimbal);      // 6. 计算 pitch 俯仰目标角度
				//yaw
        YawAngleRefSet(&gimbal);        // 7. 计算 yaw 偏航目标角度
				//设定底盘
        ChassisSpeedSet(&gimbal);       // 8. 计算底盘速度指令（0x21 帧）
			
        PitchPidCalc(&gimbal);          // 9. pitch 轴串级 PID 算力矩
        YawPidCalc(&gimbal);            // 10. yaw 轴串级 PID 算力矩
			
	
				osDelay(1);                     // 延时 1ms，形成 1kHz 控制周期
    }
}

/**
 * @brief 云台上电初始化：设置默认模式 + 初始化所有 PID 参数
 * @note  这里初始化 6 组 PID，分成两套：
 *         - gimbal_*  ：手动控制用的 PID（外环角度环 + 内环角速度环）
 *         - vision_*  ：视觉自瞄用的 PID（一般参数更猛，自瞄更跟手）
 *        注意：下面 gimbal_shell_pid[YAW] 被初始化了两次，第一次
 *        LimitIntegral/LimitOutput 写反了，第二次才是对的，以第二次为准。
 */
static void GimbalInit(void)
{
		gimbal.robot_mode_set = ZERO_FORCE_MODE;
    gimbal.super_power_mode = SUPER_POWER_CLOSE;
    gimbal.chassis_mode = CHASSIS_FOLLOW_GIMBAL_MODE;


    /* ===== 云台俯仰轴(pitch) 外环「角度环」PID 参数 ===== */
    gimbal_shell_pid[PITCH].Param.KP = PITCH_SHELL_PID_KP;
    gimbal_shell_pid[PITCH].Param.KI = PITCH_SHELL_PID_KI;
    gimbal_shell_pid[PITCH].Param.KD = PITCH_SHELL_PID_KD;
    gimbal_shell_pid[PITCH].Param.LimitIntegral = PITCH_SHELL_PID_MAX_IOUT;
    gimbal_shell_pid[PITCH].Param.LimitOutput = PITCH_SHELL_PID_MAX_OUT;
    gimbal_shell_pid[PITCH].Type = PID_POSITION;
		
		/* ===== 云台俯仰轴(pitch) 内环「角速度环」PID 参数 ===== */
    gimbal_core_pid[PITCH].Param.KP = PITCH_CORE_PID_KP;
    gimbal_core_pid[PITCH].Param.KI = PITCH_CORE_PID_KI;
    gimbal_core_pid[PITCH].Param.KD = PITCH_CORE_PID_KD;
    gimbal_core_pid[PITCH].Param.LimitIntegral = PITCH_CORE_PID_MAX_IOUT; 
    gimbal_core_pid[PITCH].Param.LimitOutput = PITCH_CORE_PID_MAX_OUT;
    gimbal_core_pid[PITCH].Type = PID_POSITION; 

//    /* ===== 云台偏航轴(yaw) 外环「角度环」PID 参数 ===== */
    gimbal_shell_pid[YAW].Param.KP = YAW_SHELL_PID_KP;
    gimbal_shell_pid[YAW].Param.KI = YAW_SHELL_PID_KI;
    gimbal_shell_pid[YAW].Param.KD = YAW_SHELL_PID_KD;
    gimbal_shell_pid[YAW].Param.LimitIntegral = YAW_SHELL_PID_MAX_IOUT;
    gimbal_shell_pid[YAW].Param.LimitOutput = YAW_SHELL_PID_MAX_OUT;
    gimbal_shell_pid[YAW].Type = PID_POSITION;

    /* ===== 云台偏航轴(yaw) 内环「角速度环」PID 参数 ===== */
    gimbal_core_pid[YAW].Param.KP = YAW_CORE_PID_KP;
    gimbal_core_pid[YAW].Param.KI = YAW_CORE_PID_KI;
    gimbal_core_pid[YAW].Param.KD = YAW_CORE_PID_KD;
    gimbal_core_pid[YAW].Param.LimitIntegral = YAW_CORE_PID_MAX_IOUT;   
    gimbal_core_pid[YAW].Param.LimitOutput = YAW_CORE_PID_MAX_OUT;  
    gimbal_core_pid[YAW].Type = PID_POSITION;   


		/* ===== 视觉自瞄 俯仰轴(pitch) 外环「角度环」PID 参数 ===== */
    vision_shell_pid[PITCH].Param.KP = PITCH_VISION_SHELL_PID_KP;
    vision_shell_pid[PITCH].Param.KI = PITCH_VISION_SHELL_PID_KI;   
    vision_shell_pid[PITCH].Param.KD = PITCH_VISION_SHELL_PID_KD;   
    vision_shell_pid[PITCH].Param.LimitIntegral = PITCH_VISION_SHELL_PID_MAX_IOUT; 
    vision_shell_pid[PITCH].Param.LimitOutput = PITCH_VISION_SHELL_PID_MAX_OUT;
    vision_shell_pid[PITCH].Type = PID_DERIVATIVE_ON_MEASURE;
		    
    /* ===== 视觉自瞄 俯仰轴(pitch) 内环「角速度环」PID 参数 ===== */
    vision_core_pid[PITCH].Param.KP = VISION_PITCH_CORE_PID_KP;
    vision_core_pid[PITCH].Param.KI = VISION_PITCH_CORE_PID_KI; 
    vision_core_pid[PITCH].Param.KD = VISION_PITCH_CORE_PID_KD; 
    vision_core_pid[PITCH].Param.LimitIntegral = VISION_PITCH_CORE_PID_MAX_IOUT; 
    vision_core_pid[PITCH].Param.LimitOutput = VISION_PITCH_CORE_PID_MAX_OUT;
    vision_core_pid[PITCH].Type = PID_POSITION;/* PID类型：位置式PID */

    /* ===== 视觉自瞄 偏航轴(yaw) 外环「角度环」PID 参数 ===== */
    vision_shell_pid[YAW].Param.KP = YAW_VISION_SHELL_PID_KP;
    vision_shell_pid[YAW].Param.KI = YAW_VISION_SHELL_PID_KI;
    vision_shell_pid[YAW].Param.KD = YAW_VISION_SHELL_PID_KD;
    vision_shell_pid[YAW].Param.LimitIntegral = YAW_VISION_SHELL_PID_MAX_IOUT;
    vision_shell_pid[YAW].Param.LimitOutput = YAW_VISION_SHELL_PID_MAX_OUT;
    vision_shell_pid[YAW].Type = PID_DERIVATIVE_ON_MEASURE;
        
    /* ===== 视觉自瞄 偏航轴(yaw) 内环「角速度环」PID 参数 ===== */
    vision_core_pid[YAW].Param.KP = VISION_YAW_CORE_PID_KP;
    vision_core_pid[YAW].Param.KI = VISION_YAW_CORE_PID_KI; 
    vision_core_pid[YAW].Param.KD = VISION_YAW_CORE_PID_KD; 
    vision_core_pid[YAW].Param.LimitIntegral = VISION_YAW_CORE_PID_MAX_IOUT; 
    vision_core_pid[YAW].Param.LimitOutput = VISION_YAW_CORE_PID_MAX_OUT;
    vision_core_pid[YAW].Type = PID_POSITION;  


    /* ===== 底盘跟随云台用的 PID（算底盘旋转量，见 ChassisSpeedSet）===== */
    follow_pid.Param.KP = FOLLOW_PID_KP;
    follow_pid.Param.KI = FOLLOW_PID_KI;
    follow_pid.Param.KD = FOLLOW_PID_KD;
    follow_pid.Param.LimitIntegral = FOLLOW_PID_MAX_IOUT;
    follow_pid.Param.LimitOutput = FOLLOW_PID_MAX_OUT;
    follow_pid.Type = PID_POSITION;
		
		/* ===== GM6020 双环 PID（位置环 + 速度环）参数 ===== */
    gm6020_shell_pid.Param.KP = GM6020_SHELL_PID_KP;
    gm6020_shell_pid.Param.KI = GM6020_SHELL_PID_KI;
    gm6020_shell_pid.Param.KD = GM6020_SHELL_PID_KD;
    gm6020_shell_pid.Param.LimitIntegral = GM6020_SHELL_PID_MAX_IOUT;
    gm6020_shell_pid.Param.LimitOutput  = GM6020_SHELL_PID_MAX_OUT;
    gm6020_shell_pid.Type = PID_POSITION;

    gm6020_core_pid.Param.KP = GM6020_CORE_PID_KP;
    gm6020_core_pid.Param.KI = GM6020_CORE_PID_KI;
    gm6020_core_pid.Param.KD = GM6020_CORE_PID_KD;
    gm6020_core_pid.Param.LimitIntegral = GM6020_CORE_PID_MAX_IOUT;
    gm6020_core_pid.Param.LimitOutput  = GM6020_CORE_PID_MAX_OUT;
    gm6020_core_pid.Type = PID_POSITION;
		
    /* 初始化视觉速度估计低通滤波器 */
    LowPassFilter1p_Init(&vision_vel_lpf[YAW],   VISION_VEL_LPF_ALPHA);
    LowPassFilter1p_Init(&vision_vel_lpf[PITCH], VISION_VEL_LPF_ALPHA);
    /* 初始化发给视觉的Pitch角度低通滤波器 */
    LowPassFilter1p_Init(&pitch_tx_lpf, PITCH_TX_LPF_ALPHA);

}
/**
 * @brief 读取所有反馈数据（每 1ms 调用一次）
 * @note  读的反馈包括：
 *         - pitch/yaw 电机角度（相对角）→ 用于判断车头车尾、算设定
 *         - IMU 的绝对角度（yaw/pitch）和陀螺仪角速度 → 用于串级 PID 内环
 *         - 视觉数据（自瞄偏移量）→ 存到 vision_data
 *         - 视觉数据有效性计时：偏移量在阈值内才刷新时间戳，
 *           超过 500ms 没刷新就认为视觉丢失（见 pidCalc 里的超时判断）
 */
static void GetDataFdb(Gimbal_t* gimbal_fdb)
{
    gimbal_fdb->time_last = gimbal_fdb->time_new;
    gimbal_fdb->time_new.model = xTaskGetTickCount();

		//看函数返回值  覆盖？？？
    gimbal_fdb->time_new.gimbal[PITCH] = GetPitchMotorFdb(&(gimbal_fdb->pitch_motor_fdb));
    gimbal_fdb->time_new.gimbal[YAW] = GetYawMotorFdb(&(gimbal_fdb->yaw_motor_fdb));
    gimbal_fdb->time_new.gimbal[PITCH] = GetMotorPitchRelativeAngle(&(gimbal_fdb->angle.relative_fdb[PITCH]));
    gimbal_fdb->time_new.gimbal[YAW] = GetMotorYawRelativeAngle(&(gimbal_fdb->angle.relative_fdb[YAW]));
		
		//标记头还是尾
    if ((gimbal_fdb->angle.relative_fdb[YAW] > 90 && gimbal_fdb->angle.relative_fdb[YAW] < 180) || (gimbal_fdb->angle.relative_fdb[YAW] < -90&&gimbal_fdb->angle.relative_fdb[YAW] > -180))
        head_flag = TAIL;
    else if (gimbal_fdb->angle.relative_fdb[YAW] < 90 && gimbal_fdb->angle.relative_fdb[YAW] > -90)
        head_flag = HEAD;

    GetIMUYawAngleFdb(&(gimbal_fdb->angle.absolute_fdb[YAW]));
    GetIMUPitchAngleFdb(&(gimbal_fdb->angle.absolute_fdb[PITCH]));
    GetIMUPitchGyroFdb(&(gimbal_fdb->gyro.fdb[PITCH]));
    GetIMUYawGyroFdb(&(gimbal_fdb->gyro.fdb[YAW]));

    GetVisionData(&(gimbal.vision_data));

static uint32_t last_check_time = 0;
uint32_t current_time = HAL_GetTick();

if (current_time - last_check_time >= 10) 
{  
    last_check_time = current_time;
    
    if (gimbal_fdb->vision_data.vision_flag == 1) 
    {

        if (fabs(gimbal_fdb->vision_data.vision_x_offect) < 360.0f) 
        {
            vision_last_valid_time[YAW] = current_time;
        }
        
        if (fabs(gimbal_fdb->vision_data.vision_y_offect) < 90.0f) 
        {
            vision_last_valid_time[PITCH] = current_time;
        }
    }
    else 
    {

    }
}
    TimeErrorCalc(gimbal_fdb);
}

/**
 * @brief 计算各传感器两次更新之间的时间差（毫秒）
 * @note  用来检测各数据源（遥控器/IMU/电机/视觉）是否卡顿，时间差存进 time_err。
 */
static void TimeErrorCalc(Gimbal_t* gimbal)
{
    gimbal->time_err.model = gimbal->time_new.model - gimbal->time_last.model;
    gimbal->time_err.rc = gimbal->time_new.rc - gimbal->time_last.rc;
    gimbal->time_err.imu = gimbal->time_new.imu - gimbal->time_last.imu;
    gimbal->time_err.gimbal[0] = gimbal->time_new.gimbal[0] - gimbal->time_last.gimbal[0];
    gimbal->time_err.gimbal[1] = gimbal->time_new.gimbal[1] - gimbal->time_last.gimbal[1];
    gimbal->time_err.gimbal[2] = gimbal->time_new.gimbal[2] - gimbal->time_last.gimbal[2];
    gimbal->time_err.vision = gimbal->time_new.vision - gimbal->time_last.vision;
}

int Key_ctrl_flag = 0;

int Key_r_flag = 0;
int8_t pause = 0;
int8_t pause_last = 0;
int8_t UI_force_mode = 0;
int8_t ui_rotate_mode = 0;
int8_t ui_rotate_mode_last = 0;

/**
 * @brief 根据遥控器/键鼠设定云台工作模式（核心的按键逻辑）
 * @note  用「边沿触发」检测按键：只有当某个键从"没按"变成"按下"那一下才动作，
 *        避免按住时重复触发。主要逻辑：
 *         - 拨杆 s[0]=2/3 或按 Z 键(UI_force_mode)：进入普通模式 NORMAL_MODE
 *         - 否则：零力模式 ZERO_FORCE_MODE（安全）
 *         - 按 Shift 键/拨杆切换：底盘旋转模式 ↔ 底盘跟随模式（循环）
 *         - 按 R 键：底盘「不跟随云台」↔「跟随云台」切换
 *         - 按 Q 键：把拨杆 s[0]/s[1] 清零（一键归零，方便切换）
 *         - 按住鼠标右键 或 右上拨杆 s[1]=1 档：进入视觉自瞄模式 VISION_MODE
 */
static void GimbalModeSet(Gimbal_t* gimbal_mode_set)
{		
    gimbal_mode_set->robot_last_mode = gimbal_mode_set->robot_mode_set;
    gimbal_mode_set->motor_last_mode[PITCH] = gimbal_mode_set->motor_mode[PITCH];
    gimbal_mode_set->motor_last_mode[YAW] = gimbal_mode_set->motor_mode[YAW];
		

    static  uint16_t REF_key_robot_mode = 0;
    static  uint16_t REF_key_robot_mode_last = 0;
    static int8_t REF_mouse_right = 0;
    static int8_t REF_mouse_right_last = 0;
    static int8_t REF_mouse_middle = 0;
    static int8_t REF_mouse_middle_last = 0;
		

    REF_key_robot_mode_last = REF_key_robot_mode ;
    REF_key_robot_mode = VT13_Info.Key.V;      // 【图传VT13】键盘
    REF_mouse_right_last = REF_mouse_right;
    REF_mouse_right = VT13_Info.Mouse.Press_R; // 【图传VT13】鼠标右键
    REF_mouse_middle_last = REF_mouse_middle;
    ui_rotate_mode_last = ui_rotate_mode;
    REF_mouse_middle = VT13_Info.Mouse.Press_M;// 【图传VT13】鼠标中键
    ui_rotate_mode = VT13_Info.RC.Left  ;      // 【图传VT13】左键(旋转模式开关)
    pause_last = pause;
    pause = VT13_Info.RC.Stop;                 // 【图传VT13】停止键(切零力)

    if (((REF_key_robot_mode & KEY_PRESSED_OFFSET_Z) > 0 && (REF_key_robot_mode_last & KEY_PRESSED_OFFSET_Z) == 0) || (pause == 1 && pause_last == 0))
    {
        UI_force_mode++;
        UI_force_mode = UI_force_mode % 2;
    }

		//改 
		//s[0]=1 小陀螺（大pitch展开 云台受控 底盘旋转）
		//s[0]=3 收缩模式（大pitch趴下 小pitch水平 yaw在限位范围移动 yaw要先转到轮组方向）
		//s[0]=2 无力模式 
		if(remote_ctrl.rc.s[0] ==1)
		{
			gimbal_mode_set->robot_mode_set = NORMAL_MODE;
			gimbal_mode_set->motor_mode[YAW] = ABSOLUTE_MODE;
			gimbal_mode_set->motor_mode[PITCH] = ABSOLUTE_MODE;
			gm6020_enable = 1; //大pitch立起来
			gm6020_target_angle = GM6020_EXTEND_ANGLE;
		}
		else if(remote_ctrl.rc.s[0] == 3)
		{
			gimbal_mode_set->robot_mode_set = NORMAL_MODE;
			gimbal_mode_set->motor_mode[PITCH] = ABSOLUTE_MODE;   // 小pitch保持水平
      gimbal_mode_set->motor_mode[YAW] = ABSOLUTE_MODE;     // yaw 正常遥控
			gm6020_enable = 1;
			gm6020_target_angle = GM6020_FOLD_ANGLE;
		}
		else if(remote_ctrl.rc.s[0] == 2)
		{
			gimbal_mode_set->robot_mode_set = ZERO_FORCE_MODE;
        gimbal_mode_set->motor_mode[PITCH] = RELATIVE_MODE;
        gimbal_mode_set->motor_mode[YAW] = RELATIVE_MODE;
        gm6020_enable = 0;                          // 大pitch松
		}
		
    if (((REF_key_robot_mode & KEY_PRESSED_OFFSET_SHIFT) > 0 && (REF_key_robot_mode_last & KEY_PRESSED_OFFSET_SHIFT) == 0)||(ui_rotate_mode == 1 && ui_rotate_mode_last == 0))
    {
        Key_ctrl_flag++;
        if (Key_ctrl_flag == 1)
        {
            vision_flag = 1;
            gimbal_mode_set->chassis_mode = CHASSIS_ROTATE_MODE;
        }
        else if (Key_ctrl_flag == 2)
        {
            gimbal_mode_set->chassis_mode = CHASSIS_FOLLOW_GIMBAL_MODE;
            Key_ctrl_flag = 0;
            vision_flag = 0;
        }
        else
        {
            vision_flag = 0;
            gimbal_mode_set->chassis_mode = CHASSIS_FOLLOW_GIMBAL_MODE;
        }
    }
		
    if ((REF_key_robot_mode & KEY_PRESSED_OFFSET_R) > 0 && (REF_key_robot_mode_last & KEY_PRESSED_OFFSET_R) == 0)
    {
        Key_r_flag++;
        if (Key_r_flag == 1)
        {
            gimbal_mode_set->chassis_mode = CHASSIS_NO_FOLLOW_GIMBAL_MODE;
        }
        else if (Key_r_flag == 2)
        {
            gimbal_mode_set->chassis_mode = CHASSIS_FOLLOW_GIMBAL_MODE;
            Key_r_flag = 0;
        }
        else
        {
					gimbal_mode_set->chassis_mode = CHASSIS_FOLLOW_GIMBAL_MODE;
        }
    }
		

    if ((REF_key_robot_mode & KEY_PRESSED_OFFSET_Q) > 0 && (REF_key_robot_mode_last & KEY_PRESSED_OFFSET_Q) == 0)
    {
        remote_ctrl.rc.s[0] = 0;
        remote_ctrl.rc.s[1] = 0;
    }
		
    /*【修改】自瞄触发条件：原只有「图传鼠标右键 REF_mouse_right / VT13 拨杆 Switch==2」，
     * 现新增 remote_ctrl.rc.s[1]==1（右上拨杆 1 档=0x01），拨到该档也进入自瞄 VISION_MODE。 */
    if (REF_mouse_right == 1||VT13_Info.RC.Switch==2 || remote_ctrl.rc.s[1] == 1)
    {
        gimbal_mode_set->motor_mode[YAW] = VISION_MODE;
        gimbal_mode_set->motor_mode[PITCH] = VISION_MODE;
        gimbal_mode_set->right_click = 1;
    }
    else
    {
        gimbal_mode_set->motor_mode[YAW] = ABSOLUTE_MODE;
        gimbal_mode_set->motor_mode[PITCH] = ABSOLUTE_MODE;
        gimbal_mode_set->right_click = 0;
    }
}


//改 底盘模式
//s[0]=1  小陀螺
//s[0]=3  云底
//s[0]=2  云底（无力）
static void ChassisModeSet(Gimbal_t* ch_mode_set)
{
    ch_mode_set->chassis_last_mode = ch_mode_set->chassis_mode;

    if (remote_ctrl.rc.s[0] == 1)
    {  
        ch_mode_set->chassis_mode = CHASSIS_ROTATE_MODE;  
    }
    else if (remote_ctrl.rc.s[0] == 3)
    {
        ch_mode_set->chassis_mode = CHASSIS_FOLLOW_GIMBAL_MODE;
    }
    else if (remote_ctrl.rc.s[0] == 2)   
    {
        ch_mode_set->chassis_mode = CHASSIS_FOLLOW_GIMBAL_MODE;  //无力模式会直接在ChassisSpeedSet中清0
    }
}

/**
 * @brief 模式切换时的角度「无缝过渡」
 * @note  切换模式的瞬间，把「目标角度」直接设成「当前反馈角度」，
 *        避免设定值和反馈值差一大截导致电机猛地甩一下（跳变）。
 *        例如从相对模式切到绝对模式，就把 absolute_set 赋成当前 absolute_fdb。
 */
static void GimbalModeChangeControlTransit(Gimbal_t* mode_change)
{

    if (mode_change->motor_last_mode[PITCH] != RELATIVE_MODE && mode_change->motor_mode[PITCH] == RELATIVE_MODE)
    {
        mode_change->angle.relative_set[PITCH] = mode_change->angle.relative_fdb[PITCH];
    }
    else if (mode_change->motor_last_mode[PITCH] != ABSOLUTE_MODE && mode_change->motor_mode[PITCH] == ABSOLUTE_MODE)
    {
        mode_change->angle.absolute_set[PITCH] = mode_change->angle.absolute_fdb[PITCH];
    }
    else if (mode_change->motor_last_mode[PITCH] != VISION_MODE && mode_change->motor_mode[PITCH] == VISION_MODE)
    {
        mode_change->vision_data.angle_set[PITCH] = 0;
    }


    if (mode_change->motor_last_mode[YAW] != RELATIVE_MODE && mode_change->motor_mode[YAW] == RELATIVE_MODE)
    {
        mode_change->angle.relative_set[YAW] = mode_change->angle.relative_fdb[YAW];
    }
    else if (mode_change->motor_last_mode[YAW] != ABSOLUTE_MODE && mode_change->motor_mode[YAW] == ABSOLUTE_MODE)
    {
        mode_change->angle.absolute_set[YAW] = mode_change->angle.absolute_fdb[YAW];
    }
    else if (mode_change->motor_last_mode[YAW] != VISION_MODE && mode_change->motor_mode[YAW] == VISION_MODE)
    {
        mode_change->vision_data.angle_set[YAW] = 0;
    }


    if (mode_change->robot_last_mode == ZERO_FORCE_MODE && mode_change->robot_mode_set != ZERO_FORCE_MODE)
    {
        mode_change->angle.relative_set[PITCH] = mode_change->angle.relative_fdb[PITCH];
        mode_change->angle.relative_set[YAW] = mode_change->angle.relative_fdb[YAW];

        mode_change->angle.absolute_set[PITCH] = mode_change->angle.absolute_fdb[PITCH];
        mode_change->angle.absolute_set[YAW] = mode_change->angle.absolute_fdb[YAW];
    }
}

/**
 * @brief 计算俯仰轴(pitch)的目标角度
 * @note  普通模式：把遥控器 ch[1] 摇杆 + 鼠标 Y 轴 的输入累加成角度增量。
 *        CHANNEL_PITCH_SCALE_FACTOR 是摇杆→角度的比例系数（越大角度动得越慢）。
 *        累加完会做限幅（见 addPitchAngle），防止俯仰打到结构极限。
 */
static void PitchAngleRefSet(Gimbal_t* angle_set)
{
    float add_angle = 0;
    if (angle_set->robot_mode_set == ZERO_FORCE_MODE)
    {
        pitch_ref_vel = 0.0f;   //【新增】无力模式无指令，参考速度清零
        return;
    }

    else if (angle_set->robot_mode_set == NORMAL_MODE)
    {   
			  //新增
				if(remote_ctrl.rc.s[0] == 3)
				{
					angle_set->angle.absolute_set[PITCH] = PITCH_LEVEL_ANGLE;
					pitch_ref_vel = 0.0f;
				}
				else
				{
        // 俯仰目标角增量 = 摇杆输入 之和：【DT7】ch[1](右摇杆上下) + 【DT7】mouse.y(鼠标Y)
        //                               +【图传VT13】Channel[1] +【图传VT13】Mouse.
        add_angle = -(float)remote_ctrl.rc.ch[1] / CHANNEL_PITCH_SCALE_FACTOR - (float)VT13_Info.RC.Channel[1] / CHANNEL_PITCH_SCALE_FACTOR - (float)remote_ctrl.mouse.y / MOUSE_SCALE_FACTOR - VT13_Info.Mouse.Y / MOUSE_SCALE_FACTOR;
        addPitchAngle(angle_set, -add_angle);

        //【新增】速度前馈：把本周期角度增量换算成参考角速度(°/s)。
        //        addPitchAngle 传的是 -add_angle，所以实际增量是 -add_angle，参考速度也跟着取反。
        pitch_ref_vel = -add_angle / GIMBAL_CONTROL_PERIOD_S;
			  }
    }
    else if (angle_set->robot_mode_set == VISION_FOLLOW)
    {
        angle_set->vision_data.angle_set[PITCH] = 0;
        pitch_ref_vel = 0.0f;   //【新增】视觉模式走 vision_target_vel 前馈，手动前馈关闭
    }
}

/**
 * @brief 计算偏航轴(yaw)的目标角度
 * @note  普通模式：把遥控器 ch[0] 摇杆 + 鼠标 X 轴 的输入累加成角度增量。
 *        和 pitch 一样，累加完做限幅/环绕处理（yaw 是 ±180° 环绕的）。
 */
static void YawAngleRefSet(Gimbal_t* angle_set)
{
    float add_angle;

    if (angle_set->robot_mode_set == ZERO_FORCE_MODE)
    {
        yaw_ref_vel = 0.0f;   //【新增】无力模式无指令，参考速度清零
        return;
    }

    else if (angle_set->robot_mode_set == NORMAL_MODE)
    {
				//调试发现将 VT13_Info.RC.Channel[0]改个正负 左推摇杆 车（云台+底盘）右转
        // 偏航目标角增量 = 摇杆输入 之和：【DT7】ch[0](右摇杆左右) +【图传VT13】Wheel(滚轮)
        //                              +【图传VT13】Channel[0] +【图传VT13】Mouse.X
        // 想只用 DT7：删掉下面带 VT13_Info 的几项即可
        add_angle = -(float)remote_ctrl.rc.ch[0] / (float)CHANNEL_YAW_SCALE_FACTOR - (float)VT13_Info.RC.Wheel / CHANNEL_YAW_SCALE_FACTOR - (VT13_Info.RC.Channel[0]) / CHANNEL_YAW_SCALE_FACTOR - VT13_Info.Mouse.X /  MOUSE_SCALE_FACTOR;

        addYawAngle(angle_set, add_angle);
				
				
        //【新增】速度前馈：把本周期角度增量换算成参考角速度(°/s)。
        //        增量(°/周期) ÷ 周期(s) = 角速度(°/s)，供 pidCalc 前馈到内环速度设定。
        yaw_ref_vel = add_angle / GIMBAL_CONTROL_PERIOD_S;
    }
    else if (angle_set->robot_mode_set == VISION_FOLLOW)
    {
        angle_set->vision_data.angle_set[YAW] = 0;
        yaw_ref_vel = 0.0f;   //【新增】视觉模式走 vision_target_vel 前馈，手动前馈关闭
		}
}
float max_step = 0.02;
/**
 * @brief 计算底盘速度指令（通过 CAN 0x21 帧发给底盘）
 * @note  底盘是麦克纳姆轮全向底盘，指令是三个分量：
 *         - CHASSIS_X / CHASSIS_Y：平面平移速度（前后/左右，单位 m/s 量级）
 *         - CHASSIS_Z：绕自身旋转角速度
 *        关键逻辑：
 *        1. 摇杆/键鼠输入先经 ramp_speed 平滑（避免突变）
 *        2. 按云台 yaw 相对角度做坐标旋转，把「以云台为参考的前后左右」
 *           换算成「以底盘为参考的前后左右」
 *        3. 车头/车尾(head_flag)切换：yaw 转过 ±90° 时车头车尾对调，
 *           底盘旋转方向要反过来（这里用 follow_pid 算旋转量来纠正）
 *        4. 视觉模式时旋转量置 0（自瞄时底盘不转，只有平移）
 */
static void ChassisSpeedSet(Gimbal_t* chassis_speed)
{
    // ===== 静态/局部变量说明 =====
    // set_chassis_head / set_chassis_tail：车头/车尾跟随的「死区」阈值(单位:度)
    //  - 车头模式：yaw 相对角在 ±0.5° 内就算对准，不再给旋转量
    //  - 车尾模式：yaw 相对角贴近 ±180°(偏差 0.5° 内)就算对准
    static float set_chassis_head = 0.5, set_chassis_tail = 179.5;
    float angle_set = 0.0f;   // 本函数内未再使用，历史遗留(可删)
	
    // x_temp/y_temp 必须是 static：ramp_speed 需要「上一次的值」来算平滑增量
    static float x_temp, y_temp;
	
    // 无力模式 / 初始化模式：底盘静止，三轴速度清零后直接返回
    if (chassis_speed->robot_mode_set == ZERO_FORCE_MODE)
    {
        chassis_speed->chassis_set[CHASSIS_X] = 0;
        chassis_speed->chassis_set[CHASSIS_Y] = 0;
        chassis_speed->chassis_set[CHASSIS_Z] = 0;
        return;
    }

    // ============ 模式 1：云台底盘跟随 ============
    // 特点：底盘朝向会自动跟着云台 yaw 转，始终保持车头与云台一致。
    // 内部又分两种子情况(见下面的 if)：
    //  - 云台不在自瞄(手动)：平移 + 用 follow_pid 让底盘跟着云台转回去
    //  - 云台在自瞄(VISION_MODE)：只平移、不旋转，旋转交给云台自己
    if (chassis_speed->chassis_mode == CHASSIS_FOLLOW_GIMBAL_MODE)
    {
			  
        // 云台不在自瞄(至少一个轴不是 VISION_MODE)：手动操作，底盘「平移 + 跟随旋转」都做
        if (chassis_speed->motor_mode[PITCH] != VISION_MODE || chassis_speed->motor_mode[YAW] != VISION_MODE)
        {
            // 1) 读前后/左右输入并平滑：x_temp=左右、y_temp=前后。
            //    摇杆：【DT7】remote_ctrl.rc.ch[2]/ch[3] 与【图传VT13】Channel[3]/[2] 两套叠加 + 键盘(W/A/S/D)，
            //    ramp_speed 用上一次的值做增量限幅(max_step)，避免速度突变。
            // 2) 下方 sin/cos 是把「云台坐标系」的速度旋转成「底盘坐标系」。
            // 3) 再按 head_flag 用车头/车尾 PID 补出旋转量(Z)，让车头跟上云台。
            x_temp = ramp_speed(x_temp, Int16ToFloat(remote_ctrl.rc.ch[2]) * REMOTE_CHANNLE_TO_CHASSIS_SPEED  + VT13_Info.RC.Channel[3] * REMOTE_CHANNLE_TO_CHASSIS_SPEED - ((remote_ctrl.key.v & KEY_PRESSED_OFFSET_A) > 0) * KEY_SCALE_FACTOR + ((remote_ctrl.key.v & KEY_PRESSED_OFFSET_D) > 0) * KEY_SCALE_FACTOR -
                                ((VT13_Info.Key.V & KEY_PRESSED_OFFSET_A) > 0) * key_scale_factor  + ((VT13_Info.Key.V & KEY_PRESSED_OFFSET_D) > 0) * key_scale_factor, max_step);

            y_temp = ramp_speed(y_temp, Int16ToFloat(remote_ctrl.rc.ch[3]) * REMOTE_CHANNLE_TO_CHASSIS_SPEED + VT13_Info.RC.Channel[2] * REMOTE_CHANNLE_TO_CHASSIS_SPEED + ((remote_ctrl.key.v & KEY_PRESSED_OFFSET_W) > 0) * KEY_SCALE_FACTOR - ((remote_ctrl.key.v & KEY_PRESSED_OFFSET_S) > 0) * KEY_SCALE_FACTOR +
                                ((VT13_Info.Key.V & KEY_PRESSED_OFFSET_W) > 0) * key_scale_factor  - ((VT13_Info.Key.V & KEY_PRESSED_OFFSET_S) > 0) * key_scale_factor, max_step);

            chassis_speed->chassis_set[CHASSIS_X] = (y_temp * sin(-chassis_speed->angle.relative_fdb[YAW] * ANGLE_TO_RAD) + x_temp * cos(-chassis_speed->angle.relative_fdb[YAW] * ANGLE_TO_RAD));
            chassis_speed->chassis_set[CHASSIS_Y] = (y_temp * cos(-chassis_speed->angle.relative_fdb[YAW] * ANGLE_TO_RAD) - x_temp * sin(-chassis_speed->angle.relative_fdb[YAW] * ANGLE_TO_RAD));


            // ===== 车头模式：让底盘转到「车头正对云台」(yaw 相对角 → 0°) =====
            // 技巧：先直接把 yaw 反馈值以每帧 0.5° 步长往 0 拉(软回中)，
            //       等效于把 PID 看到的误差平滑减小，避免突然给大旋转速度。
            // 注意：下方 if/else 两个分支其实完全一样(历史遗留)，效果就是「恒做 PID 回中」。
            if (head_flag == HEAD)
            {
                if (chassis_speed->angle.relative_fdb[YAW] > 0)
                {
                    if (chassis_speed->angle.relative_fdb[YAW] - 0.5 > 0)chassis_speed->angle.relative_fdb[YAW] -= 0.5;
                    else chassis_speed->angle.relative_fdb[YAW] = 0;
                }
                else
                {
                    if (chassis_speed->angle.relative_fdb[YAW] + 0.5 < 0)chassis_speed->angle.relative_fdb[YAW] += 0.5;
                    else chassis_speed->angle.relative_fdb[YAW] = 0;
                }
                if ((chassis_speed->angle.relative_fdb[YAW] < set_chassis_head) && (chassis_speed->angle.relative_fdb[YAW] > -set_chassis_head))
                    chassis_speed->chassis_set[CHASSIS_Z] = PID_Calculate(&follow_pid, 0, chassis_speed->angle.relative_fdb[YAW]); //-6.4
                else
                    chassis_speed->chassis_set[CHASSIS_Z] = PID_Calculate(&follow_pid, 0, chassis_speed->angle.relative_fdb[YAW]);
            }
            // ===== 车尾模式：云台相对底盘转了约 180°，对准方向是 ±180° 而非 0° =====
            // 先把 yaw 反馈以每帧 0.5° 步长往 ±180° 拉(软回中)，
            // 已经贴近 ±180°(死区内)则不再转；否则按所在半区往 +180° 或 -180° 方向转回去。
            else if (head_flag == TAIL)
            {
                if (chassis_speed->angle.relative_fdb[YAW] - 180 > 0)
                {
                    if (chassis_speed->angle.relative_fdb[YAW] - 179.5 < 0)chassis_speed->angle.relative_fdb[YAW] += 0.5;
                    else chassis_speed->angle.relative_fdb[YAW] = 180;
                }
                else
                {
                    if (chassis_speed->angle.relative_fdb[YAW] + 179.5 > 0)chassis_speed->angle.relative_fdb[YAW] -= 0.5;
                    else chassis_speed->angle.relative_fdb[YAW] = -180;
                }
                if (((chassis_speed->angle.relative_fdb[YAW] > set_chassis_tail) && (chassis_speed->angle.relative_fdb[YAW] < 180)) ||
                        ((chassis_speed->angle.relative_fdb[YAW] < -set_chassis_tail) && (chassis_speed->angle.relative_fdb[YAW] > -180)))
                    chassis_speed->chassis_set[CHASSIS_Z] = 0;
                else if (chassis_speed->angle.relative_fdb[YAW] > 90)
                    chassis_speed->chassis_set[CHASSIS_Z] =  PID_Calculate(&follow_pid, 180, chassis_speed->angle.relative_fdb[YAW]);
                else if (chassis_speed->angle.relative_fdb[YAW] < -90)
                    chassis_speed->chassis_set[CHASSIS_Z] =  PID_Calculate(&follow_pid, -180, chassis_speed->angle.relative_fdb[YAW]);
            }
        }
        else
        {
            // ===== 自瞄中(VISION_MODE)：底盘只平移、不旋转(Z 恒为 0) =====
            // 云台自己转去追目标，底盘只管前后左右走；下方注释掉的旧代码(直接当速度、不做旋转)已废弃。
//           chassis_speed->chassis_set[CHASSIS_X] = ramp_speed(chassis_speed->chassis_set[CHASSIS_X], Int16ToFloat(remote_ctrl.rc.ch[2]) * REMOTE_CHANNLE_TO_CHASSIS_SPEED  + VT13_Info.RC.Channel[3] * REMOTE_CHANNLE_TO_CHASSIS_SPEED - ((remote_ctrl.key.v & KEY_PRESSED_OFFSET_A) > 0) * KEY_SCALE_FACTOR + ((remote_ctrl.key.v & KEY_PRESSED_OFFSET_D) > 0) * KEY_SCALE_FACTOR -
//                                ((VT13_Info.Key.V & KEY_PRESSED_OFFSET_A) > 0) * key_scale_factor  + ((VT13_Info.Key.V & KEY_PRESSED_OFFSET_D) > 0) * key_scale_factor, max_step);

//           chassis_speed->chassis_set[CHASSIS_Y] = ramp_speed(chassis_speed->chassis_set[CHASSIS_Y], Int16ToFloat(remote_ctrl.rc.ch[3]) * REMOTE_CHANNLE_TO_CHASSIS_SPEED + VT13_Info.RC.Channel[2] * REMOTE_CHANNLE_TO_CHASSIS_SPEED + ((remote_ctrl.key.v & KEY_PRESSED_OFFSET_W) > 0) * KEY_SCALE_FACTOR - ((remote_ctrl.key.v & KEY_PRESSED_OFFSET_S) > 0) * KEY_SCALE_FACTOR +
//                                ((VT13_Info.Key.V & KEY_PRESSED_OFFSET_W) > 0) * key_scale_factor  - ((VT13_Info.Key.V & KEY_PRESSED_OFFSET_S) > 0) * key_scale_factor, max_step);            
//					chassis_speed->chassis_set[CHASSIS_Z] = 0;
		x_temp = ramp_speed(x_temp,
							Int16ToFloat(remote_ctrl.rc.ch[2]) * REMOTE_CHANNLE_TO_CHASSIS_SPEED
							+ VT13_Info.RC.Channel[3] * REMOTE_CHANNLE_TO_CHASSIS_SPEED
							- ((remote_ctrl.key.v & KEY_PRESSED_OFFSET_A) > 0) * KEY_SCALE_FACTOR
							+ ((remote_ctrl.key.v & KEY_PRESSED_OFFSET_D) > 0) * KEY_SCALE_FACTOR
							- ((VT13_Info.Key.V & KEY_PRESSED_OFFSET_A) > 0) * key_scale_factor
							+ ((VT13_Info.Key.V & KEY_PRESSED_OFFSET_D) > 0) * key_scale_factor,
							max_step);

		y_temp = ramp_speed(y_temp,
							Int16ToFloat(remote_ctrl.rc.ch[3]) * REMOTE_CHANNLE_TO_CHASSIS_SPEED
							+ VT13_Info.RC.Channel[2] * REMOTE_CHANNLE_TO_CHASSIS_SPEED
							+ ((remote_ctrl.key.v & KEY_PRESSED_OFFSET_W) > 0) * KEY_SCALE_FACTOR
							- ((remote_ctrl.key.v & KEY_PRESSED_OFFSET_S) > 0) * KEY_SCALE_FACTOR
							+ ((VT13_Info.Key.V & KEY_PRESSED_OFFSET_W) > 0) * key_scale_factor
							- ((VT13_Info.Key.V & KEY_PRESSED_OFFSET_S) > 0) * key_scale_factor,
							max_step);
////////////////////////////////////////////////////////////////////////调试准备（把它改成负号看看）
    // 2. 按云台YAW相对角度做坐标旋转（与非视觉模式一致）
		chassis_speed->chassis_set[CHASSIS_X] =
						(y_temp * sin(-chassis_speed->angle.relative_fdb[YAW] * ANGLE_TO_RAD)
						+ x_temp * cos(-chassis_speed->angle.relative_fdb[YAW] * ANGLE_TO_RAD));

		chassis_speed->chassis_set[CHASSIS_Y] =
						(y_temp * cos(-chassis_speed->angle.relative_fdb[YAW] * ANGLE_TO_RAD)
						- x_temp * sin(-chassis_speed->angle.relative_fdb[YAW] * ANGLE_TO_RAD));	
		chassis_speed->chassis_set[CHASSIS_Z] = 0;

        }
    }
    // ============ 模式 2：云台底盘不跟随 ============
    // 特点：云台和底盘彻底解耦，云台怎么转底盘都不跟着转。
    // 平移(前后左右)照常用摇杆+WASD；旋转(Z)直接用摇杆 ch[0] + 鼠标 X 控制(-0.05 是灵敏度)。
    else if (chassis_speed->chassis_mode == CHASSIS_NO_FOLLOW_GIMBAL_MODE)
    {
           chassis_speed->chassis_set[CHASSIS_X] = ramp_speed(chassis_speed->chassis_set[CHASSIS_X], Int16ToFloat(remote_ctrl.rc.ch[2]) * REMOTE_CHANNLE_TO_CHASSIS_SPEED  + VT13_Info.RC.Channel[3] * REMOTE_CHANNLE_TO_CHASSIS_SPEED - ((remote_ctrl.key.v & KEY_PRESSED_OFFSET_A) > 0) * KEY_SCALE_FACTOR + ((remote_ctrl.key.v & KEY_PRESSED_OFFSET_D) > 0) * KEY_SCALE_FACTOR -
                                ((VT13_Info.Key.V & KEY_PRESSED_OFFSET_A) > 0) * key_scale_factor  + ((VT13_Info.Key.V & KEY_PRESSED_OFFSET_D) > 0) * key_scale_factor, max_step);

           chassis_speed->chassis_set[CHASSIS_Y] = ramp_speed(chassis_speed->chassis_set[CHASSIS_Y], Int16ToFloat(remote_ctrl.rc.ch[3]) * REMOTE_CHANNLE_TO_CHASSIS_SPEED + VT13_Info.RC.Channel[2] * REMOTE_CHANNLE_TO_CHASSIS_SPEED + ((remote_ctrl.key.v & KEY_PRESSED_OFFSET_W) > 0) * KEY_SCALE_FACTOR - ((remote_ctrl.key.v & KEY_PRESSED_OFFSET_S) > 0) * KEY_SCALE_FACTOR +
                                ((VT13_Info.Key.V & KEY_PRESSED_OFFSET_W) > 0) * key_scale_factor  - ((VT13_Info.Key.V & KEY_PRESSED_OFFSET_S) > 0) * key_scale_factor, max_step);            
					 chassis_speed->chassis_set[CHASSIS_Z] = -0.05*(Int16ToFloat(remote_ctrl.rc.ch[0]) * REMOTE_CHANNLE_TO_CHASSIS_SPEED + VT13_Info.RC.Channel[0] * REMOTE_CHANNLE_TO_CHASSIS_SPEED + remote_ctrl.mouse.x*100 / MOUSE_SCALE_FACTOR * KEY_SCALE_FACTOR + VT13_Info.Mouse.X*100 / MOUSE_SCALE_FACTOR);

    }
    // ============ 模式 3：底盘旋转(小陀螺) ============
    // 特点：底盘一边平移一边绕自身旋转。旋转量 Z 用一个随平移速度衰减的式子：
    //   Z = -3 * exp(-1.1 * sqrt(X²+Y²))，
    //   即「静止时转最快(≈-3)，平移越快转得越慢」，避免转着转着把平移方向带飞。
    else if (chassis_speed->chassis_mode == CHASSIS_ROTATE_MODE)
    {
           x_temp = ramp_speed(x_temp, Int16ToFloat(remote_ctrl.rc.ch[2]) * REMOTE_CHANNLE_TO_CHASSIS_SPEED  + VT13_Info.RC.Channel[3] * REMOTE_CHANNLE_TO_CHASSIS_SPEED - ((remote_ctrl.key.v & KEY_PRESSED_OFFSET_A) > 0) * KEY_SCALE_FACTOR + ((remote_ctrl.key.v & KEY_PRESSED_OFFSET_D) > 0) * KEY_SCALE_FACTOR -
                                ((VT13_Info.Key.V & KEY_PRESSED_OFFSET_A) > 0) * key_scale_factor  + ((VT13_Info.Key.V & KEY_PRESSED_OFFSET_D) > 0) * key_scale_factor, max_step);

            y_temp = ramp_speed(y_temp, Int16ToFloat(remote_ctrl.rc.ch[3]) * REMOTE_CHANNLE_TO_CHASSIS_SPEED + VT13_Info.RC.Channel[2] * REMOTE_CHANNLE_TO_CHASSIS_SPEED + ((remote_ctrl.key.v & KEY_PRESSED_OFFSET_W) > 0) * KEY_SCALE_FACTOR - ((remote_ctrl.key.v & KEY_PRESSED_OFFSET_S) > 0) * KEY_SCALE_FACTOR +
                                ((VT13_Info.Key.V & KEY_PRESSED_OFFSET_W) > 0) * key_scale_factor  - ((VT13_Info.Key.V & KEY_PRESSED_OFFSET_S) > 0) * key_scale_factor, max_step);
        chassis_speed->chassis_set[CHASSIS_X] = (y_temp * sin(-chassis_speed->angle.relative_fdb[YAW] * ANGLE_TO_RAD) + x_temp * cos(-chassis_speed->angle.relative_fdb[YAW] * ANGLE_TO_RAD));
        chassis_speed->chassis_set[CHASSIS_Y] = (y_temp * cos(-chassis_speed->angle.relative_fdb[YAW] * ANGLE_TO_RAD) - x_temp * sin(-chassis_speed->angle.relative_fdb[YAW] * ANGLE_TO_RAD));
        chassis_speed->chassis_set[CHASSIS_Z] = -4.5 * expf(-1.1f * sqrtf(chassis_speed->chassis_set[CHASSIS_X] * chassis_speed->chassis_set[CHASSIS_X] + chassis_speed->chassis_set[CHASSIS_Y] * chassis_speed->chassis_set[CHASSIS_Y]));  //-3
//        chassis_speed->chassis_set[CHASSIS_Z]=-2;  
    }



}
/**
 * @brief 俯仰轴 PID 计算（串级 PID 入口）
 * @note  零力模式直接力矩置 0；其余模式统一走 pidCalc() 做串级 PID。
 *        实际的控制算法都在 pidCalc 里，这里只是按模式分发。
 */
static void PitchPidCalc(Gimbal_t* pitch_pid_calc)
{

    if (pitch_pid_calc->robot_mode_set == ZERO_FORCE_MODE)
    {
        pitch_pid_calc->motor_set[PITCH].Torque = 0;
    }

    else if (pitch_pid_calc->robot_mode_set == NORMAL_MODE)
    {
        pidCalc(pitch_pid_calc, PITCH);
    }
    else if (pitch_pid_calc->robot_mode_set == VISION_FOLLOW)
    {
        pidCalc(pitch_pid_calc, PITCH);
    }
}
/**
 * @brief 偏航轴 PID 计算（串级 PID 入口）
 * @note  和 PitchPidCalc 一样，按模式分发到 pidCalc()。
 */
static void YawPidCalc(Gimbal_t* yaw_pid_calc)
{


    if (yaw_pid_calc->robot_mode_set == ZERO_FORCE_MODE)
    {
        yaw_pid_calc->motor_set[YAW].Torque = 0;
    }

    else if (yaw_pid_calc->robot_mode_set == NORMAL_MODE)
    {
        pidCalc(yaw_pid_calc, YAW);
    }
    else if (yaw_pid_calc->robot_mode_set == VISION_FOLLOW)
    {
        pidCalc(yaw_pid_calc, YAW);
    }
}


/**
 * @brief 给俯仰轴目标角度累加一个增量，并做限幅
 * @note  绝对模式累加到 absolute_set（限幅 PITCH_ABSOLUTE_MIN/MAX），
 *        相对模式累加到 relative_set（限幅 PITCH_RELATIVE_MIN/MAX）。
 *        限幅范围定义在 Control_Task.h，调车改那里。
 */
static void addPitchAngle(Gimbal_t* pitch, float add)
{
    if (pitch->motor_mode[PITCH] == ABSOLUTE_MODE)
    {
        pitch->angle.absolute_set[PITCH] += add;
        MinMaxConstrain(&pitch->angle.absolute_set[PITCH], PITCH_ABSOLUTE_MIN_ANGLE, PITCH_ABSOLUTE_MAX_ANGLE);
    }  
    else if (pitch->motor_mode[PITCH] == RELATIVE_MODE)
    {
        pitch->angle.relative_set[PITCH] += add;
        MinMaxConstrain(&pitch->angle.relative_set[PITCH], PITCH_RELATIVE_MIN_ANGLE, PITCH_RELATIVE_MAX_ANGLE);
    }
}

/**
 * @brief 给偏航轴目标角度累加一个增量，并做限幅/环绕
 * @note  绝对模式累加到 absolute_set（用 ConstrainLoop 做 ±180° 环绕），
 *        相对模式累加到 relative_set（限幅 YAW_RELATIVE_MIN/MAX）。
 */
static void addYawAngle(Gimbal_t* yaw, float add)
{
    if (yaw->motor_mode[YAW] == ABSOLUTE_MODE)
    {
        yaw->angle.absolute_set[YAW] += add;
        ConstrainLoop(&(yaw->angle.absolute_set[YAW]), yaw->angle.absolute_set[YAW], ANGLE_MIN, ANGLE_MAX);
    }
    else if (yaw->motor_mode[YAW] == RELATIVE_MODE)
    {
        yaw->angle.relative_set[YAW] += add;
        MinMaxConstrain(&yaw->angle.relative_set[YAW], YAW_RELATIVE_MIN_ANGLE, YAW_RELATIVE_MAX_ANGLE);
    }
}
float Pitch_set = 0, Yaw_set = 0;

/**
 * @brief 串级 PID 核心计算（pitch/yaw 通用，axis 区分）
 * @note  双环串级：外环角度环(gimbal_shell_pid/vision_shell_pid)算出目标角速度，
 *        内环角速度环(gimbal_core_pid/vision_core_pid)算出最终力矩。
 *        三种电机模式：
 *         - VISION_MODE  视觉自瞄：目标角度 = 视觉偏移 + 手动设定，走 vision_* PID
 *         - ABSOLUTE_MODE 绝对模式：以 IMU 绝对角度为反馈，走 gimbal_* PID
 *         - RELATIVE_MODE 相对模式：以电机相对角度为反馈，走 gimbal_* PID
 *        视觉模式额外做了：超时保护(500ms 丢视觉自动退到绝对模式)、
 *        速度前馈(vision_target_vel)、旋转靶预测补偿。
 */
static void pidCalc(Gimbal_t* pid_calc, Axis_e axis)
{
    
    
    float angle_set = 0.0f;
    uint32_t current_time = HAL_GetTick();
    uint32_t data_timeout = current_time - vision_last_valid_time[axis];
		
    // ================== 1. 视觉模式(自瞄) ==================
    // 「云台怎么跟上目标」的核心就在这：
    //   妙算发来目标应指向的角度 vision_x/y_offect(度) → 加上手动修正 Yaw/Pitch_set
    //   → 得到目标绝对角度 → 外环角度环PID(算出目标角速度) → 内环角速度环PID(算出力矩)
    //   → 力矩发给达妙电机，云台就转过去对准目标。
    //   视觉把 offset 算得越准、PID 越跟手，枪口就越稳地咬住目标。
    if (pid_calc->motor_mode[axis] == VISION_MODE)
    {
        if (data_timeout > VISION_DATA_TIMEOUT_MS)
        {
            pid_calc->motor_mode[axis] = ABSOLUTE_MODE;
            pid_calc->angle.absolute_set[axis] = pid_calc->angle.absolute_fdb[axis];
            vision_target_vel[axis] = 0;   // 超时清零速度估计
        }
        else
        {
            // --- 视觉目标速度获取（已在Vision_Info_Update中以视觉帧率从原始数据计算）---
            vision_target_vel[axis] = LowPassFilter1p_Update(&vision_vel_lpf[axis],
                (axis == YAW) ? pid_calc->vision_data.vision_vel_x
                              : pid_calc->vision_data.vision_vel_y);
            // -----------------------------------------

            if (axis == YAW)
            {
                // 预测补偿：旋转靶外推视觉偏移到当前时刻
                float yaw_offset = pid_calc->vision_data.vision_x_offect;  
							if (Vision_Data.target_type != 0)   //非0为旋转靶   若为旋转靶则要预测
                {
                    yaw_offset += vision_target_vel[YAW] * VISION_PREDICT_TIME_S;
                }
                float target_yaw_abs = yaw_offset + Yaw_set;                             //当前位置+偏移量=目标位置

                // 防止跨越半圈
                target_yaw_abs = PreventFullCircleConstrain(target_yaw_abs, pid_calc->angle.absolute_fdb[axis]);

								
                // 外环PID + 速度前馈 → 内环角速度设定  
            // if(Vision_Data.target_type==0){
            // pid_calc->gyro.set[axis] = PID_Calculate(&vision_shell_pid[axis], target_yaw_abs, pid_calc->angle.absolute_fdb[axis])
            //                          + vision_target_vel[YAW] * VISION_VEL_FF_GAIN;}
             //else{
                pid_calc->gyro.set[axis] = PID_Calculate(&vision_shell_pid[axis], target_yaw_abs, pid_calc->angle.absolute_fdb[axis]);
             //}

                pid_calc->motor_set[axis].Torque = PID_Calculate(&vision_core_pid[axis], pid_calc->gyro.set[axis], pid_calc->gyro.fdb[axis]);
            }
            else if (axis == PITCH)
            {
                // 预测补偿：旋转靶外推视觉偏移到当前时刻
                float pitch_offset = pid_calc->vision_data.vision_y_offect;
                if (Vision_Data.target_type != 0)
                {
                    pitch_offset += vision_target_vel[PITCH] * VISION_PREDICT_TIME_S;
                }
                float target_pitch_abs = pitch_offset + Pitch_set;

                // 外环PID + 速度前馈 → 内环角速度设定
                pid_calc->gyro.set[axis] = PID_Calculate(&vision_shell_pid[axis], -target_pitch_abs, pid_calc->angle.absolute_fdb[axis]);

                pid_calc->motor_set[axis].Torque = PID_Calculate(&vision_core_pid[axis], pid_calc->gyro.set[axis], pid_calc->gyro.fdb[axis]);
            }
        }
    }

    
    if (pid_calc->motor_mode[axis] == ABSOLUTE_MODE)
    {
        if (axis == PITCH)
        {
            angle_set = PreventFullCircleConstrain(pid_calc->angle.absolute_set[axis], pid_calc->angle.absolute_fdb[axis]);
            //【新增】速度前馈：角度环输出 + 参考角速度×增益，直接喂给内环速度设定
            pid_calc->gyro.set[axis] = PID_Calculate(&gimbal_shell_pid[axis], angle_set, pid_calc->angle.absolute_fdb[axis])
                                     + pitch_vel_ff_gain * pitch_ref_vel;
            pid_calc->motor_set[axis].Torque = PID_Calculate(&gimbal_core_pid[axis], pid_calc->gyro.set[axis], pid_calc->gyro.fdb[axis]);
        }
        else if (axis == YAW)
        {
            if (pid_calc->chassis_mode == CHASSIS_NO_FOLLOW_GIMBAL_MODE)
            {
                pid_calc->gyro.set[axis] = PID_Calculate(&gimbal_shell_pid[axis], 0, pid_calc->angle.relative_fdb[axis]);
            }
            else
            {
                angle_set = PreventFullCircleConstrain(pid_calc->angle.absolute_set[axis], pid_calc->angle.absolute_fdb[axis]);
                //【新增】速度前馈：角度环输出 + 参考角速度×增益，直接喂给内环速度设定。
                //        摇杆一动，内环立刻拿到目标速度，不用等角度误差慢慢累积，响应更快。
                pid_calc->gyro.set[axis] = PID_Calculate(&gimbal_shell_pid[axis], angle_set, pid_calc->angle.absolute_fdb[axis])
                                         + yaw_vel_ff_gain * yaw_ref_vel;
            }
            pid_calc->motor_set[axis].Torque = PID_Calculate(&gimbal_core_pid[axis], pid_calc->gyro.set[axis], pid_calc->gyro.fdb[axis]);
        }
    }

    
    else if (pid_calc->motor_mode[axis] == RELATIVE_MODE)
    {
        if (axis == PITCH)
        {
            angle_set = PreventFullCircleConstrain(pid_calc->angle.relative_set[axis], pid_calc->angle.relative_fdb[axis]);
            //【新增】速度前馈：同绝对模式，角度环输出 + 参考角速度×增益喂给内环速度设定
            pid_calc->gyro.set[axis] = PID_Calculate(&gimbal_shell_pid[axis], angle_set, pid_calc->angle.relative_fdb[axis])
                                     + pitch_vel_ff_gain * pitch_ref_vel;
            pid_calc->motor_set[axis].Torque = PID_Calculate(&gimbal_core_pid[axis], pid_calc->gyro.set[axis], pid_calc->gyro.fdb[axis]);
        }
        else if (axis == YAW)
        {
            angle_set = PreventFullCircleConstrain(pid_calc->angle.relative_set[axis], pid_calc->angle.relative_fdb[axis]);
            //【新增】速度前馈：同绝对模式，角度环输出 + 参考角速度×增益喂给内环速度设定
            pid_calc->gyro.set[axis] = PID_Calculate(&gimbal_shell_pid[axis], angle_set, pid_calc->angle.relative_fdb[axis])
                                     + yaw_vel_ff_gain * yaw_ref_vel;
            pid_calc->motor_set[axis].Torque = PID_Calculate(&gimbal_core_pid[axis], pid_calc->gyro.set[axis], pid_calc->gyro.fdb[axis]);
        }
    }
}

/**
 * @brief GM6020 双环 PID（位置环 + 速度环），算出电压
 * @note  外环 位置环：角度误差(°) → 目标转速(rpm)
 *        内环 速度环：转速误差(rpm) → 电压
 *        gm6020_enable=0 时电压恒 0（安全，不发力）。
 */
static void GM6020PidCalc(void)
{
    float angle_set;
    float target_rpm;

    if (gm6020_enable == 0)   // 没使能：电压归零，并清空 PID 状态，避免使能瞬间猛推
    {
        gm6020_voltage = 0;
        PID_Calc_Clear(&gm6020_shell_pid);
        PID_Calc_Clear(&gm6020_core_pid);
        return;
    }

    // 防止目标角度和反馈角度跨 ±180° 绕大圈（走最短的近路）
    angle_set = PreventFullCircleConstrain(gm6020_target_angle, GM6020_Motor.Data.Angle);

    // 外环 位置环：角度误差 → 目标转速(rpm)
    target_rpm = PID_Calculate(&gm6020_shell_pid, angle_set, GM6020_Motor.Data.Angle);

    // 内环 速度环：转速误差 → 电压
    gm6020_voltage = (int16_t)PID_Calculate(&gm6020_core_pid, target_rpm, (float)GM6020_Motor.Data.Velocity);
}



static inline int16_t float_to_int16(float f, int16_t scale)
{
    int32_t temp = (int32_t)roundf(f * scale);
    if (temp > INT16_MAX) return INT16_MAX;
    if (temp < INT16_MIN) return INT16_MIN;
    return (int16_t)temp;
}

static inline int32_t float_to_int32(float f, int32_t scale)
{
    int64_t temp = (int64_t)roundf(f * scale);
    if (temp > INT32_MAX) return INT32_MAX;
    if (temp < INT32_MIN) return INT32_MIN;
    return (int32_t)temp;
}


/**
 * @brief 斜坡函数：让速度从 last_speed 平滑逼近 target_speed
 * @note  每次最多变化 max_step，避免速度突变。
 *        目标速度为 0 时把步长放大 3 倍（刹车更快更果断）。
 *        调车时 max_step 越大响应越快但越容易顿挫。
 */
float ramp_speed(float last_speed, float target_speed, float max_step)
{

    float delta = target_speed - last_speed;


    if (target_speed == 0)
        max_step *= 3;
    if (delta > max_step)
    {
        return last_speed + max_step;
    }
    else if (delta < -max_step)
    {
        return last_speed - max_step;
    }
    else
    {
        return target_speed; 
    }

}

/**
 * @brief 把底盘速度指令打包成 CAN 0x21 帧发给底盘
 * @note  数据格式：Data[0..5] = X/Y/Z 三个速度（各 16bit，放大 1000 倍发整数），
 *        Data[6..7] = reset_flag（复位标志）。
 *        底盘那边解出这帧就知道要往哪跑、转多快。
 */
void TransmitChassis_Info(FDCAN_TxFrame_TypeDef *FDCAN_TxFrame)
{
    int16_t Data_chassis[4];

    Data_chassis[0] = gimbal.chassis_set[CHASSIS_X] * 1000;
    Data_chassis[1] = gimbal.chassis_set[CHASSIS_Y] * 1000;
    Data_chassis[2] = gimbal.chassis_set[CHASSIS_Z] * 1000;
    Data_chassis[3] = gimbal.reset_flag;

    FDCAN_TxFrame->Header.Identifier = 0x21;
    FDCAN_TxFrame->Data[0] = (uint8_t)(Data_chassis[0] >> 8);
    FDCAN_TxFrame->Data[1] = (uint8_t)Data_chassis[0];
    FDCAN_TxFrame->Data[2] = (uint8_t)(Data_chassis[1] >> 8);
    FDCAN_TxFrame->Data[3] = (uint8_t)Data_chassis[1];
    FDCAN_TxFrame->Data[4] = (uint8_t)(Data_chassis[2] >> 8);
    FDCAN_TxFrame->Data[5] = (uint8_t)Data_chassis[2];
    FDCAN_TxFrame->Data[6] = (uint8_t)(Data_chassis[3] >> 8);
    FDCAN_TxFrame->Data[7] = (uint8_t)Data_chassis[3];

    USER_FDCAN_AddMessageToTxFifoQ(FDCAN_TxFrame);
};
int16_t watchUI[4];

/**
 * @brief 把云台状态打包成 CAN 0x22 帧发出去（给底盘/上位机/UI 灯）
 * @note  Data[0..1] 是 16 个 bit 标志位（给上位机 UI 显示状态用）：
 *         bit0=旋转模式 bit1=不跟随 bit2=零力 bit3=发射就绪 bit4=发射中
 *         bit5=视觉有目标 bit6=视觉开火 bit7=复位标志
 *        Data[2..3] = pitch 绝对角度（低通滤波后 ×100 发整数）
 *        Data[4..5] = 拨弹轮当前电流设定
 */
void TransmitGimbal_Info(FDCAN_TxFrame_TypeDef *FDCAN_TxFrame)
{
    int16_t Data_UIandYAW[4] = {0};  

    
    // bit0: rotate_state
    if (gimbal.chassis_mode == CHASSIS_ROTATE_MODE) {
        Data_UIandYAW[0] |= (1 << 0);
    }
    // bit1: follow_state
    if (gimbal.chassis_mode == CHASSIS_NO_FOLLOW_GIMBAL_MODE) {
        Data_UIandYAW[0] |= (1 << 1); 
    } 
		// bit2: zero_force_state
		if (gimbal.robot_mode_set == ZERO_FORCE_MODE) {
        Data_UIandYAW[0] |= (1 << 2);
    }
		// bit3: zero_force_state
		if (shoot.shoot_mode == SHOOT_READY) {
        Data_UIandYAW[0] |= (1 << 3);
    }		
		// bit4: zero_force_state
		if (shoot.shoot_mode == SHOOT_NORMAL ) {
        Data_UIandYAW[0] |= (1 << 4);
    }
    // bit5: vision_robot_state
    if (Vision_Data.vision_flag == 1) {
        Data_UIandYAW[0] |= (1 << 5);
    }
    // bit6: vision_shoot_state
    if (Vision_Data.shoot_flag == 1) {
        Data_UIandYAW[0] |= (1 << 6);
    }
    // bit7: reset_flag
    if (gimbal.reset_flag) {
        Data_UIandYAW[0] |= (1 << 7);
    }

    
    Data_UIandYAW[1] = (int16_t)(LowPassFilter1p_Update(&pitch_tx_lpf, gimbal.angle.absolute_fdb[0]) * 100.0f);

    
    Data_UIandYAW[2] = shoot.sht_motor_set.current_set;
    
    
    Data_UIandYAW[3] = 0; 

    
    FDCAN_TxFrame->Header.Identifier = 0x22;  
    FDCAN_TxFrame->Data[0] = (uint8_t)(Data_UIandYAW[0] >> 8);
    FDCAN_TxFrame->Data[1] = (uint8_t)Data_UIandYAW[0];
    FDCAN_TxFrame->Data[2] = (uint8_t)(Data_UIandYAW[1] >> 8);
    FDCAN_TxFrame->Data[3] = (uint8_t)Data_UIandYAW[1];
    FDCAN_TxFrame->Data[4] = (uint8_t)(Data_UIandYAW[2] >> 8);
    FDCAN_TxFrame->Data[5] = (uint8_t)Data_UIandYAW[2];
    FDCAN_TxFrame->Data[6] = (uint8_t)(Data_UIandYAW[3] >> 8);
    FDCAN_TxFrame->Data[7] = (uint8_t)Data_UIandYAW[3];

    USER_FDCAN_AddMessageToTxFifoQ(FDCAN_TxFrame);
    
    
    watchUI[0] = Data_UIandYAW[0];
    watchUI[1] = Data_UIandYAW[1];
    watchUI[2] = Data_UIandYAW[2];
    watchUI[3] = Data_UIandYAW[3];
}