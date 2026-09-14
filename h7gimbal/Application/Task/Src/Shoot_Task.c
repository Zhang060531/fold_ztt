#include "Shoot_task.h"
#include "Control_Task.h"
#include "main.h"
#include "pid.h"
#include "remote_control.h"
#include "Motor.h"
#include "bsp_can.h"
#include "cmsis_os.h"
#include "Referee_System.h"
#include <stdio.h>
#include "Image_Transmission.h"
#include <math.h>
#include "INS_Task.h"

/**
 * =====================================================================
 * 文件说明：发射机构任务 (Shoot_Task.c) —— 摩擦轮 + 拨弹轮控制
 * ---------------------------------------------------------------------
 * FreeRTOS 任务 Shoot_Task 每 1ms 循环，控制发射。主流程(见 Shoot_Task)：
 *  1. 读反馈：摩擦轮/拨弹轮转速角度 + 算摩擦轮加速度 + 更新本地热量
 *  2. 触发处理：检测开火按键(鼠标右键/左键/UI)，判断卡弹
 *  3. 更新发射状态机(Shoot_Mode_Update)
 *  4. 摩擦轮设速(Shoot_Speed_Set)：两个摩擦轮反向转，把子弹甩出去
 *  5. 拨弹轮位置控制(Shoot_Position_Set)：转到目标角度拨一颗子弹
 *  6. PID 计算(Shoot_PID_Calculate)：串级 PID + 前馈 + 热量限速
 *
 * ===== 调车关注 =====
 *  - 摩擦轮转速在 FRICTION_MOTOR_SPEED_SET 宏，改射速就改它
 *  - 每发子弹拨弹轮转 45°(SHOOT_ANGLE_PER_BULLET)
 *  - 热量保护：裁判系统热量 + 本地估算热量，过热会限速/禁止发射
 * =====================================================================
 */

/*********************************************************/
/* 宏定义                                                */
/*********************************************************/
#define SHOOT_ANGLE_PER_BULLET    45.0f   // 每发子弹拨弹轮需要转动的角度
#define ANGLE_ERROR_THRESHOLD     5.0f    // 角度误差阈值(度)
#define FRICTION_MOTOR_COUNT      2       // [MODIFIED] 摩擦轮数量改为2
#define SPEED_CTRL_MAX_RPM        18000.0f // 速度环最大转速
#define ENCODER_RESOLUTION        8192.0f // 编码器分辨率
#define FRICTION_MOTOR_SPEED_SET  6200    // 摩擦轮转速

// 堵转保护参数
#define JAMMED_RPM                15      // 判定堵转的最大转速限制
#define JAMMED_CUR                14000   // 判定堵转的最小电流阈值
#define JAM_REVERSE_TIME          160     // 反拨持续时间 (ms)

// 热量与前馈参数
#define HEAT_17MM                 10      // 17mm弹丸单发热量
#define COOLING_VALUE_DEFAULT     40      // 默认冷却速率(每秒)
#define FRI_ACCEL_FF_K            0.02f // 摩擦轮角加速度前馈系数0.02

/*********************************************************/
/* 外部变量声明                                          */
/*********************************************************/
extern Gimbal_t gimbal;
extern Image_Transmission_Info_TypeDef Image_Transmission_Info;
extern VT13_Info_TypeDef VT13_Info;
extern TIM_HandleTypeDef htim1;
extern int Remote_Interrupt_Flag;
extern int Remote_Interrupt_Flag_last;
extern Vision_Data_t Vision_Data;
extern uint8_t visionMode_test2;

/*********************************************************/
/* 全局变量定义                                          */
/*********************************************************/
Shoot_t shoot;

// PID控制器
PID_Info_TypeDef fri_motor_pid[2];            // [MODIFIED] 摩擦轮速度环PID 改为2个
PID_Info_TypeDef sht_motor_shell_pid;         // 拨弹轮位置环PID (外环)
PID_Info_TypeDef sht_motor_core_pid;          // 拨弹轮速度环PID (内环)

// 状态标志
volatile uint8_t shoot_trigger_flag = 0;     // 单次触发标志
volatile uint8_t bullet_ready_flag = 0;      // 子弹到位标志
static uint32_t last_trigger_time = 0;       // 上次触发时间
float target_continuous_angle = 0.0f;        // 目标连续角度（支持多圈）

// 堵转保护变量
uint8_t jammed_flag = 0;                     // 卡弹标志位
uint8_t jam_detect_count = 0;                // 卡弹防误触滤波
uint16_t jam_timer = 0;                      // 反拨计时器

// 前馈与热量控制变量
float fri_accel[2] = {0};                    // [MODIFIED] 摩擦轮角加速度 改为2个
float last_fri_rpm[2] = {0};                 // [MODIFIED] 摩擦轮上次转速 改为2个
uint32_t last_ff_tick = 0;                   // 上次前馈计算时间戳

float local_heat = 0.0f;                     // 本地估算热量 (防延迟)
uint16_t last_referee_heat = 0;              // 记录裁判系统上次的热量值
uint32_t last_heat_tick = 0;                 // 热量更新时间戳

// 用于检测键位变化的变量
int s1_val_last = 0;
int c1_val_last = 0;
int s2_val_last = 0;

// 拨弹轮控制模式
typedef enum
{
    SHT_CTRL_POSITION = 0,     // 单位置环控制
    SHT_CTRL_SPEED,            // 纯速度环控制 (反转时使用)
    SHT_CTRL_HYBRID            // 串级混合控制 (正常射击)
} Sht_Ctrl_Mode_e;

Sht_Ctrl_Mode_e sht_ctrl_mode = SHT_CTRL_HYBRID;

/*********************************************************/
/* 函数声明                                              */
/*********************************************************/
static void Shoot_Init(void);
static void Shoot_Get_Feedback(Shoot_t* shoot);
static Shoot_Mode_e Shoot_Mode_Update(Shoot_t* shoot);
static void Shoot_Speed_Set(Shoot_t* shoot);
static void Shoot_Position_Set(Shoot_t* shoot);
static void Shoot_PID_Calculate(Shoot_t* shoot);
static void Shoot_Trigger_Handler(Shoot_t* shoot);
static float Normalize_Angle(float angle);
static float Normalize_Angle_Error(float error);
static float Get_Angle_Error(float target, float current);

// 键鼠缓存
static int16_t ch0_val = 0, ch0_last_val = 0;
static int16_t ch1_val = 0, ch1_last_val = 0;
static uint16_t REF_key_robot_mode = 0, REF_key_robot_mode_last = 0;
static int8_t REF_mouse_right = 0, REF_mouse_right_last = 0;
static int8_t REF_mouse_left = 0, REF_mouse_left_last = 0;
static int8_t REF_mouse_middle = 0, REF_mouse_middle_last = 0;
static uint8_t ui_shoot_mode = 0, ui_shoot_mode_last = 0;
static int8_t ui_fri_mode = 0, ui_fri_mode_last = 0;

/*********************************************************/

void Shoot_Task(void const *pvParameters)
{
    Shoot_Init();

    while (1)
    {
        Shoot_Get_Feedback(&shoot);          // 获取数据 + 计算加速度 + 本地热量更新
        Shoot_Trigger_Handler(&shoot);       // 按键触发处理
        shoot.shoot_mode = Shoot_Mode_Update(&shoot);
        Shoot_Speed_Set(&shoot);             // 摩擦轮设速
        Shoot_Position_Set(&shoot);          // 拨盘位置控制 (含卡弹检测与反拨)
        Shoot_PID_Calculate(&shoot);         // PID计算 (含前馈与热量限速)
        osDelay(1);
    }
}

static void Shoot_Get_Feedback(Shoot_t* shoot)
{
    // 1. 获取常规电机数据 [MODIFIED] 删除FR3
    GetMotorFR1Fdb(&shoot->fri_motor_fdb[FR1]);
    GetMotorFR2Fdb(&shoot->fri_motor_fdb[FR2]);

    shoot->fri_motor_fdb[FR1].Data.Velocity_filter = filter_FR1(Int16ToFloat(shoot->fri_motor_fdb[FR1].Data.Velocity), FRI_FILTER);
    shoot->fri_motor_fdb[FR2].Data.Velocity_filter = filter_FR2(Int16ToFloat(shoot->fri_motor_fdb[FR2].Data.Velocity), FRI_FILTER);

    GetShootMotorFdb(&shoot->sht_motor_fdb);
    shoot->sht_motor_fdb.Data.Velocity_filter = filter_sht(Int16ToFloat(shoot->sht_motor_fdb.Data.Velocity), FRI_FILTER);
    shoot->shoot_motor.continuous_angle_fdb = shoot->sht_motor_fdb.Data.Continuous_Angle;

    // 2. 计算摩擦轮加速度用于前馈
    uint32_t current_tick = HAL_GetTick();
    float dt = (current_tick - last_ff_tick) / 1000.0f;
    if(dt > 0.0f && dt < 0.1f) 
    {
        for(int i = 0; i < FRICTION_MOTOR_COUNT; i++) {
            float current_rpm = shoot->fri_motor_fdb[i].Data.Velocity_filter;
            fri_accel[i] = (current_rpm - last_fri_rpm[i]) / dt;
            last_fri_rpm[i] = current_rpm;
        }
    }
    last_ff_tick = current_tick;

    // 3. 本地热量更新
    float heat_dt = (current_tick - last_heat_tick) / 1000.0f;
    if(heat_dt > 0.0f && heat_dt < 0.5f) {
        if(shoot->shooter_barrel_heat != last_referee_heat) {
            local_heat = shoot->shooter_barrel_heat + heat_dt * shoot->shooter_barrel_cooling_value; 
            last_referee_heat = shoot->shooter_barrel_heat;
        } else {
            local_heat -= shoot->shooter_barrel_cooling_value  * heat_dt;
        }
    }
    if(local_heat < 0.0f) local_heat = 0.0f;
    if(shoot->shooter_barrel_heat == 0) local_heat = 0.0f;
    
    last_heat_tick = current_tick;
}

Shoot_Mode_e Shoot_Mode_Update(Shoot_t* shoot)
{
    static Shoot_Mode_e last_mode = SHOOT_STOP;
    Shoot_Mode_e new_mode = shoot->shoot_mode;

    int s2_val = remote_ctrl.rc.s[0];

    // 状态覆写逻辑
    if (s2_val == 2) {
        if (new_mode == SHOOT_STOP) new_mode = SHOOT_READY;
    } else if (s2_val == 3) {
        if (new_mode == SHOOT_READY || new_mode == SHOOT_NORMAL) new_mode = SHOOT_STOP;
    }
    
    if (((REF_key_robot_mode & KEY_PRESSED_OFFSET_F) > 0 && (REF_key_robot_mode_last & KEY_PRESSED_OFFSET_F) == 0)||(ui_fri_mode==1&&ui_fri_mode_last==0)) {
        if (new_mode == SHOOT_STOP) new_mode = SHOOT_READY;
        else if (new_mode == SHOOT_READY || new_mode == SHOOT_NORMAL) new_mode = SHOOT_STOP;
    }

    // ================= 核心修复部分 =================

    // 1. 边缘触发逻辑：仅在状态真正发生跳变时执行初始化清算
    if (new_mode != last_mode)
    {
        switch (new_mode)
        {
        case SHOOT_READY:
				case SHOOT_NORMAL:
            shoot->fri_speed = FRICTION_MOTOR_SPEED_SET;
            bullet_ready_flag = 1;
            if (last_mode == SHOOT_STOP || last_mode == SHOOT_INVERSION) 
            {
                target_continuous_angle = shoot->shoot_motor.continuous_angle_fdb;
                if(sht_motor_shell_pid.PID_Calc_Clear != NULL) {
                    sht_motor_shell_pid.PID_Calc_Clear(&sht_motor_shell_pid);
                    sht_motor_core_pid.PID_Calc_Clear(&sht_motor_core_pid);
                }
                jammed_flag = 0; 
            }
            break;

        case SHOOT_INVERSION:
            shoot->fri_speed = FRICTION_MOTOR_SPEED_SET;
            break;

        case SHOOT_STOP:
            shoot->fri_speed = 0;
            bullet_ready_flag = 0;
            shoot_trigger_flag = 0;
            target_continuous_angle = shoot->shoot_motor.continuous_angle_fdb;
            jammed_flag = 0;
            break;
            
        default:
            shoot->fri_speed = 0;
            bullet_ready_flag = 0;
            shoot_trigger_flag = 0;
            target_continuous_angle = shoot->shoot_motor.continuous_angle_fdb;
            break;
        }
        last_mode = new_mode;
    }

    // 2. 连续状态逻辑：只要处于 NORMAL 就随时允许消耗准备标志进行触发
    if (new_mode == SHOOT_NORMAL) {
        if (bullet_ready_flag) {
            bullet_ready_flag = 0;
            shoot_trigger_flag = 1; 
        }
    }

    // ===============================================

    return new_mode;
}
int timedd,a=0,bbb=0;
static void Shoot_Trigger_Handler(Shoot_t* shoot)
{
    // 1. 缓存键鼠和遥控器数据 (保持原样)
    ch0_last_val = ch0_val; ch0_val = remote_ctrl.rc.ch[0];
    ch1_last_val = ch1_val; ch1_val = remote_ctrl.rc.ch[1];
    REF_key_robot_mode_last = REF_key_robot_mode; REF_key_robot_mode = VT13_Info.Key.V;
    REF_mouse_right_last = REF_mouse_right; REF_mouse_right = VT13_Info.Mouse.Press_R;
    REF_mouse_left_last = REF_mouse_left; REF_mouse_left = VT13_Info.Mouse.Press_L;
    REF_mouse_middle_last = REF_mouse_middle; REF_mouse_middle = VT13_Info.Mouse.Press_M;
    ui_fri_mode_last = ui_fri_mode; ui_fri_mode = VT13_Info.RC.Right;
    ui_shoot_mode_last = ui_shoot_mode; ui_shoot_mode = VT13_Info.RC.Trigger;
    
    int s1_val = remote_ctrl.rc.s[1];
    int c1_val = remote_ctrl.rc.ch[4];
    
    // 2. 检查热量是否允许发弹
    uint8_t heat_permit = (shoot->shooter_barrel_heat_limit - local_heat) >= (HEAT_17MM * 2.0f);

    // 提取所有触发源的状态（遥控器拨轮、鼠标左键、UI按键）
   // uint8_t is_trigger_active = (REF_mouse_left == 1) || (ui_shoot_mode == 1);
	uint8_t is_trigger_active = (REF_mouse_right == 1) || (ui_shoot_mode == 1);
    // 视觉触发保持原有逻辑（通常视觉自瞄确认对准后由代码控制连发）
    uint8_t vision_trigger = 0;
    bbb=vision_trigger;
    // 静态变量用于记录按键的历史状态和按下时刻
    static uint8_t was_trigger_active = 0;
    static uint32_t press_start_time = 0;
    timedd = press_start_time ;
    // 判定是否为“刚按下的瞬间”（上升沿）
    uint8_t trigger_just_pressed = is_trigger_active && !was_trigger_active;
    if (trigger_just_pressed) {
        press_start_time = HAL_GetTick(); // 记录按下的毫秒时间戳
    }
//    // 触发条件 1：刚按下的瞬间，无延迟立刻发一发（兼顾单发和连发的第一发）
//    if (trigger_just_pressed) {
//        should_shoot = 1;
//    }
//    // 触发条件 2：按住时间超过 100ms，判定为长按连发，持续置位
//  else if (is_trigger_active && (HAL_GetTick() - press_start_time > 100)) {
//        should_shoot = 1;
//    } 
    // 触发条件 3：视觉自瞄开火
	if(REF_mouse_right==1||VT13_Info.RC.Trigger==2){
        vision_trigger=((Vision_Data.shoot_flag == 1 && REF_mouse_right==1)||(ui_shoot_mode == 1 && Vision_Data.shoot_flag == 1)||(REF_mouse_left==1));
	}
	else{
        vision_trigger=REF_mouse_left == 1||ui_shoot_mode == 1;
    }
	
	uint8_t should_shoot = 0;
	if (vision_trigger) {
    should_shoot = 1;
    }
    // 最终执行拨弹
    if (should_shoot && heat_permit) {
  //if (should_shoot) {
        // 必须处于 SHOOT_READY 状态（意味着上一发已经拨完）且未卡弹
        if (shoot->shoot_mode == SHOOT_READY && !jammed_flag) 
        {
            a++;
            shoot->shoot_mode = SHOOT_NORMAL;
            last_trigger_time = HAL_GetTick();
			local_heat += HEAT_17MM;
        }
    }
    
    // 更新历史状态，供下一次循环比对
    was_trigger_active = is_trigger_active;
    
    s1_val_last = s1_val;
    c1_val_last = c1_val;
}
int fri1=0;
int fri2=0;
static void Shoot_Speed_Set(Shoot_t* shoot)
{
    switch (shoot->shoot_mode) {
        case SHOOT_READY:
        case SHOOT_NORMAL:
        case SHOOT_INVERSION:
            shoot->fri_motor_set[FR1].speed_rpm_set = -shoot->fri_speed;
            shoot->fri_motor_set[FR2].speed_rpm_set = shoot->fri_speed;
		fri1=shoot->fri_motor_fdb[FR1].Data.Velocity;
				fri2=-shoot->fri_motor_fdb[FR2].Data.Velocity;
            break;
        case SHOOT_STOP:
        default:
            shoot->fri_motor_set[FR1].speed_rpm_set = 0;
            shoot->fri_motor_set[FR2].speed_rpm_set = 0;
            break;
    }
}

static void Shoot_Position_Set(Shoot_t* shoot)
{
    static uint8_t position_control_enabled = 0;
    float current_continuous_angle = shoot->shoot_motor.continuous_angle_fdb;
    float angle_error;

//    if (shoot->shoot_mode == SHOOT_READY || shoot->shoot_mode == SHOOT_NORMAL) {
//        if (abs(shoot->sht_motor_fdb.Data.Current) >= JAMMED_CUR && 
//            abs(shoot->sht_motor_fdb.Data.Velocity) <= JAMMED_RPM) {
//            jam_detect_count++;
//            if(jam_detect_count > 10) { 
//                jammed_flag = 1;
//            }
//        } else {
//            jam_detect_count = 0;
//        }
//    }

//    if (jammed_flag) {
//        jam_timer++;
//        sht_ctrl_mode = SHT_CTRL_SPEED;
//        
//        // [已修改] 卡弹反拨方向改为正数，原为 -300.0f
//        shoot->sht_motor_set.speed_rpm_set = 300.0f; 
//        
//        if (jam_timer >= JAM_REVERSE_TIME) {
//            jammed_flag = 0;
//            jam_timer = 0;
//            target_continuous_angle = current_continuous_angle;
//            shoot->shoot_mode = SHOOT_READY;
//            position_control_enabled = 1;
//        }
//        return; 
//    }

    switch (shoot->shoot_mode)
    {
    case SHOOT_READY:
        if (!position_control_enabled) {
            target_continuous_angle = current_continuous_angle;
            position_control_enabled = 1;
        }
        sht_ctrl_mode = SHT_CTRL_HYBRID;
        shoot->shoot_motor.continuous_angle_set = target_continuous_angle;
        bullet_ready_flag = 1;
        break;

    case SHOOT_NORMAL:
        if (shoot_trigger_flag) {
            target_continuous_angle += 2*SHOOT_ANGLE_PER_BULLET; 
            
            shoot_trigger_flag = 0;
            bullet_ready_flag = 0;
            position_control_enabled = 1;
        }
        sht_ctrl_mode = SHT_CTRL_HYBRID;
        shoot->shoot_motor.continuous_angle_set = target_continuous_angle;

        angle_error = Get_Angle_Error(target_continuous_angle, current_continuous_angle);
        if (position_control_enabled && fabs(angle_error) < ANGLE_ERROR_THRESHOLD) {
            bullet_ready_flag = 1;
            shoot->shoot_mode = SHOOT_READY;
        }
        break;

    case SHOOT_INVERSION:
        position_control_enabled = 0;
        sht_ctrl_mode = SHT_CTRL_SPEED;
        
        shoot->sht_motor_set.speed_rpm_set = 300.0f;
        break;

    case SHOOT_STOP:
    default:
        target_continuous_angle = current_continuous_angle;
        shoot->shoot_motor.continuous_angle_set = target_continuous_angle;
        sht_ctrl_mode = SHT_CTRL_POSITION;
        position_control_enabled = 0;
        break;
    }
}
float last_aaaaaa=0;
float aaaaaa=0;
static void Shoot_PID_Calculate(Shoot_t* shoot)
{
    // 双摩擦轮速度环PID计算 + 前馈补偿
	float pid_out[2]={0};
	float ff_out[2]={0};
    for (int i = 0; i < FRICTION_MOTOR_COUNT; i++) {
        pid_out[i] = PID_Calculate(
                &fri_motor_pid[i],
                Int16ToFloat(shoot->fri_motor_set[i].speed_rpm_set),
                Int16ToFloat(shoot->fri_motor_fdb[i].Data.Velocity)
        );
		ff_out[i]=fri_accel[i] * FRI_ACCEL_FF_K; 
		
        if(shoot->fri_motor_set[i].speed_rpm_set == 0) ff_out[i]= 0; 
		
        shoot->fri_motor_set[i].current_set = pid_out[i] - ff_out[i];
	}

    float current_continuous_angle = shoot->shoot_motor.continuous_angle_fdb;
    
    switch (sht_ctrl_mode)
    {
    case SHT_CTRL_SPEED:
        shoot->sht_motor_set.current_set = PID_Calculate(
                                               &sht_motor_core_pid,
                                               shoot->sht_motor_set.speed_rpm_set,
                                               shoot->sht_motor_fdb.Data.Velocity
                                           );
        break;

    case SHT_CTRL_POSITION:
    case SHT_CTRL_HYBRID:
        {
            float angle_err_abs = fabs(Get_Angle_Error(shoot->shoot_motor.continuous_angle_set, current_continuous_angle));
			int allow_bullet_num = (shoot->shooter_barrel_heat_limit - local_heat) / HEAT_17MM;
			if (allow_bullet_num < 4.5) {
                shoot->shoot_motor.continuous_angle_set = current_continuous_angle;
            }
            float pos_output = PID_Calculate(
                                   &sht_motor_shell_pid,
                                   shoot->shoot_motor.continuous_angle_set,
                                   current_continuous_angle
                               );
			
            aaaaaa=shoot->sht_motor_fdb.Data.Velocity;
			if(aaaaaa>last_aaaaaa)			last_aaaaaa=aaaaaa;
		
            float dynamic_max_rpm = SPEED_CTRL_MAX_RPM;

            if (pos_output > dynamic_max_rpm) pos_output = dynamic_max_rpm;
            else if (pos_output < -dynamic_max_rpm) pos_output = -dynamic_max_rpm;

            if (angle_err_abs < 5.0f)
            {
                sht_motor_core_pid.Integral = 0.0f; 
                sht_motor_core_pid.Iout = 0.0f;
                
                if (pos_output > 800.0f) pos_output = 800.0f;
                if (pos_output < -800.0f) pos_output = -800.0f;
            }

            float spd_output = PID_Calculate(
                                   &sht_motor_core_pid,
                                   pos_output, 
                                   shoot->sht_motor_fdb.Data.Velocity
                               );

            shoot->sht_motor_set.current_set = spd_output;
        }
        break;

    default:
        shoot->sht_motor_set.current_set = 0;
        break;
    }

    if (shoot->sht_motor_set.current_set > SHOOT_MOTOR_CORE_PID_MAX_OUT) {
        shoot->sht_motor_set.current_set = SHOOT_MOTOR_CORE_PID_MAX_OUT;
    } else if (shoot->sht_motor_set.current_set < -SHOOT_MOTOR_CORE_PID_MAX_OUT) {
        shoot->sht_motor_set.current_set = -SHOOT_MOTOR_CORE_PID_MAX_OUT;
    }
}					

static void Shoot_Init(void)
{
    shoot.shoot_mode = SHOOT_STOP;
    shoot.fri_speed = FRICTION_MOTOR_SPEED_SET;
    shoot.shoot_motor.number = 0;
    shoot.shoot_motor.shoot_angle_fdb = 0.0f;
    shoot.shoot_motor.shoot_angle_fdb_last = 0.0f;
    shoot.shoot_motor.shoot_angle_set = 0.0f;
    shoot.shoot_motor.shoot_ecd_fdb_last = 0.0f;
    shoot.shoot_motor.shoot_ecd_fdb_error = 0.0f;
    shoot.shoot_relative_fdb = 0.0f;
    shoot.shoot_relative_set = 0.0f;
    shoot.shoot_motor.continuous_angle_fdb = 0.0f;
    shoot.shoot_motor.continuous_angle_set = 0.0f;

    s1_val_last = 0;
    c1_val_last = 0;
    s2_val_last = 0;

    sht_ctrl_mode = SHT_CTRL_HYBRID;
    target_continuous_angle = 0.0f;

    // 摩擦轮PID初始化
    for (int i = 0; i < FRICTION_MOTOR_COUNT; i++) {
        fri_motor_pid[i].Param.KP = FRICTION_MOTOR_PID_KP;
        fri_motor_pid[i].Param.KI = FRICTION_MOTOR_PID_KI;
        fri_motor_pid[i].Param.KD = FRICTION_MOTOR_PID_KD;
        fri_motor_pid[i].Param.LimitIntegral = FRICTION_MOTOR_PID_MAX_IOUT;
        fri_motor_pid[i].Param.LimitOutput = FRICTION_MOTOR_PID_MAX_OUT;
        fri_motor_pid[i].Type = PID_INTEGRAL_SEPARATION;
        fri_motor_pid[i].Param.IntSeparationErr = 500.0f; 
    }

    // 拨弹轮位置环PID参数 (外环)
    sht_motor_shell_pid.Param.KP = SHOOT_MOTOR_SHELL_PID_KP;
    sht_motor_shell_pid.Param.KI = 0.0f; 
    sht_motor_shell_pid.Param.KD = SHOOT_MOTOR_SHELL_PID_KD;
    sht_motor_shell_pid.Param.LimitIntegral = SHOOT_MOTOR_SHELL_PID_MAX_IOUT;
    sht_motor_shell_pid.Param.LimitOutput = SPEED_CTRL_MAX_RPM; 
    sht_motor_shell_pid.Type = PID_INTEGRAL_SEPARATION;
    sht_motor_shell_pid.Param.IntSeparationErr = 15.0f; 

    // 拨弹轮速度环PID参数 (内环)
    sht_motor_core_pid.Param.KP = SHOOT_MOTOR_CORE_PID_KP;
    sht_motor_core_pid.Param.KI = SHOOT_MOTOR_CORE_PID_KI;
    sht_motor_core_pid.Param.KD = SHOOT_MOTOR_CORE_PID_KD;
    sht_motor_core_pid.Param.LimitIntegral = SHOOT_MOTOR_CORE_PID_MAX_IOUT;
    sht_motor_core_pid.Param.LimitOutput = SHOOT_MOTOR_CORE_PID_MAX_OUT;
    sht_motor_core_pid.Type = PID_INTEGRAL_SEPARATION;
    sht_motor_core_pid.Param.IntSeparationErr = 800.0f; 
}

static float Normalize_Angle(float angle) {
    while (angle >= 360.0f) angle -= 360.0f;
    while (angle < 0.0f) angle += 360.0f;
    if (angle > 359.0f) angle = 0.0f;
    return angle;
}

static float Normalize_Angle_Error(float error) {
    while (error > 180.0f) error -= 360.0f;
    while (error < -180.0f) error += 360.0f;
    return error;
}

static float Get_Angle_Error(float target, float current) {
    return target - current;
}

/**
 * @brief 解析裁判系统发来的 0x23 帧（发射/热量数据）
 * @note  数据格式：Data[0..1]=弹速×100，Data[2..3]=枪口热量，
 *        Data[4..5]=热量上限，Data[6..7]=冷却速率。帧 ID 必须是 0x23。
 */
void Shoot_Info_Update(uint32_t *Identifier, uint8_t *Rx_Buf, Shoot_t *ShootData) {
    if (*Identifier != 0x23) return;
    int16_t speed_raw = (Rx_Buf[0] << 8) | Rx_Buf[1];
    ShootData->initial_speed = (float)speed_raw / 100.0f; 
    ShootData->shooter_barrel_heat = (Rx_Buf[2] << 8) | Rx_Buf[3];
    ShootData->shooter_barrel_heat_limit = (Rx_Buf[4] << 8) | Rx_Buf[5];
    ShootData->shooter_barrel_cooling_value = (Rx_Buf[6] << 8) | Rx_Buf[7];
}
