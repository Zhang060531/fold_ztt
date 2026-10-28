#include "Shoot_task.h"
#include "Control_Task.h"
#include "main.h"
#include "pid.h"
#include "remote_control.h"
#include "Motor.h"
#include "bsp_can.h"
#include "cmsis_os.h"
#include "Referee_System.h"
#include "Image_Transmission.h"
#include <math.h>
#include "INS_Task.h"

/**
 * =====================================================================
 * 文件说明：发射机构任务 (Shoot_Task.c) —— 摩擦轮 + 拨弹轮控制
 * ---------------------------------------------------------------------
 * FreeRTOS 任务 Shoot_Task 每 1ms 循环，控制发射。
 *
 * 主流程是一条清晰流水线，每个函数只干一件事：
 *
 *  Shoot_Get_Feedback  读反馈：电机转速/角度 + 算摩擦轮加速度 + 更新本地热量
 *  Shoot_Input_Read    读输入：键鼠/遥控器/UI → 产出 fire_request(是否请求开火)
 *  Shoot_Mode_Update   状态机：唯一会改 shoot_mode / 目标角 / 热量的地方
 *  Shoot_Speed_Set     设转速：按状态给两个摩擦轮目标转速
 *  Shoot_Position_Set  设位置：按状态给拨弹轮目标角(或反转转速)
 *  Shoot_PID_Calculate 算PID：串级 PID + 前馈 + 热量限速 → 输出电流
 *
 * ===== 发射状态机 Shoot_Mode_e =====
 *  SHOOT_STOP      停止：摩擦轮停，拨弹盘软锁定(目标角持续跟随当前角)
 *  SHOOT_READY     就绪：摩擦轮转起来，拨弹盘锁住等开火
 *  SHOOT_NORMAL    发射中：拨弹盘目标角已 +45°，转到位后回 READY
 *  SHOOT_INVERSION 预留：堵转反转(清卡弹)，暂未使用
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
#define SHOOT_ANGLE_PER_BULLET    45.0f    // 每发子弹拨弹轮需要转动的角度
#define ANGLE_ERROR_THRESHOLD     5.0f     // 角度误差阈值(度)，拨到位判定用
#define FRICTION_MOTOR_COUNT      2        // [MODIFIED] 摩擦轮数量改为2
/* ==================== [改动1] 拨弹盘安全供弹转速上限 ====================
 * 背景：原代码拨弹轮外环输出限幅用的是 SPEED_CTRL_MAX_RPM(18000)，
 *       拨弹起步误差一大就把目标转速顶到很高 → 拨盘猛冲 → 空拨。
 * 改法：新增 SHOOT_FEED_MAX_RPM，把拨弹盘最高转速限制在一个安全值。
 * 回退方法（效果不好时）：把下面两处 [改动1] 的 SHOOT_FEED_MAX_RPM
 *       改回 SPEED_CTRL_MAX_RPM，或直接删掉本宏即可。摩擦轮不受影响。
 * 调参：起始 3000，按"不空拨的最高转速"实测 ±500 微调。
 * ==================================================================== */
#define SHOOT_FEED_MAX_RPM        5200.0f  // 拨弹盘安全供弹转速上限(实测调)
#define FRICTION_MOTOR_SPEED_SET  6200     // 摩擦轮转速

// 热量与前馈参数
#define HEAT_17MM                 10      // 17mm弹丸单发热量
#define FRI_ACCEL_FF_K            0.02f   // 摩擦轮角加速度前馈系数0.02

/*********************************************************/
/* 外部变量声明                                          */
/*********************************************************/
extern VT13_Info_TypeDef VT13_Info;
extern Vision_Data_t Vision_Data;

/*********************************************************/
/* 全局变量定义                                          */
/*********************************************************/
Shoot_t shoot;

// PID控制器
PID_Info_TypeDef fri_motor_pid[2];            // [MODIFIED] 摩擦轮速度环PID 改为2个
PID_Info_TypeDef sht_motor_shell_pid;         // 拨弹轮位置环PID (外环)
PID_Info_TypeDef sht_motor_core_pid;          // 拨弹轮速度环PID (内环)

// 目标连续角度（支持多圈，每发 +45°）
float target_continuous_angle = 0.0f;

// 前馈与热量控制变量
float fri_accel[2] = {0};                     // [MODIFIED] 摩擦轮角加速度 改为2个
float last_fri_rpm[2] = {0};                  // [MODIFIED] 摩擦轮上次转速 改为2个
uint32_t last_ff_tick = 0;                    // 上次前馈计算时间戳

float local_heat = 0.0f;                      // 本地估算热量 (防延迟)
uint16_t last_referee_heat = 0;               // 记录裁判系统上次的热量值
uint32_t last_heat_tick = 0;                  // 热量更新时间戳

/*********************************************************/
/* 函数声明                                              */
/*********************************************************/
static void Shoot_Init(void);
static void Shoot_Get_Feedback(Shoot_t* shoot);
static void Shoot_Input_Read(Shoot_t* shoot);
static Shoot_Mode_e Shoot_Mode_Update(Shoot_t* shoot);
static void Shoot_Speed_Set(Shoot_t* shoot);
static void Shoot_Position_Set(Shoot_t* shoot);
static void Shoot_PID_Calculate(Shoot_t* shoot);

// 输入缓存(Input_Read 更新当前值 + 存上一拍，供 Mode_Update 做边沿检测)
static uint8_t fire_request = 0;              // 本次循环是否请求开火
static uint16_t REF_key_robot_mode = 0, REF_key_robot_mode_last = 0;
static int8_t REF_mouse_right = 0, REF_mouse_right_last = 0;
static int8_t REF_mouse_left = 0, REF_mouse_left_last = 0;

static uint8_t ui_shoot_mode = 0, ui_shoot_mode_last = 0;
static int8_t ui_fri_mode = 0, ui_fri_mode_last = 0;

/*********************************************************/

void Shoot_Task(void const *pvParameters)
{
    Shoot_Init();

    while (1)
    {
        Shoot_Get_Feedback(&shoot);          // 读反馈 + 算加速度 + 更新热量
        Shoot_Input_Read(&shoot);            // 读输入 → 产出 fire_request
        shoot.shoot_mode = Shoot_Mode_Update(&shoot);  // 状态机(唯一改状态/目标角/热量)
        Shoot_Speed_Set(&shoot);             // 摩擦轮设速
        Shoot_Position_Set(&shoot);          // 拨弹轮设位置
        Shoot_PID_Calculate(&shoot);         // PID + 前馈 + 限幅
        osDelay(1);
    }
}

static void Shoot_Get_Feedback(Shoot_t* shoot)
{
    // 1. 获取常规电机数据 [MODIFIED]
    GetMotorFR1Fdb(&shoot->fri_motor_fdb[FR1]);
    GetMotorFR2Fdb(&shoot->fri_motor_fdb[FR2]);

    shoot->fri_motor_fdb[FR1].Data.Velocity_filter = filter_FR1(Int16ToFloat(shoot->fri_motor_fdb[FR1].Data.Velocity), FRI_FILTER);
    shoot->fri_motor_fdb[FR2].Data.Velocity_filter = filter_FR2(Int16ToFloat(shoot->fri_motor_fdb[FR2].Data.Velocity), FRI_FILTER);

    GetShootMotorFdb(&shoot->sht_motor_fdb);  //获得拨盘电机数据
    shoot->sht_motor_fdb.Data.Velocity_filter = filter_sht(Int16ToFloat(shoot->sht_motor_fdb.Data.Velocity), FRI_FILTER);
    shoot->shoot_motor.continuous_angle_fdb = shoot->sht_motor_fdb.Data.Continuous_Angle;

    // 2. 计算摩擦轮加速度用于前馈
    uint32_t current_tick = HAL_GetTick();
    float dt = (current_tick - last_ff_tick) / 1000.0f;
    if (dt > 0.0f && dt < 0.1f)
    {
        for (int i = 0; i < FRICTION_MOTOR_COUNT; i++) {
            float current_rpm = shoot->fri_motor_fdb[i].Data.Velocity_filter;
            fri_accel[i] = (current_rpm - last_fri_rpm[i]) / dt;
            last_fri_rpm[i] = current_rpm;
        }
    }
    last_ff_tick = current_tick;

    // 3. 本地热量更新
    float heat_dt = (current_tick - last_heat_tick) / 1000.0f;
    if (heat_dt > 0.0f && heat_dt < 0.5f) {
        if (shoot->shooter_barrel_heat != last_referee_heat) {
            local_heat = shoot->shooter_barrel_heat + heat_dt * shoot->shooter_barrel_cooling_value;
            last_referee_heat = shoot->shooter_barrel_heat;
        } else {
            local_heat -= shoot->shooter_barrel_cooling_value * heat_dt;
        }
    }
    if (local_heat < 0.0f) local_heat = 0.0f;
    if (shoot->shooter_barrel_heat == 0) local_heat = 0.0f;

    last_heat_tick = current_tick;
}

static void Shoot_Input_Read(Shoot_t* shoot)
{
    // 图传+键鼠：先存上一拍，再读当前值，供 Mode_Update 做边沿检测
    REF_key_robot_mode_last = REF_key_robot_mode;
    REF_key_robot_mode = VT13_Info.Key.V;              // 【图传VT13】键盘

    REF_mouse_right_last = REF_mouse_right;
    REF_mouse_right = VT13_Info.Mouse.Press_R;         // 【图传VT13】鼠标右键

    REF_mouse_left_last = REF_mouse_left;
    REF_mouse_left = VT13_Info.Mouse.Press_L;          // 【图传VT13】鼠标左键

    ui_fri_mode_last = ui_fri_mode;
    ui_fri_mode = VT13_Info.RC.Right;                  // 【图传VT13】右键(摩擦轮开关)

    ui_shoot_mode_last = ui_shoot_mode;
    ui_shoot_mode = VT13_Info.RC.Trigger;              // 【图传VT13】扳机键(发射)

    // 【DT7】拨轮 ch[4] 当作发射扳机：转离中位(超过阈值)就发射
    //   ch[4] 是 DT7 左上角的拨轮(旋钮)，解析后中位为 0，转离中位越远值越大。
    //   阈值 500 可调：转到底没反应就改小(比如 300)，容易误触发就改大(比如 650)。
    uint8_t dt7_fire = (remote_ctrl.rc.ch[4] > 500) || (remote_ctrl.rc.ch[4] < -500);

    // 开火请求 vision_trigger（分「自瞄」和「手动」两种情况）
    //    自瞄时（按住鼠标右键，或 RC.Switch 拨到自瞄档 ==2）：
    //        必须视觉确认对准了( Vision_Data.shoot_flag == 1 )才允许开火；
    //        另外鼠标左键随时可手动补一发。
    //    手动时（没进自瞄）：
    //        鼠标左键 或 UI 扳机键 直接触发。
    if (REF_mouse_right == 1 || VT13_Info.RC.Switch == 2) {      //进入vision自瞄模式
        fire_request = ((Vision_Data.shoot_flag == 1 && REF_mouse_right == 1)
                        || (ui_shoot_mode == 1 && Vision_Data.shoot_flag == 1)
                        || (REF_mouse_left == 1) || dt7_fire);
    } else {    //没进入自瞄模式
        fire_request = REF_mouse_left == 1 || ui_shoot_mode == 1 || dt7_fire;
    }
}

Shoot_Mode_e Shoot_Mode_Update(Shoot_t* shoot)
{
    static Shoot_Mode_e last_mode = SHOOT_STOP;
    Shoot_Mode_e new_mode = shoot->shoot_mode;

    /* === 第 1 步：开火（先处理，和原逻辑顺序一致） ===
     * 打出一颗子弹必须同时满足：
     *   (a) 有触发信号 fire_request
     *   (b) 热量允许：剩余热量还够至少 2 发
     *   (c) 当前状态是 SHOOT_READY（上一发已经拨到位）
     */
    uint8_t heat_permit = (shoot->shooter_barrel_heat_limit - local_heat) >= (HEAT_17MM * 2.0f);
    if (fire_request && heat_permit && new_mode == SHOOT_READY) {
        target_continuous_angle += SHOOT_ANGLE_PER_BULLET;   // 拨一发：目标角 +45°
        local_heat += HEAT_17MM;                             // 加热量
        new_mode = SHOOT_NORMAL;
    }

    /* === 第 2 步：开关切换 ===
     * 遥控器 S2：2 = 就绪，3 = 停止
     * 键盘 F 键 / UI 摩擦轮开关：在 停止↔就绪 之间切换(边沿触发)
     */
    int s2_val = remote_ctrl.rc.s[0];
    if (s2_val == 2) {
        if (new_mode == SHOOT_STOP) new_mode = SHOOT_READY;
    } else if (s2_val == 3) {
        if (new_mode == SHOOT_READY || new_mode == SHOOT_NORMAL) new_mode = SHOOT_STOP;
    }

    if (((REF_key_robot_mode & KEY_PRESSED_OFFSET_F) > 0 && (REF_key_robot_mode_last & KEY_PRESSED_OFFSET_F) == 0)
        || (ui_fri_mode == 1 && ui_fri_mode_last == 0)) {
        if (new_mode == SHOOT_STOP) new_mode = SHOOT_READY;
        else if (new_mode == SHOOT_READY || new_mode == SHOOT_NORMAL) new_mode = SHOOT_STOP;
    }

    /* === 第 3 步：拨弹到位（NORMAL → READY） === */
    if (new_mode == SHOOT_NORMAL &&
        fabs(target_continuous_angle - shoot->shoot_motor.continuous_angle_fdb) < ANGLE_ERROR_THRESHOLD) {
        new_mode = SHOOT_READY;
    }

    /* === 第 4 步：状态跳变时做一次初始化（边沿动作） === */
    if (new_mode != last_mode)
    {
        switch (new_mode)
        {
        case SHOOT_READY:
        case SHOOT_NORMAL:
            // 从停止/反转恢复时，重新同步拨盘目标角并清零积分，防止起步猛冲
            if (last_mode == SHOOT_STOP || last_mode == SHOOT_INVERSION)
            {
                target_continuous_angle = shoot->shoot_motor.continuous_angle_fdb;
                if (sht_motor_shell_pid.PID_Calc_Clear != NULL) {
                    sht_motor_shell_pid.PID_Calc_Clear(&sht_motor_shell_pid);
                    sht_motor_core_pid.PID_Calc_Clear(&sht_motor_core_pid);
                }
            }
            break;

        case SHOOT_INVERSION:
            // 预留：堵转反转，待后续补充
            break;

        case SHOOT_STOP:
        default:
            break;
        }
        last_mode = new_mode;
    }

    /* === 第 5 步：连续动作（每周期都做） === */
    if (new_mode == SHOOT_STOP) {
        // 停止时拨弹盘软锁定：目标角持续跟随当前角，被外力推动也不较劲
        target_continuous_angle = shoot->shoot_motor.continuous_angle_fdb;
    }

    return new_mode;
}

static void Shoot_Speed_Set(Shoot_t* shoot)
{
    // 摩擦轮目标转速完全由状态决定：两个摩擦轮反向转，把子弹甩出去
    switch (shoot->shoot_mode)
    {
    case SHOOT_READY:       // 就绪：转起来
    case SHOOT_NORMAL:      // 发射中：继续转
        shoot->fri_motor_set[FR1].speed_rpm_set = -FRICTION_MOTOR_SPEED_SET;
        shoot->fri_motor_set[FR2].speed_rpm_set =  FRICTION_MOTOR_SPEED_SET;
        break;

    case SHOOT_INVERSION:   // 预留：堵转反转，待后续补充
        break;

    case SHOOT_STOP:        // 停止：停
    default:
        shoot->fri_motor_set[FR1].speed_rpm_set = 0;
        shoot->fri_motor_set[FR2].speed_rpm_set = 0;
        break;
    }
}

static void Shoot_Position_Set(Shoot_t* shoot)
{
    // 拨弹轮目标：只负责按当前状态给目标值，不改状态机/目标角(那都是 Mode_Update 的事)
    switch (shoot->shoot_mode)
    {
    case SHOOT_READY:       // 就绪：锁住当前位置等开火
    case SHOOT_NORMAL:      // 发射中：目标角已在 Mode_Update +45°，这里把目标交给位置环
    case SHOOT_STOP:        // 停止：目标角被 Mode_Update 持续跟随，等于软锁定
        shoot->shoot_motor.continuous_angle_set = target_continuous_angle;
        break;

    case SHOOT_INVERSION:   // 预留：堵转反转，待后续补充
        break;

    default:
        shoot->shoot_motor.continuous_angle_set = target_continuous_angle;
        break;
    }
}


static void Shoot_PID_Calculate(Shoot_t* shoot)
{
    // 双摩擦轮速度环PID计算 + 前馈补偿
    for (int i = 0; i < FRICTION_MOTOR_COUNT; i++) {
        float pid_out = PID_Calculate(
            &fri_motor_pid[i],
            Int16ToFloat(shoot->fri_motor_set[i].speed_rpm_set),
            Int16ToFloat(shoot->fri_motor_fdb[i].Data.Velocity)
        );
        // 加速度前馈：目标转速为 0 时不补
        float ff_out = (shoot->fri_motor_set[i].speed_rpm_set == 0) ? 0.0f : fri_accel[i] * FRI_ACCEL_FF_K;

        shoot->fri_motor_set[i].current_set = pid_out - ff_out;
    }

		
    float current_continuous_angle = shoot->shoot_motor.continuous_angle_fdb;
    // 拨弹盘控制：位置环(外) + 速度环(内) 串级
    // (INVERSION 反转清卡的纯速度环处理已删除，待后续补充)
    float angle_err_abs = fabs(shoot->shoot_motor.continuous_angle_set - current_continuous_angle);
    int allow_bullet_num = (shoot->shooter_barrel_heat_limit - local_heat) / HEAT_17MM;  //还能打几发
    if (allow_bullet_num < 4.5) {
        shoot->shoot_motor.continuous_angle_set = current_continuous_angle;  //把当前角度赋给目标角度 防止超热量
    }

    float pos_output = PID_Calculate(
        &sht_motor_shell_pid,
        shoot->shoot_motor.continuous_angle_set,
        current_continuous_angle
    );

    if (pos_output > SHOOT_FEED_MAX_RPM) pos_output = SHOOT_FEED_MAX_RPM;
    else if (pos_output < -SHOOT_FEED_MAX_RPM) pos_output = -SHOOT_FEED_MAX_RPM;

    if (angle_err_abs < ANGLE_ERROR_THRESHOLD)   //这个括号内的内容在误差角度大于5°则不生效
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

    if (shoot->sht_motor_set.current_set > SHOOT_MOTOR_CORE_PID_MAX_OUT) {
        shoot->sht_motor_set.current_set = SHOOT_MOTOR_CORE_PID_MAX_OUT;
    } else if (shoot->sht_motor_set.current_set < -SHOOT_MOTOR_CORE_PID_MAX_OUT) {
        shoot->sht_motor_set.current_set = -SHOOT_MOTOR_CORE_PID_MAX_OUT;
    }
}

static void Shoot_Init(void)
{
    shoot.shoot_mode = SHOOT_STOP;

    shoot.shoot_motor.continuous_angle_fdb = 0.0f;
    shoot.shoot_motor.continuous_angle_set = 0.0f;

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
    sht_motor_shell_pid.Param.LimitOutput = SHOOT_FEED_MAX_RPM;  // [改动1] 原为 SPEED_CTRL_MAX_RPM(18000)，拨弹盘限速
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
   	//ShootData->shooter_barrel_heat_limit = 60 ;  //这个是裁判系统给我发的 但是没人发 目前是0 我设为60 不然不允许发射
    ShootData->shooter_barrel_cooling_value = (Rx_Buf[6] << 8) | Rx_Buf[7];
}
