/* ===== 文件头 =====
 * 文件名  : UI_Task.c
 * 作用    : UI 刷新任务。周期读取底盘/云台/裁判系统数据，换算成图形参数
 *           (血条角度、数字、颜色、状态指示)，再调用 RMUI 图形库刷新到
 *           裁判系统客户端(操作手屏幕)。
 * 结构    :
 *   1. 外部数据结构引用(chassis / Referee_System_Info / GiambalData / PowerData)
 *   2. UI_Data_Calculate(): 把各状态量换算成图形元素的角度/数值/颜色
 *   3. UI_Task(): FreeRTOS 任务，周期调用 初始化/计算/更新发送
 * 调用关系:
 *   freertos.c -> 创建 UI_Task 线程
 *   UI_Task -> ui_init_g()(启动时) / UI_Data_Calculate() / ui_update_g()(周期)
 *   ui_g.h   -> 提供各图形元素宏(ui_g_1_bar0 等)
 * ===== 调车关注 =====
 *   - 本文件是"UI 显示什么"的核心: 想改哪个元素显示什么值，改 UI_Data_Calculate。
 *   - 任务周期约 1ms(osDelay(1))，实际刷新受 SEND_MESSAGE 的 osDelay(33) 限流。
 *   - 四个血条角度范围(绕中心、半径387):
 *       bar0 左上 275°~314°(pitch)  bar1 左下 226°~266°(电容电压)
 *       bar2 右下  94°~133°(功率)   bar3 右上  46°~ 85°(热量)
 *   - number 字段按"千倍放大存储"(如 pitch*1000)，客户端按小数位还原显示。
 */
#include "main.h"
#include "string.h"
#include "UI_Task.h"
#include "cmsis_os.h"
#include "Referee_System.h"
#include "Control_Task.h"
#include "stdio.h"
#include "ui.h"

/* 全局变量 ui: 疑似历史遗留/调试用变量，本工程未见实际使用 */
int ui = 0;

// 引入你需要用到的外部数据结构
/* 引入需要读取的外部数据结构(由其它任务维护):
 *   chassis            底盘信息(总功率/电机反馈等)
 *   Referee_System_Info 裁判系统信息(功率限制/枪管热量等)
 *   GiambalData        云台信息(pitch/各状态标志)
 *   PowerData          超级电容/功率信息
 */
extern Chassis_Info_Typedef chassis;
extern Referee_System_Info_TypeDef Referee_System_Info; // 修正引用
extern Gimbal_Info_Typedef GiambalData;      
extern SuperPower_Info_Typedef PowerData;   

 
/**
 * @brief 计算并更新 UI 各图形元素的角度/数值/颜色
 * @note  由 UI_Task 周期调用。把云台 pitch、电容电压、底盘功率、枪管热量、
 *        各状态标志等换算成血条角度、数字(number 字段按千倍放大存储)与颜色。
 */
void UI_Data_Calculate(void)
{
    /* ================= 1. 左上 Bar0: 云台 Pitch ================= */
    float pitch = GiambalData.pitch; 
    float pitch_ratio = (pitch - (-10.0f)) / (45.0f - (-10.0f)); 
    if (pitch_ratio > 1.0f) pitch_ratio = 1.0f;
    if (pitch_ratio < 0.0f) pitch_ratio = 0.0f;
    
    ui_g_1_bar0->start_angle = 275; 
    ui_g_1_bar0->end_angle = 275 + (uint32_t)(pitch_ratio * (314 - 275)); 
    if (ui_g_1_bar0->start_angle == ui_g_1_bar0->end_angle) ui_g_1_bar0->end_angle += 1;
    
    ui_g_2_pitchangle->number = (int32_t)(pitch * 1000); 

    /* ================= 2. 左下 Bar1: 电容电压 ================= */
    float cap_voltage = PowerData.capa_voltage;
    float cap_ratio = (cap_voltage - 6.0f) / (22.0f - 6.0f); 
    if (cap_ratio > 1.0f) cap_ratio = 1.0f;
    if (cap_ratio < 0.0f) cap_ratio = 0.0f;
    
    ui_g_1_bar1->end_angle = 266; 
    ui_g_1_bar1->start_angle = 266 - (uint32_t)(cap_ratio * (266 - 226)); 
    if (ui_g_1_bar1->start_angle == ui_g_1_bar1->end_angle) ui_g_1_bar1->start_angle -= 1;
    
    ui_g_2_capa_voltage->number = (int32_t)(cap_voltage * 1000); 

    /* ================= 3. 右下 Bar2: 底盘真实功率监控 (替换发弹数) ================= */
    float ref_limit = Referee_System_Info.robot_status.chassis_power_limit;
    if (ref_limit < 30.0f) ref_limit = 75.0f; 
    float dynamic_max_power = ref_limit;
    float current_power = chassis.super_power.tot_power;
    if (current_power < 0.0f) current_power = 0.0f;

    // UI 上显示总功率数值
    ui_g_2_total_power->number = (int32_t)(current_power* 1000);

    float power_ratio = current_power / dynamic_max_power;
    if (power_ratio > 1.0f) power_ratio = 1.0f;
    
    // 动态计算功率条比例 (Bar2 范围: 94° 到 133°)
    ui_g_1_bar2->start_angle = 94; 
    ui_g_1_bar2->end_angle = 94 + (uint32_t)(power_ratio * (133 - 94)); 
    if (ui_g_1_bar2->start_angle == ui_g_1_bar2->end_angle) ui_g_1_bar2->end_angle += 1;

    // 功率条颜色警示：超基础限幅变红，逼近限幅变橙，正常为绿
    if (current_power > ref_limit) {
        ui_g_1_bar2->color = 0; // 透支电容中 (阵营闪烁色)
    } else if (current_power > ref_limit * 0.85f) {
        ui_g_1_bar2->color = 3; // 逼近限幅 (橙色)
    } else {
        ui_g_1_bar2->color = 2; // 安全区间 (绿色)
    }

    /* ================= 4. 右上 Bar3: 42mm 热量 & 限制刻度线 (保持原始逻辑不变) ================= */
    uint16_t heat = Referee_System_Info.power_heat_data.shooter_42mm_barrel_heat;
    uint16_t heat_limit = Referee_System_Info.robot_status.shooter_barrel_heat_limit;
    if (heat_limit == 0) heat_limit = 200; // 防止断连时除 0 崩溃

    float heat_ratio = (float)heat / heat_limit;
    if (heat_ratio > 1.0f) heat_ratio = 1.0f;
    
    // Bar3 范围: 85° 往上到 46°
    ui_g_1_bar3->end_angle = 85; 
    ui_g_1_bar3->start_angle = 85 - (uint32_t)(heat_ratio * (85 - 46)); 
    if (ui_g_1_bar3->start_angle == ui_g_1_bar3->end_angle) ui_g_1_bar3->start_angle -= 1;
    
    ui_g_2_currentheart->number = heat; 

    // 刻度线位置
    float target_heat_mark = (float)heat_limit - 100.0f;
    if (target_heat_mark < 0.0f) target_heat_mark = 0.0f; 
    float limit_ratio = target_heat_mark / (float)heat_limit;
    if (limit_ratio > 1.0f) limit_ratio = 1.0f;
    
    uint32_t limit_angle = 85 - (uint32_t)(limit_ratio * (85 - 46)); 
    ui_g_2_heartlimit->start_angle = limit_angle;
    ui_g_2_heartlimit->end_angle = limit_angle + 1; // 保持 1 度的宽度

    // 发射时变红逻辑
    static uint16_t last_heat = 0;
    static uint8_t firing_timer = 0;

    if (heat > last_heat) {
        firing_timer = 5; 
    }
    last_heat = heat;

    if (firing_timer > 0) {
        firing_timer--;
        ui_g_1_bar3->color = 0; // 警示发射中
    } else {
        ui_g_1_bar3->color = 3; // 恢复橙色
    }

    /* ================= 5. 底盘模式指示器 (Robot Mode) ================= */
    if (GiambalData.zero_force_state == 1) {
        ui_g_3_robotmode->number = 0;      // 无力
    } else if (GiambalData.rotate_state == 1) {
        ui_g_3_robotmode->number = 2;      // 小陀螺
    } else if (GiambalData.follow_state == 1) {
        ui_g_3_robotmode->number = 3;      // 无云底跟随
    } else {
        ui_g_3_robotmode->number = 1;      // 正常跟随
    }

    /* ================= 6. 视觉状态指示器 (Vision Mode) ================= */
    if (GiambalData.vision_shoot_state == 1) {
        ui_g_3_visionmode->number = 2;      // 有发射标志
        ui_g_1_detect->color = 5;           
    } else if (GiambalData.vision_robot_state == 1) {
        ui_g_3_visionmode->number = 1;      // 有视觉
        ui_g_1_detect->color = 2;           
    } else {
        ui_g_3_visionmode->number = 0;      // 全无
        ui_g_1_detect->color = 8;           
    }

    /* ================= 7. 发射模式指示器 (Shoot Mode) ================= */
    if (GiambalData.shoot_normal_state == 1) {
        ui_g_3_shootmode->number = 2;       // 发射模式
    } else if (GiambalData.shoot_ready_state == 1) {
        ui_g_3_shootmode->number = 1;       // 准备模式
    } else {
        ui_g_3_shootmode->number = 0;       // 发射停
    }

    /* ================= 8. 电机离线状态映射 (Wheel & Steer) ================= */
    ui_g_4_wheel1->number = chassis.chassis_motor_fdb[0].is_offline ? 1 : 0;
    ui_g_4_wheel1->color  = chassis.chassis_motor_fdb[0].is_offline ? 5 : 2; 

    ui_g_4_wheel2->number = chassis.chassis_motor_fdb[1].is_offline ? 1 : 0;
    ui_g_4_wheel2->color  = chassis.chassis_motor_fdb[1].is_offline ? 5 : 2; 

    ui_g_4_wheel3->number = chassis.chassis_motor_fdb[2].is_offline ? 1 : 0;
    ui_g_4_wheel3->color  = chassis.chassis_motor_fdb[2].is_offline ? 5 : 2; 

    ui_g_4_wheel4->number = chassis.chassis_motor_fdb[3].is_offline ? 1 : 0;
    ui_g_4_wheel4->color  = chassis.chassis_motor_fdb[3].is_offline ? 5 : 2; 

	    /* ================= 9. 枪口指向圆点 ================= */
    /* 注: 下面一大段"枪口指向圆点坐标映射"代码已全部注释禁用，
     * 圆点当前只在 ui_g.c 的 ui_init_g 里静态初始化在 (915,540)。
     * 如需让它动态跟随云台 Yaw/Pitch，取消注释并按实际字段修正映射即可。 */
    ui_interface_round_t *vdot = (ui_interface_round_t *)&ui_g_now_figures[16];

    // Yaw 映射到屏幕 X（pitch字段实际存储的是Yaw）
    // 云台Yaw范围通常是-180~180，映射到屏幕X: 560~1360
//    float gimbal_yaw = GiambalData.pitch;   // 变量名叫pitch但实际是Yaw
//    float screen_x = 960.0f + gimbal_yaw * 2.0f;  // 2 px/度
//    if (screen_x < 560.0f)  screen_x = 560.0f;
//    if (screen_x > 1360.0f) screen_x = 1360.0f;

//    // Pitch 映射到屏幕 Y（gimbal_pitch 是新增字段）
//    // Pitch范围通常-20~45°，映射到屏幕Y: 200~800
//    float yaw = GiambalData.yaw;
//    float screen_y = 540.0f - yaw * 6.0f;  // 6 px/度，负号=抬头向上
//    if (screen_y < 150.0f) screen_y = 150.0f;
//    if (screen_y > 850.0f) screen_y = 850.0f;

//    // 根据视觉状态改颜色
//    if (GiambalData.vision_shoot_state == 1) {
//        vdot->color = 1;   // 锁定 → 红色
//        vdot->r     = 8;   // 小一点，精确
//    } else if (GiambalData.vision_robot_state == 1) {
//        vdot->color = 3;   // 跟踪中 → 黄色
//        vdot->r     = 12;  // 稍大，显眼
//    } else {
//        vdot->color = 2;   // 无目标 → 绿色
//        vdot->r     = 10;
//    }

//    vdot->start_x = (uint32_t)screen_x;
//    vdot->start_y = (uint32_t)screen_y;

}

/**
 * @brief UI 刷新任务(FreeRTOS 线程入口)
 * @param argument 任务参数(未使用)
 * @note  启动时先 ui_init_g() 全量初始化并发送一次，随后循环:
 *        计算 UI 数据 -> ui_update_g() 检测变化并发送 -> osDelay(1) 让出 CPU。
 *        注: UI_Task_SysTick 只在被注释掉的"周期强制重置"逻辑中使用，当前未生效。
 */
void UI_Task(void const * argument)
{
    TickType_t UI_Task_SysTick = 0;

    // 任务开启前进行一次全量初始化
    ui_init_g();

    while (1)
    {
        UI_Task_SysTick = osKernelSysTick();

//        // 仅在特定周期触发强制重置，防止正常情况下闪烁
//         if(UI_Task_SysTick % 1000 == 0){
//             ui_init_g();
//         }
			
        /* 无力(零力)状态时强制重初始化 UI，避免画面残留/闪烁。
         * 注意: 疑似bug/不一致 —— 这里读 chassis.gimbal_data.zero_force_state，
         * 而 UI_Data_Calculate 里读的是 GiambalData.zero_force_state，
         * 两者是否为同一数据源需确认；若不同则此判断可能永远不成立。 */
         if(chassis.gimbal_data.zero_force_state == 1){
             ui_init_g();
         }
        
        UI_Data_Calculate();
        ui_update_g();
        osDelay(1); // 保证系统调度不断
    }
}