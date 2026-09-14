/* ===== 文件头 =====
 * 文件名  : ui_g.h
 * 作用    : 本工程(底盘)所有 UI 图形元素的"命名映射表"。
 *           把 RM UI Designer 生成的 ui_g_now_figures[35] / ui_g_now_strings[9]
 *           数组元素，用易读的宏名强转成具体图形类型，方便 UI_Task.c 直接读写
 *           (如 ui_g_1_bar0->start_angle)。
 * 结构    :
 *   1. extern 声明图形/字符串数组与脏标记数组(实现在 ui_g.c)
 *   2. 图形宏:  ui_g_XX_yyyyy -> 数组某元素强转成 line/rect/round/arc/number/string
 *   3. 字符串宏:ui_g_02_xxx -> ui_g_now_strings[i]
 *   4. max_send_count 宏: 每个元素的最大重发次数(重发几次保证裁判系统收到)
 *   5. MANUAL_DIRTY 宏: 若定义则提供 *_dirty 脏标记宏(手动标记脏)
 *   6. ui_init_g / ui_update_g 接口声明
 * 调用关系:
 *   ui_g.c   -> 定义数组并实现 ui_init_g/ui_update_g
 *   UI_Task.c-> 通过本文件的宏读写图形元素
 * ===== 调车关注 =====
 *   - 下标 16 被复用作"枪口指向圆点"(ui_g_4_vdot)，
 *     下标 17/19/20 对应已注释掉的 steer1~4 槽位(见 ui_g.c 被注释代码)。
 *   - 改某个图形属性后要让它被重发: 用 ui_set 宏，或手动设对应的 *_dirty。
 *   - 图形宏的命名规律: ui_g_<图层>_<名字>，如 ui_g_1_bar0 表示图层1的圆弧进度条0。
 */
//
// Created by RM UI Designer
// Dynamic Edition
//

#ifndef UI_g_H
#define UI_g_H

#include "ui_interface.h"

/* ===== 图形/字符串数据数组(实现在 ui_g.c) ===== */
extern ui_interface_figure_t ui_g_now_figures[35];  /* 35 条图形指令的当前内容 */
extern uint8_t ui_g_dirty_figure[35];               /* 每条图形需要重发的剩余次数(>0 表示待发送) */
extern ui_interface_string_t ui_g_now_strings[9];   /* 9 条字符指令的当前内容 */
extern uint8_t ui_g_dirty_string[9];                /* 每条字符需要重发的剩余次数 */

extern uint8_t ui_g_max_send_count[44];             /* 每条图形/字符的最大重发次数(44 = 35 + 9) */

/* ===== 图形元素宏: 把数组元素强转成具体图形类型指针 ===== */
#define ui_g_00_line0 ((ui_interface_line_t*)&(ui_g_now_figures[0]))      /* 直线0: 中轴线(竖) */
#define ui_g_00_line1 ((ui_interface_line_t*)&(ui_g_now_figures[1]))      /* 直线1: 上方横线 */
#define ui_g_1_bar0 ((ui_interface_arc_t*)&(ui_g_now_figures[2]))         /* 圆弧0(左上): 云台 pitch 指示 */
#define ui_g_1_bar1 ((ui_interface_arc_t*)&(ui_g_now_figures[3]))         /* 圆弧1(左下): 电容电压 */
#define ui_g_1_bar2 ((ui_interface_arc_t*)&(ui_g_now_figures[4]))         /* 圆弧2(右下): 底盘功率 */
#define ui_g_1_bar3 ((ui_interface_arc_t*)&(ui_g_now_figures[5]))         /* 圆弧3(右上): 42mm 热量 */
#define ui_g_00_line2 ((ui_interface_line_t*)&(ui_g_now_figures[6]))      /* 直线2: 中轴线(竖) */
#define ui_g_1_detect ((ui_interface_round_t*)&(ui_g_now_figures[7]))     /* 圆: 视觉检测状态指示点 */
#define ui_g_2_pitchangle ((ui_interface_number_t*)&(ui_g_now_figures[8]))/* 数字: pitch 角度数值 */
#define ui_g_2_capa_voltage ((ui_interface_number_t*)&(ui_g_now_figures[9]))/* 数字: 电容电压数值 */
#define ui_g_2_heartlimit ((ui_interface_arc_t*)&(ui_g_now_figures[10]))  /* 圆弧: 热量限制刻度线 */
#define ui_g_00_line3 ((ui_interface_line_t*)&(ui_g_now_figures[11]))     /* 直线3: 右侧横线 */
#define ui_g_3_shootmode ((ui_interface_number_t*)&(ui_g_now_figures[12]))/* 数字: 发射模式 */
#define ui_g_3_robotmode ((ui_interface_number_t*)&(ui_g_now_figures[13]))/* 数字: 底盘模式 */
#define ui_g_3_visionmode ((ui_interface_number_t*)&(ui_g_now_figures[14]))/* 数字: 视觉模式 */
#define ui_g_00_visionrange ((ui_interface_rect_t*)&(ui_g_now_figures[15]))/* 矩形: 视觉范围框 */
#define ui_g_4_vdot ((ui_interface_round_t*)&(ui_g_now_figures[16]))      /* 圆: 枪口指向圆点(复用 steer1 槽) */
#define ui_g_01_limit0 ((ui_interface_arc_t*)&(ui_g_now_figures[18]))     /* 圆弧: 刻度线0(pitch 上限) */
#define ui_g_4_wheel1 ((ui_interface_number_t*)&(ui_g_now_figures[21]))   /* 数字: 轮1 离线状态 */
#define ui_g_4_wheel2 ((ui_interface_number_t*)&(ui_g_now_figures[22]))   /* 数字: 轮2 离线状态 */
#define ui_g_4_wheel3 ((ui_interface_number_t*)&(ui_g_now_figures[23]))   /* 数字: 轮3 离线状态 */
#define ui_g_4_wheel4 ((ui_interface_number_t*)&(ui_g_now_figures[24]))   /* 数字: 轮4 离线状态 */
#define ui_g_2_total_power ((ui_interface_number_t*)&(ui_g_now_figures[25]))/* 数字: 底盘总功率数值 */
#define ui_g_2_currentheart ((ui_interface_number_t*)&(ui_g_now_figures[26]))/* 数字: 当前热量数值 */
#define ui_g_00_line4 ((ui_interface_line_t*)&(ui_g_now_figures[27]))     /* 直线4: 左车道线 */
#define ui_g_00_line5 ((ui_interface_line_t*)&(ui_g_now_figures[28]))     /* 直线5: 右车道线 */
#define ui_g_01_limit1 ((ui_interface_arc_t*)&(ui_g_now_figures[29]))     /* 圆弧: 刻度线1(电容上限) */
#define ui_g_00_line6 ((ui_interface_line_t*)&(ui_g_now_figures[30]))     /* 直线6: 顶部短横线 */
#define ui_g_01_limit2 ((ui_interface_arc_t*)&(ui_g_now_figures[31]))     /* 圆弧: 刻度线2(功率下限) */
#define ui_g_01_limit3 ((ui_interface_arc_t*)&(ui_g_now_figures[32]))     /* 圆弧: 刻度线3 */
#define ui_g_01_limit4 ((ui_interface_arc_t*)&(ui_g_now_figures[33]))     /* 圆弧: 刻度线4 */
#define ui_g_01_limit5 ((ui_interface_arc_t*)&(ui_g_now_figures[34]))     /* 圆弧: 刻度线5 */

/* ===== 字符串元素宏 ===== */
#define ui_g_02_pitch (&(ui_g_now_strings[0]))   /* 字符 "pitch" */
#define ui_g_02_cap (&(ui_g_now_strings[1]))     /* 字符 "cap" */
#define ui_g_02_power (&(ui_g_now_strings[2]))   /* 字符 "power" */
#define ui_g_02_heat (&(ui_g_now_strings[3]))    /* 字符 "heat" */
#define ui_g_02_mode (&(ui_g_now_strings[4]))    /* 字符 "mode" */
#define ui_g_02_shoot (&(ui_g_now_strings[5]))   /* 字符 "shoot" */
#define ui_g_02_vision (&(ui_g_now_strings[6]))  /* 字符 "vision" */
#define ui_g_3_wheel (&(ui_g_now_strings[7]))    /* 字符 "wheel" */
#define ui_g_3_steer (&(ui_g_now_strings[8]))    /* 字符 "steer"(已注释不用) */

/* ===== max_send_count 宏: 每个元素的最大重发次数 =====
 * 前 35 项对应图形，后 9 项对应字符串。当前全部为 1(只发一次)。 */
#define ui_g_00_line0_max_send_count (ui_g_max_send_count[0])
#define ui_g_00_line1_max_send_count (ui_g_max_send_count[1])
#define ui_g_1_bar0_max_send_count (ui_g_max_send_count[2])
#define ui_g_1_bar1_max_send_count (ui_g_max_send_count[3])
#define ui_g_1_bar2_max_send_count (ui_g_max_send_count[4])
#define ui_g_1_bar3_max_send_count (ui_g_max_send_count[5])
#define ui_g_00_line2_max_send_count (ui_g_max_send_count[6])
#define ui_g_1_detect_max_send_count (ui_g_max_send_count[7])
#define ui_g_2_pitchangle_max_send_count (ui_g_max_send_count[8])
#define ui_g_2_capa_voltage_max_send_count (ui_g_max_send_count[9])
#define ui_g_2_heartlimit_max_send_count (ui_g_max_send_count[10])
#define ui_g_00_line3_max_send_count (ui_g_max_send_count[11])
#define ui_g_3_shootmode_max_send_count (ui_g_max_send_count[12])
#define ui_g_3_robotmode_max_send_count (ui_g_max_send_count[13])
#define ui_g_3_visionmode_max_send_count (ui_g_max_send_count[14])
#define ui_g_00_visionrange_max_send_count (ui_g_max_send_count[15])
#define ui_g_4_steer1_max_send_count (ui_g_max_send_count[16])
#define ui_g_4_steer2_max_send_count (ui_g_max_send_count[17])
#define ui_g_01_limit0_max_send_count (ui_g_max_send_count[18])
#define ui_g_4_steer3_max_send_count (ui_g_max_send_count[19])
#define ui_g_4_steer4_max_send_count (ui_g_max_send_count[20])
#define ui_g_4_wheel1_max_send_count (ui_g_max_send_count[21])
#define ui_g_4_wheel2_max_send_count (ui_g_max_send_count[22])
#define ui_g_4_wheel3_max_send_count (ui_g_max_send_count[23])
#define ui_g_4_wheel4_max_send_count (ui_g_max_send_count[24])
#define ui_g_2_total_power_max_send_count (ui_g_max_send_count[25])
#define ui_g_2_currentheart_max_send_count (ui_g_max_send_count[26])
#define ui_g_00_line4_max_send_count (ui_g_max_send_count[27])
#define ui_g_00_line5_max_send_count (ui_g_max_send_count[28])
#define ui_g_01_limit1_max_send_count (ui_g_max_send_count[29])
#define ui_g_00_line6_max_send_count (ui_g_max_send_count[30])
#define ui_g_01_limit2_max_send_count (ui_g_max_send_count[31])
#define ui_g_01_limit3_max_send_count (ui_g_max_send_count[32])
#define ui_g_01_limit4_max_send_count (ui_g_max_send_count[33])
#define ui_g_01_limit5_max_send_count (ui_g_max_send_count[34])

#define ui_g_02_pitch_max_send_count (ui_g_max_send_count[35])
#define ui_g_02_cap_max_send_count (ui_g_max_send_count[36])
#define ui_g_02_power_max_send_count (ui_g_max_send_count[37])
#define ui_g_02_heat_max_send_count (ui_g_max_send_count[38])
#define ui_g_02_mode_max_send_count (ui_g_max_send_count[39])
#define ui_g_02_shoot_max_send_count (ui_g_max_send_count[40])
#define ui_g_02_vision_max_send_count (ui_g_max_send_count[41])
#define ui_g_3_wheel_max_send_count (ui_g_max_send_count[42])
#define ui_g_3_steer_max_send_count (ui_g_max_send_count[43])

/* ===== 若定义 MANUAL_DIRTY，才提供 *_dirty 脏标记宏 =====
 * 默认未定义 MANUAL_DIRTY，脏检测由 ui_g.c 里的 memcmp 自动完成；
 * 定义后则可手动写 *_dirty 强制某元素重发。 */
#ifdef MANUAL_DIRTY
#define ui_g_00_line0_dirty (ui_g_dirty_figure[0])
#define ui_g_00_line1_dirty (ui_g_dirty_figure[1])
#define ui_g_1_bar0_dirty (ui_g_dirty_figure[2])
#define ui_g_1_bar1_dirty (ui_g_dirty_figure[3])
#define ui_g_1_bar2_dirty (ui_g_dirty_figure[4])
#define ui_g_1_bar3_dirty (ui_g_dirty_figure[5])
#define ui_g_00_line2_dirty (ui_g_dirty_figure[6])
#define ui_g_1_detect_dirty (ui_g_dirty_figure[7])
#define ui_g_2_pitchangle_dirty (ui_g_dirty_figure[8])
#define ui_g_2_capa_voltage_dirty (ui_g_dirty_figure[9])
#define ui_g_2_heartlimit_dirty (ui_g_dirty_figure[10])
#define ui_g_00_line3_dirty (ui_g_dirty_figure[11])
#define ui_g_3_shootmode_dirty (ui_g_dirty_figure[12])
#define ui_g_3_robotmode_dirty (ui_g_dirty_figure[13])
#define ui_g_3_visionmode_dirty (ui_g_dirty_figure[14])
#define ui_g_00_visionrange_dirty (ui_g_dirty_figure[15])
#define ui_g_4_steer1_dirty (ui_g_dirty_figure[16])
#define ui_g_4_steer2_dirty (ui_g_dirty_figure[17])
#define ui_g_01_limit0_dirty (ui_g_dirty_figure[18])
#define ui_g_4_steer3_dirty (ui_g_dirty_figure[19])
#define ui_g_4_steer4_dirty (ui_g_dirty_figure[20])
#define ui_g_4_wheel1_dirty (ui_g_dirty_figure[21])
#define ui_g_4_wheel2_dirty (ui_g_dirty_figure[22])
#define ui_g_4_wheel3_dirty (ui_g_dirty_figure[23])
#define ui_g_4_wheel4_dirty (ui_g_dirty_figure[24])
#define ui_g_2_total_power_dirty (ui_g_dirty_figure[25])
#define ui_g_2_currentheart_dirty (ui_g_dirty_figure[26])
#define ui_g_00_line4_dirty (ui_g_dirty_figure[27])
#define ui_g_00_line5_dirty (ui_g_dirty_figure[28])
#define ui_g_01_limit1_dirty (ui_g_dirty_figure[29])
#define ui_g_00_line6_dirty (ui_g_dirty_figure[30])
#define ui_g_01_limit2_dirty (ui_g_dirty_figure[31])
#define ui_g_01_limit3_dirty (ui_g_dirty_figure[32])
#define ui_g_01_limit4_dirty (ui_g_dirty_figure[33])
#define ui_g_01_limit5_dirty (ui_g_dirty_figure[34])

#define ui_g_02_pitch_dirty (ui_g_dirty_string[0])
#define ui_g_02_cap_dirty (ui_g_dirty_string[1])
#define ui_g_02_power_dirty (ui_g_dirty_string[2])
#define ui_g_02_heat_dirty (ui_g_dirty_string[3])
#define ui_g_02_mode_dirty (ui_g_dirty_string[4])
#define ui_g_02_shoot_dirty (ui_g_dirty_string[5])
#define ui_g_02_vision_dirty (ui_g_dirty_string[6])
#define ui_g_3_wheel_dirty (ui_g_dirty_string[7])
#define ui_g_3_steer_dirty (ui_g_dirty_string[8])
#endif

/* 初始化所有图形/字符串的默认内容并首次全量发送(实现在 ui_g.c) */
void ui_init_g();
/* 检测图形/字符串是否变化，变化则标记脏并调用 ui_scan_and_send 发送(实现在 ui_g.c) */
void ui_update_g();

#endif // UI_g_H
