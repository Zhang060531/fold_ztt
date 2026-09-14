/* ===== 文件头 =====
 * 文件名  : ui_g.c
 * 作用    : 本工程所有 UI 图形/字符串的"默认内容初始化"与"变化检测发送"。
 *           骨架由 RM UI Designer 生成，这里人工填写了各图形元素的坐标/颜色/大小。
 * 结构    :
 *   1. 数据数组: ui_g_now_figures[35] / ui_g_now_strings[9] 及脏标记、
 *      max_send_count 重发次数、last_* 上次快照
 *   2. ui_init_g(): 给每个图形/字符串赋初值，全量发送一次(operate_type=1 添加)
 *   3. ui_update_g(): memcmp 检测变化 -> 标记脏 -> ui_scan_and_send 发送
 * 调用关系:
 *   UI_Task.c -> ui_init_g()(任务启动时) / ui_update_g()(周期刷新)
 *   ui_interface.c(ui_scan_and_send) <- 被 ui_init_g/ui_update_g 调用
 * ===== 调车关注 =====
 *   - 各图形坐标是 1920x1080 客户端像素坐标(原点左上角)，屏幕中心 (960,540)。
 *   - 四个圆弧进度条(bar0~bar3)绕中心 (960,540)、半径 387 排布:
 *       bar0 左上 275°~314°(云台 pitch)   bar1 左下 226°~266°(电容电压)
 *       bar2 右下  94°~133°(底盘功率)     bar3 右上  46°~ 85°(42mm 热量)
 *     角度约定(本工程由 RM UI Designer 生成): 0°=正上方，顺时针为正
 *     (90°=正右 180°=正下 270°=正左)，与实际屏幕显示有出入时以显示为准。
 *   - 想改某个图形的位置/颜色/大小，改下面 ui_init_g 里的初值即可。
 *   - 动态数值(血条角度/数字/颜色)由 UI_Task.c 的 UI_Data_Calculate 周期更新。
 */
//
// Created by RM UI Designer
// Dynamic Edition
//

#include "string.h"
#include "ui_interface.h"
#include "ui_g.h"

#define TOTAL_FIGURE 35   /* 图形总数(35 条) */
#define TOTAL_STRING 9    /* 字符串总数(9 条) */

/* 当前图形内容(35 条)、图形脏标记、当前字符串内容(9 条)、字符串脏标记 */
ui_interface_figure_t ui_g_now_figures[TOTAL_FIGURE];
uint8_t ui_g_dirty_figure[TOTAL_FIGURE];
ui_interface_string_t ui_g_now_strings[TOTAL_STRING];
uint8_t ui_g_dirty_string[TOTAL_STRING];

/* 每条图形/字符串的最大重发次数，当前全部为 1(只发一次)。
 * 调大某元素对应值可提高其送达可靠性(多重复发送)。 */
uint8_t ui_g_max_send_count[TOTAL_FIGURE + TOTAL_STRING] = {
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
};

/* 上次发送时的快照(未定义 MANUAL_DIRTY 时启用自动脏检测，用于 memcmp 比较) */
#ifndef MANUAL_DIRTY
ui_interface_figure_t ui_g_last_figures[TOTAL_FIGURE];
ui_interface_string_t ui_g_last_strings[TOTAL_STRING];
#endif

/* 简化调用: 用本文件的数组去调用 ui_interface.c 里的 ui_scan_and_send */
#define SCAN_AND_SEND() ui_scan_and_send(ui_g_now_figures, ui_g_dirty_figure, ui_g_now_strings, ui_g_dirty_string, TOTAL_FIGURE, TOTAL_STRING)

/**
 * @brief 初始化所有图形/字符串的默认内容，并全量发送一次到客户端
 * @note 发送完成后把所有元素的 operate_type 由 1(添加) 改成 2(修改)，
 *       后续周期只需"修改"即可刷新，不必每次重建。
 */
void ui_init_g() {
    /* 直线0: 中轴线(竖)，从屏幕中心(960,624)画到(960,900)，灰色，线宽2 */
    ui_g_00_line0->figure_type = 0;
    ui_g_00_line0->operate_type = 1;
    ui_g_00_line0->layer = 0;
    ui_g_00_line0->color = 8;
    ui_g_00_line0->start_x = 960;
    ui_g_00_line0->start_y = 624;
    ui_g_00_line0->width = 2;
    ui_g_00_line0->end_x = 960;
    ui_g_00_line0->end_y = 900;

    /* 直线1: 上方横线，(868,540)→(637,540)，灰色，线宽2 */
    ui_g_00_line1->figure_type = 0;
    ui_g_00_line1->operate_type = 1;
    ui_g_00_line1->layer = 0;
    ui_g_00_line1->color = 8;
    ui_g_00_line1->start_x = 868;
    ui_g_00_line1->start_y = 540;
    ui_g_00_line1->width = 2;
    ui_g_00_line1->end_x = 637;
    ui_g_00_line1->end_y = 540;

    /* 圆弧0(左上): 云台 pitch 血条，粉色，中心(960,540) 半径387 角度275°~314° 线宽14 */
    ui_g_1_bar0->figure_type = 4;
    ui_g_1_bar0->operate_type = 1;
    ui_g_1_bar0->layer = 1;
    ui_g_1_bar0->color = 1;
    ui_g_1_bar0->start_x = 960;
    ui_g_1_bar0->start_y = 540;
    ui_g_1_bar0->width = 14;
    ui_g_1_bar0->start_angle = 275;
    ui_g_1_bar0->end_angle = 314;
    ui_g_1_bar0->rx = 387;
    ui_g_1_bar0->ry = 387;

    /* 圆弧1(左下): 电容电压血条，黄色，中心(960,540) 半径387 角度226°~266° 线宽14 */
    ui_g_1_bar1->figure_type = 4;
    ui_g_1_bar1->operate_type = 1;
    ui_g_1_bar1->layer = 1;
    ui_g_1_bar1->color = 6;
    ui_g_1_bar1->start_x = 960;
    ui_g_1_bar1->start_y = 540;
    ui_g_1_bar1->width = 14;
    ui_g_1_bar1->start_angle = 226;
    ui_g_1_bar1->end_angle = 266;
    ui_g_1_bar1->rx = 387;
    ui_g_1_bar1->ry = 387;

    /* 圆弧2(右下): 底盘功率血条，黄色，中心(960,540) 半径387 角度94°~133° 线宽14 */
    ui_g_1_bar2->figure_type = 4;
    ui_g_1_bar2->operate_type = 1;
    ui_g_1_bar2->layer = 1;
    ui_g_1_bar2->color = 6;
    ui_g_1_bar2->start_x = 960;
    ui_g_1_bar2->start_y = 540;
    ui_g_1_bar2->width = 14;
    ui_g_1_bar2->start_angle = 94;
    ui_g_1_bar2->end_angle = 133;
    ui_g_1_bar2->rx = 387;
    ui_g_1_bar2->ry = 387;

    /* 圆弧3(右上): 42mm 枪管热量血条，橙色，中心(960,540) 半径387 角度46°~85° 线宽14 */
    ui_g_1_bar3->figure_type = 4;
    ui_g_1_bar3->operate_type = 1;
    ui_g_1_bar3->layer = 1;
    ui_g_1_bar3->color = 3;
    ui_g_1_bar3->start_x = 960;
    ui_g_1_bar3->start_y = 540;
    ui_g_1_bar3->width = 14;
    ui_g_1_bar3->start_angle = 46;
    ui_g_1_bar3->end_angle = 85;
    ui_g_1_bar3->rx = 387;
    ui_g_1_bar3->ry = 387;

    /* 直线2: 中轴线(上段)，(960,459)→(960,117)，灰色，线宽2 */
    ui_g_00_line2->figure_type = 0;
    ui_g_00_line2->operate_type = 1;
    ui_g_00_line2->layer = 0;
    ui_g_00_line2->color = 8;
    ui_g_00_line2->start_x = 960;
    ui_g_00_line2->start_y = 459;
    ui_g_00_line2->width = 2;
    ui_g_00_line2->end_x = 960;
    ui_g_00_line2->end_y = 117;

    /* 圆: 视觉检测状态指示点，绿色，中心(960,540) 半径 r=21 */
    ui_g_1_detect->figure_type = 2;
    ui_g_1_detect->operate_type = 1;
    ui_g_1_detect->layer = 1;
    ui_g_1_detect->color = 2;
    ui_g_1_detect->start_x = 960;
    ui_g_1_detect->start_y = 540;
    ui_g_1_detect->width = 2;
    ui_g_1_detect->r = 21;

    /* 数字(浮点): pitch 角度数值，灰色，字号20，位置(610,568) */
    ui_g_2_pitchangle->figure_type = 5;
    ui_g_2_pitchangle->operate_type = 1;
    ui_g_2_pitchangle->layer = 2;
    ui_g_2_pitchangle->color = 8;
    ui_g_2_pitchangle->start_x = 610;
    ui_g_2_pitchangle->start_y = 568;
    ui_g_2_pitchangle->width = 2;
    ui_g_2_pitchangle->font_size = 20;
    ui_g_2_pitchangle->number = 12345;

    /* 数字(浮点): 电容电压数值，灰色，字号20，位置(610,520) */
    ui_g_2_capa_voltage->figure_type = 5;
    ui_g_2_capa_voltage->operate_type = 1;
    ui_g_2_capa_voltage->layer = 2;
    ui_g_2_capa_voltage->color = 8;
    ui_g_2_capa_voltage->start_x = 610;
    ui_g_2_capa_voltage->start_y = 520;
    ui_g_2_capa_voltage->width = 2;
    ui_g_2_capa_voltage->font_size = 20;
    ui_g_2_capa_voltage->number = 12345;

    /* 圆弧: 热量限制刻度线，灰色，线宽40，半径370，角度68°~69° */
    ui_g_2_heartlimit->figure_type = 4;
    ui_g_2_heartlimit->operate_type = 1;
    ui_g_2_heartlimit->layer = 1;
    ui_g_2_heartlimit->color = 8;
    ui_g_2_heartlimit->start_x = 960;
    ui_g_2_heartlimit->start_y = 540;
    ui_g_2_heartlimit->width = 40;
    ui_g_2_heartlimit->start_angle = 68;
    ui_g_2_heartlimit->end_angle = 69;
    ui_g_2_heartlimit->rx = 370;
    ui_g_2_heartlimit->ry = 370;

    /* 直线3: 右侧横线，(1053,540)→(1284,540)，灰色，线宽2 */
    ui_g_00_line3->figure_type = 0;
    ui_g_00_line3->operate_type = 1;
    ui_g_00_line3->layer = 0;
    ui_g_00_line3->color = 8;
    ui_g_00_line3->start_x = 1053;
    ui_g_00_line3->start_y = 540;
    ui_g_00_line3->width = 2;
    ui_g_00_line3->end_x = 1284;
    ui_g_00_line3->end_y = 540;

    /* 数字(整数): 发射模式指示(0停/1准备/2发射)，橙色，字号35，位置(350,630) */
    ui_g_3_shootmode->figure_type = 6;
    ui_g_3_shootmode->operate_type = 1;
    ui_g_3_shootmode->layer = 2;
    ui_g_3_shootmode->color = 3;
    ui_g_3_shootmode->start_x = 350;
    ui_g_3_shootmode->start_y = 630;
    ui_g_3_shootmode->width = 4;
    ui_g_3_shootmode->font_size = 35;
    ui_g_3_shootmode->number = 1;

    /* 数字(整数): 底盘模式指示(0无力/1跟随/2小陀螺/3无云底跟随)，橙色，字号35，位置(350,560) */
    ui_g_3_robotmode->figure_type = 6;
    ui_g_3_robotmode->operate_type = 1;
    ui_g_3_robotmode->layer = 2;
    ui_g_3_robotmode->color = 3;
    ui_g_3_robotmode->start_x = 350;
    ui_g_3_robotmode->start_y = 560;
    ui_g_3_robotmode->width = 4;
    ui_g_3_robotmode->font_size = 35;
    ui_g_3_robotmode->number = 0;

    /* 数字(整数): 视觉模式指示(0全无/1有视觉/2有发射标志)，橙色，字号35，位置(350,686) */
    ui_g_3_visionmode->figure_type = 6;
    ui_g_3_visionmode->operate_type = 1;
    ui_g_3_visionmode->layer = 2;
    ui_g_3_visionmode->color = 3;
    ui_g_3_visionmode->start_x = 350;
    ui_g_3_visionmode->start_y = 686;
    ui_g_3_visionmode->width = 4;
    ui_g_3_visionmode->font_size = 35;
    ui_g_3_visionmode->number = 2;

    /* 矩形: 视觉范围框，灰色，对角顶点(700,715)与(1148,390)，线宽3 */
    ui_g_00_visionrange->figure_type = 1;             //////////////////////////////////////////////////
    ui_g_00_visionrange->operate_type = 1;
    ui_g_00_visionrange->layer = 1;
    ui_g_00_visionrange->color = 8;
    ui_g_00_visionrange->start_x = 700;
    ui_g_00_visionrange->start_y = 715;   //上
    ui_g_00_visionrange->width = 3;
    ui_g_00_visionrange->end_x = 1148;
    ui_g_00_visionrange->end_y = 390;    //下

    /* 注: 以下 ui_g_4_steer1~steer4 为"舵轮"状态数字，原设计已注释弃用，
     *     对应数组槽位 figures[16~20] 部分被复用(16 复用为枪口圆点)。 */
//    ui_g_4_steer1->figure_type = 6;
//    ui_g_4_steer1->operate_type = 1;
//    ui_g_4_steer1->layer = 2;
//    ui_g_4_steer1->color = 2;
//    ui_g_4_steer1->start_x = 1670;
//    ui_g_4_steer1->start_y = 570;
//    ui_g_4_steer1->width = 4;
//    ui_g_4_steer1->font_size = 35;
//    ui_g_4_steer1->number = 1;
    /* 圆: 枪口指向圆点，红色，位置(915,540)，半径 r=5，线宽6 */
    ui_interface_round_t *vdot = (ui_interface_round_t *)&ui_g_now_figures[16];
    vdot->figure_type = 2;       // 2 = 圆形
    vdot->operate_type = 1;
    vdot->layer = 2;
    vdot->color = 0;             // red
    vdot->start_x = 915;         // 水平位置
    vdot->start_y = 540;
    vdot->width = 6;
    vdot->r = 5;                // 半径 10px

//    ui_g_4_steer2->figure_type = 6;
//    ui_g_4_steer2->operate_type = 1;
//    ui_g_4_steer2->layer = 2;
//    ui_g_4_steer2->color = 2;
//    ui_g_4_steer2->start_x = 1710;
//    ui_g_4_steer2->start_y = 570;
//    ui_g_4_steer2->width = 4;
//    ui_g_4_steer2->font_size = 35;
//    ui_g_4_steer2->number = 1;

    /* 圆弧: 刻度线0(pitch 上限)，灰色，线宽18，角度315°~317°，半径387 */
    ui_g_01_limit0->figure_type = 4;
    ui_g_01_limit0->operate_type = 1;
    ui_g_01_limit0->layer = 0;
    ui_g_01_limit0->color = 8;
    ui_g_01_limit0->start_x = 960;
    ui_g_01_limit0->start_y = 540;
    ui_g_01_limit0->width = 18;
    ui_g_01_limit0->start_angle = 315;
    ui_g_01_limit0->end_angle = 317;
    ui_g_01_limit0->rx = 387;
    ui_g_01_limit0->ry = 387;

 //   ui_g_4_steer3->figure_type = 6;
//    ui_g_4_steer3->operate_type = 1;
//    ui_g_4_steer3->layer = 2;
//    ui_g_4_steer3->color = 2;
//    ui_g_4_steer3->start_x = 1750;
//    ui_g_4_steer3->start_y = 570;
//    ui_g_4_steer3->width = 4;
//    ui_g_4_steer3->font_size = 35;
//    ui_g_4_steer3->number = 1;

//    ui_g_4_steer4->figure_type = 6;
//    ui_g_4_steer4->operate_type = 1;
//    ui_g_4_steer4->layer = 2;
//    ui_g_4_steer4->color = 2;
//    ui_g_4_steer4->start_x = 1790;
//    ui_g_4_steer4->start_y = 570;
//    ui_g_4_steer4->width = 4;
//    ui_g_4_steer4->font_size = 35;
//    ui_g_4_steer4->number = 1;

    /* 数字(整数): 轮1 离线状态(0在线/1离线)，橙色，字号35，位置(1670,520) */
    ui_g_4_wheel1->figure_type = 6;
    ui_g_4_wheel1->operate_type = 1;
    ui_g_4_wheel1->layer = 2;
    ui_g_4_wheel1->color = 3;
    ui_g_4_wheel1->start_x = 1670;
    ui_g_4_wheel1->start_y = 520;
    ui_g_4_wheel1->width = 4;
    ui_g_4_wheel1->font_size = 35;
    ui_g_4_wheel1->number = 0;

    /* 数字(整数): 轮2 离线状态，橙色，字号35，位置(1710,520) */
    ui_g_4_wheel2->figure_type = 6;
    ui_g_4_wheel2->operate_type = 1;
    ui_g_4_wheel2->layer = 2;
    ui_g_4_wheel2->color = 3;
    ui_g_4_wheel2->start_x = 1710;
    ui_g_4_wheel2->start_y = 520;
    ui_g_4_wheel2->width = 4;
    ui_g_4_wheel2->font_size = 35;
    ui_g_4_wheel2->number = 0;

    /* 数字(整数): 轮3 离线状态，橙色，字号35，位置(1750,520) */
    ui_g_4_wheel3->figure_type = 6;
    ui_g_4_wheel3->operate_type = 1;
    ui_g_4_wheel3->layer = 2;
    ui_g_4_wheel3->color = 3;
    ui_g_4_wheel3->start_x = 1750;
    ui_g_4_wheel3->start_y = 520;
    ui_g_4_wheel3->width = 4;
    ui_g_4_wheel3->font_size = 35;
    ui_g_4_wheel3->number = 0;

    /* 数字(整数): 轮4 离线状态，橙色，字号35，位置(1790,520) */
    ui_g_4_wheel4->figure_type = 6;
    ui_g_4_wheel4->operate_type = 1;
    ui_g_4_wheel4->layer = 2;
    ui_g_4_wheel4->color = 3;
    ui_g_4_wheel4->start_x = 1790;
    ui_g_4_wheel4->start_y = 520;
    ui_g_4_wheel4->width = 4;
    ui_g_4_wheel4->font_size = 35;
    ui_g_4_wheel4->number = 0;

    /* 数字(浮点): 底盘总功率数值，灰色，字号20，位置(1221,530) */
    ui_g_2_total_power->figure_type = 5;
    ui_g_2_total_power->operate_type = 1;
    ui_g_2_total_power->layer = 2;
    ui_g_2_total_power->color = 8;
    ui_g_2_total_power->start_x = 1221;
    ui_g_2_total_power->start_y = 530;
    ui_g_2_total_power->width = 2;
    ui_g_2_total_power->font_size = 20;
    ui_g_2_total_power->number = 12345;

    /* 数字(整数): 当前枪管热量数值，灰色，字号20，位置(1221,568) */
    ui_g_2_currentheart->figure_type = 6;
    ui_g_2_currentheart->operate_type = 1;
    ui_g_2_currentheart->layer = 2;
    ui_g_2_currentheart->color = 8;
    ui_g_2_currentheart->start_x = 1221;
    ui_g_2_currentheart->start_y = 568;
    ui_g_2_currentheart->width = 2;
    ui_g_2_currentheart->font_size = 20;
    ui_g_2_currentheart->number = 12345;

    /* 直线4: 左车道线，红色，从(48,0)画到(715,400)，线宽4 */
    ui_g_00_line4->figure_type = 0;                 //左车道线
    ui_g_00_line4->operate_type = 1;
    ui_g_00_line4->layer = 0;
    ui_g_00_line4->color = 0;
    ui_g_00_line4->start_x = 48	;     //左下
    ui_g_00_line4->start_y = 0;
    ui_g_00_line4->width = 4;
    ui_g_00_line4->end_x = 715;
    ui_g_00_line4->end_y = 400;

    /* 直线5: 右车道线，红色，从(1696,0)画到(1358,260)，线宽4 */
    ui_g_00_line5->figure_type = 0;
    ui_g_00_line5->operate_type = 1;
    ui_g_00_line5->layer = 0;
    ui_g_00_line5->color = 0;
    ui_g_00_line5->start_x = 1696;                   //右下车道线
    ui_g_00_line5->start_y = 0;
    ui_g_00_line5->width = 4;
    ui_g_00_line5->end_x = 1358;                     //右上车道线
    ui_g_00_line5->end_y = 260;

    /* 圆弧: 刻度线1(电容电压上限)，灰色，线宽18，角度267°~273°，半径387 */
    ui_g_01_limit1->figure_type = 4;
    ui_g_01_limit1->operate_type = 1;
    ui_g_01_limit1->layer = 0;
    ui_g_01_limit1->color = 8;
    ui_g_01_limit1->start_x = 960;
    ui_g_01_limit1->start_y = 540;
    ui_g_01_limit1->width = 18;
    ui_g_01_limit1->start_angle = 267;
    ui_g_01_limit1->end_angle = 273;
    ui_g_01_limit1->rx = 387;
    ui_g_01_limit1->ry = 387;

    /* 直线6: 顶部短横线，灰色，(920,100)→(1000,100)，线宽5 */
    ui_g_00_line6->figure_type = 0;
    ui_g_00_line6->operate_type = 1;
    ui_g_00_line6->layer = 0;
    ui_g_00_line6->color = 8;
    ui_g_00_line6->start_x = 920;
    ui_g_00_line6->start_y = 100;
    ui_g_00_line6->width = 5;
    ui_g_00_line6->end_x = 1000;
    ui_g_00_line6->end_y = 100;

    /* 圆弧: 刻度线2(功率下限)，灰色，线宽18，角度223°~225°，半径387 */
    ui_g_01_limit2->figure_type = 4;
    ui_g_01_limit2->operate_type = 1;
    ui_g_01_limit2->layer = 0;
    ui_g_01_limit2->color = 8;
    ui_g_01_limit2->start_x = 960;
    ui_g_01_limit2->start_y = 540;
    ui_g_01_limit2->width = 18;
    ui_g_01_limit2->start_angle = 223;
    ui_g_01_limit2->end_angle = 225;
    ui_g_01_limit2->rx = 387;
    ui_g_01_limit2->ry = 387;

    /* 圆弧: 刻度线3，灰色，线宽18，角度135°~137°，半径387 */
    ui_g_01_limit3->figure_type = 4;
    ui_g_01_limit3->operate_type = 1;
    ui_g_01_limit3->layer = 0;
    ui_g_01_limit3->color = 8;
    ui_g_01_limit3->start_x = 960;
    ui_g_01_limit3->start_y = 540;
    ui_g_01_limit3->width = 18;
    ui_g_01_limit3->start_angle = 135;
    ui_g_01_limit3->end_angle = 137;
    ui_g_01_limit3->rx = 387;
    ui_g_01_limit3->ry = 387;

    /* 圆弧: 刻度线4，灰色，线宽18，角度87°~93°，半径387 */
    ui_g_01_limit4->figure_type = 4;
    ui_g_01_limit4->operate_type = 1;
    ui_g_01_limit4->layer = 0;
    ui_g_01_limit4->color = 8;
    ui_g_01_limit4->start_x = 960;
    ui_g_01_limit4->start_y = 540;
    ui_g_01_limit4->width = 18;
    ui_g_01_limit4->start_angle = 87;
    ui_g_01_limit4->end_angle = 93;
    ui_g_01_limit4->rx = 387;
    ui_g_01_limit4->ry = 387;

    /* 圆弧: 刻度线5(热量附近)，灰色，线宽18，角度43°~45°，半径387 */
    ui_g_01_limit5->figure_type = 4;
    ui_g_01_limit5->operate_type = 1;
    ui_g_01_limit5->layer = 0;
    ui_g_01_limit5->color = 8;
    ui_g_01_limit5->start_x = 960;
    ui_g_01_limit5->start_y = 540;
    ui_g_01_limit5->width = 18;
    ui_g_01_limit5->start_angle = 43;
    ui_g_01_limit5->end_angle = 45;
    ui_g_01_limit5->rx = 387;
    ui_g_01_limit5->ry = 387;

    /* 字符 "pitch": 云台 pitch 标签，灰色，字号20，位置(728,843) */
    ui_g_02_pitch->figure_type = 7;
    ui_g_02_pitch->operate_type = 1;
    ui_g_02_pitch->layer = 0;
    ui_g_02_pitch->color = 8;
    ui_g_02_pitch->start_x = 728;
    ui_g_02_pitch->start_y = 843;
    ui_g_02_pitch->width = 3;
    ui_g_02_pitch->font_size = 20;
    ui_g_02_pitch->str_length = 5;
    strcpy(ui_g_02_pitch->string, "pitch");

    /* 字符 "cap": 电容电压标签，灰色，字号20，位置(720,257) */
    ui_g_02_cap->figure_type = 7;
    ui_g_02_cap->operate_type = 1;
    ui_g_02_cap->layer = 0;
    ui_g_02_cap->color = 8;
    ui_g_02_cap->start_x = 720;
    ui_g_02_cap->start_y = 257;
    ui_g_02_cap->width = 3;
    ui_g_02_cap->font_size = 20;
    ui_g_02_cap->str_length = 3;
    strcpy(ui_g_02_cap->string, "cap");

    /* 字符 "power": 功率标签，灰色，字号20，位置(1123,257) */
    ui_g_02_power->figure_type = 7;
    ui_g_02_power->operate_type = 1;
    ui_g_02_power->layer = 0;
    ui_g_02_power->color = 8;
    ui_g_02_power->start_x = 1123;
    ui_g_02_power->start_y = 257;
    ui_g_02_power->width = 3;
    ui_g_02_power->font_size = 20;
    ui_g_02_power->str_length = 5;
    strcpy(ui_g_02_power->string, "power");

    /* 字符 "heat": 热量标签，灰色，字号20，位置(1132,851) */
    ui_g_02_heat->figure_type = 7;
    ui_g_02_heat->operate_type = 1;
    ui_g_02_heat->layer = 0;
    ui_g_02_heat->color = 8;
    ui_g_02_heat->start_x = 1132;
    ui_g_02_heat->start_y = 851;
    ui_g_02_heat->width = 3;
    ui_g_02_heat->font_size = 20;
    ui_g_02_heat->str_length = 4;
    strcpy(ui_g_02_heat->string, "heat");

    /* 字符 "mode": 底盘模式标签，红色，字号35，位置(50,560) */
    ui_g_02_mode->figure_type = 7;
    ui_g_02_mode->operate_type = 1;
    ui_g_02_mode->layer = 1;
    ui_g_02_mode->color = 0;
    ui_g_02_mode->start_x = 50;
    ui_g_02_mode->start_y = 560;
    ui_g_02_mode->width = 5;
    ui_g_02_mode->font_size = 35;
    ui_g_02_mode->str_length = 4;
    strcpy(ui_g_02_mode->string, "mode");

    /* 字符 "shoot": 发射模式标签，红色，字号35，位置(50,630) */
    ui_g_02_shoot->figure_type = 7;
    ui_g_02_shoot->operate_type = 1;
    ui_g_02_shoot->layer = 1;
    ui_g_02_shoot->color = 0;
    ui_g_02_shoot->start_x = 50;
    ui_g_02_shoot->start_y = 630;
    ui_g_02_shoot->width = 5;
    ui_g_02_shoot->font_size = 35;
    ui_g_02_shoot->str_length = 5;
    strcpy(ui_g_02_shoot->string, "shoot");

    /* 字符 "vision": 视觉模式标签，红色，字号35，位置(50,686) */
    ui_g_02_vision->figure_type = 7;
    ui_g_02_vision->operate_type = 1;
    ui_g_02_vision->layer = 1;
    ui_g_02_vision->color = 0;
    ui_g_02_vision->start_x = 50;
    ui_g_02_vision->start_y = 686;
    ui_g_02_vision->width = 5;
    ui_g_02_vision->font_size = 35;
    ui_g_02_vision->str_length = 6;
    strcpy(ui_g_02_vision->string, "vision");

    /* 字符 "wheel": 轮子状态标签，红色，字号35，位置(1470,520) */
    ui_g_3_wheel->figure_type = 7;
    ui_g_3_wheel->operate_type = 1;
    ui_g_3_wheel->layer = 1;
    ui_g_3_wheel->color = 0;
    ui_g_3_wheel->start_x = 1470;
    ui_g_3_wheel->start_y = 520;
    ui_g_3_wheel->width = 5;
    ui_g_3_wheel->font_size = 35;
    ui_g_3_wheel->str_length = 5;
    strcpy(ui_g_3_wheel->string, "wheel");

//    ui_g_3_steer->figure_type = 7;
//    ui_g_3_steer->operate_type = 1;
//    ui_g_3_steer->layer = 1;
//    ui_g_3_steer->color = 0;
//    ui_g_3_steer->start_x = 1470;
//    ui_g_3_steer->start_y = 570;
//    ui_g_3_steer->width = 5;
//    ui_g_3_steer->font_size = 35;
//    ui_g_3_steer->str_length = 5;
//    strcpy(ui_g_3_steer->string, "steer");

    /* 给每条图形分配唯一的 figure_name(3 字节序号，大端存放)，
     * 并置 operate_type=1(添加)、记快照、标记脏(首次全量发送)。 */
    uint32_t idx = 0;
    for (int i = 0; i < TOTAL_FIGURE; i++) {
        ui_g_now_figures[i].figure_name[2] = idx & 0xFF;
        ui_g_now_figures[i].figure_name[1] = (idx >> 8) & 0xFF;
        ui_g_now_figures[i].figure_name[0] = (idx >> 16) & 0xFF;
        ui_g_now_figures[i].operate_type = 1;
#ifndef MANUAL_DIRTY
        ui_g_last_figures[i] = ui_g_now_figures[i];
#endif
        ui_g_dirty_figure[i] = 1;
        idx++;
    }
    /* 同样给 9 条字符串分配 figure_name(序号接在图形之后，继续 idx++) */
    for (int i = 0; i < TOTAL_STRING; i++) {
        ui_g_now_strings[i].figure_name[2] = idx & 0xFF;
        ui_g_now_strings[i].figure_name[1] = (idx >> 8) & 0xFF;
        ui_g_now_strings[i].figure_name[0] = (idx >> 16) & 0xFF;
        ui_g_now_strings[i].operate_type = 1;
#ifndef MANUAL_DIRTY
        ui_g_last_strings[i] = ui_g_now_strings[i];
#endif
        ui_g_dirty_string[i] = 1;
        idx++;
    }

    SCAN_AND_SEND();                     /* 首次全量发送(添加全部图形/字符串) */

    /* 发送完成后把 operate_type 由 1(添加) 改为 2(修改)，
     * 后续 ui_update_g 周期刷新时只需"修改"即可，不必重建。 */
    for (int i = 0; i < TOTAL_FIGURE; i++) {
        ui_g_now_figures[i].operate_type = 2;
    }
    for (int i = 0; i < TOTAL_STRING; i++) {
        ui_g_now_strings[i].operate_type = 2;
    }
}

/**
 * @brief 周期刷新: 逐个 memcmp 比较图形/字符串与上次快照，有变化则标记脏并发送
 * @note 未定义 MANUAL_DIRTY 时才做 memcmp 自动检测；
 *       定义 MANUAL_DIRTY 则需用户手动设置 *_dirty 标记后调用本函数。
 */
void ui_update_g() {
#ifndef MANUAL_DIRTY
    for (int i = 0; i < TOTAL_FIGURE; i++) {
        if (memcmp(&ui_g_now_figures[i], &ui_g_last_figures[i], sizeof(ui_g_now_figures[i])) != 0) {
            ui_g_dirty_figure[i] = ui_g_max_send_count[i];
            ui_g_last_figures[i] = ui_g_now_figures[i];
        }
    }
    for (int i = 0; i < TOTAL_STRING; i++) {
        if (memcmp(&ui_g_now_strings[i], &ui_g_last_strings[i], sizeof(ui_g_now_strings[i])) != 0) {
            ui_g_dirty_string[i] = ui_g_max_send_count[TOTAL_FIGURE + i];
            ui_g_last_strings[i] = ui_g_now_strings[i];
        }
    }
#endif
    SCAN_AND_SEND();
}
