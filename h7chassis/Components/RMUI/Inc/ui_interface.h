/* ===== 文件头 =====
 * 文件名  : ui_interface.h
 * 作用    : RMUI 图形库的对外接口声明：发送宏、帧打包处理函数、
 *           "扫描脏图形并发送"的主入口，以及删除图层接口。
 * 结构    :
 *   1. 依赖包含: ui_types.h(结构体)、usart.h(UART)、cmsis_os.h(osDelay)
 *   2. extern 全局变量 ui_self_id
 *   3. print_message: 调试打印
 *   4. SEND_MESSAGE 宏: 通过串口 DMA 发送一帧并延时(限流)
 *   5. ui_set 宏: 修改图形字段并立即标记为脏(需重发)
 *   6. 各帧打包函数 ui_proc_*_frame / 删除图层 / 扫描发送
 * 调用关系:
 *   ui_interface.c -> 实现本文件声明的所有函数
 *   ui_g.c         -> 调用 ui_scan_and_send()
 *   UI_Task.c      -> 经 ui_g.h 间接使用
 * ===== 调车关注 =====
 *   - SEND_MESSAGE 里 huart1 是裁判系统串口，osDelay(33) 用于限流，
 *     避免一次性发太多帧导致裁判系统丢包。调大/调小该延时会影响刷新率。
 *   - ui_set 宏是"改字段 + 标记重发"的快捷方式，改图形内容时建议用它，
 *     否则需要手动设置对应的 *_dirty 标志。
 *   - 具体打包格式（帧头/SOF/CRC16）在 ui_types.h 与 ui_interface.c 中有详细说明。
 */
//
// Created by bismarckkk on 2025/3/22.
// Dynamic Edition
//

#ifndef UI_INTERFACE_H
#define UI_INTERFACE_H

#include "ui_types.h"
#include "usart.h"
#include "cmsis_os.h"

extern int ui_self_id; /* 自身(机器人)ID，本工程未实际参与打包(帧内 send_id 用机器人ID) */

void print_message(const uint8_t* message, int length); /* 以十六进制逐字节打印一帧，用于调试 */

// User Code Begin
/* 发送一帧数据: 通过串口1 DMA 发送，随后 osDelay(33) 限流。
 * 注意: 33ms 延时决定了整体 UI 刷新上限，调车时可酌情调整。 */
#define SEND_MESSAGE(message, length)  HAL_UART_Transmit_DMA(&huart1, message, length); osDelay(33);
// User Code End

/* 修改图形某个字段并立即把它标记为"脏"(需重发 max_send_count 次)。
 * 用法示例: ui_set(ui_g_1_bar0, color, 2);
 * 原理: 赋值后把 obj##_dirty 设为 obj##_max_send_count，等待 ui_scan_and_send 扫描发送。 */
#define ui_set(obj, field, value) { \
    obj->field = value; \
    obj##_dirty = obj##_max_send_count; \
}

/* 以下函数分别给 1/2/5/7 图形帧、字符帧、删除帧填充帧头/长度/CRC 等字段，实现在 ui_interface.c */
void ui_proc_1_frame(ui_1_frame_t *msg);
void ui_proc_2_frame(ui_2_frame_t *msg);
void ui_proc_5_frame(ui_5_frame_t *msg);
void ui_proc_7_frame(ui_7_frame_t *msg);
void ui_proc_string_frame(ui_string_frame_t *msg);
void ui_proc_delete_frame(ui_delete_frame_t *msg);

/* 删除图层: delete_type 为删除方式(0=删除全部/1=删除指定图层等，见裁判系统协议)，layer 为图层号 */
void ui_delete_layer(const uint8_t delete_type, const uint8_t layer);

/* RMUI 主入口: 扫描脏图形/脏字符串，按 7/5/2/1 打包成帧发送。
 * 每发送一次，对应 dirty 计数减 1，直到减到 0 不再发送(实现"重发几次保证到达")。 */
void ui_scan_and_send(const ui_interface_figure_t* ui_now_figures, uint8_t* ui_dirty_figure, const ui_interface_string_t* ui_now_strings, uint8_t* ui_dirty_string, int total_figures, int total_strings);

#endif //UI_INTERFACE_H
