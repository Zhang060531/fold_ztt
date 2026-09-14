/* ===== 文件头 =====
 * 文件名  : ui.h
 * 作用    : RMUI 图形库的"总入口"头文件。用户代码只需 #include "ui.h"
 *           即可获得全部 UI 相关声明：图形结构(ui_interface.h)、
 *           本工程的图形/字符串宏定义(ui_g.h)。
 * 结构    :
 *   1. C++ 兼容保护 extern "C"
 *   2. 包含 ui_interface.h（图形数据类型 + 帧打包接口）
 *   3. 包含 ui_g.h（本工程各图形元素的宏名，如 ui_g_1_bar0 等）
 * 调用关系:
 *   UI_Task.c -> #include "ui.h"
 *   ui_g.c    -> 包含 ui_interface.h / ui_g.h
 * ===== 调车关注 =====
 *   本文件是纯聚合头文件，不含逻辑，无需改动。
 *   调车要操作的图形元素宏名都在 ui_g.h 中定义。
 */
//
// Created by RM UI Designer
// Dynamic Edition
//

#ifndef UI_H
#define UI_H
#ifdef __cplusplus
extern "C" {
#endif

#include "ui_interface.h" /* 图形结构体 + 帧打包/发送接口 */

#include "ui_g.h"         /* 本工程各图形元素宏名 + 初始化/更新接口 */

#ifdef __cplusplus
}
#endif

#endif // UI_H
