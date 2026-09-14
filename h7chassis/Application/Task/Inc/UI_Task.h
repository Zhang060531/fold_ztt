/* ===== 文件头 =====
 * 文件名  : UI_Task.h
 * 作用    : UI 刷新任务的声明头文件。
 *           对应源文件 UI_Task.c 中的 UI_Task 线程（FreeRTOS 任务入口），
 *           周期调用 RMUI 图形库把底盘/云台状态画到裁判系统客户端（操作手屏幕）。
 * 结构    : 本文件保留 CubeMX 生成的空壳，内容极少：
 *           1. 头文件保护宏  #ifndef UI_TASK_H / #define UI_TASK_H
 *           2. stdint.h 包含（原工程标准写法）
 *           （注：UI_Task / UI_Data_Calculate 的函数声明未在本文件维护，
 *             任务入口由 FreeRTOS 创建线程处直接 extern 使用，故这里为空。）
 * 调用关系:
 *   UI_Task.c  -> 包含本文件（#include "UI_Task.h"）
 *   freertos.c -> 创建 UI_Task 线程（线程入口实现在 UI_Task.c）
 * ===== 调车关注 =====
 *   本文件基本无需改动。调车时关注的 UI 刷新周期、刷新元素
 *   （血条/功率/底盘状态/底盘跟随模式等）都在 UI_Task.c 与
 *   Components/RMUI 目录下维护，不在本文件。
 */
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : UI_Task.h
  * @brief          : UI task
  * @author         : Your Name
  * @date           : 2025/01/22
  * @version        : v1.0
  ******************************************************************************
  * @attention      : None
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef UI_TASK_H
#define UI_TASK_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"


#endif //UI_TASK_H
