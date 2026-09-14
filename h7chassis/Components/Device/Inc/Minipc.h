/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : MiniPC.h
  * @brief          : MiniPC interfaces functions 
  * @author         : GrassFan Wang
  * @date           : 2025/02/10
  * @version        : v1.0
  ******************************************************************************
  * @attention      : None
  ******************************************************************************
  */
/* USER CODE END Header */

/**
 ******************************************************************************
 * @file    MiniPC.h
 * @brief   上位机（妙算 MiniPC）通信接口声明
 *
 * 【设备是什么】
 * 妙算（MiniPC / Manifold）是大疆的车载微型计算机（运行 Linux），
 * 常作为上位机跑视觉识别算法（自动瞄准）。MCU 通过 USB 虚拟串口(CDC)与它通信。
 *
 * 【通过什么接口连接】
 * USB CDC 虚拟串口（由 usbd_cdc_if.c 提供 CDC_Transmit_HS / CDC_Receive_HS）。
 *
 * 【数据协议大概长什么样】
 * 上层自定义字节流：MCU 把云台姿态等状态发给妙算，妙算把识别到的目标位置、
 * 控制指令等发回 MCU。本文件只声明收发接口，具体帧格式由调用方约定。
 *
 * 【被谁调用】
 *   - MiniPC_Transmit_Info()：MCU 向上位机发送数据；
 *   - MiniPC_Recvive_Info()：USB 接收回调(CDC_Receive_HS)里被调用，处理上位机发来的数据。
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef DEVICE_MINIPC_H
#define DEVICE_MINIPC_H


/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "stdbool.h" 


extern void MiniPC_Transmit_Info(uint8_t *Buff);

extern void MiniPC_Recvive_Info(uint8_t* Buff, const uint32_t *Len);
#endif