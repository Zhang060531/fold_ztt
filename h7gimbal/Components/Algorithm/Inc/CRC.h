/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : crc.h
  * @brief          : crc check 
  * @author         : GrassFan Wang
  * @date           : 2023/01/22
  * @version        : v1.0
  ******************************************************************************
  * @attention      : None
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef CRC_H
#define CRC_H

/* ==== 文件说明 ================================================================
 * CRC.h —— CRC 循环冗余校验（Cyclic Redundancy Check）头文件
 * ---------------------------------------------------------------------------
 * 作用：为通信链路提供 CRC8 / CRC16 校验，用来发现数据在传输中出现的位错误。
 *       发送方把数据算出一个校验值填在包尾，接收方用同样算法重算一遍再对比，
 *       一致说明数据没坏，不一致说明传输出错（受干扰/丢包），应丢弃这包数据。
 *
 * 核心函数（实现在 CRC.c）：
 *   - Get_CRC8_Check_Sum / Get_CRC16_Check_Sum : 计算校验值
 *   - Verify_CRC8_Check_Sum / Verify_CRC16_Check_Sum : 校验整包数据是否正确
 *   - Append_CRC8_Check_Sum / Append_CRC16_Check_Sum : 把校验值附加到包尾
 *
 * 在整车里的用途：云台板与上位机（裁判系统/视觉/自研调试软件）通信时，
 *   用来保证数据完整性；解析协议前先做 CRC 校验，错了就直接丢弃。
 * 新手提示：CRC 不是加密，只是"查错"。它检测随机位翻转很有效，但不能防篡改。
 * ========================================================================== */

#include "stdint.h"
#include "stdbool.h"

 
/* 计算 CRC8 校验值（查表法）
 * pchMessage: 待计算的数据指针；dwLength: 数据长度（不含校验字节）；
 * ucCRC8: 初始值（通常传 CRC8_INIT=0xFF，支持分片续算）；返回 8bit 校验值 */
extern uint8_t Get_CRC8_Check_Sum(unsigned char *pchMessage,unsigned int dwLength,unsigned char ucCRC8);

/* 校验 CRC8：对整包(数据+校验字节)重算并与包尾校验字节比较，一致返回 true */
extern bool Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);

/* 把 CRC8 校验值写到缓冲区最后一个字节（包尾） */
extern void Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);

/* 计算 CRC16 校验值（查表法），返回 16bit 校验值 */
extern uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC);

/* 校验 CRC16：整包重算并比较包尾两个字节（低字节在前） */
extern bool Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);

/* 把 CRC16 校验值写到缓冲区最后两个字节（低字节在前、高字节在后） */
extern void Append_CRC16_Check_Sum(uint8_t * pchMessage,uint32_t dwLength);
#endif