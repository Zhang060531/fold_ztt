/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bsp_uart.h
  * @brief          : The header file of bsp_can.h 
  * @author         : GrassFan Wang
  * @date           : 2025/01/22
  * @version        : v1.0
  ******************************************************************************
  * @attention      : Pay attention to extern the functions and structure
  ******************************************************************************
  */
/* USER CODE END Header */

/* ================================================================================ */
/* 文件说明：BSP 层 UART（通用异步串口）驱动 —— 头文件                               */
/* -------------------------------------------------------------------------------- */
/* 【UART 是干什么的】                                                               */
/*   UART 是异步串行通信（收发双方约定好波特率，按位收发数据，无需时钟线）。           */
/*   本工程用它连接多个外部模块：裁判系统、图传、SBUS 遥控器接收机、VoFa 上位机。      */
/*                                                                                  */
/* 【关键函数】                                                                     */
/*   - BSP_USART_Init()                   初始化各串口及 DMA 双缓冲接收             */
/*   - USART_Vofa_Justfloat_Transmit()    向 VoFa 上位机发送 3 个 float 波形数据    */
/* ================================================================================ */

#ifndef BSP_UART_H
#define BSP_UART_H

#ifdef __cplusplus
extern "C" {
#endif

/* Include------------------------------------------------------------------*/
#include "stm32h7xx.h"

/**
 * @brief  初始化各串口：配置 USART1(裁判/图传)、USART5(SBUS)，并开启 DMA 双缓冲接收
 * @param  无
 * @retval 无
 */
extern void BSP_USART_Init(void);
	   		 
/* ==================== 【VoFa 专用】发送函数声明（走 USART7） ==================== */
/**
 * @brief  向 VoFa+ 上位机发送 3 个 float 数据（JustFloat 协议）
 * @param  SendValue1/SendValue2/SendValue3  要发送的 3 个浮点数（如 PID 的电流/速度/位置）
 * @retval 无
 * @note   通过 USART7 以 DMA 方式发送，VoFa 端选 JustFloat 协议即可看到曲线
 */
extern void USART_Vofa_Justfloat_Transmit(float SendValue1,float SendValue2,float SendValue3);

/**
 * @brief  向 VoFa+ 上位机发送 4 个 float 数据（JustFloat 协议）
 * @param  SendValue1/SendValue2/SendValue3/SendValue4  要发送的 4 个浮点数（如 yaw 内外环 4 通道）
 * @retval 无
 * @note   通过 USART7 以 DMA 方式发送，VoFa 端选 JustFloat 协议、通道数填 4
 */
extern void USART_Vofa_Justfloat_Transmit4(float SendValue1,float SendValue2,float SendValue3,float SendValue4);
#endif