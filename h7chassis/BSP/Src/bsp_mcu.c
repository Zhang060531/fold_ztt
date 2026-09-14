/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bsp_mcu.c
  * @brief          : MCU peripheral initialization functions
  * @author         : GrassFan Wang
  * @date           : 2025/01/22
  * @version        : v1.0
  ******************************************************************************
  * @attention      : none
  ******************************************************************************
  */
/* USER CODE END Header */

/* ================================================================================ */
/* 文件说明：BSP 层 MCU 统一初始化入口 —— 实现文件                                   */
/* -------------------------------------------------------------------------------- */
/* 【作用】                                                                         */
/*   把整个板子需要的外设初始化集中起来，按「先外设、后设备」的顺序调用：               */
/*     1) BSP 外设层：PWM -> GPIO -> CAN -> UART -> ADC -> USB                      */
/*     2) 设备层：BMI088 惯性传感器（加速度计+陀螺仪）                                */
/*                                                                                  */
/* 【初始化顺序为什么这样排】                                                        */
/*   先初始化基础外设（PWM/GPIO/通信总线等），再初始化依赖这些外设的具体设备。          */
/*   例如 BMI088 依赖 SPI + GPIO 片选，所以放在最后。                                 */
/*                                                                                  */
/* 【被谁调用】                                                                     */
/*   main() 启动后调用一次 MCU_Init()。                                              */
/* ================================================================================ */

/* Includes ------------------------------------------------------------------*/
#include "bsp_mcu.h"
#include "bsp_gpio.h"
#include "bsp_can.h"
#include "bsp_pwm.h"
#include "bsp_uart.h"
#include "bsp_adc.h"
#include "bmi088.h"
#include "usb_device.h"

/**
  * @brief  初始化整个 MCU：依次初始化所有 BSP 外设和设备
  * @param  无
  * @retval 无
  * @note   顺序：先外设（PWM/GPIO/CAN/UART/ADC/USB），后设备（BMI088）
  */
void MCU_Init(void)
{
  /* ----------------------- BSP Init ----------------------- */
  /* 依次初始化各 BSP 外设：
     BSP_PWM_Init()      : TIM3 CH4 PWM，驱动 BMI088 加热片（恒温）
     BSP_GPIO_Init()     : GPIO（当前为空）
     BSP_FDCAN_Init()    : FDCAN 总线，与电机/电调通信（bsp_can.c）
     BSP_USART_Init()    : 串口 + DMA 接收（裁判/图传/SBUS/VoFa）
     BSP_ADC_Init()      : ADC 采样（测电池电压）
     MX_USB_DEVICE_Init(): USB 设备 */
  BSP_PWM_Init();
	BSP_GPIO_Init();
  BSP_FDCAN_Init();
  BSP_USART_Init();
	BSP_ADC_Init();
	MX_USB_DEVICE_Init();
  /* ----------------------- Device Init ----------------------- */
  /* 最后初始化设备：BMI088 惯性传感器（依赖 SPI 和 GPIO 片选） */
  BMI088_Init();
}
//------------------------------------------------------------------------------
