/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bsp_can.h
  * @brief          : The header file of bsp_can.c
  * @author         : GrassFan Wang
  * @date           : 2025/01/22
  * @version        : v1.0
  ******************************************************************************
  * @attention      : Pay attention to extern the functions and structure
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BSP_CAN_H
#define BSP_CAN_H
#ifdef __cplusplus
extern "C" {
#endif


/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx.h"

/* ===== 文件说明：CAN 底层驱动头文件 (bsp_can.h) =====
 * 作用：声明 FDCAN 收发帧结构体、超级电容功率结构体，以及 CAN 收发/初始化/换算接口。
 * 三路 CAN 的分工详见 bsp_can.c 文件头，这里只放「类型定义 + 全局变量声明 + 函数原型」。
 *
 * ===== 调车关注 =====
 *  - FDCAN_TxFrame_TypeDef / FDCAN_RxFrame_TypeDef：把 HAL 的收发头 + 8 字节数据
 *    打包成一个结构体，上层只需填 Data[8] 和 Header.Identifier 就能收/发一帧。
 *  - SuperPower_Info_Typedef：超级电容功率结构体（全局实例 PowerData），
 *    保存从电管(接收帧 0x211)读回的功率/电压等数据，供功率限制算法使用。
 */

/**
 * @brief FDCAN 发送帧结构体：HAL 发送句柄 + 发送头 + 8 字节载荷
 */
typedef struct{
		FDCAN_HandleTypeDef *hcan;   /*!< 指向对应的 FDCAN 外设句柄(hfdcan1/2/3) */
    FDCAN_TxHeaderTypeDef Header; /*!< HAL 发送头：帧ID/帧类型/长度/FD 格式等 */
    uint8_t				Data[8];      /*!< 8 字节载荷(经典 CAN 最多 8 字节) */
}FDCAN_TxFrame_TypeDef;

/**
 * @brief FDCAN 接收帧结构体：HAL 接收句柄 + 接收头 + 8 字节载荷
 */
typedef struct {
		FDCAN_HandleTypeDef *hcan;   /*!< 指向对应的 FDCAN 外设句柄 */
    FDCAN_RxHeaderTypeDef Header; /*!< HAL 接收头：帧ID/长度/时间戳等 */
    uint8_t 			Data[8];      /*!< 8 字节接收数据 */
}FDCAN_RxFrame_TypeDef;

/* 帧 ID 信息结构体：记录某设备(电机/电管等)的发送/接收 CAN ID */
typedef struct
{
  uint32_t TxIdentifier;   /*!< Specifies FDCAN transmit identifier  发送帧 ID */
  uint32_t RxIdentifier;   /*!< Specifies FDCAN recieved identifier  接收帧 ID */

}CANFrameInfo_typedef;

/* 超级电容/功率管理结构体：从电管(接收帧 0x211)读功率数据，做超功率/超电控制 */
typedef struct
{
	CANFrameInfo_typedef FDCANFrame;    /*!< information for the CAN Transfer  本设备收发 ID(接收 0x211) */
	float PowerData[4];                 /*!< 功率原始数据暂存(当前未使用) */

	float tot_power;                    /*!< 电管当前总功率 (W) */

	float input_voltage;                /*!< 输入电压 (V，当前未使用) */

	float capacitive_voltage;           /*!< 电容电压 (V，当前未使用) */

	float input_current;                /*!< 输入电流 (A，当前未使用) */

	float power_set;                    /*!< 功率设定 (W，当前未使用) */

	float curren_chassis_power;         /*!< 底盘当前消耗功率 (W) */

	float set_chassis_power;            /*!< 底盘目标功率 (W，电管限功率值) */

	uint16_t SurperPower_Flag;          /*!< 超级电容标志位(当前未使用) */

	float capa_voltage;                 /*!< 电容电压 (V) */

	uint32_t SurperPowerTime;           /*!< 超级电容时间戳(当前未使用) */
}
SuperPower_Info_Typedef;

/* int16 -> float 转换(处理符号位，等价于简单强转) */
float Int16ToFloat(int16_t data);
/* int16 -> int32 转换(处理符号位) */
int Int16ToInt32(int16_t data);



/* Externs ------------------------------------------------------------------*/
extern  FDCAN_TxFrame_TypeDef   FDCAN1_TxFrame;  /*!< FDCAN1 发送帧(经典 CAN，大疆电机) */
extern  FDCAN_TxFrame_TypeDef   FDCAN2_TxFrame;  /*!< FDCAN2 发送帧(FD CAN，DM4310/云台通信) */
extern  FDCAN_TxFrame_TypeDef   FDCAN3_TxFrame;  /*!< FDCAN3 发送帧(经典 CAN，视觉/妙算) */
extern SuperPower_Info_Typedef PowerData;        /*!< 超级电容功率数据全局变量 */
extern void  USER_FDCAN_AddMessageToTxFifoQ(FDCAN_TxFrame_TypeDef *FDCAN_TxFrame); /*!< 把一帧塞进 CAN 发送 FIFO 队列 */
extern void BSP_FDCAN_Init(void);                /*!< 初始化三路 FDCAN 的过滤器 + 中断 + 启动 */
extern float uint_to_float(int X_int, float X_min, float X_max, int Bits);   /*!< uint 整数 -> 浮点物理量(DM 电机用) */
extern  int float_to_uint(float x, float x_min, float x_max, int bits);      /*!< 浮点物理量 -> uint 整数(DM 电机用) */
extern void Sent_Superpower_Data(FDCAN_TxFrame_TypeDef *FDCAN_TxFrame,float set_power);//uint16_t temp_power /*!< 向电管发送目标功率(帧 0x210) */

#endif