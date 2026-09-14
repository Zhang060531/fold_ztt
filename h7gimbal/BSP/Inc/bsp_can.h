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

/**
 * =====================================================================
 * 文件说明：CAN 底层驱动头文件 (bsp_can.h)
 * ---------------------------------------------------------------------
 * 定义 FDCAN 收发帧结构、超级电容功率结构、CAN 收发接口。
 * 三路 CAN 的分工见 bsp_can.c 文件头说明。
 * =====================================================================
 */

/**
 * @brief The structure that contains the Information of FDCAN Transmit.
 */
typedef struct{
		FDCAN_HandleTypeDef *hcan;
    FDCAN_TxHeaderTypeDef Header;
    uint8_t				Data[8];
}FDCAN_TxFrame_TypeDef;

/**
 * @brief The structure that contains the Information of FDCAN Receive.
 */
typedef struct {
		FDCAN_HandleTypeDef *hcan;
    FDCAN_RxHeaderTypeDef Header;
    uint8_t 			Data[8];
}FDCAN_RxFrame_TypeDef;

typedef struct
{
  uint32_t TxIdentifier;   /*!< Specifies FDCAN transmit identifier */
  uint32_t RxIdentifier;   /*!< Specifies FDCAN recieved identifier */

}CANFrameInfo_typedef;

/* 超级电容/功率管理结构体：从裁判系统读功率数据，做超功率控制 */
typedef struct
{
	CANFrameInfo_typedef FDCANFrame;    /*!< information for the CAN Transfer */
	float PowerData[4];
	
	float tot_power;
	
	float input_voltage;
	
	float capacitive_voltage;
	
	float input_current;
	
	float power_set;
	
	float curren_chassis_power;
	
	float set_chassis_power;
	
	uint16_t SurperPower_Flag;
	
	float capa_voltage;
	
	uint32_t SurperPowerTime;
}
SuperPower_Info_Typedef;

float Int16ToFloat(int16_t data);
int Int16ToInt32(int16_t data);



/* Externs ------------------------------------------------------------------*/
extern  FDCAN_TxFrame_TypeDef   FDCAN1_TxFrame;
extern  FDCAN_TxFrame_TypeDef   FDCAN2_TxFrame;
extern  FDCAN_TxFrame_TypeDef   FDCAN3_TxFrame;
extern void  USER_FDCAN_AddMessageToTxFifoQ(FDCAN_TxFrame_TypeDef *FDCAN_TxFrame);
extern void BSP_FDCAN_Init(void);
extern float uint_to_float(int X_int, float X_min, float X_max, int Bits);
extern  int float_to_uint(float x, float x_min, float x_max, int bits);
extern void sentToGimbal(FDCAN_TxFrame_TypeDef *FDCAN_TxFrame,int16_t ecd, int16_t speed_rpm, int16_t current, int16_t temperate);
//extern uint32_t GetMotorFR1Fdb(DJI_Motor_Info_Typedef * motor);
//extern uint32_t GetMotorFR2Fdb(DJI_Motor_Info_Typedef * motor);
//extern uint32_t GetMotorFR3Fdb(DJI_Motor_Info_Typedef * motor);
//extern uint32_t GetPitchMotorFdb(DM_Motor_Info_Typedef* motor);
//extern uint32_t GetYawMotorFdb(DM_Motor_Info_Typedef* motor);
//extern uint32_t GetMotorPitchRelativeAngle(float* angle);
//extern uint32_t GetMotorYawRelativeAngle(float* angle);
//extern uint32_t GetShootMotorFdb(DJI_Motor_Info_Typedef* motor);
#endif