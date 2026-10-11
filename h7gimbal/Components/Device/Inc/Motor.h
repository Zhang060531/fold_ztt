/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : Motor.h
  * @brief          : The header file of Motor.h 
  * @author         : GrassFan Wang
  * @date           : 2025/01/2
  * @version        : v1.0
  ******************************************************************************
  * @attention      : None
  ******************************************************************************
  */
/* USER CODE END Header */

/**
 * =====================================================================
 * 文件说明：电机定义与接口 (Motor.h)
 * ---------------------------------------------------------------------
 * 本文件是「云台电机层」的头文件，声明了所有电机相关的枚举、数据结构
 * 和函数。云台上总共用到两类、共 6 个电机：
 *
 *  1. DJI 大疆电机（走大疆电调 CAN 协议，发电流、收转速/角度/温度）：
 *     - 摩擦轮电机 M3508 ×3  → 实例 FR_Motor[3]，靠摩擦轮甩出子弹
 *     - 拨弹轮电机 M2006 ×1  → 实例 Shoot_Motor，把子弹拨进摩擦轮
 *
 *  2. 达妙(Damiao) DM4310 电机 ×2（走 MIT 模式 CAN 协议）：
 *     - 实例 DM_4310_Motor[2]，分别是云台 pitch(俯仰) 和 yaw(偏航) 关节
 *
 * ===== 调车重点关注 =====
 *  - 所有电机实例的 CAN ID 定义在 Motor.c 里：
 *      TxIdentifier = 本机发送的帧 ID（下发给电机）
 *      RxIdentifier = 电机回传的帧 ID（本机接收）
 *    CAN ID 不对，电机就收不到指令 / 回不了数据，是调车最先要检查的。
 *  - DM_4310_Motor 的 Param_Range(P_MAX/V_MAX/T_MAX) 是 MIT 协议量程，
 *    必须和达妙上位机(DM_Tools)里设的一致，否则换算出的位置/速度/力矩会错。
 * =====================================================================
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef DEVICE_MOTOR_H
#define DEVICE_MOTOR_H

/* Includes ------------------------------------------------------------------*/
#include "config.h"
#include "stm32h723xx.h"
#include "cmsis_os.h"
#include "bsp_can.h"

// 三个摩擦轮电机的下标，方便用 FR_Motor[M1] 这种写法访问
#define M1 0   // 摩擦轮电机 1
#define M2 1   // 摩擦轮电机 2
#define M3 2   // 摩擦轮电机 3



/**
 * @brief typedef enum that contains the type of DJI Motor Device.
 */
// DJI 电机型号枚举（对应大疆电调 CAN 协议里的不同电机）
typedef enum{
	  DJI_GM6020,    // 云台常用的大扭矩关节电机（本云台没用到，纯枚举占位）
    DJI_M3508,     // 摩擦轮电机，本车用来甩子弹
	  DJI_M350851,   // M3508 的另一个变体（未使用）
    DJI_M2006,     // 拨弹轮电机，把子弹拨进摩擦轮
}DJI_Motor_Type_e;

/**
 * @brief  typedef enum that control mode the type of DM_Motor.
 */
// 达妙(DM)电机的工作模式
typedef enum
{
  MIT,                // 最常用：发 位置+速度+KP+KD+力矩，电机内部做 MIT 阻抗控制
	POSITION_VELOCITY,  // 发 位置+速度
	VELOCITY,           // 只发 速度
}DM_Motor_Control_Mode_Type_e;

/**
 * @brief  typedef enum that CMD of DM_Motor .
 */
// 发给达妙电机的控制命令（放在 MIT 帧的最后一个字节 Data[7]）
typedef enum{
  Motor_Enable,             // 使能电机 (0xFC)
  Motor_Disable,            // 失能电机 (0xFD)
  Motor_Save_Zero_Position, // 保存当前位置为零点 (0xFE)
	Motor_Clear,              // 清除错误 (0xFB)
  DM_Motor_CMD_Type_Num,
}DM_Motor_CMD_Type_e;

/**
 * @brief typedef structure that contains the information for the motor FDCAN transmit and recieved .
 */
typedef struct
{
  uint32_t TxIdentifier;   /*!< Specifies FDCAN transmit identifier */
  uint32_t RxIdentifier;   /*!< Specifies FDCAN recieved identifier */
} Motor_CANFrameInfo_typedef;

/**
 * @brief typedef structure that contains the data for the Motor Device.
 */
// DJI 电机（M3508/M2006/M3508等）的数据结构，存电机回传信息
typedef struct
{
  bool Initlized;            // 初始化标志：第一次收到数据后置 true
  int16_t  Current;          // 电机实际电流（转矩电流，单位约 A 的量化值）
  int16_t  Velocity;         // 电机转速 (RPM，转/分)
  int16_t  Encoder;          // 编码器角度原始值（0~8191，一圈 8192 线）
  int16_t  Last_Encoder;     // 上一次的编码器值，用于算增量/判断过零
  float    Angle;            // 换算后的角度（度）
  uint8_t  Temperature;      // 电机温度
  float    Velocity_filter;  // 低通滤波后的转速
	int32_t Total_Round;       // 记录转子走过的总圈数（累计，判断正反转）
	float   Continuous_Angle;  // 除以减速比后的输出轴绝对连续角度（可超 360°）
} DJI_Motor_Data_Typedef;


/**
 * @brief typedef structure that contains the param range for the DM_Motor .
 */
// 达妙电机 MIT 协议的量程（单位要跟达妙上位机一致）
typedef struct
{
  float  P_MAX;   // 位置量程上限（对称范围 ±P_MAX），单位通常是 rad 或自定义
  float  V_MAX;   // 速度量程上限（±V_MAX），单位 rad/s
  float  T_MAX;   // 力矩量程上限（±T_MAX），单位 N·m
} DM_Motor_Param_Range_Typedef;

/**
 * @brief typedef structure that contains the data for the DJI Motor Device.
 */
// 达妙电机回传数据（MIT 模式反馈帧解析后存这里）
typedef struct
{
  bool Initlized;    // 初始化标志
  uint8_t  State;    // 电机状态字（使能/错误标志，取反馈帧高位）
  uint16_t  P_int;   // 位置原始值 (uint16)
  uint16_t  V_int;   // 速度原始值 (uint16)
  uint16_t  T_int;   // 力矩原始值 (uint16)
  float  Position;   // 换算后的位置（物理单位）
  float  Velocity;   // 换算后的速度（物理单位）
  float  Torque;     // 换算后的力矩（物理单位）
  float  Temperature_MOS;   // MOS 管温度（板载功率管温度）
  float  Temperature_Rotor; // 转子/绕组温度
  float  Angle;      // 角度（度）
	float  Relative_angle;   // 相对角度：位置量程 [-P_MAX,P_MAX] 映射到 [ANGLE_MIN,ANGLE_MAX]
} DM_Motor_Data_Typedef;

/**
 * @brief typedef structure that contains the information for the Damiao Motor Device.
 */
typedef struct
{
  DJI_Motor_Type_e Type;   /*!< Type of Motor */
  Motor_CANFrameInfo_typedef FDCANFrame;    /*!< information for the CAN Transfer */
  DJI_Motor_Data_Typedef Data;   /*!< information for the Motor Device */
  uint32_t Motor_time;
} DJI_Motor_Info_Typedef;


// DJI 电机（M3508/M2006）的控制输出结构
typedef struct
{
  float speed_rpm_set;          // 目标转速设定 (RPM)，经速度环 PID 算出
  int16_t current_set;          // 目标电流（整数，最终下发给电机的值）
  float current_set_f;          // 目标电流（浮点，PID 中间量）
  int16_t current_set_limited;  // 限幅后的目标电流
} DJI_Motor_Contorl_Info_Typedef;

/**
 * @brief typedef structure that contains the information for the DJI Motor Device.
 */
typedef struct
{
  DM_Motor_Control_Mode_Type_e Control_Mode;
  Motor_CANFrameInfo_typedef FDCANFrame;   
  DM_Motor_Param_Range_Typedef Param_Range; 
  DM_Motor_Data_Typedef Data;   
  uint32_t Motor_time;
} DM_Motor_Info_Typedef;

/**
 * @brief typedef structure that contains the control information for the DM Motor Device .
 */
// 达妙电机的控制输出结构（MIT 模式一次下发这 5 个量）
typedef struct
{
  float Position;   // 目标位置
  float Velocity;   // 目标速度
  float KP;         // 位置刚度（MIT 的 P，单位 N·m/rad 之类）
  float KD;         // 速度阻尼（MIT 的 D）
  float Torque;     // 前馈力矩
  float Angle;      // 目标角度（度）
} DM_Motor_Contorl_Info_Typedef;

/* Externs ------------------------------------------------------------------*/
// 全局电机实例（定义在 Motor.c）：
//   FR_Motor[3]    = 3 个摩擦轮 M3508
//   Shoot_Motor    = 1 个拨弹轮 M2006
//   DM_4310_Motor[2] = 2 个达妙 DM4310（[0]=pitch 俯仰, [1]=yaw 偏航）
extern DJI_Motor_Info_Typedef Shoot_Motor,FR_Motor[3];
extern DM_Motor_Info_Typedef DM_4310_Motor[2];
/* Function Prototypes */
extern void DJI_Motor_Info_Update(uint32_t *Identifier, uint8_t *Rx_Buf,DJI_Motor_Info_Typedef *DJI_Motor);
extern void DM_Motor_Info_Update(uint32_t *Identifier,uint8_t *Rx_Buf,DM_Motor_Info_Typedef *DM_Motor);
extern void TransmitShootMotorCurrent(FDCAN_TxFrame_TypeDef *FDCAN_TxFrame,int16_t shoot_current);
extern void DM_Motor_Command(FDCAN_TxFrame_TypeDef *FDCAN_TxFrame,DM_Motor_Info_Typedef *DM_Motor,uint8_t CMD);
extern void DM_Motor_CAN_TxMessage(FDCAN_TxFrame_TypeDef *FDCAN_TxFrame,DM_Motor_Info_Typedef *DM_Motor,float Postion, float Velocity, float KP, float KD, float Torque);
extern void SectionTransform(float* output, float input, float out_min, float out_zero, float out_max, float in_min, float in_zero, float in_max);
extern void MinMaxConstrain(float* input, float min, float max);
extern float Abs(float num);
extern float PreventFullCircleConstrain(float set, float fdb);
extern float filter_FR1(float temp,float alpha);
extern float filter_FR2(float temp,float alpha);

extern float filter_sht(float temp,float alpha);
extern void ConstrainLoop(float* output, float input, float min, float max);
extern void TransmitFriMotorCurrent(FDCAN_TxFrame_TypeDef *FDCAN_TxFrame,int16_t m1, int16_t m2);



#endif //DEVICE_MOTOR_H