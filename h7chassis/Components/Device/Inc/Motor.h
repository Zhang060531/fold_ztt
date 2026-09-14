/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : Motor.h
  * @brief          : The header file of Motor.h 
  ******************************************************************************
  */
/* USER CODE END Header */

/* =====
 * 文件: Motor.h —— 电机抽象层头文件(大疆经典CAN / 达妙DM / 灵控LK 电机)
 * 作用: 定义三类电机的数据结构、控制模式枚举、CAN 帧信息与收发函数原型:
 *   - DJI 电机(M3508/M2006/GM6020) 走经典 CAN, 数据格式固定 8 字节。
 *   - DM 电机(DM4310 等) 走 MIT 协议, 位置16位/速度12位/KP12位/KD12位/力矩12位。
 *   - LK 灵控电机(MS/MF/MH/MG, 本工程作舵向电机) 走 0x140/0x180 系列 ID。
 * 结构/调用关系:
 *   - 底层 CAN 接收中断调用 DJI_Motor_Info_Update / DM_Motor_Info_Update /
 *     LK_Motor_Info_Update 解析反馈。
 *   - 上层控制任务调用 Transmit*MotorCurrent / DM_Motor_CAN_TxMessage /
 *     LK_Motor_* 系列函数下发指令。
 *   - Check_All_Motors_Offline() 在控制任务循环中检测离线。
 * ===== 调车关注 =====
 *   - 大疆电机 ID: 底盘 M3508 发 0x200 收 0x201~0x204; 发射 M2006 发 0x1FF 收 0x205。
 *   - DM4310 发 0x02 收 0x12(MIT 模式), P_MAX/V_MAX/T_MAX 决定编解码量程。
 *   - LK 舵向电机发 0x141~0x144 收 0x181~0x184(注意 .c 中全局初始化 Tx==Rx, 需核对)。
 *   - MOTOR_OFFLINE_TIMEOUT=100ms, 超过未收到反馈即判离线。
 * =====
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef DEVICE_MOTOR_H
#define DEVICE_MOTOR_H

/* Includes ------------------------------------------------------------------*/
#include "config.h"
#include "stm32h723xx.h"
#include "cmsis_os.h"
#include "bsp_can.h"
#include "stdbool.h"

/* 电机编号宏(数组下标用) */
#define M1 0     /* 电机1 */
#define M2 1     /* 电机2 */
#define M3 2     /* 电机3 */
#define M4 3     /* 电机4 */
#define YAW 4    /* 云台 yaw 轴电机 */
#define SHOOT 5  /* 发射机构电机 */

/* 离线检测超时时间 (ms): 超过该时长未收到反馈即判定电机离线 */
#define MOTOR_OFFLINE_TIMEOUT 100


/**
 * @brief 大疆经典 CAN 电机型号
 * @note  GM6020: 云台 yaw/pitch 电机(带编码器); M3508: 底盘动力电机;
 *        M350851: M3508 减速箱版本(减速比 51); M2006: 发射机构拨盘电机。
 */
typedef enum{
    DJI_GM6020,
    DJI_M3508,
    DJI_M350851,
    DJI_M2006,
}DJI_Motor_Type_e;

/**
 * @brief 达妙(DM)电机控制模式
 * @note  MIT: 经典 MIT 模式(位置/速度/KP/KD/力矩全下发), 本工程 DM4310 用该模式;
 *        POSITION_VELOCITY / VELOCITY: DM 的另一种指令帧格式(直接发 float 字节)。
 */
typedef enum
{
  MIT,
  POSITION_VELOCITY,
  VELOCITY,
}DM_Motor_Control_Mode_Type_e;

/**
 * @brief DM 电机特殊指令(通过数据位 0xFC/0xFD/0xFE 下发)
 */
typedef enum{
  Motor_Enable,              /* 使能(0xFC) */
  Motor_Disable,             /* 失能(0xFD) */
  Motor_Save_Zero_Position,  /* 保存当前位置为零点(0xFE) */
  DM_Motor_CMD_Type_Num,
}DM_Motor_CMD_Type_e;


typedef struct
{
  uint32_t TxIdentifier;   /*!< Specifies FDCAN transmit identifier */
  uint32_t RxIdentifier;   /*!< Specifies FDCAN recieved identifier */
} Motor_CANFrameInfo_typedef;

/**
 * @brief 大疆经典 CAN 电机的反馈数据结构
 * @note  单位: Current 为电调电流指令值(-16384~16384 对应 -20A~20A);
 *        Velocity 为 rpm; Encoder 为 0~8191 编码器值; Angle 为换算后的累计角度(°);
 *        Temperature 为温度(℃); Velocity_filter 为滤波后的线速度(m/s)。
 */
typedef struct
{
  bool Initlized;   /*!< 是否已完成首次角度初始化 */
  int16_t  Current;   /*!< 电流反馈(电调单位, -16384~16384 对应 ±20A) */
  int16_t  Velocity;    /*!< 转速(RPM) */
  int16_t  Encoder;   /*!< 编码器原始值(0~8191) */
  int16_t  Last_Encoder;   /*!< 上一次编码器原始值 */
  float    Angle;   /*!< 累计角度(°) */
  uint8_t  Temperature;   /*!< 电机温度(℃) */
  float    Velocity_filter;    /*!< 滤波后的线速度(m/s) */
} DJI_Motor_Data_Typedef;


/**
 * @brief DM 电机 MIT 协议编解码量程(用于 float<->uint 缩放)
 * @note  P_MAX 位置量程(rad), V_MAX 速度量程(rad/s), T_MAX 力矩量程(N·m)。
 */
typedef struct
{
  float  P_MAX;
  float  V_MAX;
  float  T_MAX;
} DM_Motor_Param_Range_Typedef;


/**
 * @brief DM 电机反馈数据
 * @note  P_int/V_int/T_int 为 MIT 协议原始位段(16/12/12 位), 经 uint_to_float
 *        按 P_MAX/V_MAX/T_MAX 量程换算为实际物理量(Position: rad, Velocity: rad/s,
 *        Torque: N·m); State 为高 4 位状态码。
 */
typedef struct
{
  bool Initlized;    /*!< 初始化标志 */
  uint8_t  State;    /*!< 电机状态(高4位) */
  uint16_t  P_int;   /*!< 位置原始值(16位) */
  uint16_t  V_int;   /*!< 速度原始值(12位) */
  uint16_t  T_int;   /*!< 力矩原始值(12位) */
  float  Position;   /*!< 位置(rad) */
  float  Velocity;   /*!< 速度(rad/s) */
  float  Torque;     /*!< 力矩(N·m) */
  float  Temperature_MOS;   /*!< 驱动板(MOS)温度(℃) */
  float  Temperature_Rotor; /*!< 电机转子温度(℃) */
  float  Angle;      /*!< 累计角度(°) */
} DM_Motor_Data_Typedef;

/**
 * @brief 大疆电机完整信息(类型 + CAN 帧 + 反馈数据 + 离线检测)
 */
typedef struct
{
  DJI_Motor_Type_e Type;   /*!< 电机型号 */
  Motor_CANFrameInfo_typedef FDCANFrame;    /*!< CAN 收发 ID */
  DJI_Motor_Data_Typedef Data;   /*!< 电机反馈数据 */
  uint32_t Motor_time;      /*!< 最后收到反馈的 tick(用于离线检测) */
  bool is_offline;         /* 【新增】离线标志 */
} DJI_Motor_Info_Typedef;


/**
 * @brief 大疆电机控制指令
 * @note  speed_rpm_set: 目标转速(rpm); current_set: 目标电流(电调单位);
 *        current_set_f: 浮点目标电流(限流计算用); current_set_limited: 限流后电流。
 */
typedef struct
{
  float speed_rpm_set;
  int16_t current_set;
  float current_set_f;
  int16_t current_set_limited;
} DJI_Motor_Contorl_Info_Typedef;


typedef struct
{
  DM_Motor_Control_Mode_Type_e Control_Mode;
  Motor_CANFrameInfo_typedef FDCANFrame;   
  DM_Motor_Param_Range_Typedef Param_Range; 
  DM_Motor_Data_Typedef Data;   
  uint32_t Motor_time;
  bool is_offline;                     /* 【新增】离线标志 */
} DM_Motor_Info_Typedef;

/**
 * @brief DM 电机控制指令(MIT 模式)
 * @note  Position/Velocity/Torque 为目标物理量(rad, rad/s, N·m);
 *        KP/KD 为 MIT 位置-速度环增益(无量纲, 编码时 KP 量程 0~500, KD 量程 0~5)。
 */
typedef struct
{
  float Position;
  float Velocity;
  float KP;
  float KD;
  float Torque;
  float Angle;
} DM_Motor_Contorl_Info_Typedef;

/* Externs ------------------------------------------------------------------*/
extern DJI_Motor_Info_Typedef Shoot_Motor,Chassis_Motor[4];
extern DM_Motor_Info_Typedef DM_4310_Motor[1];
extern DM_Motor_Contorl_Info_Typedef DM_Motor_Contorl_Info[4];

/* Function Prototypes */
/* --- 大疆经典 CAN 电机 --- */
extern void DJI_Motor_Info_Update(uint32_t *Identifier, uint8_t *Rx_Buf,DJI_Motor_Info_Typedef *DJI_Motor);
extern void TransmitChassisMotorCurrent(FDCAN_TxFrame_TypeDef *FDCAN_TxFrame,int16_t m1, int16_t m2, int16_t m3, int16_t m4);
extern void TransmitsteerMotorCurrent(FDCAN_TxFrame_TypeDef *FDCAN_TxFrame,int16_t m1, int16_t m2, int16_t m3, int16_t m4);
extern void TransmitShootMotorCurrent(FDCAN_TxFrame_TypeDef *FDCAN_TxFrame,int16_t shoot_current);

/* --- 达妙 DM 电机(MIT 协议) --- */
extern void DM_Motor_Info_Update(uint32_t *Identifier,uint8_t *Rx_Buf,DM_Motor_Info_Typedef *DM_Motor);
extern void DM_Motor_Command(FDCAN_TxFrame_TypeDef *FDCAN_TxFrame,DM_Motor_Info_Typedef *DM_Motor,uint8_t CMD);
extern void DM_Motor_CAN_TxMessage(FDCAN_TxFrame_TypeDef *FDCAN_TxFrame,DM_Motor_Info_Typedef *DM_Motor,float Postion, float Velocity, float KP, float KD, float Torque);


/* 【新增】离线检测函数 */
extern void Check_All_Motors_Offline(void);

#endif //DEVICE_MOTOR_H