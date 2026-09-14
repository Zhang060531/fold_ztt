/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : Referee_System.h
  * @brief          : Updated to RoboMaster 2026 Protocol V2.0.0
  * @author         : GrassFan Wang
  * @date           : 2026/07/25
  * @version        : v2.0
  ******************************************************************************
  */
/* USER CODE END Header */

/**
 ******************************************************************************
 * @file    Referee_System.h
 * @brief   裁判系统（RoboMaster 官方比赛主控）通信协议的数据结构与常量定义
 *
 * 【设备是什么】
 * 裁判系统是 RoboMaster 比赛官方装在每台机器人上的「裁判主控盒」，
 * 通过串口(UART)与 MCU 通信。它负责比赛判罚并实时下发比赛信息：
 * 血量、子弹余量、底盘功率限制、枪口热量、比赛阶段、机器人坐标、Buff 状态等。
 * 机器人拿到这些数据后可以做功率限制、能量管理、自适应战术等。
 *
 * 【通过什么接口连接】
 * UART 串口（本工程配合 DMA 双缓冲接收，接收缓冲区见 Referee_System.c）。
 *
 * 【数据协议大概长什么样】
 * 一帧 = 帧头(5字节) + CMD_ID(2字节) + 数据区(变长) + CRC16(2字节)：
 *   帧头 5 字节：SOF(0xA5 固定帧头) + Data_Length(2字节,小端) + Seq(包序号) + CRC8(帧头校验)
 *   随后 2 字节 CMD_ID 决定本帧是哪种数据，接收端用 switch 按 CMD_ID 分发解析；
 *   帧尾 2 字节 CRC16 用于整帧校验。校验分两级：帧头 CRC8 + 整帧 CRC16，都通过才解析。
 *
 * 【被谁调用】
 *   - Referee_System_Frame_Update()：接收任务收到完整帧后调用，做帧校验与解析；
 *   - Referee_System_Data_Update()：上层任务拷贝最新裁判数据使用。
 *
 * 【字节序说明】
 * 协议里多字节数据一律为「小端」（低字节在前），所以解析时要把字节拼成 16/32/64 位，
 * 见 Referee_System.c 中的 bit8TObit16 / bit8TObit32 / bit8TOfloat32 等辅助函数。
 ******************************************************************************
 */

#ifndef REFEREE_INFO_H
#define REFEREE_INFO_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "stdbool.h"

/* Exported defines ----------------------------------------------------------*/

#define REFEREE_RXFRAME_LENGTH  136   // frame_header 5bytes, cmd_id 2bytes, data_max 127bytes, crc16 2bytes

/**
 * @brief Referee Communication protocol format
 */
#define FrameHeader_Length    5U     /*!< length of frame header */
#define CMDID_Length          2U     /*!< length of CMD ID */
#define CRC16_Length          2U     /*!< length of CRC16 */

/**
 * @brief Cmd ID List (RM2026 V2.0.0)
 */
#define GAME_STATUS_ID                 0x0001U  /* Game status data */
#define GAME_RESULT_ID                 0x0002U  /* Game result data */
#define GAME_ROBOT_HP_ID               0x0003U  /* Robot HP data */
#define EVENET_DATA_ID                 0x0101U  /* Site event data */
#define REFEREE_WARNING_ID             0x0104U  /* Referee warning data */
#define DART_INFO_ID                   0x0105U  /* Dart shoot data */
#define ROBOT_STATUS_ID                0x0201U  /* Robot status data */
#define POWER_HEAT_DATA_ID             0x0202U  /* Real-time power heat data */
#define ROBOT_POS_ID                   0x0203U  /* Robot position data */
#define BUFF_ID                        0x0204U  /* Robot buff data */
#define HURT_DATA_ID                   0x0206U  /* Robot hurt data */
#define SHOOT_DATA_ID                  0x0207U  /* Real-time shoot data */
#define PROJECTILE_ALLOWANCE_ID        0x0208U  /* Bullet remain data */
#define RFID_STATUS_ID                 0x0209U  /* RFID status data */
#define DART_CLIENT_CMD_ID             0x020AU  /* Dart Client cmd data */
#define GROUND_ROBOT_POSITION_ID       0x020BU  /* Ground robot position */
#define RADAR_MARAKING_DATA_ID         0x020CU  /* Radar marking progress */
#define SENTRY_INFO_ID                 0x020DU  /* Sentry autonomous decision info */
#define RADAR_INFO_ID                  0x020EU  /* Radar autonomous decision info */
#define ROBOT_INTERACTION_DATA_ID      0x0301U  /* Robot interaction data */
#define MAP_COMMAND_ID                 0x0303U  /* Mini-map command data */
#define MAP_ROBOT_DATA_ID              0x0305U  /* Mini-map receive data */
#define MAP_DATA_ID                    0x0307U  /* Mini-map path data */
#define CUSTOM_INFO_ID                 0x0308U  /* Custom info data */

/* Exported types ------------------------------------------------------------*/
/* #pragma pack(1)：让下面的结构体按 1 字节对齐（不做内存填充），
 * 这样结构体在内存中的布局与串口收到的原始字节流一一对应，不会出现填充字节。 */
#pragma pack(1)

/**
 * @brief Frame Header, 5 Bytes
 */
typedef struct 
{
  uint8_t  SOF;          /*!< Fixed 0xA5 */
  uint16_t Data_Length;  /*!< Data length */
  uint8_t  Seq;          /*!< Sequence number */
  uint8_t  CRC8;         /*!< CRC8 Checksum */
} FrameHeader_TypeDef;

/**
 * @brief 0x0001: Game Status
 */
typedef struct           
{
  uint8_t  game_type : 4;        
  uint8_t  game_progress : 4;    
  uint16_t stage_remain_time;    
  uint64_t SyncTimeStamp;        
} game_status_t;

/**
 * @brief 0x0002: Game Result
 */
typedef struct
{
  uint8_t winner;
} game_result_t;

/**
 * @brief 0x0003: Robot HP Data (Updated in RM2026: 20 Bytes)
 */
typedef struct
{
  uint16_t ally_1_robot_HP;   /*!< Red/Blue Hero HP */
  uint16_t ally_2_robot_HP;   /*!< Red/Blue Engineer HP */
  uint16_t ally_3_robot_HP;   /*!< Red/Blue 3 Infantry HP */
  uint16_t ally_4_robot_HP;   /*!< Red/Blue 4 Infantry HP */
  int16_t  damage_difference; /*!< Ally total damage minus Enemy total damage */
  uint16_t ally_7_robot_HP;   /*!< Ally Sentry HP */
  uint16_t ally_outpost_HP;   /*!< Ally Outpost HP */
  uint16_t ally_base_HP;      /*!< Ally Base HP */
  uint16_t enemy_outpost_HP;  /*!< Enemy Outpost HP */
  uint16_t enemy_base_HP;     /*!< Enemy Base HP */
} game_robot_HP_t;

/**
 * @brief 0x0101: Site Event Data
 */
typedef union
{
  uint32_t event_data;
} event_data_t;

/**
 * @brief 0x0104: Referee Warning
 */
typedef struct 
{ 
  uint8_t level; 
  uint8_t offending_robot_id; 
  uint8_t count; 
} referee_warning_t; 

/**
 * @brief 0x0105: Dart Info
 */
typedef struct
{
  uint8_t  dart_remaining_time;
  uint16_t dart_info;
} dart_info_t;

/**
 * @brief 0x0201: Robot Status (Updated in RM2026: 17 Bytes)
 */
typedef struct
{
  uint8_t  robot_id;
  uint8_t  robot_level;
  uint16_t current_HP;
  uint16_t maximum_HP;
  uint16_t shooter_barrel_cooling_value;
  uint16_t shooter_barrel_heat_limit;
  uint16_t chassis_power_limit;
  float    bullet_speed_limit; /*!< RM2026 New Field */
  uint8_t  power_management_gimbal_output : 1;
  uint8_t  power_management_chassis_output : 1;
  uint8_t  power_management_shooter_output : 1;
} robot_status_t;

/**
 * @brief 0x0202: Power Heat Data (Updated in RM2026: 14 Bytes)
 */
typedef struct
{
  uint16_t reserved1;
  uint16_t reserved2;
  float    reserved3;
  uint16_t buffer_energy;
  uint16_t shooter_17mm_barrel_heat;
  uint16_t shooter_42mm_barrel_heat;
} power_heat_data_t;

/**
 * @brief 0x0203: Robot Position Data
 */
typedef struct
{
  float x;     /* m */
  float y;     /* m */
  float angle; /* degree */
} robot_pos_t;

/**
 * @brief 0x0204: Robot Buff Data
 */
typedef struct
{
  uint8_t  recovery_buff;     
  uint16_t cooling_buff;      /*!< Updated to uint16_t in RM2026 */
  uint8_t  defence_buff;      
  uint8_t  vulnerability_buff;
  uint16_t attack_buff;       
  uint8_t  remaining_energy;  
} buff_t;

/**
 * @brief 0x0206: Hurt Data
 */
typedef struct
{
  uint8_t armor_id : 4; 
  uint8_t HP_deduction_reason : 4;
} hurt_data_t;

/**
 * @brief 0x0207: Real-time Shoot Data
 */
typedef struct
{
  uint8_t bullet_type;      
  uint8_t shooter_number;   
  uint8_t launching_frequency; 
  float   initial_speed;    
} shoot_data_t;

/**
 * @brief 0x0208: Projectile Allowance (Updated in RM2026: 8 Bytes)
 */
typedef struct
{
  uint16_t projectile_allowance_17mm; 
  uint16_t projectile_allowance_42mm;  
  uint16_t remaining_gold_coin; 
  uint16_t projectile_allowance_fortress; /*!< RM2026 New Field */
} projectile_allowance_t;

/**
 * @brief 0x0209: RFID Status (Updated in RM2026: 5 Bytes)
 */
typedef struct
{
  uint32_t rfid_status;
  uint8_t  rfid_status_2; /*!< RM2026 New Field */
} rfid_status_t;

/**
 * @brief 0x020A: Dart Client Cmd
 */
typedef struct
{
  uint8_t  dart_launch_opening_status;
  uint8_t  reserved;
  uint16_t target_change_time;
  uint16_t latest_launch_cmd_time;
} dart_client_cmd_t;

/**
 * @brief 0x020B: Ground Robot Position
 */
typedef struct
{
  float hero_x;
  float hero_y;
  float engineer_x;
  float engineer_y;
  float standard_3_x;
  float standard_3_y;
  float standard_4_x;
  float standard_4_y;
  float reserved1;
  float reserved2;
} ground_robot_position_t;

/**
 * @brief 0x020C: Radar Marking Data
 */
typedef struct
{
  uint16_t mark_progress; 
} radar_mark_data_t;

/**
 * @brief 0x020D: Sentry Autonomous Decision Info (Updated in RM2026: 14 Bytes)
 */
typedef struct
{
  uint32_t sentry_info;  
  uint16_t sentry_info_2;
  uint64_t sentry_info_3; /*!< RM2026 New Field */
} sentry_info_t;

/**
 * @brief 0x020E: Radar Info
 */
typedef struct
{
  uint8_t radar_info;
} radar_info_t;

/**
 * @brief 0x0301: Robot Interaction Data
 */
typedef struct { 
  uint16_t data_cmd_id;
  uint16_t sender_id;
  uint16_t receiver_id;
  uint8_t  user_data[112];
} robot_interaction_data_t;

/**
 * @brief 0x0303: Mini-map Interactive Cmd
 */
typedef struct
{
  float    target_position_x;
  float    target_position_y;
  uint8_t  cmd_keyboard;
  uint8_t  target_robot_id;   
  uint16_t cmd_source;
} map_command_t;

/**
 * @brief 0x0305: Mini-map Robot Position Data
 */
typedef struct
{
  uint16_t opponent_hero_position_x;
  uint16_t opponent_hero_position_y;
  uint16_t opponent_engineer_position_x;
  uint16_t opponent_engineer_position_y;
  uint16_t opponent_infantry_3_position_x;
  uint16_t opponent_infantry_3_position_y;
  uint16_t opponent_infantry_4_position_x;
  uint16_t opponent_infantry_4_position_y;
  uint16_t opponent_aerial_position_x;
  uint16_t opponent_aerial_position_y;
  uint16_t opponent_sentry_position_x;
  uint16_t opponent_sentry_position_y;

  uint16_t ally_hero_position_x;
  uint16_t ally_hero_position_y;
  uint16_t ally_engineer_position_x;
  uint16_t ally_engineer_position_y;
  uint16_t ally_infantry_3_position_x;
  uint16_t ally_infantry_3_position_y;
  uint16_t ally_infantry_4_position_x;
  uint16_t ally_infantry_4_position_y;
  uint16_t ally_aerial_position_x;
  uint16_t ally_aerial_position_y;
  uint16_t ally_sentry_position_x;
  uint16_t ally_sentry_position_y;
} map_robot_data_t;

/**
 * @brief 0x0307: Sentry / Auto Path Data
 */
typedef struct
{
  uint8_t  intention;
  uint16_t start_position_x;
  uint16_t start_position_y;
  int8_t   delta_x[49];
  int8_t   delta_y[49];
  uint16_t sender_id;
} map_data_t;

/**
 * @brief 0x0308: Custom Info
 */
typedef struct
{
  uint16_t sender_id;
  uint16_t receiver_id;
  uint8_t  user_data[30];
} custom_info_t;

/**
 * @brief Referee System Info Main Typedef Structure
 * @note  裁判系统解析结果的「总结构体」。内部用 #ifdef 按 CMD_ID 宏条件编译，
 *        只把工程真正用到的那些数据类型包含进来（用到的 CMD_ID 会定义成宏）。
 *        Index/DataLength 是解析过程用的临时游标，其余成员按协议对应各 CMD_ID 的数据。
 */
typedef struct
{
  uint8_t  Index;
  uint16_t DataLength;

#ifdef GAME_STATUS_ID
  game_status_t game_status;
#endif

#ifdef GAME_RESULT_ID
  game_result_t game_result;
#endif

#ifdef GAME_ROBOT_HP_ID
  game_robot_HP_t game_robot_HP;
#endif  

#ifdef EVENET_DATA_ID
  event_data_t event_data;
#endif

#ifdef REFEREE_WARNING_ID
  referee_warning_t referee_warning;
#endif

#ifdef DART_INFO_ID
  dart_info_t dart_info;
#endif

#ifdef ROBOT_STATUS_ID
  robot_status_t robot_status;
#endif

#ifdef POWER_HEAT_DATA_ID
  power_heat_data_t power_heat_data;
#endif

#ifdef ROBOT_POS_ID
  robot_pos_t robot_pos;
#endif

#ifdef BUFF_ID
  buff_t buff;
#endif

#ifdef HURT_DATA_ID
  hurt_data_t hurt_data;
#endif

#ifdef SHOOT_DATA_ID
  shoot_data_t shoot_data;
#endif

#ifdef PROJECTILE_ALLOWANCE_ID
  projectile_allowance_t projectile_allowance;
#endif

#ifdef RFID_STATUS_ID
  rfid_status_t rfid_status;
#endif

#ifdef DART_CLIENT_CMD_ID
  dart_client_cmd_t dart_client_cmd;
#endif

#ifdef GROUND_ROBOT_POSITION_ID
  ground_robot_position_t ground_robot_position;
#endif 

#ifdef RADAR_MARAKING_DATA_ID
  radar_mark_data_t radar_mark_data;
#endif

#ifdef SENTRY_INFO_ID
  sentry_info_t sentry_info;
#endif

#ifdef RADAR_INFO_ID
  radar_info_t radar_info;
#endif

#ifdef ROBOT_INTERACTION_DATA_ID
  robot_interaction_data_t robot_interaction_data;
#endif

#ifdef MAP_COMMAND_ID
  map_command_t map_command;
#endif

#ifdef MAP_ROBOT_DATA_ID
  map_robot_data_t map_robot_data;
#endif

#ifdef MAP_DATA_ID
  map_data_t map_data;
#endif

#ifdef CUSTOM_INFO_ID
  custom_info_t custom_info;
#endif
} Referee_System_Info_TypeDef;

#pragma pack()

/* Exported variables ---------------------------------------------------------*/
extern uint8_t Referee_System_Info_MultiRx_Buf[2][REFEREE_RXFRAME_LENGTH];
extern Referee_System_Info_TypeDef Referee_System_Info;

/* Exported functions prototypes ---------------------------------------------*/
extern void Referee_System_Frame_Update(uint8_t *Buff);
void Referee_System_Info_Update(uint8_t *Buff, Referee_System_Info_TypeDef *Referee_System_Info);
extern void Referee_System_Data_Update(Referee_System_Info_TypeDef *ref_data);

#endif // REFEREE_INFO_H
