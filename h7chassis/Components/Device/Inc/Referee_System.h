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
 * @note  CMD_ID 是每帧数据区的"类型标签"，接收端 switch 按它决定怎么解析。
 *        这些宏在本工程里全部 #define 了，所以 Referee_System_Info 会包含所有类型，
 *        但底盘实际只用到其中几个（带 ★ 的）：ROBOT_STATUS、POWER_HEAT_DATA、
 *        SHOOT_DATA、ROBOT_POS、BUFF、GAME_ROBOT_HP、PROJECTILE_ALLOWANCE。
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
 * @brief 0x0001: Game Status（比赛状态）
 * @note  game_type 和 game_progress 是两个 4 位的「位域」，共用一个字节 p_data[0]：
 *        低 4 位是 game_type，高 4 位是 game_progress，见 .c 里的解析。
 */
typedef struct
{
  uint8_t  game_type : 4;        /*!< 比赛类型：0=机甲大师赛 1=机甲大师单项赛 2=联盟赛… */
  uint8_t  game_progress : 4;    /*!< 比赛阶段：0=未开始 1=准备阶段 2=自检 3=5秒倒计时 4=进行中 5=结算 */
  uint16_t stage_remain_time;    /*!< 当前阶段剩余时间，单位秒 */
  uint64_t SyncTimeStamp;        /*!< 同步时间戳(Unix 时间)，可用于多机时间同步 */
} game_status_t;

/**
 * @brief 0x0002: Game Result
 */
typedef struct
{
  uint8_t winner; /*!< 比赛结果：0=平局 1=红方胜 2=蓝方胜 */
} game_result_t;

/**
 * @brief 0x0003: Robot HP Data（全场血量信息，RM2026 共 20 字节）
 * @note  「ally」= 我方(己方队伍)机器人，「enemy」= 敌方。1~7 号是机器人编号，
 *         不同编号对应不同兵种(1英雄 2工程 3/4步兵 7哨兵)。血量为 0 表示已阵亡。
 */
typedef struct
{
  uint16_t ally_1_robot_HP;   /*!< 我方 1 号(英雄) 血量 */
  uint16_t ally_2_robot_HP;   /*!< 我方 2 号(工程) 血量 */
  uint16_t ally_3_robot_HP;   /*!< 我方 3 号(步兵) 血量 */
  uint16_t ally_4_robot_HP;   /*!< 我方 4 号(步兵) 血量 */
  int16_t  damage_difference; /*!< 伤害差 = 我方总伤害 - 敌方总伤害(有符号) */
  uint16_t ally_7_robot_HP;   /*!< 我方 7 号(哨兵) 血量 */
  uint16_t ally_outpost_HP;   /*!< 我方前哨站血量 */
  uint16_t ally_base_HP;      /*!< 我方基地血量 */
  uint16_t enemy_outpost_HP;  /*!< 敌方前哨站血量 */
  uint16_t enemy_base_HP;     /*!< 敌方基地血量 */
} game_robot_HP_t;

/**
 * @brief 0x0101: Site Event Data
 */
typedef union
{
  uint32_t event_data; /*!< 场地事件标志(32位位图，每一位代表一种场地事件) */
} event_data_t;

/**
 * @brief 0x0104: Referee Warning
 */
typedef struct 
{ 
  uint8_t level;               /*!< 警告等级 */
  uint8_t offending_robot_id;  /*!< 违规机器人编号 */
  uint8_t count;               /*!< 违规次数 */
} referee_warning_t; 

/**
 * @brief 0x0105: Dart Info
 */
typedef struct
{
  uint8_t  dart_remaining_time; /*!< 飞镖剩余发射时间(秒) */
  uint16_t dart_info;           /*!< 飞镖状态信息(位标志) */
} dart_info_t;

/**
 * @brief 0x0201: Robot Status（本机状态，RM2026 共 17 字节）★本项目最核心的结构体★
 * @note  这是底盘最常用的数据：功率限制、血量、枪口热量都从这里来。
 */
typedef struct
{
  uint8_t  robot_id;                        /*!< 本机机器人编号(1~7，决定兵种)，也用于图传/交互的收发 ID */
  uint8_t  robot_level;                     /*!< 本机等级(升级增益用) */
  uint16_t current_HP;                      /*!< 当前血量 */
  uint16_t maximum_HP;                      /*!< 最大血量(满血值) */
  uint16_t shooter_barrel_cooling_value;    /*!< 枪口冷却值(每秒散热多少，17mm 枪管) */
  uint16_t shooter_barrel_heat_limit;       /*!< 枪口热量上限(超过这个值就不能开火) */
  uint16_t chassis_power_limit;             /*!< ★底盘功率上限(单位 W)，功率环就按它限制总输出 */
  float    bullet_speed_limit;              /*!< 弹速上限(m/s)，RM2026 新增字段 */
  uint8_t  power_management_gimbal_output : 1;   /*!< 云台输出使能(1=允许云台用电) */
  uint8_t  power_management_chassis_output : 1;  /*!< ★底盘输出使能(1=允许底盘用电)，超功率会被裁判关掉 */
  uint8_t  power_management_shooter_output : 1;  /*!< 发射机构输出使能 */
} robot_status_t;

/**
 * @brief 0x0202: Power Heat Data（功率与热量实时数据，RM2026 共 14 字节）★本项目核心结构体★
 * @note  buffer_energy(缓冲能量) 和 shooter_*_barrel_heat(枪口热量) 是功率环/射速控制的依据。
 */
typedef struct
{
  uint16_t reserved1;                 /*!< 保留位 1(旧版协议里的意义已废弃，不要用) */
  uint16_t reserved2;                 /*!< 保留位 2 */
  float    reserved3;                 /*!< 保留位 3(float) */
  uint16_t buffer_energy;             /*!< ★缓冲能量(J)，步兵/英雄满值默认 60J、飞坡增益期间 250J，用于功率环"回血" */
  uint16_t shooter_17mm_barrel_heat;  /*!< 17mm 枪管当前热量(接近上限就不能再开火) */
  uint16_t shooter_42mm_barrel_heat;  /*!< 42mm 枪管当前热量(本工程用于上位机显示) */
} power_heat_data_t;

/**
 * @brief 0x0203: Robot Position Data
 */
typedef struct
{
  float x;     /*!< X 坐标(m)，裁判地图坐标系 */
  float y;     /*!< Y 坐标(m) */
  float angle; /*!< 朝向角(度) */
} robot_pos_t;

/**
 * @brief 0x0204: Robot Buff Data（机器人增益状态，吃 BUFF 后这里会变）
 */
typedef struct
{
  uint8_t  recovery_buff;      /*!< 回血增益(每秒回多少血) */
  uint16_t cooling_buff;       /*!< 冷却增益(枪口散热加成，RM2026 改为 16 位) */
  uint8_t  defence_buff;       /*!< 防御增益(减伤) */
  uint8_t  vulnerability_buff; /*!< 易伤状态(受到的伤害增加，通常出现在某阶段) */
  uint16_t attack_buff;        /*!< 攻击增益(伤害加成) */
  uint8_t  remaining_energy;   /*!< 剩余能量(增益消耗的能量) */
} buff_t;

/**
 * @brief 0x0206: Hurt Data（受伤信息）
 * @note  两个 4 位位域共用一个字节：低 4 位 armor_id，高 4 位 HP_deduction_reason。
 */
typedef struct
{
  uint8_t armor_id : 4;            /*!< 被击中的装甲板编号(哪块装甲板被打) */
  uint8_t HP_deduction_reason : 4; /*!< 扣血原因(0=装甲 1=模块 2=基地 3=违规等) */
} hurt_data_t;

/**
 * @brief 0x0207: Real-time Shoot Data（实时射击数据）★本项目用到(发给云台)★
 */
typedef struct
{
  uint8_t bullet_type;        /*!< 弹种(17mm / 42mm / 飞镖等) */
  uint8_t shooter_number;     /*!< 发射机构编号(哪个枪口在打) */
  uint8_t launching_frequency; /*!< 当前射速(发/秒) */
  float   initial_speed;      /*!< 弹丸初速(m/s)，本工程通过 CAN 0x23 转发给云台 */
} shoot_data_t;

/**
 * @brief 0x0208: Projectile Allowance（弹丸余量，RM2026 共 8 字节）
 */
typedef struct
{
  uint16_t projectile_allowance_17mm;      /*!< 17mm 弹丸剩余可发射数 */
  uint16_t projectile_allowance_42mm;      /*!< 42mm 弹丸剩余可发射数 */
  uint16_t remaining_gold_coin;            /*!< 剩余金币(用于购买增益/弹丸) */
  uint16_t projectile_allowance_fortress;  /*!< 堡垒弹丸余量(RM2026 新增) */
} projectile_allowance_t;

/**
 * @brief 0x0209: RFID Status (Updated in RM2026: 5 Bytes)
 */
typedef struct
{
  uint32_t rfid_status;  /*!< RFID 刷卡状态(32位位图) */
  uint8_t  rfid_status_2; /*!< RM2026 New Field */
} rfid_status_t;

/**
 * @brief 0x020A: Dart Client Cmd
 */
typedef struct
{
  uint8_t  dart_launch_opening_status; /*!< 飞镖发射口开启状态 */
  uint8_t  reserved;                   /*!< 保留位 */
  uint16_t target_change_time;         /*!< 目标切换时间 */
  uint16_t latest_launch_cmd_time;     /*!< 最近一次发射命令时间 */
} dart_client_cmd_t;

/**
 * @brief 0x020B: Ground Robot Position
 */
typedef struct
{
  float hero_x;        /*!< 英雄机器人 X 坐标(m) */
  float hero_y;        /*!< 英雄机器人 Y 坐标(m) */
  float engineer_x;    /*!< 工程机器人 X 坐标(m) */
  float engineer_y;    /*!< 工程机器人 Y 坐标(m) */
  float standard_3_x;  /*!< 3号步兵 X 坐标(m) */
  float standard_3_y;  /*!< 3号步兵 Y 坐标(m) */
  float standard_4_x;  /*!< 4号步兵 X 坐标(m) */
  float standard_4_y;  /*!< 4号步兵 Y 坐标(m) */
  float reserved1;     /*!< 保留位 1 */
  float reserved2;     /*!< 保留位 2 */
} ground_robot_position_t;

/**
 * @brief 0x020C: Radar Marking Data
 */
typedef struct
{
  uint16_t mark_progress; /*!< 雷达标定进度 */
} radar_mark_data_t;

/**
 * @brief 0x020D: Sentry Autonomous Decision Info (Updated in RM2026: 14 Bytes)
 */
typedef struct
{
  uint32_t sentry_info;  /*!< 哨兵自主决策信息 */
  uint16_t sentry_info_2; /*!< 哨兵自主决策信息 2 */
  uint64_t sentry_info_3; /*!< RM2026 New Field */
} sentry_info_t;

/**
 * @brief 0x020E: Radar Info
 */
typedef struct
{
  uint8_t radar_info; /*!< 雷达自主决策信息 */
} radar_info_t;

/**
 * @brief 0x0301: Robot Interaction Data
 */
typedef struct { 
  uint16_t data_cmd_id;    /*!< 交互数据命令 ID */
  uint16_t sender_id;      /*!< 发送方机器人编号 */
  uint16_t receiver_id;    /*!< 接收方机器人编号 */
  uint8_t  user_data[112]; /*!< 用户自定义数据(112字节) */
} robot_interaction_data_t;

/**
 * @brief 0x0303: Mini-map Interactive Cmd
 */
typedef struct
{
  float    target_position_x; /*!< 目标位置 X 坐标 */
  float    target_position_y; /*!< 目标位置 Y 坐标 */
  uint8_t  cmd_keyboard;      /*!< 键盘指令 */
  uint8_t  target_robot_id;   /*!< 目标机器人编号 */
  uint16_t cmd_source;        /*!< 指令来源 */
} map_command_t;

/**
 * @brief 0x0305: Mini-map Robot Position Data
 */
typedef struct
{
  uint16_t opponent_hero_position_x;       /*!< 敌方英雄 X 坐标 */
  uint16_t opponent_hero_position_y;       /*!< 敌方英雄 Y 坐标 */
  uint16_t opponent_engineer_position_x;   /*!< 敌方工程 X 坐标 */
  uint16_t opponent_engineer_position_y;   /*!< 敌方工程 Y 坐标 */
  uint16_t opponent_infantry_3_position_x; /*!< 敌方3号步兵 X 坐标 */
  uint16_t opponent_infantry_3_position_y; /*!< 敌方3号步兵 Y 坐标 */
  uint16_t opponent_infantry_4_position_x; /*!< 敌方4号步兵 X 坐标 */
  uint16_t opponent_infantry_4_position_y; /*!< 敌方4号步兵 Y 坐标 */
  uint16_t opponent_aerial_position_x;     /*!< 敌方空中机器人 X 坐标 */
  uint16_t opponent_aerial_position_y;     /*!< 敌方空中机器人 Y 坐标 */
  uint16_t opponent_sentry_position_x;     /*!< 敌方哨兵 X 坐标 */
  uint16_t opponent_sentry_position_y;     /*!< 敌方哨兵 Y 坐标 */

  uint16_t ally_hero_position_x;           /*!< 我方英雄 X 坐标 */
  uint16_t ally_hero_position_y;           /*!< 我方英雄 Y 坐标 */
  uint16_t ally_engineer_position_x;       /*!< 我方工程 X 坐标 */
  uint16_t ally_engineer_position_y;       /*!< 我方工程 Y 坐标 */
  uint16_t ally_infantry_3_position_x;     /*!< 我方3号步兵 X 坐标 */
  uint16_t ally_infantry_3_position_y;     /*!< 我方3号步兵 Y 坐标 */
  uint16_t ally_infantry_4_position_x;     /*!< 我方4号步兵 X 坐标 */
  uint16_t ally_infantry_4_position_y;     /*!< 我方4号步兵 Y 坐标 */
  uint16_t ally_aerial_position_x;         /*!< 我方空中机器人 X 坐标 */
  uint16_t ally_aerial_position_y;         /*!< 我方空中机器人 Y 坐标 */
  uint16_t ally_sentry_position_x;         /*!< 我方哨兵 X 坐标 */
  uint16_t ally_sentry_position_y;         /*!< 我方哨兵 Y 坐标 */
} map_robot_data_t;

/**
 * @brief 0x0307: Sentry / Auto Path Data
 */
typedef struct
{
  uint8_t  intention;         /*!< 意图(移动/进攻等) */
  uint16_t start_position_x;  /*!< 路径起点 X 坐标 */
  uint16_t start_position_y;  /*!< 路径起点 Y 坐标 */
  int8_t   delta_x[49];       /*!< 路径点 X 方向偏移序列(49个点) */
  int8_t   delta_y[49];       /*!< 路径点 Y 方向偏移序列(49个点) */
  uint16_t sender_id;         /*!< 发送方机器人编号 */
} map_data_t;

/**
 * @brief 0x0308: Custom Info
 */
typedef struct
{
  uint16_t sender_id;     /*!< 发送方机器人编号 */
  uint16_t receiver_id;   /*!< 接收方机器人编号 */
  uint8_t  user_data[30]; /*!< 用户自定义数据(30字节) */
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
