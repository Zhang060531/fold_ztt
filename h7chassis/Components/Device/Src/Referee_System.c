/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : Referee_System_Info.c
  * @brief          : Referee_System_Info interfaces functions (RM2026 V2.0.0)
  * @author         : GrassFan Wang
  * @date           : 2026/07/25
  * @version        : v2.0
  ******************************************************************************
  */
/* USER CODE END Header */

/**
 ******************************************************************************
 * @file    Referee_System.c
 * @brief   裁判系统串口数据解析实现
 *
 * 【本文件做什么】
 * 把串口收到的原始字节流，按照 RoboMaster 裁判系统协议解析成结构化数据。
 * 三个核心函数：
 *   - Referee_System_Frame_Update()：帧校验入口（CRC8 校验帧头 + CRC16 校验整帧）；
 *   - Referee_System_Info_Update()：按 CMD_ID 分发解析数据区到对应结构体；
 *   - Referee_System_Data_Update()：上层拷贝最新裁判数据。
 *
 * 【字节序】
 * 协议多字节字段为小端（低字节在前）。辅助函数 bit8TObit16/32/64/float32
 * 用 union 把字节数组按小端重新拼成目标类型，避免手动移位写错。
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "Referee_System.h"
#include "CRC.h"
#include "string.h"

/* Exported variables ---------------------------------------------------------*/
__attribute__((section (".AXI_SRAM"))) uint8_t Referee_System_Info_MultiRx_Buf[2][REFEREE_RXFRAME_LENGTH];
Referee_System_Info_TypeDef Referee_System_Info;

/* Private function prototypes -----------------------------------------------*/
static uint32_t bit8TObit32(uint8_t change_info[4]);
static uint64_t bit8TObit64(uint8_t change_info[8]);
static float    bit8TOfloat32(uint8_t change_info[4]);
static int16_t  bit8TObit16(uint8_t change_info[2]);

/**
  * @brief  解析接收缓冲区中的裁判系统数据帧（缓冲区里可能粘着多帧）
  * @param  *Buff: 指向接收帧缓冲区的指针
  * @retval None
  * @note   帧结构：帧头5字节 + CMD_ID 2字节 + 数据区 + CRC16 2字节。
  *         解析流程：
  *           1) while 循环每次定位帧头 0xA5；
  *           2) 用 CRC8 校验帧头 5 字节，失败则 break 结束本轮解析；
  *           3) 从帧头第 2、3 字节读出数据区长度(小端)，再加上帧头/CMD_ID/CRC16 长度
  *              得到整帧总长 DataLength；
  *           4) 用 CRC16 校验整帧，通过才调用 Referee_System_Info_Update 解析数据；
  *           5) Index 前移一个整帧长度，继续解析下一帧。
  */
void Referee_System_Frame_Update(uint8_t *Buff)
{
  Referee_System_Info.Index = 0;
  Referee_System_Info.DataLength = 0;

  while (Buff[Referee_System_Info.Index] == 0xA5)
  {
    /* CRC8 verification */
    if (Verify_CRC8_Check_Sum(&Buff[Referee_System_Info.Index], FrameHeader_Length) == true)
    {
      /* Update data length */
      Referee_System_Info.DataLength = (uint16_t)(Buff[Referee_System_Info.Index+2] << 8 | Buff[Referee_System_Info.Index+1]) 
                                        + FrameHeader_Length + CMDID_Length + CRC16_Length;

      /* CRC16 verification */
      if (Verify_CRC16_Check_Sum(&Buff[Referee_System_Info.Index], Referee_System_Info.DataLength) == true)
      {
        Referee_System_Info_Update(Buff, &Referee_System_Info);
      }
    }
    else
    {
      break;
    }

    Referee_System_Info.Index += Referee_System_Info.DataLength;
  }
}

/**
  * @brief  按 CMD_ID 解析数据区，把字节流填进对应结构体
  * @param  *Buff: 指向整帧缓冲区
  * @param  *Referee_System_Info: 解析结果写入的目标结构体
  * @retval None
  * @note   先取出 CMD_ID（帧头后 2 字节），再让 p_data 指向数据区首地址
  *         （帧头5字节 + CMD_ID 2字节之后），然后用 switch 按 CMD_ID 分发。
  *         每个 case 里的下标就是该字段在数据区里的字节偏移（从 0 开始），
  *         多字节字段用 bit8TObit16/32/float32 按小端拼装。
  */
void Referee_System_Info_Update(uint8_t *Buff, Referee_System_Info_TypeDef *Referee_System_Info)
{
  uint16_t cmd_id = bit8TObit16(&Buff[Referee_System_Info->Index + FrameHeader_Length]);
  uint8_t *p_data = &Buff[Referee_System_Info->Index + FrameHeader_Length + CMDID_Length];

  switch (cmd_id)
  {
#ifdef GAME_STATUS_ID 
    case GAME_STATUS_ID:
      Referee_System_Info->game_status.game_type          = p_data[0] & 0x0F;
      Referee_System_Info->game_status.game_progress      = (p_data[0] & 0xF0) >> 4;
      Referee_System_Info->game_status.stage_remain_time = bit8TObit16(&p_data[1]);
      Referee_System_Info->game_status.SyncTimeStamp     = bit8TObit64(&p_data[3]);
      break;
#endif

#ifdef GAME_RESULT_ID
    case GAME_RESULT_ID:
      Referee_System_Info->game_result.winner = p_data[0];
      break;
#endif

#ifdef GAME_ROBOT_HP_ID
    case GAME_ROBOT_HP_ID:
      Referee_System_Info->game_robot_HP.ally_1_robot_HP  = bit8TObit16(&p_data[0]);
      Referee_System_Info->game_robot_HP.ally_2_robot_HP  = bit8TObit16(&p_data[2]);
      Referee_System_Info->game_robot_HP.ally_3_robot_HP  = bit8TObit16(&p_data[4]);
      Referee_System_Info->game_robot_HP.ally_4_robot_HP  = bit8TObit16(&p_data[6]);
      Referee_System_Info->game_robot_HP.damage_difference= bit8TObit16(&p_data[8]);
      Referee_System_Info->game_robot_HP.ally_7_robot_HP  = bit8TObit16(&p_data[10]);
      Referee_System_Info->game_robot_HP.ally_outpost_HP  = bit8TObit16(&p_data[12]);
      Referee_System_Info->game_robot_HP.ally_base_HP     = bit8TObit16(&p_data[14]);
      Referee_System_Info->game_robot_HP.enemy_outpost_HP = bit8TObit16(&p_data[16]);
      Referee_System_Info->game_robot_HP.enemy_base_HP    = bit8TObit16(&p_data[18]);
      break;
#endif

#ifdef EVENET_DATA_ID
    case EVENET_DATA_ID:
      Referee_System_Info->event_data.event_data = bit8TObit32(&p_data[0]);
      break;
#endif

#ifdef REFEREE_WARNING_ID
    case REFEREE_WARNING_ID:
      Referee_System_Info->referee_warning.level               = p_data[0];
      Referee_System_Info->referee_warning.offending_robot_id = p_data[1];
      Referee_System_Info->referee_warning.count              = p_data[2];
      break;
#endif

#ifdef DART_INFO_ID
    case DART_INFO_ID:
      Referee_System_Info->dart_info.dart_remaining_time = p_data[0];
      Referee_System_Info->dart_info.dart_info           = bit8TObit16(&p_data[1]);
      break;
#endif

#ifdef ROBOT_STATUS_ID
    case ROBOT_STATUS_ID:
      Referee_System_Info->robot_status.robot_id                        = p_data[0];
      Referee_System_Info->robot_status.robot_level                     = p_data[1];
      Referee_System_Info->robot_status.current_HP                      = bit8TObit16(&p_data[2]);
      Referee_System_Info->robot_status.maximum_HP                      = bit8TObit16(&p_data[4]);
      Referee_System_Info->robot_status.shooter_barrel_cooling_value    = bit8TObit16(&p_data[6]);
      Referee_System_Info->robot_status.shooter_barrel_heat_limit       = bit8TObit16(&p_data[8]);
      Referee_System_Info->robot_status.chassis_power_limit             = bit8TObit16(&p_data[10]);
      Referee_System_Info->robot_status.bullet_speed_limit              = bit8TOfloat32(&p_data[12]);
      Referee_System_Info->robot_status.power_management_gimbal_output  = (p_data[16] & 0x01);
      Referee_System_Info->robot_status.power_management_chassis_output = (p_data[16] & 0x02) >> 1;
      Referee_System_Info->robot_status.power_management_shooter_output = (p_data[16] & 0x04) >> 2;
      break;
#endif

#ifdef POWER_HEAT_DATA_ID
    case POWER_HEAT_DATA_ID:
      Referee_System_Info->power_heat_data.buffer_energy             = bit8TObit16(&p_data[8]);
      Referee_System_Info->power_heat_data.shooter_17mm_barrel_heat = bit8TObit16(&p_data[10]);
      Referee_System_Info->power_heat_data.shooter_42mm_barrel_heat = bit8TObit16(&p_data[12]);
      break;
#endif

#ifdef ROBOT_POS_ID
    case ROBOT_POS_ID:
      Referee_System_Info->robot_pos.x     = bit8TOfloat32(&p_data[0]);
      Referee_System_Info->robot_pos.y     = bit8TOfloat32(&p_data[4]);
      Referee_System_Info->robot_pos.angle = bit8TOfloat32(&p_data[8]);
      break;
#endif

#ifdef BUFF_ID
    case BUFF_ID:
      Referee_System_Info->buff.recovery_buff      = p_data[0];
      Referee_System_Info->buff.cooling_buff       = bit8TObit16(&p_data[1]);
      Referee_System_Info->buff.defence_buff       = p_data[3];
      Referee_System_Info->buff.vulnerability_buff = p_data[4];
      Referee_System_Info->buff.attack_buff        = bit8TObit16(&p_data[5]);
      Referee_System_Info->buff.remaining_energy   = p_data[7];
      break;
#endif

#ifdef HURT_DATA_ID
    case HURT_DATA_ID:
      Referee_System_Info->hurt_data.armor_id            = p_data[0] & 0x0F;
      Referee_System_Info->hurt_data.HP_deduction_reason = (p_data[0] & 0xF0) >> 4;
      break;
#endif

#ifdef SHOOT_DATA_ID
    case SHOOT_DATA_ID:
      Referee_System_Info->shoot_data.bullet_type          = p_data[0];
      Referee_System_Info->shoot_data.shooter_number      = p_data[1];
      Referee_System_Info->shoot_data.launching_frequency = p_data[2];
      Referee_System_Info->shoot_data.initial_speed       = bit8TOfloat32(&p_data[3]);
      break;
#endif

#ifdef PROJECTILE_ALLOWANCE_ID
    case PROJECTILE_ALLOWANCE_ID:
      Referee_System_Info->projectile_allowance.projectile_allowance_17mm     = bit8TObit16(&p_data[0]);
      Referee_System_Info->projectile_allowance.projectile_allowance_42mm     = bit8TObit16(&p_data[2]);
      Referee_System_Info->projectile_allowance.remaining_gold_coin           = bit8TObit16(&p_data[4]);
      Referee_System_Info->projectile_allowance.projectile_allowance_fortress = bit8TObit16(&p_data[6]);
      break;
#endif

#ifdef RFID_STATUS_ID
    case RFID_STATUS_ID:
      Referee_System_Info->rfid_status.rfid_status   = bit8TObit32(&p_data[0]);
      Referee_System_Info->rfid_status.rfid_status_2 = p_data[4];
      break;
#endif

#ifdef DART_CLIENT_CMD_ID
    case DART_CLIENT_CMD_ID:
      Referee_System_Info->dart_client_cmd.dart_launch_opening_status = p_data[0];
      Referee_System_Info->dart_client_cmd.reserved                    = p_data[1];
      Referee_System_Info->dart_client_cmd.target_change_time        = bit8TObit16(&p_data[2]);
      Referee_System_Info->dart_client_cmd.latest_launch_cmd_time    = bit8TObit16(&p_data[4]);
      break;
#endif

#ifdef GROUND_ROBOT_POSITION_ID
    case GROUND_ROBOT_POSITION_ID:
      Referee_System_Info->ground_robot_position.hero_x       = bit8TOfloat32(&p_data[0]);
      Referee_System_Info->ground_robot_position.hero_y       = bit8TOfloat32(&p_data[4]);
      Referee_System_Info->ground_robot_position.engineer_x   = bit8TOfloat32(&p_data[8]);
      Referee_System_Info->ground_robot_position.engineer_y   = bit8TOfloat32(&p_data[12]);
      Referee_System_Info->ground_robot_position.standard_3_x = bit8TOfloat32(&p_data[16]);
      Referee_System_Info->ground_robot_position.standard_3_y = bit8TOfloat32(&p_data[20]);
      Referee_System_Info->ground_robot_position.standard_4_x = bit8TOfloat32(&p_data[24]);
      Referee_System_Info->ground_robot_position.standard_4_y = bit8TOfloat32(&p_data[28]);
      break;
#endif

#ifdef RADAR_MARAKING_DATA_ID
    case RADAR_MARAKING_DATA_ID:
      Referee_System_Info->radar_mark_data.mark_progress = bit8TObit16(&p_data[0]);
      break;
#endif

#ifdef SENTRY_INFO_ID
    case SENTRY_INFO_ID:
      Referee_System_Info->sentry_info.sentry_info   = bit8TObit32(&p_data[0]);
      Referee_System_Info->sentry_info.sentry_info_2 = bit8TObit16(&p_data[4]);
      Referee_System_Info->sentry_info.sentry_info_3 = bit8TObit64(&p_data[6]);
      break;
#endif

#ifdef RADAR_INFO_ID
    case RADAR_INFO_ID:
      Referee_System_Info->radar_info.radar_info = p_data[0];
      break;
#endif

#ifdef ROBOT_INTERACTION_DATA_ID
    case ROBOT_INTERACTION_DATA_ID:
      Referee_System_Info->robot_interaction_data.data_cmd_id = bit8TObit16(&p_data[0]);
      Referee_System_Info->robot_interaction_data.sender_id   = bit8TObit16(&p_data[2]);
      Referee_System_Info->robot_interaction_data.receiver_id = bit8TObit16(&p_data[4]);
      for (uint8_t i = 0; i < 112; i++) {
        Referee_System_Info->robot_interaction_data.user_data[i] = p_data[6 + i];
      }
      break;
#endif

#ifdef MAP_COMMAND_ID
    case MAP_COMMAND_ID:
      Referee_System_Info->map_command.target_position_x = bit8TOfloat32(&p_data[0]); 
      Referee_System_Info->map_command.target_position_y = bit8TOfloat32(&p_data[4]); 
      Referee_System_Info->map_command.cmd_keyboard      = p_data[8];
      Referee_System_Info->map_command.target_robot_id  = p_data[9];
      Referee_System_Info->map_command.cmd_source        = bit8TObit16(&p_data[10]);
      break;
#endif

#ifdef MAP_ROBOT_DATA_ID
    case MAP_ROBOT_DATA_ID:
      Referee_System_Info->map_robot_data.opponent_hero_position_x       = bit8TObit16(&p_data[0]);
      Referee_System_Info->map_robot_data.opponent_hero_position_y       = bit8TObit16(&p_data[2]);
      Referee_System_Info->map_robot_data.opponent_engineer_position_x   = bit8TObit16(&p_data[4]);
      Referee_System_Info->map_robot_data.opponent_engineer_position_y   = bit8TObit16(&p_data[6]);
      Referee_System_Info->map_robot_data.opponent_infantry_3_position_x = bit8TObit16(&p_data[8]);
      Referee_System_Info->map_robot_data.opponent_infantry_3_position_y = bit8TObit16(&p_data[10]);
      Referee_System_Info->map_robot_data.opponent_infantry_4_position_x = bit8TObit16(&p_data[12]);
      Referee_System_Info->map_robot_data.opponent_infantry_4_position_y = bit8TObit16(&p_data[14]);
      Referee_System_Info->map_robot_data.opponent_aerial_position_x     = bit8TObit16(&p_data[16]);
      Referee_System_Info->map_robot_data.opponent_aerial_position_y     = bit8TObit16(&p_data[18]);
      Referee_System_Info->map_robot_data.opponent_sentry_position_x     = bit8TObit16(&p_data[20]);
      Referee_System_Info->map_robot_data.opponent_sentry_position_y     = bit8TObit16(&p_data[22]);

      Referee_System_Info->map_robot_data.ally_hero_position_x           = bit8TObit16(&p_data[24]);
      Referee_System_Info->map_robot_data.ally_hero_position_y           = bit8TObit16(&p_data[26]);
      Referee_System_Info->map_robot_data.ally_engineer_position_x       = bit8TObit16(&p_data[28]);
      Referee_System_Info->map_robot_data.ally_engineer_position_y       = bit8TObit16(&p_data[30]);
      Referee_System_Info->map_robot_data.ally_infantry_3_position_x     = bit8TObit16(&p_data[32]);
      Referee_System_Info->map_robot_data.ally_infantry_3_position_y     = bit8TObit16(&p_data[34]);
      Referee_System_Info->map_robot_data.ally_infantry_4_position_x     = bit8TObit16(&p_data[36]);
      Referee_System_Info->map_robot_data.ally_infantry_4_position_y     = bit8TObit16(&p_data[38]);
      Referee_System_Info->map_robot_data.ally_aerial_position_x         = bit8TObit16(&p_data[40]);
      Referee_System_Info->map_robot_data.ally_aerial_position_y         = bit8TObit16(&p_data[42]);
      Referee_System_Info->map_robot_data.ally_sentry_position_x         = bit8TObit16(&p_data[44]);
      Referee_System_Info->map_robot_data.ally_sentry_position_y         = bit8TObit16(&p_data[46]);
      break;
#endif

#ifdef MAP_DATA_ID
    case MAP_DATA_ID:
      Referee_System_Info->map_data.intention        = p_data[0];
      Referee_System_Info->map_data.start_position_x = bit8TObit16(&p_data[1]);
      Referee_System_Info->map_data.start_position_y = bit8TObit16(&p_data[3]);
      for (uint8_t i = 0; i < 49; i++) {
        Referee_System_Info->map_data.delta_x[i] = p_data[5 + i];
      }
      for (uint8_t i = 0; i < 49; i++) {
        Referee_System_Info->map_data.delta_y[i] = p_data[54 + i];
      }
      Referee_System_Info->map_data.sender_id = bit8TObit16(&p_data[103]);
      break;
#endif

#ifdef CUSTOM_INFO_ID
    case CUSTOM_INFO_ID:
      Referee_System_Info->custom_info.sender_id   = bit8TObit16(&p_data[0]);
      Referee_System_Info->custom_info.receiver_id = bit8TObit16(&p_data[2]);
      for (uint8_t i = 0; i < 30; i++) {
        Referee_System_Info->custom_info.user_data[i] = p_data[4 + i];
      }
      break;
#endif 

    default:
      break;
  }
}

/**
 * @brief  把最新的裁判系统数据整体拷贝到调用者提供的结构体中
 * @param  ref_data：目标结构体指针
 * @retval 无
 * @note   简单整结构体赋值拷贝，上层任务拿它做功率限制/能量管理等决策。
 */
void Referee_System_Data_Update(Referee_System_Info_TypeDef *ref_data) {
  *ref_data = Referee_System_Info;
}

/* ---------- 字节拼装辅助函数 ----------
 * 协议里的多字节字段都是小端（低字节在前），下面几个函数用 union 把
 * 连续字节按小端重新解释成 16/32/64 位整数或 float，避免手动移位写错。
 * 例如 bit8TObit16(&p_data[2]) 会把 p_data[2](低字节) 和 p_data[3](高字节)
 * 拼成一个 int16_t。 */
static uint32_t bit8TObit32(uint8_t change_info[4])
{
  union {
    uint32_t bit32;
    uint8_t  byte[4];
  } u32val;

  u32val.byte[0] = change_info[0];
  u32val.byte[1] = change_info[1];
  u32val.byte[2] = change_info[2];
  u32val.byte[3] = change_info[3];

  return u32val.bit32;
}

static uint64_t bit8TObit64(uint8_t change_info[8])
{
  union {
    uint64_t bit64;
    uint8_t  byte[8];
  } u64val;

  for (uint8_t i = 0; i < 8; i++) {
    u64val.byte[i] = change_info[i];
  }

  return u64val.bit64;
}

static float bit8TOfloat32(uint8_t change_info[4])
{
  union {
    float float32;
    uint8_t byte[4];
  } u32val;

  u32val.byte[0] = change_info[0];
  u32val.byte[1] = change_info[1];
  u32val.byte[2] = change_info[2];
  u32val.byte[3] = change_info[3];

  return u32val.float32;
}

static int16_t bit8TObit16(uint8_t change_info[2])
{
  union {
    int16_t bit16;
    uint8_t byte[2];
  } u16val;

  u16val.byte[0] = change_info[0];
  u16val.byte[1] = change_info[1];

  return u16val.bit16;
}