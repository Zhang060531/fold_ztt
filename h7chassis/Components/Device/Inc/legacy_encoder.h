#ifndef LEGACY_ENCODER_H
#define LEGACY_ENCODER_H
#include "stm32h7xx_hal.h"

#include <stdbool.h>
#include <stdint.h>

typedef enum {
		LEGACY_ENCODER_EVENT_NONE = 0,  //不管
    LEGACY_ENCODER_EVENT_ANGLE,     //收到角度了 dev->angle_deg已更新
    LEGACY_ENCODER_EVENT_ID,        //收到读ID结果 dev->reported_id已更新
    LEGACY_ENCODER_EVENT_ACK,       //收到"ok!" 设ID/设频率成功
    LEGACY_ENCODER_EVENT_ERROR      //参数非法(空指针/非标准帧/非数据帧)
} LegacyEncoderEvent;

//一个编码器的(状态+数据)结构体
typedef struct {
    FDCAN_HandleTypeDef *hcan;      //挂在哪一路CAN（fdcan2）
    uint16_t node_id;               //当前编码器节点号(发帧ID)
    uint16_t pending_node_id;       //
    uint8_t last_command;           //
    float angle_deg;                //
		uint32_t reported_id;           //这个没啥用好像
} LegacyEncoder;

void LegacyEncoder_Init(LegacyEncoder *dev, FDCAN_HandleTypeDef *hcan, uint16_t node_id);
bool LegacyEncoder_SetId(LegacyEncoder *dev, uint16_t new_id);
bool LegacyEncoder_ReadAngleOnce(LegacyEncoder *dev);
bool LegacyEncoder_StartAuto(LegacyEncoder *dev);
bool LegacyEncoder_Stop(LegacyEncoder *dev);
bool LegacyEncoder_ReadId(LegacyEncoder *dev);
bool LegacyEncoder_SetPeriodCode(LegacyEncoder *dev, uint8_t code);
bool LegacyEncoder_SetZero(LegacyEncoder *dev);
LegacyEncoderEvent LegacyEncoder_OnCanFrame(LegacyEncoder *dev,
                                             const FDCAN_RxHeaderTypeDef *header,
                                             const uint8_t data[8]);

#endif
