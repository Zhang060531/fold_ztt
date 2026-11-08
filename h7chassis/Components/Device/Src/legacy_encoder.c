#include "legacy_encoder.h"
#include <string.h>

//共用底层发送函数
static bool send_frame(LegacyEncoder *dev, uint8_t command, uint8_t dlc,
                       uint16_t parameter) {
    FDCAN_TxHeaderTypeDef tx = {0};
		
    uint8_t data[8] = {0};

    if ((dev == NULL) || (dev->hcan == NULL)) return false;
    data[0] = command;
    if (dlc == 8U) {
        data[4] = (uint8_t)(parameter & 0xFFU);
        data[5] = (uint8_t)(parameter >> 8);
    }
    tx.Identifier = dev->node_id;
    tx.IdType = FDCAN_STANDARD_ID;
    tx.TxFrameType = FDCAN_DATA_FRAME;
    tx.DataLength = dlc;
		tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE;  //错误状态指示：主动错误 
    tx.BitRateSwitch = FDCAN_BRS_OFF;           //经典 CAN 不切换速率
    tx.FDFormat = FDCAN_CLASSIC_CAN;            // 经典 CAN 格式(不是 FD CAN) 
    tx.TxEventFifoControl = FDCAN_NO_TX_EVENTS; // 不记录发送事件
    tx.MessageMarker = 0;
    if (HAL_FDCAN_AddMessageToTxFifoQ(dev->hcan, &tx, data) != HAL_OK) return false;
    dev->last_command = command;
    return true;
}
//初始化 (绑定哪路CAN + 编码器的ID是多少)
void LegacyEncoder_Init(LegacyEncoder *dev, FDCAN_HandleTypeDef *hcan, uint16_t node_id) {
    if (dev == NULL) return;
    memset(dev, 0, sizeof(*dev));
    dev->hcan = hcan;
    dev->node_id = node_id;
    dev->pending_node_id = node_id;
}

//改编码器ID
bool LegacyEncoder_SetId(LegacyEncoder *dev, uint16_t new_id) {
    if ((dev == NULL) || (new_id > 0x00FFU)) return false;
    dev->pending_node_id = new_id;
    return send_frame(dev, 0x01U, 8U, new_id);
}
//单次读角度
bool LegacyEncoder_ReadAngleOnce(LegacyEncoder *dev) {
    return send_frame(dev, 0x02U, 1U, 0U);
}
//开始自动上报
bool LegacyEncoder_StartAuto(LegacyEncoder *dev) {
    return send_frame(dev, 0x03U, 1U, 0U);
}
//停止上报
bool LegacyEncoder_Stop(LegacyEncoder *dev) {
    return send_frame(dev, 0x04U, 1U, 0U);
}
//读编码器ID
bool LegacyEncoder_ReadId(LegacyEncoder *dev) {
    return send_frame(dev, 0x05U, 1U, 0U);
}
//设上报频率
bool LegacyEncoder_SetPeriodCode(LegacyEncoder *dev, uint8_t code) {
    if ((code < 1U) || (code > 5U)) return false;
    return send_frame(dev, 0x06U, 8U, code);
}
//设零点
bool LegacyEncoder_SetZero(LegacyEncoder *dev) {
    return send_frame(dev, 0x07U, 1U, 0U);
}
//接收解析（**核心，必须挂进接收回调**）
LegacyEncoderEvent LegacyEncoder_OnCanFrame(LegacyEncoder *dev,
                                             const FDCAN_RxHeaderTypeDef *header,
                                             const uint8_t data[8]) {
    uint16_t accepted_id;
    if ((dev == NULL) || (header == NULL) || (data == NULL) ||
        (header->IdType != FDCAN_STANDARD_ID) || (header->RxFrameType != FDCAN_DATA_FRAME)) {
        return LEGACY_ENCODER_EVENT_ERROR;
    }
    accepted_id = (dev->last_command == 0x01U) ? dev->pending_node_id : dev->node_id;
    if (header->Identifier != accepted_id || header->DataLength != 4U) return LEGACY_ENCODER_EVENT_NONE;

    if ((dev->last_command == 0x01U || dev->last_command == 0x06U) &&
        data[0] == 'o' && data[1] == 'k' && data[2] == '!') {
        if (dev->last_command == 0x01U) dev->node_id = dev->pending_node_id;
        return LEGACY_ENCODER_EVENT_ACK;
    }
    if (dev->last_command == 0x05U) {
        dev->reported_id = (uint32_t)data[0] | ((uint32_t)data[1] << 8) |
                           ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
        return LEGACY_ENCODER_EVENT_ID;
    }
    memcpy(&dev->angle_deg, data, sizeof(dev->angle_deg));
    return LEGACY_ENCODER_EVENT_ANGLE;
}
