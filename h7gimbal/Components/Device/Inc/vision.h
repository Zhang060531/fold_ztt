/**
 * =====================================================================
 * 文件说明：视觉数据结构与接口 (vision.h)
 * ---------------------------------------------------------------------
 * 定义云台自瞄用的视觉数据。视觉由「妙算」(Jetson/上位机) 跑算法，
 * 通过 CAN 把识别到的目标信息发给主控，主控解析后存进 Vision_Data。
 *
 * ===== 核心字段 =====
 *  - vision_x_offect / vision_y_offect：目标相对枪口的偏移量，
 *    就是 pidCalc() 里自瞄的误差来源
 *  - vision_flag：是否识别到目标(1=有目标)
 *  - target_type：目标类型(区分旋转靶/装甲板，用于预测补偿)
 *  - shoot_flag：是否允许开火(视觉判断已对准)
 *  - vision_vel_x/y：视觉算出的目标角速度(速度前馈用)
 *
 * ===== 调车关注 =====
 *  - Vision_Info_Update 解析 CAN 帧，帧 ID 和字段顺序要和妙算那边对上
 * =====================================================================
 */
#ifndef __VISION_H
#define __VISION_H
#include "main.h"
#include "bsp_can.h"
#include "Shoot_Task.h"
#include "Quaternion.h"

/* 视觉工作模式（发给妙算，告诉它当前在打什么） */
typedef enum
{
	idle,       // 空闲
	auto_aim,   // 自瞄(普通装甲板)
	small_buff, // 小能量机关(旋转靶)
	big_buff,   // 大能量机关
	outpost     // 前哨站
}Mode_e;

/* 视觉数据结构：妙算通过 CAN 发来的自瞄数据，解析后存这里。
 *   angle_fdb/set[2]     角度反馈/设定(视觉相关的角度)
 *   vision_x/y_data      视觉原始数据(整数)
 *   vision_x/y_offect    目标相对枪口的偏移(自瞄误差核心) ← pidCalc 用它
 *   vision_flag          是否识别到目标(1=有目标)
 *   target_type          目标类型(0=普通装甲 非0=旋转靶，用于预测补偿)
 *   Distence             目标距离
 *   shoot_flag           是否允许开火(视觉已对准)
 *   vision_fps/period_ms 视觉帧率/帧间隔
 *   mode                 视觉工作模式(见 Mode_e)
 *   vision_vel_x/y       视觉角速度(速度前馈用)
 */
typedef struct {
		float angle_fdb[2];
  	float angle_set[2];
    int16_t vision_y_data;         
    int32_t vision_x_data;           
	  float vision_y_offect;         
    float vision_x_offect;           
	  uint8_t vision_flag;
     uint8_t target_type;  
	  int16_t Distence;
    uint8_t shoot_flag;   // 1字节
    float vision_fps;            // 视觉帧率(Hz), 每秒刷新
    uint16_t vision_period_ms;   // 相邻两帧间隔(ms)
    Mode_e mode;
    float vision_vel_x;          // X方向视觉角速度(°/s)，视觉帧率匹配微分
    float vision_vel_y;          // Y方向视觉角速度(°/s)，视觉帧率匹配微分
} Vision_Data_t;
extern Vision_Data_t Vision_Data;
void Vision_Read_Data(uint8_t *ReadFromUsart);
void GetVisionData(Vision_Data_t* Data);
extern void SentToVision(FDCAN_TxFrame_TypeDef *FDCAN_TxFrame,float Speed,Mode_e Mode,Shoot_Mode_e FireMode);
extern void SentToVisionangle(FDCAN_TxFrame_TypeDef *FDCAN_TxFrame,Quaternion_Info_Typedef Quaternion_Info);
extern void Vision_Info_Update(uint32_t *Identifier, uint8_t *Rx_Buf,Vision_Data_t *Vision_Data);
#endif
