/**
 * =====================================================================
 * 文件说明：视觉数据接收 (vision.c) —— 自瞄数据的入口
 * ---------------------------------------------------------------------
 * 云台自瞄的数据来源是「妙算」(NVIDIA Jetson 上位机，跑视觉识别算法)，
 * 它通过 CAN 把识别到的目标信息发给主控，本文件负责：
 *
 *  1. 收数据 Vision_Info_Update()：解析妙算发来的 CAN 帧(0x309)，
 *     算出「目标相对枪口的偏移角度」，存进全局 Vision_Data。
 *  2. 发数据 SentToVision() / SentToVisionangle()：主控把自己的状态
 *     (弹速、工作模式、四元数姿态) 回发给妙算，方便妙算做解算/预测。
 *
 * ===== 最关键的两个字段 (自瞄就是靠它们) =====
 *  - vision_x_offect / vision_y_offect：目标相对枪口的偏移角度(单位:度)。
 *    妙算把「枪口当前指向」和「目标实际位置」的差算出来发给我们，
 *    这个偏移量就是 Control_Task.c 里 pidCalc() 自瞄的「误差来源」。
 *    云台只要把这个偏移量追到 0，枪口就对准目标了。
 *  - shoot_flag：视觉判断「已经对准、可以开火」的标志(1=允许开火)，
 *    开火逻辑里会拿它当「视觉确认对准」的条件(见 Shoot_Task.c)。
 *
 * ===== 帧 ID 约定 (要和妙算那边对上) =====
 *  - 0x307 主控→妙算：云台姿态四元数 (SentToVisionangle)
 *  - 0x308 主控→妙算：弹速/模式/开火模式 (SentToVision)
 *  - 0x309 妙算→主控：目标偏移量等自瞄数据 (Vision_Info_Update)
 * =====================================================================
 */
#include "vision.h"
#include "Control_Task.h"
#include "main.h"
#include "cmsis_os.h"
#include "INS_Task.h"

int visionMode_test2=0;
Vision_Data_t Vision_Data;
void SentToVision(FDCAN_TxFrame_TypeDef *FDCAN_TxFrame,float Speed,Mode_e Mode,Shoot_Mode_e FireMode);
void SentToVisionangle(FDCAN_TxFrame_TypeDef *FDCAN_TxFrame,Quaternion_Info_Typedef Quaternion_Info);
void Vision_Info_Update(uint32_t *Identifier, uint8_t *Rx_Buf,Vision_Data_t *Vision_Data);

/**
 * @brief 把云台姿态四元数发给妙算 (CAN 0x307)
 * @note  妙算需要用云台当前姿态来解算「枪口指向」，从而算出目标偏移量。///////////////
 *        四元数 4 个分量各放大 1e4 转成 int16，按大端序拆成 8 字节发送。
 */
void SentToVisionangle(FDCAN_TxFrame_TypeDef *FDCAN_TxFrame,Quaternion_Info_Typedef Quaternion_Info){

	  FDCAN_TxFrame->Header.Identifier = 0x307;

    int16_t quat1 = -(int16_t)(Quaternion_Info.quat[1] * 1e4);
    int16_t quat2 = (int16_t)(Quaternion_Info.quat[2] * 1e4);
    int16_t quat3 = (int16_t)(Quaternion_Info.quat[3] * 1e4);
    int16_t quat0 = (int16_t)(Quaternion_Info.quat[0] * 1e4);

    // 按照大端序，先发高8位，再发低8位
    FDCAN_TxFrame->Data[0] = (uint8_t)(quat2 >> 8);   // 高8位
    FDCAN_TxFrame->Data[1] = (uint8_t)(quat2);        // 低8位
    FDCAN_TxFrame->Data[2] = (uint8_t)(quat1 >> 8);
    FDCAN_TxFrame->Data[3] = (uint8_t)(quat1);
    FDCAN_TxFrame->Data[4] = (uint8_t)(quat3 >> 8);
    FDCAN_TxFrame->Data[5] = (uint8_t)(quat3);
    FDCAN_TxFrame->Data[6] = (uint8_t)(quat0 >> 8);
    FDCAN_TxFrame->Data[7] = (uint8_t)(quat0);

    USER_FDCAN_AddMessageToTxFifoQ(FDCAN_TxFrame);
}


/**
 * @brief 把弹速/工作模式/开火模式发给妙算 (CAN 0x308)
 * @note  Speed=弹速(放大100倍发整数)，Mode=自瞄目标类型(见 Mode_e)，
 *        FireMode=当前发射状态(见 Shoot_Mode_e)。
 */
void SentToVision(FDCAN_TxFrame_TypeDef *FDCAN_TxFrame,float Speed,Mode_e Mode,Shoot_Mode_e FireMode){

	  FDCAN_TxFrame->Header.Identifier = 0x308;
    int16_t speed = (int16_t)(Speed * 1e2);
	  visionMode_test2=speed;
    FDCAN_TxFrame->Data[0] = (uint8_t)(speed >> 8);
    FDCAN_TxFrame->Data[1] = (uint8_t)(speed);
    FDCAN_TxFrame->Data[2] = (uint8_t)(Mode);
    FDCAN_TxFrame->Data[3] = (uint8_t)(FireMode);

    USER_FDCAN_AddMessageToTxFifoQ(FDCAN_TxFrame);
}


// 一阶低通滤波系数 (0.0 ~ 1.0)，越小越平滑但越滞后
#define VISION_LPF_ALPHA 0.95f

// 辅助函数：将角度归到 -180 ~ 180 之间 (循环处理)
float Normalize_Angle(float angle)
{
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

/**
 * @brief 解析妙算发来的自瞄数据帧 (CAN 0x309) —— 自瞄核心入口
 * @note  数据格式(和妙算约定，8 字节)：
 *         Rx_Buf[0]    bit0 = vision_flag(是否识别到目标)，bit1 = target_type(目标类型)
 *         Rx_Buf[1]    shoot_flag (是否允许开火)
 *         Rx_Buf[2..3] vision_x_data 目标相对枪口的水平(yaw)偏移原始值
 *         Rx_Buf[4..5] vision_y_data 目标相对枪口的垂直(pitch)偏移原始值
 *         Rx_Buf[6..7] 目标距离(原始整数)
 *
 *        原始偏移量是「弧度 × 1e4」的整数，这里先 ÷1e4 变回弧度，
 *        再 ÷(π/180) 转成「度」，得到真正的角度偏移 vision_x/y_offect。
 *        这就是 pidCalc() 里自瞄的误差来源。
 */
void Vision_Info_Update(uint32_t *Identifier, uint8_t *Rx_Buf, Vision_Data_t *Vision_Data)
{
    if(*Identifier != 0x309) return;

    static uint8_t last_vision_flag = 0;
    static uint32_t fps_count = 0;                            // 帧率统计 - 帧计数
    static uint32_t fps_tick = 0;                             // 帧率统计 - 计时起点
    static uint32_t last_vision_tick = 0;                    // 上一帧时间戳

    Vision_Data->vision_flag = Rx_Buf[0] & 0x01;
    Vision_Data->target_type = (Rx_Buf[0] >> 1) & 0x01;
    // ===== 视觉帧率统计 =====
    {
        uint32_t now = HAL_GetTick();
        // 帧间隔 (ms)
        Vision_Data->vision_period_ms = (now > last_vision_tick && last_vision_tick > 0)  //调试中的vision_period_ms以及下面的vision_fps是在电控这计算出来的
                                        ? (uint16_t)(now - last_vision_tick) : 0;
        last_vision_tick = now;
        // 每秒统计一次帧率
        fps_count++;
        if (now - fps_tick >= 1000)
        {
            Vision_Data->vision_fps = fps_count * 1000.0f / (float)(now - fps_tick);      //调试中的vision_fps大概一百左右（每秒发一百帧左右）
            fps_count = 0;
            fps_tick = now;
        }
    }

    Vision_Data->shoot_flag = Rx_Buf[1];

    // 提取原始数据 (保留了解决负数问题的 int16_t 强转)
    Vision_Data->vision_x_data = (int16_t)((Rx_Buf[2] << 8) | Rx_Buf[3]);
    Vision_Data->vision_y_data = (int16_t)((Rx_Buf[4] << 8) | Rx_Buf[5]);

    // 原始值(弧度×1e4的整数) → 弧度 → 度
    float raw_x_offset =((float)Vision_Data->vision_x_data) / 1e4/ (PI / 180.0f);
    float raw_y_offset =((float)Vision_Data->vision_y_data) /1e4/ (PI / 180.0f);
    float raw_distance = (float)((Rx_Buf[6] << 8) | Rx_Buf[7]);

    // 将接收到的原始X数据归到 -180 ~ 180
    raw_x_offset = Normalize_Angle(raw_x_offset);

    // vision_flag == 1 代表识别到了目标
    if (Vision_Data->vision_flag == 1)
    {
        if (last_vision_flag == 0)
        {
            // 目标刚进入视野的第一帧：直接赋值，不做滤波(避免滤波初值滞后)
            Vision_Data->vision_x_offect = raw_x_offset;
            Vision_Data->vision_y_offect = raw_y_offset;
            Vision_Data->Distence        = raw_distance;
        }
        else
        {
            // 目标持续在视野中：对偏移量做一阶低通滤波(平滑视觉抖动)
            Vision_Data->vision_x_offect = VISION_LPF_ALPHA * raw_x_offset + (1.0f - VISION_LPF_ALPHA) * Vision_Data->vision_x_offect;

            // 滤波后再次确保结果在 -180 ~ 180 范围内
            Vision_Data->vision_x_offect = Normalize_Angle(Vision_Data->vision_x_offect);

            // Y轴偏移和距离也做同样的滤波
            Vision_Data->vision_y_offect = VISION_LPF_ALPHA * raw_y_offset + (1.0f - VISION_LPF_ALPHA) * Vision_Data->vision_y_offect;
            Vision_Data->Distence        = VISION_LPF_ALPHA * raw_distance + (1.0f - VISION_LPF_ALPHA) * Vision_Data->Distence;
        }
    }
    else
    {
//        // 没有识别到目标时，清理数据防止乱飘
//        Vision_Data->vision_x_offect = 0.0f;
//        Vision_Data->vision_y_offect = 0.0f;
//        Vision_Data->Distence        = 0.0f;
    }

    // 对偏移量做限幅(防止视觉数据异常导致云台甩飞)，范围定义在 Control_Task.h
    MinMaxConstrain(&Vision_Data->vision_y_offect, PITCH_VISION_MIN_ANGLE, PITCH_VISION_MAX_ANGLE);
    MinMaxConstrain(&Vision_Data->vision_x_offect, YAW_VISION_MIN_ANGLE, YAW_VISION_MAX_ANGLE);

    // 更新历史状态，供下一帧判断「目标是否刚进入视野」
    last_vision_flag = Vision_Data->vision_flag;
}


void GetVisionData(Vision_Data_t* Data)
{
    *Data = Vision_Data;
}
