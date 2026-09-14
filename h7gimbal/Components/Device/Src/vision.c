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
void SentToVisionangle(FDCAN_TxFrame_TypeDef *FDCAN_TxFrame,Quaternion_Info_Typedef Quaternion_Info){
	
	  FDCAN_TxFrame->Header.Identifier = 0x307;

    int16_t quat1 = -(int16_t)(Quaternion_Info.quat[1] * 1e4);
    int16_t quat2 = (int16_t)(Quaternion_Info.quat[2] * 1e4);
    int16_t quat3 = (int16_t)(Quaternion_Info.quat[3] * 1e4);
    int16_t quat0 = (int16_t)(Quaternion_Info.quat[0] * 1e4);

    // °´ÕÕ´ó¶ËÐò£¬ÏÈ·¢ËÍ¸ß8Î»£¬ÔÙ·¢ËÍµÍ8Î»
    FDCAN_TxFrame->Data[0] = (uint8_t)(quat2 >> 8);   // ¸ß8Î»
    FDCAN_TxFrame->Data[1] = (uint8_t)(quat2);        // µÍ8Î»
    FDCAN_TxFrame->Data[2] = (uint8_t)(quat1 >> 8);
    FDCAN_TxFrame->Data[3] = (uint8_t)(quat1);
    FDCAN_TxFrame->Data[4] = (uint8_t)(quat3 >> 8);
    FDCAN_TxFrame->Data[5] = (uint8_t)(quat3);
    FDCAN_TxFrame->Data[6] = (uint8_t)(quat0 >> 8);
    FDCAN_TxFrame->Data[7] = (uint8_t)(quat0);
	
    USER_FDCAN_AddMessageToTxFifoQ(FDCAN_TxFrame);
}


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


// ¶¨ÒåÂË²¨ÏµÊý (0.0 ~ 1.0)
#define VISION_LPF_ALPHA 0.95f 

// ¸¨Öúº¯Êý£º½«½Ç¶È¹éµ½ -180 ~ 180 Ö®¼ä (Ñ­»·´¦Àí)
float Normalize_Angle(float angle)
{
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

void Vision_Info_Update(uint32_t *Identifier, uint8_t *Rx_Buf, Vision_Data_t *Vision_Data)
{
    if(*Identifier != 0x309) return;
    
    static uint8_t last_vision_flag = 0;
    static uint32_t fps_count = 0;                            // å¸§çŽ‡ç»Ÿè®¡ - å¸§è®¡æ•°
    static uint32_t fps_tick = 0;                             // å¸§çŽ‡ç»Ÿè®¡ - è®¡æ—¶èµ·ç‚¹
    static uint32_t last_vision_tick = 0;                    // ä¸Šä¸€å¸§æ—¶é—´æˆ³
	
    Vision_Data->vision_flag = Rx_Buf[0] & 0x01;           
    Vision_Data->target_type = (Rx_Buf[0] >> 1) & 0x01;   
    // ===== è§†è§‰å¸§çŽ‡ç»Ÿè®¡ =====
    {
        uint32_t now = HAL_GetTick();
        // å¸§é—´éš” (ms)
        Vision_Data->vision_period_ms = (now > last_vision_tick && last_vision_tick > 0)
                                        ? (uint16_t)(now - last_vision_tick) : 0;
        last_vision_tick = now;
        // æ¯ç§’ç»Ÿè®¡ä¸€æ¬¡å¸§çŽ‡
        fps_count++;
        if (now - fps_tick >= 1000)
        {
            Vision_Data->vision_fps = fps_count * 1000.0f / (float)(now - fps_tick);
            fps_count = 0;
            fps_tick = now;
        }
    }
	
    Vision_Data->shoot_flag = Rx_Buf[1];

    // ÌáÈ¡Ô­Ê¼Êý¾Ý (±£ÁôÁË½â¾ö¸ºÊýÎÊÌâµÄ int16_t Ç¿×ª)
    Vision_Data->vision_x_data = (int16_t)((Rx_Buf[2] << 8) | Rx_Buf[3]);
    Vision_Data->vision_y_data = (int16_t)((Rx_Buf[4] << 8) | Rx_Buf[5]);

    float raw_x_offset =((float)Vision_Data->vision_x_data) / 1e4/ (PI / 180.0f);
    float raw_y_offset =((float)Vision_Data->vision_y_data) /1e4/ (PI / 180.0f);
    float raw_distance = (float)((Rx_Buf[6] << 8) | Rx_Buf[7]);

    // ½«½ÓÊÕµ½µÄÔ­Ê¼XÊý¾Ý¹éµ½ -180 ~ 180
    raw_x_offset = Normalize_Angle(raw_x_offset);

    // ¼ÙÉè vision_flag == 1 ´ú±íÊ¶±ðµ½ÁËÄ¿±ê
    if (Vision_Data->vision_flag == 1) 
    {
        if (last_vision_flag == 0)
        {
            // Ä¿±ê¸Õ½øÈëÊÓÒ°µÄµÚÒ»Ö¡£ºÖ±½Ó¸³Öµ£¬¾Ü¾ø³ÙÖÍ
            Vision_Data->vision_x_offect = raw_x_offset;
            Vision_Data->vision_y_offect = raw_y_offset;
            Vision_Data->Distence        = raw_distance;
        }
        else
        {
            // Ä¿±ê³ÖÐøÔÚÊÓÒ°ÖÐ£º¶ÔXÖ±½Ó½øÐÐÒ»½×µÍÍ¨ÂË²¨
            Vision_Data->vision_x_offect = VISION_LPF_ALPHA * raw_x_offset + (1.0f - VISION_LPF_ALPHA) * Vision_Data->vision_x_offect;
            
            // ÂË²¨ºóÔÙ´ÎÈ·±£½á¹ûÔÚ -180 ~ 180 ·¶Î§ÄÚ
            Vision_Data->vision_x_offect = Normalize_Angle(Vision_Data->vision_x_offect);

            // YÖáºÍ¾àÀëÒ²½øÐÐÕý³£ÂË²¨
            Vision_Data->vision_y_offect = VISION_LPF_ALPHA * raw_y_offset + (1.0f - VISION_LPF_ALPHA) * Vision_Data->vision_y_offect;
            Vision_Data->Distence        = VISION_LPF_ALPHA * raw_distance + (1.0f - VISION_LPF_ALPHA) * Vision_Data->Distence;
        }
    }
    else 
    {
//        // Ã»ÓÐÊ¶±ðµ½Ä¿±êÊ±£¬ÇåÀíÊý¾Ý·ÀÖ¹ÂÒÆ®
//        Vision_Data->vision_x_offect = 0.0f;
//        Vision_Data->vision_y_offect = 0.0f;
//        Vision_Data->Distence        = 0.0f;
    }

    // ¸üÐÂÀúÊ·×´Ì¬£¬¹©ÏÂÒ»Ö¡Ê¹ÓÃ
		 MinMaxConstrain(&Vision_Data->vision_y_offect, PITCH_VISION_MIN_ANGLE, PITCH_VISION_MAX_ANGLE);
		 MinMaxConstrain(&Vision_Data->vision_x_offect, YAW_VISION_MIN_ANGLE, YAW_VISION_MAX_ANGLE);

		

    last_vision_flag = Vision_Data->vision_flag;
}


void GetVisionData(Vision_Data_t* Data)
{
    *Data = Vision_Data;
}