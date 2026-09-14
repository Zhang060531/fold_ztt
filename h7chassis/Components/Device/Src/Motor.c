/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : Motor.c
  * @brief          : Motor interfaces functions 
  ******************************************************************************
  */
/* USER CODE END Header */

/* =====
 * 文件: Motor.c —— 电机抽象层实现(大疆 / 达妙DM / 灵控LK)
 * 作用: 解析三类电机的 CAN 反馈帧、封装下发指令帧, 并实现 MIT 协议编解码
 *       (float_to_uint / uint_to_float) 与离线检测。
 * 结构/调用关系:
 *   - 反馈路径: CAN 接收中断 -> DJI_Motor_Info_Update / DM_Motor_Info_Update /
 *     LK_Motor_Info_Update(按 ID 匹配后解析)。
 *   - 指令路径: 控制任务 -> Transmit*MotorCurrent / DM_Motor_CAN_TxMessage /
 *     LK_Motor_Send_Command(或其封装函数) -> 组帧入 FIFO 发送。
 *   - Check_All_Motors_Offline() 由 Control_Task 循环调用做离线判断。
 * ===== 调车关注 =====
 *   - MIT 协议位宽: 位置 16 位、速度 12 位、KP 12 位、KD 12 位、力矩 12 位,
 *     每项按各自量程线性映射到 [0, 2^bits-1] 的无符号整数。
 *   - 大疆 M3508 底盘电机收 0x201~0x204、发 0x200; 发射 M2006 收 0x205。
 *   - LK 舵向电机 ID: 命令 0x141~0x144, 反馈 0x181~0x184。
 *   - DM4310 MIT 模式: 发 0x02 / 收 0x12, 量程 P_MAX=2PI rad, V_MAX=30 rad/s, T_MAX=10 N·m。
 * =====
 */

/* Includes ------------------------------------------------------------------*/
#include "Motor.h"
#include "bsp_can.h"
#include "Control_Task.h"
#include <string.h>
#include <stdio.h>


/* 发射机构电机(M2006), 发 0x1FF / 收 0x205 */
DJI_Motor_Info_Typedef Shoot_Motor =
{
    .Type = DJI_M2006,
    .FDCANFrame = { .TxIdentifier = 0x1ff, .RxIdentifier = 0x205, }
};

/* 底盘 4 个动力电机(M3508), 同发 0x200, 分别收 0x201~0x204 */
DJI_Motor_Info_Typedef Chassis_Motor[4] = {
    [0] = { .Type = DJI_M3508, .FDCANFrame = { .TxIdentifier = 0x200, .RxIdentifier = 0x201, } },
    [1] = { .Type = DJI_M3508, .FDCANFrame = { .TxIdentifier = 0x200, .RxIdentifier = 0x202, } },
    [2] = { .Type = DJI_M3508, .FDCANFrame = { .TxIdentifier = 0x200, .RxIdentifier = 0x203, } },
    [3] = { .Type = DJI_M3508, .FDCANFrame = { .TxIdentifier = 0x200, .RxIdentifier = 0x204, } },
};

/* 达妙 DM4310 电机(MIT 模式), 发 0x02 / 收 0x12;
 * P_MAX=2PI(rad) V_MAX=30(rad/s) T_MAX=10(N·m) 为编解码量程 */
DM_Motor_Info_Typedef DM_4310_Motor[1]= {
    [0] = {
        .Control_Mode = MIT,
        .Param_Range = { .P_MAX = 6.283186f, .V_MAX = 30.f, .T_MAX = 10.f },
        .FDCANFrame = { .TxIdentifier = 0x02, .RxIdentifier = 0x12, },
    },
};


static int16_t float_to_int16(float value, float scale);
static int32_t float_to_int32(float value, float scale);
static uint32_t float_to_uint32(float value, float scale);
static float int16_to_float(int16_t value, float scale);
static float int32_to_float(int32_t value, float scale);
static float uint32_to_float(uint32_t value, float scale);



/* LK 数值转换辅助: 浮点 <-> 整数的线性缩放(value*scale 或 value/scale) */
static int16_t float_to_int16(float value, float scale) { return (int16_t)(value * scale); }
static int32_t float_to_int32(float value, float scale) { return (int32_t)(value * scale); }
static uint32_t float_to_uint32(float value, float scale) { return (uint32_t)(value * scale); }
static float int16_to_float(int16_t value, float scale) { return (float)value / scale; }
static float int32_to_float(int32_t value, float scale) { return (float)value / scale; }
static float uint32_to_float(uint32_t value, float scale) { return (float)value / scale; }

/**
 * @brief 将 Input 环绕约束到 [Min_Value, Max_Value] 区间(用于角度环绕)
 * @param Input     输入值
 * @param Min_Value 区间下界
 * @param Max_Value 区间上界
 * @retval 约束后的值
 * @note  若 Max<Min 则直接返回原值; 否则按区间长度循环加减使其落回区间内。
 */
float F_Loop_Constrain(float Input, float Min_Value, float Max_Value)
{
    if (Max_Value < Min_Value) return Input;
    float len = Max_Value - Min_Value;    
    if (Input > Max_Value) { do { Input -= len; } while (Input > Max_Value); }
    else if (Input < Min_Value) { do { Input += len; } while (Input < Min_Value); }
    return Input;
}

/**
 * @brief 大疆电机编码器 -> 累计角度(备用函数, 当前工程未调用)
 * @param Data         电机数据
 * @param Torque_Ratio 减速比
 * @param MAXEncoder   编码器满量程(如 8192)
 * @retval 累计角度(°)
 * @note  注意: 疑似bug/未使用 —— 该函数目前无任何调用点, 且环绕判断逻辑与
 *        DJI_Motor_Encoder_To_Angle 不一致(方向相反), 建议确认是否需要保留。
 */
static float DJI_Motor_Encoder_To_Anglesum(DJI_Motor_Data_Typedef *Data,float Torque_Ratio,uint16_t MAXEncoder)
{
    float res1 = 0,res2 =0;
    if(Data == NULL) return 0;
    if(Data->Initlized != true) {
        Data->Last_Encoder = Data->Encoder; Data->Angle = 0; Data->Initlized = true;
    }
    if(Data->Encoder < Data->Last_Encoder) { res1 = Data->Encoder - Data->Last_Encoder + MAXEncoder; }
    else if(Data->Encoder > Data->Last_Encoder) { res1 = Data->Encoder - Data->Last_Encoder - MAXEncoder; }
    res2 = Data->Encoder - Data->Last_Encoder;
    Data->Last_Encoder = Data->Encoder;
    if(fabsf(res1) > fabsf(res2)) { Data->Angle += (float)res2/(MAXEncoder*Torque_Ratio)*360.f; }
    else { Data->Angle += (float)res1/(MAXEncoder*Torque_Ratio)*360.f; }
    return Data->Angle;
}

/**
 * @brief 大疆电机编码器 -> 累计角度(带溢出环绕处理), 输出约束到 [-180°,180°]
 * @param Data         电机数据
 * @param torque_ratio 减速比(编码器圈数 -> 输出轴圈数的换算)
 * @param MAXEncoder   编码器满量程(如 8192)
 * @retval 累计角度(°)
 * @note  根据两次编码器差值判断是否跨 0/满量程, 累加后做 -180~180 环绕约束。
 */
float DJI_Motor_Encoder_To_Angle(DJI_Motor_Data_Typedef *Data,float torque_ratio,uint16_t MAXEncoder)
{    
    float Encoder_Err = 0.f;
    if(Data->Initlized != true) {
        Data->Last_Encoder = Data->Encoder; Data->Angle = Data->Encoder/(MAXEncoder*torque_ratio)*360.f; Data->Initlized = true;
    }
    Encoder_Err = Data->Encoder - Data->Last_Encoder;
    if(Encoder_Err > MAXEncoder*0.5f) { Data->Angle += (float)(Encoder_Err - MAXEncoder)/(MAXEncoder*torque_ratio)*360.f; }
    else if(Encoder_Err < -MAXEncoder*0.5f) { Data->Angle += (float)(Encoder_Err + MAXEncoder)/(MAXEncoder*torque_ratio)*360.f; }
    else { Data->Angle += (float)(Encoder_Err)/(MAXEncoder*torque_ratio)*360.f; }
    
    Data->Last_Encoder = Data->Encoder;
    Data->Angle = F_Loop_Constrain(Data->Angle,-180.f,180.f);
    return Data->Angle;
}

/**
 * @brief 大疆电机反馈解析: 按 ID 匹配后解析 8 字节标准帧, 并按型号换算角度
 * @param Identifier 收到的 CAN ID 指针
 * @param Rx_Buf     8 字节数据
 * @param DJI_Motor  电机句柄
 * @note  标准帧: 字节0/1=编码器, 2/3=转速(rpm), 4/5=电流, 6=温度;
 *        不同型号减速比不同, 换算角度时参数见各 case。
 *        收到 0x205(发射电机) 时还会转发给云台 SentShootToGimbal。
 */
void DJI_Motor_Info_Update(uint32_t *Identifier, uint8_t *Rx_Buf,DJI_Motor_Info_Typedef *DJI_Motor)
{
    if(*Identifier != DJI_Motor->FDCANFrame.RxIdentifier) return;
    
    DJI_Motor->Motor_time=xTaskGetTickCount();
    DJI_Motor->Data.Temperature = Rx_Buf[6];
    DJI_Motor->Data.Encoder  = ((int16_t)Rx_Buf[0] << 8 | (int16_t)Rx_Buf[1]);
    DJI_Motor->Data.Velocity = ((int16_t)Rx_Buf[2] << 8 | (int16_t)Rx_Buf[3]);
    DJI_Motor->Data.Current  = ((int16_t)Rx_Buf[4] << 8 | (int16_t)Rx_Buf[5]);
    
    switch(DJI_Motor->Type)
    {
        case DJI_GM6020: DJI_Motor->Data.Angle = DJI_Motor_Encoder_To_Angle(&DJI_Motor->Data,1.f,8192); break;
        case DJI_M3508: DJI_Motor->Data.Angle = DJI_Motor_Encoder_To_Angle(&DJI_Motor->Data,3591.f/187.f,8192); break;
        case DJI_M350851: DJI_Motor->Data.Angle = DJI_Motor_Encoder_To_Angle(&DJI_Motor->Data,51.f,8192); break;
        case DJI_M2006: DJI_Motor->Data.Angle = DJI_Motor_Encoder_To_Angle(&DJI_Motor->Data,36.f,8192); break;
        default:break;
    }
		if(*Identifier == 0x205) {
				SentShootToGimbal(&FDCAN2_TxFrame, Rx_Buf); 
		}
		
}

/**
 * @brief 下发 4 个底盘动力电机(M3508)电流指令, 帧 ID 0x200
 * @param m1~m4 4 个电机的电流值(电调单位, 大端序: 高字节在前)
 */
void TransmitChassisMotorCurrent(FDCAN_TxFrame_TypeDef *FDCAN_TxFrame,int16_t m1, int16_t m2, int16_t m3, int16_t m4)
{
    FDCAN_TxFrame->Header.Identifier = 0x200;
    FDCAN_TxFrame->Data[0] = (uint8_t)(m1 >> 8); FDCAN_TxFrame->Data[1] = (uint8_t)m1;
    FDCAN_TxFrame->Data[2] = (uint8_t)(m2 >> 8); FDCAN_TxFrame->Data[3] = (uint8_t)m2;
    FDCAN_TxFrame->Data[4] = (uint8_t)(m3 >> 8); FDCAN_TxFrame->Data[5] = (uint8_t)m3;
    FDCAN_TxFrame->Data[6] = (uint8_t)(m4 >> 8); FDCAN_TxFrame->Data[7] = (uint8_t)m4;
    USER_FDCAN_AddMessageToTxFifoQ(FDCAN_TxFrame);
}

/**
 * @brief 下发 4 个舵向电机电流指令, 帧 ID 0x280
 * @param m1~m4 4 个电机的电流值(小端序: 低字节在前, 与大疆标准帧相反)
 */
void TransmitsteerMotorCurrent(FDCAN_TxFrame_TypeDef *FDCAN_TxFrame,int16_t m1, int16_t m2, int16_t m3, int16_t m4)
{
    FDCAN_TxFrame->Header.Identifier = 0x280;
    FDCAN_TxFrame->Data[0] = (uint8_t)m1; FDCAN_TxFrame->Data[1] = (uint8_t)(m1 >> 8);
    FDCAN_TxFrame->Data[2] = (uint8_t)m2; FDCAN_TxFrame->Data[3] = (uint8_t)(m2 >> 8);
    FDCAN_TxFrame->Data[4] = (uint8_t)m3; FDCAN_TxFrame->Data[5] = (uint8_t)(m3 >> 8);
    FDCAN_TxFrame->Data[6] = (uint8_t)m4; FDCAN_TxFrame->Data[7] = (uint8_t)(m4 >> 8);
    USER_FDCAN_AddMessageToTxFifoQ(FDCAN_TxFrame);
}

/**
 * @brief 下发发射机构电机电流指令, 帧 ID 0x1FF(大端序)
 */
void TransmitShootMotorCurrent(FDCAN_TxFrame_TypeDef *FDCAN_TxFrame,int16_t shoot_current)
{
    FDCAN_TxFrame->Header.Identifier = 0x1ff;
    FDCAN_TxFrame->Data[0] = (uint8_t)(shoot_current >> 8);
    FDCAN_TxFrame->Data[1] = (uint8_t)shoot_current;
    USER_FDCAN_AddMessageToTxFifoQ(FDCAN_TxFrame);
}

/**
 * @brief DM 电机特殊指令(使能/失能/保存零点)
 * @param FDCAN_TxFrame 发送帧
 * @param DM_Motor      DM 电机句柄
 * @param CMD           指令: Motor_Enable(0xFC)/Disable(0xFD)/Save_Zero(0xFE)
 * @note  前 7 字节填 0xFF, 第 8 字节写指令码。
 */
void DM_Motor_Command(FDCAN_TxFrame_TypeDef *FDCAN_TxFrame,DM_Motor_Info_Typedef *DM_Motor,uint8_t CMD)
{
    FDCAN_TxFrame->Header.Identifier = DM_Motor->FDCANFrame.TxIdentifier;
    FDCAN_TxFrame->Data[0] = 0xFF; FDCAN_TxFrame->Data[1] = 0xFF; FDCAN_TxFrame->Data[2] = 0xFF; FDCAN_TxFrame->Data[3] = 0xFF;
    FDCAN_TxFrame->Data[4] = 0xFF; FDCAN_TxFrame->Data[5] = 0xFF; FDCAN_TxFrame->Data[6] = 0xFF;
    
    switch(CMD){
        case Motor_Enable: FDCAN_TxFrame->Data[7] = 0xFC; break;
        case Motor_Disable: FDCAN_TxFrame->Data[7] = 0xFD; break;
        case Motor_Save_Zero_Position: FDCAN_TxFrame->Data[7] = 0xFE; break;
        default:break;   
    }
    USER_FDCAN_AddMessageToTxFifoQ(FDCAN_TxFrame);
}

/**
 * @brief DM 电机指令发送(MIT 模式组帧)
 * @param FDCAN_TxFrame 发送帧
 * @param DM_Motor      DM 电机句柄
 * @param Postion       目标位置(rad, 量程 ±P_MAX, 编码 16 位)
 * @param Velocity      目标速度(rad/s, 量程 ±V_MAX, 编码 12 位)
 * @param KP            位置环比例增益(量程 0~500, 编码 12 位)
 * @param KD            速度环微分增益(量程 0~5, 编码 12 位)
 * @param Torque        前馈力矩(N·m, 量程 ±T_MAX, 编码 12 位)
 * @note  MIT 协议 8 字节位域布局:
 *        [0:1] 位置高8位 | 位置低8位
 *        [2]   速度高4位
 *        [3]   速度低4位(高4位) | KP 高4位(低4位)
 *        [4]   KP 低8位
 *        [5]   KD 高4位
 *        [6]   KD 低4位(高4位) | 力矩高4位(低4位)
 *        [7]   力矩低8位
 *        各值经 float_to_uint 线性映射到 [0, 2^bits-1] 的无符号整数。
 */
void DM_Motor_CAN_TxMessage(FDCAN_TxFrame_TypeDef *FDCAN_TxFrame,DM_Motor_Info_Typedef *DM_Motor,float Postion, float Velocity, float KP, float KD, float Torque)
{
    if(DM_Motor->Control_Mode == MIT){
        uint16_t Postion_Tmp,Velocity_Tmp,Torque_Tmp,KP_Tmp,KD_Tmp;
        Postion_Tmp  =  float_to_uint(Postion, -DM_Motor->Param_Range.P_MAX,DM_Motor->Param_Range.P_MAX,16); /* 位置 16 位 */
        Velocity_Tmp =  float_to_uint(Velocity,-DM_Motor->Param_Range.V_MAX,DM_Motor->Param_Range.V_MAX,12); /* 速度 12 位 */
        Torque_Tmp   =  float_to_uint(Torque,  -DM_Motor->Param_Range.T_MAX,DM_Motor->Param_Range.T_MAX,12); /* 力矩 12 位 */
        KP_Tmp = float_to_uint(KP,0,500,12); KD_Tmp = float_to_uint(KD,0,5,12); /* KP/KD 各 12 位, 量程 0~500 / 0~5 */
        
        FDCAN_TxFrame->Header.Identifier = DM_Motor->FDCANFrame.TxIdentifier;
        /* 8 字节按位拼接: 位置[16] 速度[12] KP[12] KD[12] 力矩[12] */
        FDCAN_TxFrame->Data[0] = (uint8_t)(Postion_Tmp>>8); FDCAN_TxFrame->Data[1] = (uint8_t)(Postion_Tmp);
        FDCAN_TxFrame->Data[2] = (uint8_t)(Velocity_Tmp>>4);                                    /* 速度高 4 位 */
        FDCAN_TxFrame->Data[3] = (uint8_t)((Velocity_Tmp&0x0F)<<4) | (uint8_t)(KP_Tmp>>8);      /* 速度低 4 位 | KP 高 4 位 */
        FDCAN_TxFrame->Data[4] = (uint8_t)(KP_Tmp);                                             /* KP 低 8 位 */
        FDCAN_TxFrame->Data[5] = (uint8_t)(KD_Tmp>>4);                                          /* KD 高 4 位 */
        FDCAN_TxFrame->Data[6] = (uint8_t)((KD_Tmp&0x0F)<<4) | (uint8_t)(Torque_Tmp>>8);        /* KD 低 4 位 | 力矩高 4 位 */
        FDCAN_TxFrame->Data[7] = (uint8_t)(Torque_Tmp);                                         /* 力矩低 8 位 */
    }else if(DM_Motor->Control_Mode == POSITION_VELOCITY){
        uint8_t *Postion_Tmp,*Velocity_Tmp;
        Postion_Tmp  = (uint8_t*) & Postion; Velocity_Tmp = (uint8_t*) & Velocity;
        FDCAN_TxFrame->Header.Identifier = DM_Motor->FDCANFrame.TxIdentifier + 0x100;
        FDCAN_TxFrame->Data[0] = *(Postion_Tmp); FDCAN_TxFrame->Data[1] = *(Postion_Tmp + 1);
        FDCAN_TxFrame->Data[2] = *(Postion_Tmp + 2); FDCAN_TxFrame->Data[3] = *(Postion_Tmp + 3);
        FDCAN_TxFrame->Data[4] = *(Velocity_Tmp); FDCAN_TxFrame->Data[5] = *(Velocity_Tmp + 1);
        FDCAN_TxFrame->Data[6] = *(Velocity_Tmp + 2); FDCAN_TxFrame->Data[7] = *(Velocity_Tmp + 3);
    }else if(DM_Motor->Control_Mode == VELOCITY){
        uint8_t *Velocity_Tmp; Velocity_Tmp = (uint8_t*) & Velocity;
        FDCAN_TxFrame->Header.Identifier = DM_Motor->FDCANFrame.TxIdentifier + 0x200;
        FDCAN_TxFrame->Data[0] = *(Velocity_Tmp); FDCAN_TxFrame->Data[1] = *(Velocity_Tmp + 1);
        FDCAN_TxFrame->Data[2] = *(Velocity_Tmp + 2); FDCAN_TxFrame->Data[3] = *(Velocity_Tmp + 3);
        FDCAN_TxFrame->Data[4] = 0; FDCAN_TxFrame->Data[5] = 0; FDCAN_TxFrame->Data[6] = 0; FDCAN_TxFrame->Data[7] = 0;
    }
    USER_FDCAN_AddMessageToTxFifoQ(FDCAN_TxFrame);
}

/**
 * @brief DM 电机 MIT 协议反馈解析(解码 16/12/12 位 -> 物理量)
 * @param Identifier 收到的 CAN ID 指针
 * @param Rx_Buf     8 字节数据
 * @param DM_Motor   DM 电机句柄
 * @note  位域布局与发送一致: 位置16位、速度12位、力矩12位,
 *        经 uint_to_float 按 P_MAX/V_MAX/T_MAX 量程换算回物理量。
 */
void DM_Motor_Info_Update(uint32_t *Identifier,uint8_t *Rx_Buf,DM_Motor_Info_Typedef *DM_Motor)
{
    if(*Identifier != DM_Motor->FDCANFrame.RxIdentifier) return;
    DM_Motor->Motor_time=xTaskGetTickCount();
    DM_Motor->Data.State = Rx_Buf[0]>>4;
    DM_Motor->Data.P_int = ((uint16_t)(Rx_Buf[1]) <<8) | ((uint16_t)(Rx_Buf[2]));       /* 位置 16 位 */
    DM_Motor->Data.V_int = ((uint16_t)(Rx_Buf[3]) <<4) | ((uint16_t)(Rx_Buf[4])>>4);    /* 速度 12 位 */
    DM_Motor->Data.T_int = ((uint16_t)(Rx_Buf[4]&0xF) <<8) | ((uint16_t)(Rx_Buf[5]));   /* 力矩 12 位 */
    DM_Motor->Data.Torque=  uint_to_float(DM_Motor->Data.T_int,-DM_Motor->Param_Range.T_MAX,DM_Motor->Param_Range.T_MAX,12);
    DM_Motor->Data.Position=uint_to_float(DM_Motor->Data.P_int,-DM_Motor->Param_Range.P_MAX,DM_Motor->Param_Range.P_MAX,16);
    DM_Motor->Data.Velocity=uint_to_float(DM_Motor->Data.V_int,-DM_Motor->Param_Range.V_MAX,DM_Motor->Param_Range.V_MAX,12);
    DM_Motor->Data.Temperature_MOS   = (float)(Rx_Buf[6]);
    DM_Motor->Data.Temperature_Rotor = (float)(Rx_Buf[7]);  
}

/**
 * @brief MIT 协议解码: 无符号整数 -> 浮点物理量
 * @param X_int 原始无符号整数
 * @param X_min 物理量下界
 * @param X_max 物理量上界
 * @param Bits  位宽(如 12/16)
 * @retval 映射后的浮点物理量 = X_int * (X_max-X_min) / (2^Bits-1) + X_min
 * @note  例如位置 16 位、量程 [-2PI,2PI]: 中间值 0x8000 对应 0 rad。
 */
float uint_to_float(int X_int, float X_min, float X_max, int Bits)
{
    float span = X_max - X_min; float offset = X_min;
    return ((float)X_int)*span/((float)((1<<Bits)-1)) + offset;
}

/**
 * @brief MIT 协议编码: 浮点物理量 -> 无符号整数
 * @param x     物理量
 * @param x_min 物理量下界
 * @param x_max 物理量上界
 * @param bits  位宽(如 12/16)
 * @retval 映射后的无符号整数 = (x - x_min) * (2^bits-1) / (x_max - x_min)
 * @note  与 uint_to_float 互逆; 超出量程会溢出截断, 调用前应保证 x 在量程内。
 */
int float_to_uint(float x, float x_min, float x_max, int bits)
{
    float span = x_max - x_min; float offset = x_min;
    return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}

/**
  * @brief  检查所有电机是否离线(超过 MOTOR_OFFLINE_TIMEOUT 未收到反馈即置离线标志)
  * @note   此函数将在 Control_Task 中循环调用; 检测大疆底盘/发射电机、LK 舵向电机、DM 电机。
  */
void Check_All_Motors_Offline(void)
{
    uint32_t current_time = xTaskGetTickCount();

    /* 1. 检测 DJI 轮向电机 (Chassis) 与 发射机构电机 (Shoot) */
    for(int i = 0; i < 4; i++) {
        if((current_time - Chassis_Motor[i].Motor_time) > MOTOR_OFFLINE_TIMEOUT) {
            Chassis_Motor[i].is_offline = true;
        } else {
            Chassis_Motor[i].is_offline = false;
        }
    }
    
    if((current_time - Shoot_Motor.Motor_time) > MOTOR_OFFLINE_TIMEOUT) {
        Shoot_Motor.is_offline = true;
    } else {
        Shoot_Motor.is_offline = false;
    }



    /* 3. 检测 DM 达妙电机 (Yaw) */
    if((current_time - DM_4310_Motor[0].Motor_time) > MOTOR_OFFLINE_TIMEOUT) {
        DM_4310_Motor[0].is_offline = true;
    } else {
        DM_4310_Motor[0].is_offline = false;
    }
}