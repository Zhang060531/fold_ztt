/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : Motor.c
  * @brief          : Motor interfaces functions 
  ******************************************************************************
  * @attention      : None
  ******************************************************************************
  */
/* USER CODE END Header */

/**
 * =====================================================================
 * 文件说明：电机驱动实现 (Motor.c)
 * ---------------------------------------------------------------------
 * 实现两类电机的 CAN 收发与数据解析：
 *
 *  1. DJI 电机（摩擦轮 M3508、拨弹轮 M2006）—— 大疆电调 CAN 协议：
 *     - 收：DJI_Motor_Info_Update() 解析电机回传帧
 *           Data[0..1]=编码器, [2..3]=转速, [4..5]=电流, [6]=温度
 *     - 发：TransmitFriMotorCurrent() / TransmitShootMotorCurrent()
 *           下发电流指令（帧 ID 0x200 / 0x1FF）
 *
 *  2. 达妙 DM4310 电机（pitch/yaw）—— MIT 模式 CAN 协议：
 *     - 发：DM_Motor_CAN_TxMessage() 按 MIT 格式下发
 *           [位置16bit | 速度12bit | KP12bit | KD12bit | 力矩12bit]
 *     - 收：DM_Motor_Info_Update() 解析反馈帧并换算回物理单位
 *     - 命令：DM_Motor_Command() 使能/失能/设零点（放 Data[7]）
 *
 *  ===== 调车关注 =====
 *  - 所有电机的 CAN ID 都在本文件顶部实例定义里，改 ID 来这里。
 *  - DJI 编码器一圈 8192 线；M2006 减速比 36:1（输出轴角度要除以 36）。
 *  - MIT 协议用 12bit/16bit 映射物理量，量程由 Param_Range 决定，
 *    float_to_uint()/uint_to_float() 就是做这个映射。
 * =====================================================================
 */

/* Includes ------------------------------------------------------------------*/
#include "Motor.h"
#include "bsp_can.h"
#include "Control_Task.h"
#include <string.h>
#include <stdio.h>
#include <math.h> // 需要用到 fabsf 等数学函数

/* =====================================================================
 * 全局电机实例定义（CAN ID 都在这里，改 ID 就改这里）
 * ===================================================================== */

// 3 个摩擦轮电机 M3508：共用下发帧 0x200（一条帧带两个电机），
// 各自回传帧 0x201 / 0x202 / 0x203
DJI_Motor_Info_Typedef FR_Motor[3] = {
    [0] = {
        .Type = DJI_M3508,
        .FDCANFrame = {
            .TxIdentifier = 0x200,   // 下发：电流指令（摩擦轮 1、2 共用）
            .RxIdentifier = 0x201,   // 回传：摩擦轮 1 的转速/角度/电流/温度
        }
    },
    [1] = {
        .Type = DJI_M3508,
        .FDCANFrame = {
            .TxIdentifier = 0x200,   // 下发：电流指令（摩擦轮 1、2 共用）
            .RxIdentifier = 0x202,   // 回传：摩擦轮 2
        }
    },
    [2] = {
        .Type = DJI_M3508,
        .FDCANFrame = {
            .TxIdentifier = 0x200,   // 下发：电流指令（摩擦轮 3 单独用）
            .RxIdentifier = 0x203,   // 回传：摩擦轮 3
        }
    },
};

// 拨弹轮电机 M2006：下发帧 0x1FF，回传帧 0x25
DJI_Motor_Info_Typedef Shoot_Motor = {
        .Type = DJI_M2006,
        .FDCANFrame = {
            .TxIdentifier = 0x1ff,   // 下发：拨弹轮电流
            .RxIdentifier = 0x25,    // 回传：拨弹轮转速/角度
        }
};

// 达妙 DM4310 关节电机 ×2（pitch 俯仰 / yaw 偏航）
// 注意：达妙电机用「1 对 1」ID，Tx=控制帧, Rx=反馈帧（Rx=Tx+0x10）
DM_Motor_Info_Typedef DM_4310_Motor[2]= {
    [0] = {
        .Control_Mode = MIT,         // MIT 阻抗控制模式
        .Param_Range = {
            .P_MAX = 12.5f,          // 位置量程 ±12.5 rad（pitch 俯仰，约 4 圈）
            .V_MAX = 30.f,           // 速度量程 ±30 rad/s
            .T_MAX = 10.f            // 力矩量程 ±10 N·m
        },
        .FDCANFrame = {
            .TxIdentifier = 0x01,    // 下发：pitch 电机控制帧
            .RxIdentifier = 0x11,    // 回传：pitch 电机反馈帧
        },
    },
        [1] = {
        .Control_Mode = MIT,
        .Param_Range = {
            .P_MAX = 3.1415926f,     // 位置量程 ±π rad（yaw 偏航，半圈范围）
            .V_MAX = 30.f,
            .T_MAX = 10.f
        },
        .FDCANFrame = {
            .TxIdentifier = 0x02,    // 下发：yaw 电机控制帧
            .RxIdentifier = 0x12,    // 回传：yaw 电机反馈帧
        },
    },
};

static int16_t float_to_int16(float value, float scale);
static int32_t float_to_int32(float value, float scale);
static uint32_t float_to_uint32(float value, float scale);
static float int16_to_float(int16_t value, float scale);
static float int32_to_float(int32_t value, float scale);
static float uint32_to_float(uint32_t value, float scale);
float filter_FR1(float temp,float alpha);
float filter_FR2(float temp,float alpha);

float filter_sht(float temp,float alpha);
void ConstrainLoop(float* output, float input, float min, float max);
void TransmitFriMotorCurrent(FDCAN_TxFrame_TypeDef *FDCAN_TxFrame,int16_t m1, int16_t m2);
void TransmitShootMotorCurrent(FDCAN_TxFrame_TypeDef *FDCAN_TxFrame,int16_t shoot_current);

/* Helper conversion functions */
static int16_t float_to_int16(float value, float scale)
{
    return (int16_t)(value * scale);
}

static int32_t float_to_int32(float value, float scale)
{
    return (int32_t)(value * scale);
}

static uint32_t float_to_uint32(float value, float scale)
{
    return (uint32_t)(value * scale);
}

static float int16_to_float(int16_t value, float scale)
{
    return (float)value / scale;
}

static float int32_to_float(int32_t value, float scale)
{
    return (float)value / scale;
}

static float uint32_to_float(uint32_t value, float scale)
{
    return (float)value / scale;
}

/* Existing functions (unchanged) */
//------------------------------------------------------------------------------

float F_Loop_Constrain(float Input, float Min_Value, float Max_Value)
{
    if (Max_Value < Min_Value)
    {
        return Input;
    }
       
    float len = Max_Value - Min_Value;    

    if (Input > Max_Value)
    {
        do{
            Input -= len;
        }while (Input > Max_Value);
    }
    else if (Input < Min_Value)
    {
        do{
            Input += len;
        }while (Input < Min_Value);
    }
    return Input;
}

static float DJI_Motor_Encoder_To_Anglesum(DJI_Motor_Data_Typedef *Data,float Torque_Ratio,uint16_t MAXEncoder)
{
    float res1 = 0,res2 =0;
    
    if(Data == NULL) return 0;
    
    if(Data->Initlized != true)
    {
        Data->Last_Encoder = Data->Encoder;
        Data->Angle = 0;
        Data->Initlized = true;
    }
    
    if(Data->Encoder < Data->Last_Encoder)
    {
        res1 = Data->Encoder - Data->Last_Encoder + MAXEncoder;
    }
    else if(Data->Encoder > Data->Last_Encoder)
    {
        res1 = Data->Encoder - Data->Last_Encoder - MAXEncoder;
    }
    res2 = Data->Encoder - Data->Last_Encoder;
    
    Data->Last_Encoder = Data->Encoder;
    
    if(fabsf(res1) > fabsf(res2))
    {
        Data->Angle += (float)res2/(MAXEncoder*Torque_Ratio)*360.f;
    }
    else
    {
        Data->Angle += (float)res1/(MAXEncoder*Torque_Ratio)*360.f;
    }
    
    return Data->Angle;
}

// 把 DJI 电机编码器增量累加成角度（范围限制在 -180~180 度）
// 处理了编码器 0/8191 之间过零跳变的问题：
// 若两次差值超过半圈(MAXEncoder*0.5)，说明发生了过零，做 ±MAXEncoder 修正
float DJI_Motor_Encoder_To_Angle(DJI_Motor_Data_Typedef *Data,float torque_ratio,uint16_t MAXEncoder)
{
    float Encoder_Err = 0.f;
    
    if(Data->Initlized != true)
    {
        Data->Last_Encoder = Data->Encoder;
        Data->Angle = Data->Encoder/(MAXEncoder*torque_ratio)*360.f;
        Data->Initlized = true;
    }
    
    Encoder_Err = Data->Encoder - Data->Last_Encoder;
    
    if(Encoder_Err > MAXEncoder*0.5f)
    {
        Data->Angle += (float)(Encoder_Err - MAXEncoder)/(MAXEncoder*torque_ratio)*360.f;
    }
    else if(Encoder_Err < -MAXEncoder*0.5f)
    {
        Data->Angle += (float)(Encoder_Err + MAXEncoder)/(MAXEncoder*torque_ratio)*360.f;
    }
    else
    {
        Data->Angle += (float)(Encoder_Err)/(MAXEncoder*torque_ratio)*360.f;
    }
    
    Data->Last_Encoder = Data->Encoder;
    Data->Angle = F_Loop_Constrain(Data->Angle,-180.f,180.f);
    return Data->Angle;
}


void DJI_Motor_Encoder_To_Continuous_Angle(DJI_Motor_Data_Typedef *Data, float reduction_ratio, float resolution)
{
    // 1. 首次上电初始化，防止开机瞬间编码器跳变
    if (Data->Initlized != true)
    {
        Data->Last_Encoder = Data->Encoder;
        Data->Total_Round = 0;
        Data->Angle = (float)Data->Encoder * 360.0f / resolution;
        Data->Initlized = true;
    }

    // 2. 计算编码器差值 (使用 16 位有符号整型计算，自动处理过零溢出)
    int16_t diff = Data->Encoder - Data->Last_Encoder;

    // 3. 防丢圈逻辑：半圈阈值法 (8192 / 2 = 4096)
    if (diff < -4096) 
    {
        Data->Total_Round++; // 正向转过了一圈
    } 
    else if (diff > 4096) 
    {
        Data->Total_Round--; // 反向转过了一圈
    }

    // 4. 更新历史编码器值
    Data->Last_Encoder = Data->Encoder;

    // 5. 计算转子当前的单圈角度 (0-360)
    Data->Angle = (float)Data->Encoder * 360.0f / resolution;

    // 6. 核心：计算输出轴的绝对连续物理角度 (已除以减速比，支持无限多圈)
    Data->Continuous_Angle = ((float)Data->Total_Round * 360.0f + Data->Angle) / reduction_ratio;
}


// DJI 电机反馈帧解析（大疆电调 CAN 协议标准格式）：
//   Data[0..1] = 编码器角度 (0~8191)
//   Data[2..3] = 转速 (RPM)
//   Data[4..5] = 电流
//   Data[6]    = 温度
void DJI_Motor_Info_Update(uint32_t *Identifier, uint8_t *Rx_Buf,DJI_Motor_Info_Typedef *DJI_Motor)
{
    // 先判断这条 CAN 帧是不是发给本电机的（按 RxIdentifier 匹配）
    if(*Identifier != DJI_Motor->FDCANFrame.RxIdentifier) return;
    
    DJI_Motor->Motor_time=xTaskGetTickCount();
    DJI_Motor->Data.Temperature = Rx_Buf[6];
    DJI_Motor->Data.Encoder  = ((int16_t)Rx_Buf[0] << 8 | (int16_t)Rx_Buf[1]);
    DJI_Motor->Data.Velocity = ((int16_t)Rx_Buf[2] << 8 | (int16_t)Rx_Buf[3]);
    DJI_Motor->Data.Current  = ((int16_t)Rx_Buf[4] << 8 | (int16_t)Rx_Buf[5]);
    
    switch(DJI_Motor->Type)
    {
        case DJI_M3508:
            // 原有的普通角度更新逻辑 (-180 到 180 度)
            DJI_Motor->Data.Angle = DJI_Motor_Encoder_To_Angle(&DJI_Motor->Data,1.f,8192);
            break;
            
        case DJI_M2006:
            // 减速比传入 36.0f，编码器线数传入 8192.0f
            DJI_Motor_Encoder_To_Continuous_Angle(&DJI_Motor->Data, 36.0f, 8192.0f);
            break;
            
        default:break;
    }
}

// 下发摩擦轮电流：帧 ID 0x200，一条帧同时带两个电机
// 大疆电调规定 0x200 帧的 8 字节 = [电机1电流高8位,低8位, 电机2电流高8位,低8位, ...]
void TransmitFriMotorCurrent(FDCAN_TxFrame_TypeDef *FDCAN_TxFrame,int16_t m1, int16_t m2)
{
    FDCAN_TxFrame->Header.Identifier = 0x200;
    FDCAN_TxFrame->Data[0] = (uint8_t)(m1 >> 8);
    FDCAN_TxFrame->Data[1] = (uint8_t)m1;
    FDCAN_TxFrame->Data[2] = (uint8_t)(m2 >> 8);
    FDCAN_TxFrame->Data[3] = (uint8_t)m2;
    USER_FDCAN_AddMessageToTxFifoQ(FDCAN_TxFrame);
}

// 下发拨弹轮电流：帧 ID 0x1FF，只带拨弹轮一个电机
void TransmitShootMotorCurrent(FDCAN_TxFrame_TypeDef *FDCAN_TxFrame,int16_t shoot_current)
{
    FDCAN_TxFrame->Header.Identifier = 0x1ff;
    FDCAN_TxFrame->Data[0] = (uint8_t)(shoot_current >> 8);
    FDCAN_TxFrame->Data[1] = (uint8_t)shoot_current;
    USER_FDCAN_AddMessageToTxFifoQ(FDCAN_TxFrame);
}
// 给达妙电机发控制命令（使能/失能/设零点/清错误）
// 命令放在 MIT 帧最后一个字节 Data[7]，前 7 字节全填 0xFF（无效值）
void DM_Motor_Command(FDCAN_TxFrame_TypeDef *FDCAN_TxFrame,DM_Motor_Info_Typedef *DM_Motor,uint8_t CMD)
{
    FDCAN_TxFrame->Header.Identifier = DM_Motor->FDCANFrame.TxIdentifier;
    FDCAN_TxFrame->Data[0] = 0xFF;
    FDCAN_TxFrame->Data[1] = 0xFF;
    FDCAN_TxFrame->Data[2] = 0xFF;
    FDCAN_TxFrame->Data[3] = 0xFF;
    FDCAN_TxFrame->Data[4] = 0xFF;
    FDCAN_TxFrame->Data[5] = 0xFF;
    FDCAN_TxFrame->Data[6] = 0xFF;
    
    switch(CMD){
        case Motor_Enable:
            FDCAN_TxFrame->Data[7] = 0xFC; 
            break;
        case Motor_Disable:
            FDCAN_TxFrame->Data[7] = 0xFD; 
            break;
        case Motor_Save_Zero_Position:
            FDCAN_TxFrame->Data[7] = 0xFE; 
            break;
				case Motor_Clear:
            FDCAN_TxFrame->Data[7] = 0xFB; 
            break;
				
        default:break;   
    }
    USER_FDCAN_AddMessageToTxFifoQ(FDCAN_TxFrame);
}

// 达妙电机下发主函数（MIT 模式核心）：
// 把 位置/速度/KP/KD/力矩 五个物理量按量程映射成 16bit/12bit 整数后拼进 8 字节 CAN 帧
// MIT 帧格式（共 8 字节）：
//   P(位置) 16bit | V(速度) 12bit | KP 12bit | KD 12bit | T(力矩) 12bit
void DM_Motor_CAN_TxMessage(FDCAN_TxFrame_TypeDef *FDCAN_TxFrame,DM_Motor_Info_Typedef *DM_Motor,float Postion, float Velocity, float KP, float KD, float Torque)
{
    if(DM_Motor->Control_Mode == MIT){
        uint16_t Postion_Tmp,Velocity_Tmp,Torque_Tmp,KP_Tmp,KD_Tmp;
        Postion_Tmp  =  float_to_uint(Postion, -DM_Motor->Param_Range.P_MAX,DM_Motor->Param_Range.P_MAX,16) ;
        Velocity_Tmp =  float_to_uint(Velocity,-DM_Motor->Param_Range.V_MAX,DM_Motor->Param_Range.V_MAX,12);
        Torque_Tmp   =  float_to_uint(Torque,  -DM_Motor->Param_Range.T_MAX,DM_Motor->Param_Range.T_MAX,12);
        KP_Tmp = float_to_uint(KP,0,500,12);
        KD_Tmp = float_to_uint(KD,0,5,12);
        FDCAN_TxFrame->Header.Identifier = DM_Motor->FDCANFrame.TxIdentifier;
        FDCAN_TxFrame->Data[0] = (uint8_t)(Postion_Tmp>>8);
        FDCAN_TxFrame->Data[1] = (uint8_t)(Postion_Tmp);
        FDCAN_TxFrame->Data[2] = (uint8_t)(Velocity_Tmp>>4);
        FDCAN_TxFrame->Data[3] = (uint8_t)((Velocity_Tmp&0x0F)<<4) | (uint8_t)(KP_Tmp>>8);
        FDCAN_TxFrame->Data[4] = (uint8_t)(KP_Tmp);
        FDCAN_TxFrame->Data[5] = (uint8_t)(KD_Tmp>>4);
        FDCAN_TxFrame->Data[6] = (uint8_t)((KD_Tmp&0x0F)<<4) | (uint8_t)(Torque_Tmp>>8);
        FDCAN_TxFrame->Data[7] = (uint8_t)(Torque_Tmp);
    }else if(DM_Motor->Control_Mode == POSITION_VELOCITY){
        uint8_t *Postion_Tmp,*Velocity_Tmp;
        Postion_Tmp  = (uint8_t*) & Postion;
        Velocity_Tmp = (uint8_t*) & Velocity;
        FDCAN_TxFrame->Header.Identifier = DM_Motor->FDCANFrame.TxIdentifier + 0x100;
        FDCAN_TxFrame->Data[0] = *(Postion_Tmp);
        FDCAN_TxFrame->Data[1] = *(Postion_Tmp + 1);
        FDCAN_TxFrame->Data[2] = *(Postion_Tmp + 2);
        FDCAN_TxFrame->Data[3] = *(Postion_Tmp + 3);
        FDCAN_TxFrame->Data[4] = *(Velocity_Tmp);
        FDCAN_TxFrame->Data[5] = *(Velocity_Tmp + 1);
        FDCAN_TxFrame->Data[6] = *(Velocity_Tmp + 2);
        FDCAN_TxFrame->Data[7] = *(Velocity_Tmp + 3);
    }else if(DM_Motor->Control_Mode == VELOCITY){
        uint8_t *Velocity_Tmp;
        Velocity_Tmp = (uint8_t*) & Velocity;
        FDCAN_TxFrame->Header.Identifier = DM_Motor->FDCANFrame.TxIdentifier + 0x200;
        FDCAN_TxFrame->Data[0] = *(Velocity_Tmp);
        FDCAN_TxFrame->Data[1] = *(Velocity_Tmp + 1);
        FDCAN_TxFrame->Data[2] = *(Velocity_Tmp + 2);
        FDCAN_TxFrame->Data[3] = *(Velocity_Tmp + 3);
        FDCAN_TxFrame->Data[4] = 0;
        FDCAN_TxFrame->Data[5] = 0;
        FDCAN_TxFrame->Data[6] = 0;
        FDCAN_TxFrame->Data[7] = 0;
    }
    USER_FDCAN_AddMessageToTxFifoQ(FDCAN_TxFrame);
}

// 达妙电机反馈帧解析（MIT 模式）：
//   Data[0]    高 4 位 = 状态字（错误/使能标志）
//   Data[1..2] = 位置 16bit；Data[3..4] = 速度 12bit；Data[4..5] = 力矩 12bit
//   Data[6]    = MOS 温度；Data[7] = 转子温度
// 解析后用 uint_to_float() 按量程换算回物理单位
void DM_Motor_Info_Update(uint32_t *Identifier,uint8_t *Rx_Buf,DM_Motor_Info_Typedef *DM_Motor)
{
    FDCAN_TxFrame_TypeDef FDCANFrameYaw;
    if(*Identifier != DM_Motor->FDCANFrame.RxIdentifier) return;
    DM_Motor->Motor_time=xTaskGetTickCount();
    DM_Motor->Data.State = Rx_Buf[0]>>4;
    DM_Motor->Data.P_int = ((uint16_t)(Rx_Buf[1]) <<8) | ((uint16_t)(Rx_Buf[2]));
    DM_Motor->Data.V_int = ((uint16_t)(Rx_Buf[3]) <<4) | ((uint16_t)(Rx_Buf[4])>>4);
    DM_Motor->Data.T_int = ((uint16_t)(Rx_Buf[4]&0xF) <<8) | ((uint16_t)(Rx_Buf[5]));
    DM_Motor->Data.Torque=  uint_to_float(DM_Motor->Data.T_int,-DM_Motor->Param_Range.T_MAX,DM_Motor->Param_Range.T_MAX,12);
    DM_Motor->Data.Position=uint_to_float(DM_Motor->Data.P_int,-DM_Motor->Param_Range.P_MAX,DM_Motor->Param_Range.P_MAX,16);
    DM_Motor->Data.Velocity=uint_to_float(DM_Motor->Data.V_int,-DM_Motor->Param_Range.V_MAX,DM_Motor->Param_Range.V_MAX,12);
    DM_Motor->Data.Temperature_MOS   = (float)(Rx_Buf[6]);
    DM_Motor->Data.Temperature_Rotor = (float)(Rx_Buf[7]);
    SectionTransform(&DM_Motor->Data.Relative_angle, DM_Motor->Data.Position, ANGLE_MIN, ANGLE_ZERO, ANGLE_MAX, -DM_Motor->Param_Range.P_MAX, 0, DM_Motor->Param_Range.P_MAX);
}

// 无符号整数 → 浮点物理量（MIT 协议解码）
// 把 [0, 2^Bits-1] 的整数按量程 [X_min, X_max] 线性映射回浮点数
float uint_to_float(int X_int, float X_min, float X_max, int Bits)
{
    float span = X_max - X_min;      // 量程跨度
    float offset = X_min;            // 量程下界
    return ((float)X_int)*span/((float)((1<<Bits)-1)) + offset;
}

// 浮点物理量 → 无符号整数（MIT 协议编码，是 uint_to_float 的逆过程）
int float_to_uint(float x, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}

void ConstrainLoop(float* output, float input, float min, float max)
{
    if (input > max)
    {
        do
        {
            input = input - (max - min);
        }while(input > max);
    }
    else if(input < min)
    {
        do
        {
            input = input + (max - min);
        }while(input < min);
    }
    
    *output = input;
}

// 分段线性映射：把输入区间 [in_min,in_max]（零点 in_zero）映射到输出区间
// [out_min,out_max]（零点 out_zero），并处理输入过零跳变，结果用循环限幅到输出区间
void SectionTransform(float* output, float input, float out_min, float out_zero, float out_max, float in_min, float in_zero, float in_max)
{
    float temp_in = input - in_zero;
    float temp_out = 0;
    
    if(temp_in > (in_max - in_min) / 2)
    {
        temp_in = temp_in - (in_max - in_min);
    }
    else if(temp_in < -(in_max - in_min) / 2)
    {
        temp_in = temp_in + (in_max - in_min);
    }
    
    temp_out = temp_in * (out_max - out_min) / (in_max - in_min) + out_zero;
    ConstrainLoop(output, temp_out, out_min, out_max);
}

void MinMaxConstrain(float* input, float min, float max)
{
    if(*input > max)
    {
        *input = max;
    }
    else if(*input < min)
    {
        *input = min;
    }
}

float Abs(float num)
{
    return (num >= 0) ? num : -num;
}


/**
  * @brief 防止PITCH、YAW轴绕大圈达到设定值，计算出一个临时的设定值
  * @param 角度设定值
  * @param 角度反馈值
  * * @retval 临时角度设定值
  */
float PreventFullCircleConstrain(float set, float fdb)
{
    if(set - fdb > 180.0f)
        return set - 360.0f; 
    else if(set - fdb < -180.0f)
        return set + 360.0f; 
    else
        return set;
}
float filter_sht(float temp,float alpha) 
{
    static float value,last_value;

    value = temp * alpha + (1.0 - alpha) * last_value;
    last_value=value;

    return value;

}
float filter_FR1(float temp,float alpha) 
{
    static float value,last_value;

    value = temp * alpha + (1.0 - alpha) * last_value;
    last_value=value;

    return value;

}

float filter_FR2(float temp,float alpha) 
{
    static float value,last_value;

    value = temp * alpha + (1.0 - alpha) * last_value;
    last_value=value;

    return value;

}

