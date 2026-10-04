/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bsp_can.c
  * @brief          : bsp can functions
  * @author         : GrassFan Wang
  * @date           : 2025/01/22
  * @version        : v1.0
  ******************************************************************************
  * @attention      : Pay attention to enable the fdcan filter
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "fdcan.h"
#include "cmsis_os.h"
#include "Motor.h"
#include "Control_Task.h"

/* ===== 文件说明：CAN 底层驱动 (bsp_can.c) —— 三路 FDCAN 的收发 =====
 * 作用：负责整车三路 FDCAN 的初始化(过滤器+中断)、接收中断分发、发送接口，
 *       以及各电机/电管/云台反馈数据的读取接口。
 *
 * 三路 CAN 分工（本工程是 Swerve 全向底盘 + 超级电容）：
 *   - FDCAN1 (CLASSIC CAN，经典 CAN)：大疆电机
 *       发 0x200(4 个 M3508 底盘轮电机电流)、0x1ff(拨弹 M2006 电流)，
 *       收 0x201~0x204(4 个轮电机回传)、0x205(拨弹回传)、0x211(电管功率)。
 *   - FDCAN2 (FD CAN，可变速率 CAN)：达妙 DM4310(MIT 协议) + 与云台通信
 *       收 0x12(DM4310 回传)、0x21(底盘速度指令，来自云台/操作手)、0x22(云台状态)。
 *   - FDCAN3 (CLASSIC CAN)：视觉/妙算通信，接收处理函数目前为空(预留)。
 *
 * 关键函数：
 *   - BSP_FDCAN_Init()：配置三路 CAN 的过滤器 + 使能接收中断 + 启动
 *   - HAL_FDCAN_RxFifo0/1Callback()：接收中断回调，按 CAN 号分发到各自解析函数
 *   - USER_FDCAN_AddMessageToTxFifoQ()：把数据帧塞进发送 FIFO 队列
 *   - GetMotor*Fdb()/GetGimbalData()/GetSuperData()：给上层任务读反馈数据的接口
 *
 * ===== 调车关注 =====
 *  - 三路 CAN 的过滤器都配成「不过滤」(FilterID1/FilterID2 全 0)，即全收，
 *    靠帧 ID 在 DJI_Motor_Info_Update / DM_Motor_Info_Update 里分辨是哪个设备。
 *  - 波特率/CAN 引脚在 CubeMX 里配(hfdcan1/2/3)，这里只做过滤器 + 中断 + 启动。
 *  - 底盘与云台通信帧：0x21=云台→底盘速度指令(ChassisData)，0x22=云台状态回传(GiambalData)。
 *  - 电管功率：接收 0x211(SuperPower_Info_Update)，发送 0x210(Sent_Superpower_Data)。
 * ===== ======================================================================== */

/* 前向声明：本文件给上层任务提供的数据读取接口(实现在本文件下方) */
uint32_t GetChassisMotor1Fdb(DJI_Motor_Info_Typedef * motor);   /* 读底盘 1 号轮电机反馈 */
uint32_t GetChassisMotor2Fdb(DJI_Motor_Info_Typedef * motor);   /* 读底盘 2 号轮电机反馈 */
uint32_t GetChassisMotor3Fdb(DJI_Motor_Info_Typedef * motor);   /* 读底盘 3 号轮电机反馈 */
uint32_t GetChassisMotor4Fdb(DJI_Motor_Info_Typedef * motor);   /* 读底盘 4 号轮电机反馈 */
uint32_t GetShootFdb(DJI_Motor_Info_Typedef * motor);           /* 读拨弹电机反馈 */

uint32_t GetYawMotorFdb(DM_Motor_Info_Typedef* motor);          /* 读 DM4310 yaw 电机反馈 */
uint32_t GetChassisXSpeedSet(float* set);                       /* 读底盘 X 方向速度设定 */
uint32_t GetChassisYSpeedSet(float* set);                       /* 读底盘 Y 方向速度设定 */
uint32_t GetChassisZSpeedSet(float* set);                       /* 读底盘 Z(自旋)速度设定 */
void GetGimbalData(Gimbal_Info_Typedef* data);                  /* 读云台状态数据 */
void GetSuperData(SuperPower_Info_Typedef* data);               /* 读超级电容功率数据 */
void Sent_Superpower_Data(FDCAN_TxFrame_TypeDef *FDCAN_TxFrame,float set_power);//uint16_t temp_power /* 向电管发目标功率 */


/**
 * @brief 接收中断用到的两个全局帧缓冲(临时存放最近一帧收到的数据)
 * @note  FDCAN1/FDCAN3 收到帧进 FIFO0，用 FDCAN_RxFIFO0Frame 暂存；
 *        FDCAN2 收到帧进 FIFO1，用 FDCAN_RxFIFO1Frame 暂存。
 */
FDCAN_RxFrame_TypeDef FDCAN_RxFIFO0Frame;
FDCAN_RxFrame_TypeDef FDCAN_RxFIFO1Frame;

/* 超级电容/电管功率数据全局实例：接收帧 0x211 解析结果存这里 */
SuperPower_Info_Typedef PowerData =
{

    .FDCANFrame = {
        .RxIdentifier = 0x211,   /* 电管回传帧 ID */
    },

};

/* 拨弹机构数据(接收 0x1ff 对应的回传 0x205，此处 RxIdentifier 实际未用于解析) */
Shoot_Info_Typedef Shootdata=
{
    .FDCANFrame = {
        .RxIdentifier = 0x1ff,
    },


};

/* 底盘速度指令数据：接收帧 0x21(云台/操作手下发 X/Y/Z 速度) */
Chassis_Info_Typedef ChassisData =
{
    .FDCANFrame = {
        .RxIdentifier = 0x21,
    },


};

/* 云台状态数据：接收帧 0x22(云台回传的位姿/状态标志) */
Gimbal_Info_Typedef GiambalData =
{

    .FDCANFrame = {
        .RxIdentifier = 0x22,
    },

};


/**
 * @brief FDCAN1 发送帧(CLASSIC CAN)：大疆电机电流指令
 * @note  经典 CAN，固定 8 字节数据段，无位速率切换(BRS_OFF)
 */
FDCAN_TxFrame_TypeDef FDCAN1_TxFrame =
{
    .hcan = &hfdcan1,
    .Header.IdType = FDCAN_STANDARD_ID,
    .Header.TxFrameType = FDCAN_DATA_FRAME,
    .Header.DataLength = 8,
    .Header.ErrorStateIndicator =  FDCAN_ESI_ACTIVE,
    .Header.BitRateSwitch = FDCAN_BRS_OFF,
    .Header.FDFormat =  FDCAN_CLASSIC_CAN,
    .Header.TxEventFifoControl =  FDCAN_NO_TX_EVENTS,
    .Header.MessageMarker = 0,
};

/**
 * @brief FDCAN2 发送帧(FD CAN)：达妙 DM4310(MIT) + 与云台通信
 * @note  FD 格式，开启位速率切换(BRS_ON)，数据段波特率更高(用于 MIT 协议)
 */
FDCAN_TxFrame_TypeDef FDCAN2_TxFrame =
{
    .hcan = &hfdcan2,
    .Header.IdType = FDCAN_STANDARD_ID,
    .Header.TxFrameType = FDCAN_DATA_FRAME,
    .Header.DataLength = 8,
    .Header.ErrorStateIndicator =  FDCAN_ESI_ACTIVE,
    .Header.BitRateSwitch = FDCAN_BRS_ON,
    .Header.FDFormat =  FDCAN_FD_CAN,
    .Header.TxEventFifoControl =  FDCAN_NO_TX_EVENTS,
    .Header.MessageMarker = 0,
};

/**
 * @brief FDCAN3 发送帧(CLASSIC CAN)：视觉/妙算通信
 */
FDCAN_TxFrame_TypeDef FDCAN3_TxFrame =
{
    .hcan = &hfdcan3,
    .Header.IdType = FDCAN_STANDARD_ID,
    .Header.TxFrameType = FDCAN_DATA_FRAME,
    .Header.DataLength = 8,
    .Header.ErrorStateIndicator =  FDCAN_ESI_ACTIVE,
    .Header.BitRateSwitch = FDCAN_BRS_OFF,
    .Header.FDFormat =  FDCAN_CLASSIC_CAN,
    .Header.TxEventFifoControl =  FDCAN_NO_TX_EVENTS,
    .Header.MessageMarker = 0,
};

/**
  * @brief  初始化三路 FDCAN：配置过滤器 + 使能接收中断 + 启动
  * @param  无
  * @retval 无
  * @note   三路 CAN 过滤器都配成「不过滤全收」(FilterID1/FilterID2 全 0)，
  *         收到帧后统一按帧 ID 在各自解析函数里分辨是哪个设备。
  *         FDCAN1/FDCAN3 收到进 FIFO0，FDCAN2 收到进 FIFO1。
  */
void BSP_FDCAN_Init(void)
{

    /* ---------- FDCAN1 过滤器(经典 CAN，大疆电机) ---------- */
    FDCAN_FilterTypeDef FDCAN1_FilterConfig;

    FDCAN1_FilterConfig.IdType = FDCAN_STANDARD_ID;              /* 标准帧 ID(11 位) */
    FDCAN1_FilterConfig.FilterIndex = 0;                         /* 过滤器编号 0 */
    FDCAN1_FilterConfig.FilterType = FDCAN_FILTER_MASK;          /* 位屏蔽型过滤器 */
    FDCAN1_FilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;  /* 通过后进 FIFO0 */
    FDCAN1_FilterConfig.FilterID1 = 0x00000000;                  /* 过滤 ID：全 0 */
    FDCAN1_FilterConfig.FilterID2 = 0x00000000;                  /* 屏蔽位：全 0，即不过滤任何 ID(全收) */

    HAL_FDCAN_ConfigFilter(&hfdcan1, &FDCAN1_FilterConfig);      /* 应用过滤器配置 */

    /* 配置全局过滤器：拒绝「不匹配任何过滤器的帧」，即只有通过上面过滤器的帧才收 */
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);

    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE,0); /* 使能 FIFO0 新消息中断 */

    HAL_FDCAN_Start(&hfdcan1);                                  /* FDCAN1 开始工作 */


    FDCAN_FilterTypeDef FDCAN2_FilterConfig;//FDCAN2过滤器结构体

    FDCAN2_FilterConfig.IdType = FDCAN_STANDARD_ID;//过滤器过滤ID类型
    FDCAN2_FilterConfig.FilterIndex = 0;//过滤器编号0
    FDCAN2_FilterConfig.FilterType = FDCAN_FILTER_MASK;//过滤器类型：经典位屏蔽过滤器
    FDCAN2_FilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;//过滤后进入FIFO1区
    FDCAN2_FilterConfig.FilterID1 = 0x00000000; //消息ID过滤器
    FDCAN2_FilterConfig.FilterID2 = 0x00000000; //过滤器屏蔽 每个位均设置0，即不过滤任何ID

    HAL_FDCAN_ConfigFilter(&hfdcan2, &FDCAN2_FilterConfig);//根据过滤器结构体中指定的参数配置FDCAN接收过滤器

    //配置 FDCAN 全局过滤器
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);

    HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);//使能中断，FIFO1新消息中断

    HAL_FDCAN_EnableTxDelayCompensation(&hfdcan2);//使能FDCAN发送延时补偿

    HAL_FDCAN_ConfigTxDelayCompensation(&hfdcan2, 14, 14); //FDCAN发送延时补偿时间设置

    HAL_FDCAN_Start(&hfdcan2);//FDCAN开始工作


    /* ---------- FDCAN3 过滤器(经典 CAN，视觉/妙算) ---------- */
    FDCAN_FilterTypeDef FDCAN3_FilterConfig;

    FDCAN3_FilterConfig.IdType = FDCAN_STANDARD_ID;
    FDCAN3_FilterConfig.FilterIndex = 0;
    FDCAN3_FilterConfig.FilterType = FDCAN_FILTER_MASK;
    FDCAN3_FilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    FDCAN3_FilterConfig.FilterID1 = 0x00000000;
    FDCAN3_FilterConfig.FilterID2 = 0x00000000;

    HAL_FDCAN_ConfigFilter(&hfdcan3, &FDCAN3_FilterConfig);

    HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);

    HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

    HAL_FDCAN_Start(&hfdcan3);
}

/**
 * @brief HAL库CAN错误中断
 *
 * @param hfdcan CAN编号
 * @param ErrorStatusITs 错误状态
 */
void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
{
    if ((ErrorStatusITs & FDCAN_IT_BUS_OFF) != RESET)
    {
        // CAN总线离线, 重新启动CAN
        HAL_FDCAN_Stop(hfdcan);
        HAL_FDCAN_Start(hfdcan);
    }
}


/**
  * @brief  把一帧 CAN 数据塞进发送 FIFO 队列(非阻塞)
  * @param  FDCAN_TxFrame 指向已填好的发送帧结构体(hcan + Header + Data)
  * @retval 无
  * @note   发送前需要先填好 FDCAN_TxFrame->Header.Identifier 和 Data[0..7]，
  *         再调用本函数；底层是 HAL 的 AddMessageToTxFifoQ。
  */
void USER_FDCAN_AddMessageToTxFifoQ(FDCAN_TxFrame_TypeDef *FDCAN_TxFrame)
{

    HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_TxFrame->hcan, &FDCAN_TxFrame->Header, FDCAN_TxFrame->Data);

}

/**
  * @brief  解析超级电容/电管功率帧(0x211)并更新 PowerData
  * @param  Identifier 指向接收帧 ID(应等于 0x211)
  * @param  Rx_Buf     指向 8 字节接收数据
  * @param  PowerData  指向要更新的 SuperPower_Info_Typedef 结构体
  * @retval 无
  * @note   电管功率帧格式(小端，每项占 2 字节，偏移 0x7fff 表示 0)：
  *         [0..1] 底盘当前功率(W, 放大10倍)  [2..3] 底盘目标功率(W, 放大10倍)
  *         [4..5] 电管当前功率(W, 放大10倍)  [6..7] 电容电压(V, 放大100倍)
  */
void SuperPower_Info_Update(uint32_t *Identifier, uint8_t *Rx_Buf, SuperPower_Info_Typedef *PowerData)
{
    /* check the Identifier：ID 不匹配直接返回 */
    if (*Identifier != PowerData->FDCANFrame.RxIdentifier) return;

    /* transforms the  data：拼 16 位 -> 减 0x7fff 去偏移 -> 除 10 换算成 W */
    PowerData->curren_chassis_power = (float)(((Rx_Buf[1] << 8 | Rx_Buf[0]) - 0x7fff) / 10.f); //底盘当前功率

    PowerData->set_chassis_power = (float)(((Rx_Buf[3] << 8 | Rx_Buf[2]) - 0x7fff) / 10.f); //电容目标功率

    PowerData->tot_power = (float)(((Rx_Buf[5] << 8 | Rx_Buf[4]) - 0x7fff) / 10.f); //电管当前功率

    PowerData->capa_voltage  = (float)(Rx_Buf[7] << 8 | Rx_Buf[6]) / 100.f; //电容电压

//
}


/**
  * @brief  解析云台状态帧(0x22)并更新 GiambalData
  * @param  Identifier  指向接收帧 ID(应等于 0x22)
  * @param  Rx_Buf      指向 8 字节接收数据
  * @param  GiambalData 指向要更新的 Gimbal_Info_Typedef 结构体
  * @retval 无
  * @note   帧格式：[0..1] 状态标志位(bit0~7)  [2..3] pitch 角度(放大100倍)
  *         [4..5] 电流设定值  [6..7] 保留/复用(yaw 角度, 放大100倍)
  */
void Gimbal_Info_Update(uint32_t *Identifier, uint8_t *Rx_Buf, Gimbal_Info_Typedef *GiambalData)
{
    /* 1. 校验 Identifier 是否匹配 */
    if (*Identifier != GiambalData->FDCANFrame.RxIdentifier) return;

    /* ========================================
     * 2. 还原 标志位 (Data[0] 和 Data[1])
     * ======================================== */
    // 将两个8位字节拼接回16位无符号整型
    uint16_t flags = (Rx_Buf[0] << 8) | Rx_Buf[1];

    // 按位取出对应的状态 (严格对应发送端的 bit0 ~ bit7)
    GiambalData->rotate_state         = (flags >> 0) & 0x01; // bit0: 小陀螺
    GiambalData->follow_state         = (flags >> 1) & 0x01; // bit1: 无跟随
    GiambalData->zero_force_state     = (flags >> 2) & 0x01; // bit2: 无力模式
    GiambalData->shoot_ready_state    = (flags >> 3) & 0x01; // bit3: 准备发射
    GiambalData->shoot_normal_state   = (flags >> 4) & 0x01; // bit4: 正常发射
    GiambalData->vision_robot_state   = (flags >> 5) & 0x01; // bit5: 视觉识别到机器人
    GiambalData->vision_shoot_state   = (flags >> 6) & 0x01; // bit6: 视觉建议射击
    GiambalData->reset_flag           = (flags >> 7) & 0x01; // bit7: 复位标志

    /* ========================================
     * 3. 还原 Pitch 角度 (Data[2] 和 Data[3])
     * ======================================== */
    // 拼接回带符号的16位整型 (角度可能有负数)
    int16_t pitch_raw = (Rx_Buf[2] << 8) | Rx_Buf[3];
    // 发送端放大了100倍，这里除以100.0f转回真实的浮点角度
    GiambalData->pitch = (float)pitch_raw / 100.0f;

    /* ========================================
     * 4. 还原 电流值 (Data[4] 和 Data[5])
     * ======================================== */
    // 如果电流也是带符号的，请确保接收结构体里的 current_set 是 int16_t 类型
    GiambalData->current_set = (Rx_Buf[4] << 8) | Rx_Buf[5];

    /* ========================================
     * 5. 还原 整型保留位 (Data[6] 和 Data[7])
     * ======================================== */
     /* 注意: 疑似bug —— reserved_int 和 yaw 同时取自 Data[6..7] 同一组字节，
      * 一个按无符号 16 位解释、一个按有符号再除 100 当角度，二者语义重叠。 */
     GiambalData->reserved_int  = (Rx_Buf[6] << 8) | Rx_Buf[7];
     GiambalData->yaw  = (float)((int16_t)((Rx_Buf[6] << 8) | Rx_Buf[7])) / 100.0f;
}

/**
  * @brief  解析底盘速度指令帧(0x21)并更新 ChassisData
  * @param  Identifier  指向接收帧 ID(应等于 0x21)
  * @param  Rx_Buf      指向 8 字节接收数据
  * @param  ChassisData 指向要更新的 Chassis_Info_Typedef 结构体
  * @retval 无
  * @note   帧格式(大端，每轴 2 字节，放大 1000 倍)：[0..1] X 速度  [2..3] Y 速度
  *         [4..5] Z(自旋)速度  [6..7] 复位标志
  */
void Chassis_Info_Update(uint32_t *Identifier, uint8_t *Rx_Buf, Chassis_Info_Typedef *ChassisData)
{
    /* check the Identifier：ID 不匹配直接返回 */
    if (*Identifier != ChassisData->FDCANFrame.RxIdentifier) return;

    /* transforms the  gimbal data：解析 X/Y/Z 三轴速度(带符号, 除 1000 还原) */
    ChassisData->chassis_speed_set_time[CHASSIS_X] = xTaskGetTickCount();          /* 记录 X 指令到达时刻 */
    ChassisData->chassis_xyz_set[CHASSIS_X] = Int16ToFloat(((int16_t)((Rx_Buf[0] << 8) | Rx_Buf[1]))) / 1000.0f;
    ChassisData->chassis_speed_set_time[CHASSIS_Y] = xTaskGetTickCount();          /* 记录 Y 指令到达时刻 */
    ChassisData->chassis_xyz_set[CHASSIS_Y] = Int16ToFloat(((int16_t)((Rx_Buf[2] << 8) | Rx_Buf[3]))) / 1000.0f;
    ChassisData->chassis_speed_set_time[CHASSIS_Z] = xTaskGetTickCount();          /* 记录 Z 指令到达时刻 */
    ChassisData->chassis_xyz_set[CHASSIS_Z] = Int16ToFloat(((int16_t)((Rx_Buf[4] << 8) | Rx_Buf[5]))) / 1000.0f;
    ChassisData->reset_flag = (Rx_Buf[6] << 8) | Rx_Buf[7];                       /* 复位标志(来自云台) */

}


/**
  * @brief  FDCAN1 收到帧(FIFO0)后的解析分发函数
  * @param  Identifier 指向接收帧 ID
  * @param  Data       8 字节接收数据
  * @retval 无
  * @note   同一帧会依次喂给各电机/电管的 Update 函数，内部靠 ID 匹配，
  *         只有 ID 相符的那个才会真正解析(其余直接 return)。
  */
static void FDCAN1_RxFifo0RxHandler(uint32_t *Identifier, uint8_t Data[8])
{

    DJI_Motor_Info_Update(Identifier, Data, &Chassis_Motor[0]);   /* 底盘 1 号轮电机(收 0x201) */
    DJI_Motor_Info_Update(Identifier, Data, &Chassis_Motor[1]);   /* 底盘 2 号轮电机(收 0x202) */
    DJI_Motor_Info_Update(Identifier, Data, &Chassis_Motor[2]);   /* 底盘 3 号轮电机(收 0x203) */
    DJI_Motor_Info_Update(Identifier, Data, &Chassis_Motor[3]);   /* 底盘 4 号轮电机(收 0x204) */
    DJI_Motor_Info_Update(Identifier, Data, &Shoot_Motor);        /* 拨弹电机(收 0x205) */
    SuperPower_Info_Update(Identifier, Data, &PowerData);         /* 电管功率(收 0x211) */

}

/**
  * @brief  FDCAN3 收到帧(FIFO0)后的解析分发函数(视觉/妙算)
  * @param  Identifier 指向接收帧 ID
  * @param  Data       8 字节接收数据
  * @retval 无
  * @note   当前为空，视觉/妙算的 CAN 数据暂未接入解析。
  */
static void FDCAN3_RxFifo0RxHandler(uint32_t *Identifier, uint8_t Data[8])
{
	/* 注意: 当前为空，视觉/妙算(FDCAN3)的接收帧未做任何解析，如需使用在此补充 */
}


/**
  * @brief  FDCAN2 收到帧(FIFO1)后的解析分发函数(DM4310 + 与云台通信)
  * @param  Identifier 指向接收帧 ID
  * @param  Data       8 字节接收数据
  * @retval 无
  * @note   收 0x12(DM4310 回传)、0x21(底盘速度指令)、0x22(云台状态)。
  */
static void FDCAN2_RxFifo1RxHandler(uint32_t *Identifier, uint8_t Data[8])
{
    Gimbal_Info_Update(Identifier, Data, &GiambalData);           /* 云台状态(收 0x22) */
    Chassis_Info_Update(Identifier, Data, &ChassisData);          /* 底盘速度指令(收 0x21) */
	  DM_Motor_Info_Update(Identifier, Data, &DM_4310_Motor[0]);   /* DM4310 回传(收 0x12) */
	  /* 注意: Swerve 底盘的 LK 舵向电机(LK_Motors[4], 收 0x181~0x184)反馈
	   * 未在此解析，若舵向电机读不到数据需在此补 LK_Motor_Info_Update。 */

}

/**
 * @brief HAL库CAN错误中断
 *
 * @param hfdcan CAN编号
 * @param ErrorStatusITs 错误状态
 * @note  当 CAN 总线进入 BUS_OFF(总线离线)状态时，停止再重新启动该路 CAN 尝试恢复。
 *        并按是哪一路 CAN 分别计数，方便定位是哪一路总线在反复掉线。
 */
//void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
//{
//    if ((ErrorStatusITs & FDCAN_IT_BUS_OFF) != RESET)
//    {
//        // CAN总线离线, 重新启动CAN
//        HAL_FDCAN_Stop(hfdcan);
//        HAL_FDCAN_Start(hfdcan);

//    }
//}

/* ===== 电机反馈读取接口 =====
 * 这些函数把某个电机实例的数据整体拷出(*motor = xxx)，并返回该电机最后一次
 * 收到反馈的时间戳(xTaskGetTickCount 的 tick 值)。上层任务据此判断数据新旧。 */

uint32_t GetChassisMotor1Fdb(DJI_Motor_Info_Typedef * motor)
{
    *motor = Chassis_Motor[0];
    return Chassis_Motor[0].Motor_time;
}
uint32_t GetChassisMotor2Fdb(DJI_Motor_Info_Typedef * motor)
{
    *motor = Chassis_Motor[1];
    return Chassis_Motor[1].Motor_time;
}
uint32_t GetChassisMotor3Fdb(DJI_Motor_Info_Typedef * motor)
{
    *motor = Chassis_Motor[2];
    return Chassis_Motor[2].Motor_time;
}
uint32_t GetChassisMotor4Fdb(DJI_Motor_Info_Typedef * motor)
{
    *motor = Chassis_Motor[3];
    return Chassis_Motor[3].Motor_time;
}
uint32_t GetShootFdb(DJI_Motor_Info_Typedef * motor)
{
    *motor = Shoot_Motor;
    return Shoot_Motor.Motor_time;
}

uint32_t GetYawMotorFdb(DM_Motor_Info_Typedef* motor)
{
    *motor = DM_4310_Motor[0];
    return DM_4310_Motor[0].Motor_time;
}

/* 读底盘 X/Y/Z 速度设定值(来自云台 0x21 帧)，返回该轴的指令到达时间戳 */
uint32_t GetChassisXSpeedSet(float* set)
{
    *set = ChassisData.chassis_xyz_set[CHASSIS_X];
    return ChassisData.chassis_speed_set_time[CHASSIS_X];
}

uint32_t GetChassisYSpeedSet(float* set)
{
    *set = ChassisData.chassis_xyz_set[CHASSIS_Y];
    return ChassisData.chassis_speed_set_time[CHASSIS_Y];
}
uint32_t GetChassisZSpeedSet(float* set)
{
    *set = ChassisData.chassis_xyz_set[CHASSIS_Z];
    return ChassisData.chassis_speed_set_time[CHASSIS_Z];
}

void GetGimbalData(Gimbal_Info_Typedef* data)
{
    *data = GiambalData;
}

void GetSuperData(SuperPower_Info_Typedef* data)
{
    *data = PowerData;
}

/**
  * @brief  向电管发送底盘目标功率(帧 0x210)
  * @param  FDCAN_TxFrame 指向发送帧结构体(填 hcan 与 Header，此处填 ID 与 Data)
  * @param  set_power     目标功率(W)
  * @retval 无
  * @note   编码方式与接收 0x211 相反：功率放大 10 倍再加 0x7fff 偏移，
  *         低字节在前(Data[0] 低位, Data[1] 高位)。
  */
void Sent_Superpower_Data(FDCAN_TxFrame_TypeDef *FDCAN_TxFrame,float set_power)//uint16_t temp_power
{
	  uint16_t temp_set_power = (set_power) * 10 + 0x7fff;   /* 功率 W -> 放大10倍 + 0x7fff 偏移 */
    FDCAN_TxFrame->Header.Identifier = 0x210;              /* 电管目标功率帧 ID */
    FDCAN_TxFrame->Data[0] = temp_set_power;               /* 低字节 */
    FDCAN_TxFrame->Data[1] = temp_set_power >> 8;          /* 高字节 */
    FDCAN_TxFrame->Data[2] = 0;
    FDCAN_TxFrame->Data[3] = 0;
    FDCAN_TxFrame->Data[4] = 0;
    FDCAN_TxFrame->Data[5] = 0;
    FDCAN_TxFrame->Data[6] = 0;
    FDCAN_TxFrame->Data[7] = 0;
    USER_FDCAN_AddMessageToTxFifoQ(FDCAN_TxFrame);         /* 塞进发送 FIFO */
}

/**
  * @brief  FDCAN 接收 FIFO0 中断回调(由 HAL 在 RX FIFO0 新消息中断里调用)
  * @param  hfdcan     指向触发中断的 FDCAN 句柄
  * @param  RxFifo0ITs  触发的 FIFO0 中断标志
  * @retval 无
  * @note   FDCAN1/FDCAN3 都配成进 FIFO0，所以这里按句柄区分是哪一路 CAN，
  *         再分发到对应的解析函数。
  */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{

    HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &FDCAN_RxFIFO0Frame.Header, FDCAN_RxFIFO0Frame.Data);

    if (hfdcan == &hfdcan1)   /* FDCAN1：大疆电机 + 电管 */
    {

        FDCAN1_RxFifo0RxHandler(&FDCAN_RxFIFO0Frame.Header.Identifier, FDCAN_RxFIFO0Frame.Data);

    }

    if (hfdcan == &hfdcan3)   /* FDCAN3：视觉/妙算 */
    {

        FDCAN3_RxFifo0RxHandler(&FDCAN_RxFIFO0Frame.Header.Identifier, FDCAN_RxFIFO0Frame.Data);

    }

}

/**
  * @brief  FDCAN 接收 FIFO1 中断回调(FDCAN2 专用)
  * @param  hfdcan     指向触发中断的 FDCAN 句柄
  * @param  RxFifo0ITs  触发的 FIFO1 中断标志(形参名沿用了 RxFifo0ITs)
  * @retval 无
  * @note   FDCAN2 配成进 FIFO1，所以这里直接分发给 FDCAN2 的解析函数。
  */
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{

    HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &FDCAN_RxFIFO1Frame.Header, FDCAN_RxFIFO1Frame.Data);

    FDCAN2_RxFifo1RxHandler(&FDCAN_RxFIFO1Frame.Header.Identifier, FDCAN_RxFIFO1Frame.Data);

}

/* int16 -> float：正确处理符号位(等价于简单强转) */
float Int16ToFloat(int16_t data)
{
    if (data >= 0)
    {
        return (float)data;
    }
    else
    {
        float temp = (float) - data;
        return -temp;
    }
}

/* int16 -> int32：正确处理符号位 */
int Int16ToInt32(int16_t data)
{
    if (data >= 0)
    {
        return (int)data;
    }
    else
    {
        int temp = (int) - data;
        return -temp;
    }
}