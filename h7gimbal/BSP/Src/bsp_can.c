/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bsp_can.c
  * @brief          : bsp can functions
  * @version        : v1.0
  ******************************************************************************
  * @attention      : Pay attention to enable the fdcan filter
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "fdcan.h"
#include "bsp_can.h"
#include "cmsis_os.h"
#include "Motor.h"
#include "vision.h"
#include "Control_Task.h"

/**
 * =====================================================================
 * 文件说明：CAN 底层驱动 (bsp_can.c) —— 三路 FDCAN 的收发
 * ---------------------------------------------------------------------
 * 本工程用了 3 路 FDCAN，分工如下：
 *   - FDCAN1 (CLASSIC CAN)：大疆电机(摩擦轮 M3508、拨弹轮 M2006)
 *     发送电流指令(0x200/0x1ff)，接收电机回传(0x201/0x202/0x25 等)
 *   - FDCAN2 (FD CAN)：达妙 DM4310 云台电机 + 底盘通信
 *     用 MIT 协议控制 pitch/yaw，还发 0x21(底盘指令)/0x22(云台状态)
 *   - FDCAN3 (CLASSIC CAN)：视觉/妙算通信，收视觉数据、发弹速和姿态
 *
 * 关键函数：
 *   - BSP_FDCAN_Init()：初始化三路 CAN 的过滤器 + 中断 + 启动
 *   - HAL_FDCAN_RxFifo0/1Callback()：接收中断回调，按 CAN 号分发解析
 *   - USER_FDCAN_AddMessageToTxFifoQ()：把数据帧塞进发送队列
 *   - GetMotor*Fdb()：给上层任务读电机反馈数据的接口
 *
 * ===== 调车关注 =====
 *  - 三路 CAN 的过滤器都配成「不过滤」(全 0)，即全收，靠帧 ID 在
 *    DJI_Motor_Info_Update/DM_Motor_Info_Update 里分辨是哪个电机
 *  - 波特率/CAN 编号在 CubeMX 里配(hfdcan1/2/3)，这里只做过滤器+中断
 * =====================================================================
 */

uint32_t GetMotorFR1Fdb(DJI_Motor_Info_Typedef * motor);
uint32_t GetMotorFR2Fdb(DJI_Motor_Info_Typedef * motor);

uint32_t GetPitchMotorFdb(DM_Motor_Info_Typedef* motor);
uint32_t GetYawMotorFdb(DM_Motor_Info_Typedef* motor);
uint32_t GetMotorPitchRelativeAngle(float* angle);
uint32_t GetMotorYawRelativeAngle(float* angle);
uint32_t GetShootMotorFdb(DJI_Motor_Info_Typedef* motor);


/**
 * @brief The structure that contains the Information of FDCAN1 and FDCAN2 Receive.
 */
FDCAN_RxFrame_TypeDef FDCAN_RxFIFO0Frame;
FDCAN_RxFrame_TypeDef FDCAN_RxFIFO1Frame;
/**
 * @brief The structure that contains the Information of FDCAN1 Transmit(CLASSIC_CAN).
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
 * @brief The structure that contains the Information of FDCAN2 Transmit(FDCAN).
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
 * @brief The structure that contains the Information of FDCAN3 Transmit(CLASSIC_CAN).
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
  * @brief  Configures the FDCAN Filter.
            FDCAN1:CLASSIC_CAN  FDCAN2:FDCAN  FDCAN3:CLASSIC_CAN
  * @param  None
  * @retval None
  */
void BSP_FDCAN_Init(void)
{

    FDCAN_FilterTypeDef FDCAN1_FilterConfig;

    FDCAN1_FilterConfig.IdType = FDCAN_STANDARD_ID; 
    FDCAN1_FilterConfig.FilterIndex = 0;           
    FDCAN1_FilterConfig.FilterType = FDCAN_FILTER_MASK; 
    FDCAN1_FilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    FDCAN1_FilterConfig.FilterID1 = 0x00000000; 
    FDCAN1_FilterConfig.FilterID2 = 0x00000000; 

    HAL_FDCAN_ConfigFilter(&hfdcan1, &FDCAN1_FilterConfig); 

    HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE); 

    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

    HAL_FDCAN_Start(&hfdcan1);


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
  * @brief  Function to transmit the FDCAN message.
  * @param  *FDCAN_TxFrame :the structure that contains the Information of FDCAN
  * @retval None
  */
void USER_FDCAN_AddMessageToTxFifoQ(FDCAN_TxFrame_TypeDef *FDCAN_TxFrame)
{

    HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_TxFrame->hcan, &FDCAN_TxFrame->Header, FDCAN_TxFrame->Data);

}


/**
  * @brief  Function to converting the FDCAN1 received message to Fifo0.
    * @param  Identifier: Received the identifier.
    * @param  Data: Array that contains the received massage.
  * @retval None
  */
static void FDCAN1_RxFifo0RxHandler(uint32_t *Identifier, uint8_t Data[8])
{

	
    DJI_Motor_Info_Update(Identifier, Data, &FR_Motor[0]);
    DJI_Motor_Info_Update(Identifier, Data, &FR_Motor[1]);
		
		DJI_Motor_Info_Update(Identifier, Data, &GM6020_Motor); 
}

/**
  * @brief  Function to converting the FDCAN3 received message to Fifo0.
    * @param  Identifier: Received the identifier.
    * @param  Data: Array that contains the received massage.
  * @retval None
  */
static void FDCAN3_RxFifo0RxHandler(uint32_t *Identifier, uint8_t Data[8])
{
   Vision_Info_Update(Identifier, Data,&Vision_Data);

}


/**
  * @brief  Function to converting the FDCAN2 received message to Fifo1.
    * @param  Identifier: Received the identifier.
    * @param  Data: Array that contains the received massage.
  * @retval None
  */
static void FDCAN2_RxFifo1RxHandler(uint32_t *Identifier, uint8_t Data[8])
{
    DM_Motor_Info_Update(Identifier, Data, &DM_4310_Motor[0]);
    DM_Motor_Info_Update(Identifier, Data, &DM_4310_Motor[1]);
	  
		DJI_Motor_Info_Update(Identifier, Data, &Shoot_Motor);
	
	  Shoot_Info_Update(Identifier, Data, &shoot);

}

/**
 * @brief 把电机数据(角度/转速/电流/温度)打包成 0x306 帧发出去(给上位机/云台)
 * @note  4 个 int16 数据各占 2 字节，高位在前(大端)，共 8 字节。
 */
void sentToGimbal(FDCAN_TxFrame_TypeDef *FDCAN_TxFrame,int16_t ecd, int16_t speed_rpm, int16_t current, int16_t temperate){
	
	  FDCAN_TxFrame->Header.Identifier = 0x306;

    FDCAN_TxFrame->Data[0] = (uint8_t)(ecd >> 8);
    FDCAN_TxFrame->Data[1] = (uint8_t)ecd;
    FDCAN_TxFrame->Data[2] = (uint8_t)(speed_rpm >> 8);
    FDCAN_TxFrame->Data[3] = (uint8_t)speed_rpm;
    FDCAN_TxFrame->Data[4] = (uint8_t)(current >> 8);
    FDCAN_TxFrame->Data[5] = (uint8_t)current;
    FDCAN_TxFrame->Data[6] = (uint8_t)(temperate >> 8);
    FDCAN_TxFrame->Data[7] = (uint8_t)temperate;
	
    USER_FDCAN_AddMessageToTxFifoQ(FDCAN_TxFrame);
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
/* ==================== 电机反馈读取接口 ====================
 * 这些函数把某个电机实例的数据拷出来，返回该电机最后更新时间戳。
 * 上层任务(Control_Task/Shoot_Task)每 1ms 调这些函数读反馈。 */

uint32_t GetMotorFR1Fdb(DJI_Motor_Info_Typedef * motor)
{
    *motor = FR_Motor[0];
    return FR_Motor[0].Motor_time;
}
uint32_t GetMotorFR2Fdb(DJI_Motor_Info_Typedef * motor)
{
    *motor = FR_Motor[1];
    return FR_Motor[1].Motor_time;
}


uint32_t GetPitchMotorFdb(DM_Motor_Info_Typedef* motor)
{
    *motor = DM_4310_Motor[0];
    return DM_4310_Motor[0].Motor_time;
}
uint32_t GetYawMotorFdb(DM_Motor_Info_Typedef* motor)
{
    *motor = DM_4310_Motor[1];
    return DM_4310_Motor[1].Motor_time;
}

uint32_t GetMotorPitchRelativeAngle(float* angle)
{
    *angle = DM_4310_Motor[0].Data.Relative_angle;
    return DM_4310_Motor[0].Motor_time;
}

uint32_t GetMotorYawRelativeAngle(float* angle)
{
    *angle = DM_4310_Motor[1].Data.Relative_angle;
    return DM_4310_Motor[1].Motor_time;
}
uint32_t GetShootMotorFdb(DJI_Motor_Info_Typedef* motor)
{
    *motor = Shoot_Motor;  
	  return Shoot_Motor.Motor_time;
}


/**
  * @brief  Rx FIFO 0 callback.
  * @param  hfdcan pointer to an FDCAN_HandleTypeDef structure that contains
  *         the configuration information for the specified FDCAN.
  * @param  RxFifo0ITs indicates which Rx FIFO 0 interrupts are signaled.
  *         This parameter can be any combination of @arg FDCAN_Rx_Fifo0_Interrupts.
  * @retval None
  */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{

    HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &FDCAN_RxFIFO0Frame.Header, FDCAN_RxFIFO0Frame.Data);

    if (hfdcan == &hfdcan1)
    {
        FDCAN1_RxFifo0RxHandler(&FDCAN_RxFIFO0Frame.Header.Identifier, FDCAN_RxFIFO0Frame.Data);
    }

    if (hfdcan == &hfdcan3)
    {
        FDCAN3_RxFifo0RxHandler(&FDCAN_RxFIFO0Frame.Header.Identifier, FDCAN_RxFIFO0Frame.Data);
    }

}

/**
  * @brief  Rx FIFO 1 callback.
  * @param  hfdcan pointer to an FDCAN_HandleTypeDef structure that contains
  *         the configuration information for the specified FDCAN.
  * @param  RxFifo1ITs indicates which Rx FIFO 1 interrupts are signaled.
  *         This parameter can be any combination of @arg FDCAN_Rx_Fifo1_Interrupts.
  * @retval None
  */
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{

    HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &FDCAN_RxFIFO1Frame.Header, FDCAN_RxFIFO1Frame.Data);

    FDCAN2_RxFifo1RxHandler(&FDCAN_RxFIFO1Frame.Header.Identifier, FDCAN_RxFIFO1Frame.Data);

}
/* int16 → float 转换(其实就是简单强转，符号位正确处理) */
float Int16ToFloat(int16_t data)
{
	if (data >= 0)
	{
		return (float)data;
	}
	else
	{
		float temp = (float)-data;
		return -temp;
	}
}

int Int16ToInt32(int16_t data)
{
	if (data >= 0)
	{
		return (int)data;
	}
	else
	{
		int temp = (int)-data;
		return -temp;
	}
}