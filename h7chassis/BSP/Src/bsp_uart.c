/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bsp_uart.c
  * @brief          : bsp uart functions
  * @author         : GrassFan Wang
  * @date           : 2025/04/27
  * @version        : v1.0
  ******************************************************************************
  * @attention      : Pay attention to init the  BSP_USART_Init functions
  ******************************************************************************
  */
/* USER CODE END Header */

#include "bsp_uart.h"
#include "usart.h"
#include "remote_control.h"
#include "Referee_System.h"
#include "Image_Transmission.h"

/* ===== 文件说明：BSP 层 UART 驱动 (bsp_uart.c) —— 各串口用途与 DMA 双缓冲接收 =====
 * 作用：初始化各串口并用 DMA「双缓冲 + 空闲中断」接收不定长数据，收到完整帧后
 *       分发到对应解析函数。本工程串口用途如下：
 *   - USART1：裁判系统(Referee_System) 与 图传(Image_Transmission) 二选一，
 *             由下方宏 USART1_RX_Switch 切换，默认接裁判系统(波特率 115200)。
 *   - USART5：SBUS 遥控器(DR16)接收机，100k 波特率，18 字节一帧。
 *   - USART7：VoFa+ 上位机调试(发送 JustFloat 波形数据)。
 *   - USART2 / USART3 / USART10：预留(接收处理函数当前为空)。
 *
 * 关键机制：DMA 双缓冲(M0/M1 交替收) + UART 空闲中断(IDLE)，一帧收完进中断，
 *          切换缓冲指针并解析，实现无阻塞连续接收。
 *
 * ===== 调车关注 =====
 *  - 裁判系统/图传在 USART1 上只能二选一，改 USART1_RX_Switch 后需重编译。
 *  - 各串口引脚/波特率/DMA 在 CubeMX 里配(MX_USARTx_UART_Init)，这里只做双缓冲。
 */

/* 各串口接收处理函数前向声明(static, 本文件内实现) */
static void USER_USART5_RxHandler(UART_HandleTypeDef *huart,uint16_t Size);

static void USER_USART2_RxHandler(UART_HandleTypeDef *huart,uint16_t Size);

static void USER_USART3_RxHandler(UART_HandleTypeDef *huart,uint16_t Size);

static void USART_RxDMA_MultiBuffer_Init(UART_HandleTypeDef *, uint32_t *, uint32_t *, uint32_t );

/* 用于读取 PLL2 时钟频率的变量(USART1 时钟源为 PLL2Q) */
PLL2_ClocksTypeDef PLL2_ClockFreq;

/* USART1 复用切换开关：0=裁判系统(Referee_System)  1=图传(Image_Transmission) */
#define USART1_RX_Switch  0  //Referee_System 0  Image_Transmission 1

/**
  * @brief  初始化各串口：USART1(裁判/图传) + USART5(SBUS)，并开启 DMA 双缓冲接收
  * @param  无
  * @retval 无
  * @note   波特率通过直接改 USARTx->BRR 寄存器设置(时钟来自 PLL2Q)。
  */
void BSP_USART_Init(void){

	 HAL_RCCEx_GetPLL2ClockFreq(&PLL2_ClockFreq);// Get PLL2 P Q R  Clock Frequency
	 uint32_t USART1_ClockFreq = PLL2_ClockFreq.PLL2_Q_Frequency;//USART1 use PLL2Q Clock Frequency

	#if USART1_RX_Switch
	 /* ---------- 开关=1：USART1 接图传(波特率 921600) ---------- */
	 USART1->CR1 &= ~USART_CR1_UE; //Disable USART1
   USART1->BRR = (uint32_t)(USART1_ClockFreq/921600);// Set baudrate 921600
	 USART1->CR1 |= USART_CR1_UE;// Enable USART1
	 USART_RxDMA_MultiBuffer_Init(&huart1,(uint32_t *)Image_Trans_MultiRx_Buff[0],(uint32_t *)Image_Trans_MultiRx_Buff[1],IMAGE_TRANS_RX_LENGTH);

	#else

	 /* ---------- 开关=0：USART1 接裁判系统(波特率 115200) ---------- */
	 USART1->CR1 &= ~USART_CR1_UE;
   USART1->BRR = (uint32_t)(USART1_ClockFreq/115200);// Set baudrate 115200
	 USART1->CR1 |= USART_CR1_UE;
	 USART_RxDMA_MultiBuffer_Init(&huart1,(uint32_t *)Referee_System_Info_MultiRx_Buf[0],(uint32_t *)Referee_System_Info_MultiRx_Buf[1],REFEREE_RXFRAME_LENGTH);

	#endif

	//USART3 Init
	 /* ---------- USART5 接 SBUS 遥控器接收机(18 字节一帧) ---------- */
	 USART_RxDMA_MultiBuffer_Init(&huart5,(uint32_t *)SBUS_MultiRx_Buf[0],(uint32_t *)SBUS_MultiRx_Buf[1],SBUS_RX_BUF_NUM);

}

/**
  * @brief  配置某串口的 DMA 双缓冲接收(开空闲中断)
  * @param  huart          指向 UART 句柄
  * @param  DstAddress     缓冲区 M0 地址(第一个接收缓冲)
  * @param  SecondMemAddress 缓冲区 M1 地址(第二个接收缓冲)
  * @param  DataLength     每个缓冲区的字节长度
  * @retval 无
  * @note   DMA 开启双缓冲模式(DBM)，M0/M1 交替接收，避免单缓冲在解析期间丢数据。
  */
static void USART_RxDMA_MultiBuffer_Init(UART_HandleTypeDef *huart, uint32_t *DstAddress, uint32_t *SecondMemAddress, uint32_t DataLength){

 huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;   /* 接收方式：收到空闲(IDLE)即认为一帧结束 */

 huart->RxXferSize    = DataLength * 2;              /* 两个缓冲共占用的总长度(字节, 按 uint8 计) */

 SET_BIT(huart->Instance->CR3,USART_CR3_DMAR);       /* 使能 UART 的 DMA 接收请求 */

 __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);          /* 使能 UART 空闲中断 */

  do{                                                /* 等待 DMA 停止(确保能安全改配置) */
      __HAL_DMA_DISABLE(huart->hdmarx);
  }while(((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR & DMA_SxCR_EN);

  /* Configure the source memory Buffer address  */
  ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->PAR = (uint32_t)&huart->Instance->RDR;   /* 外设地址=UART 数据寄存器 RDR */

  /* Configure the destination memory Buffer address */
  ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->M0AR = (uint32_t)DstAddress;             /* 内存地址 M0 */

  /* Configure DMA Stream destination address */
  ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->M1AR = (uint32_t)SecondMemAddress;       /* 内存地址 M1 */

  /* Configure the length of data to be transferred from source to destination */
  ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->NDTR = DataLength;                       /* 每个缓冲的接收长度 */

  /* Enable double memory buffer */
  SET_BIT(((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR, DMA_SxCR_DBM);               /* 开启 DMA 双缓冲模式 */

  /* Enable DMA */
  __HAL_DMA_ENABLE(huart->hdmarx);

}



/**
  * @brief  USART5 接收完成回调：解析 SBUS 遥控器帧
  * @param  huart UART 句柄
  * @param  Size  本次接收到的数据长度
  * @retval 无
  * @note   判断当前 DMA 正在用哪个缓冲(M0/M1)，把另一块已收满的缓冲
  *          交给 SBUS_TO_RC() 解析成遥控器结构体 remote_ctrl。
  */
static void USER_USART5_RxHandler(UART_HandleTypeDef *huart,uint16_t Size){

    /* Current memory buffer used is Memory 0 */
  if(((((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR) & DMA_SxCR_CT ) == RESET){

					/* Disable DMA */
					__HAL_DMA_DISABLE(huart->hdmarx);

				  /* Switch Memory 0 to Memory 1*/
					((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR |= DMA_SxCR_CT;

				  /* Reset the receive count */
					__HAL_DMA_SET_COUNTER(huart->hdmarx,SBUS_RX_BUF_NUM*2);

				  /* Juge whether size is equal to the length of the received data */
					if(Size == SBUS_RX_BUF_NUM)
					{

						/* Memory 0 data update to remote_ctrl*/
						SBUS_TO_RC(SBUS_MultiRx_Buf[0],&remote_ctrl);

					}

			}
			/* Current memory buffer used is Memory 1 */
			else{
					/* Disable DMA */
					__HAL_DMA_DISABLE(huart->hdmarx);

				  /* Switch Memory 1 to Memory 0*/
					((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR &= ~(DMA_SxCR_CT);

					/* Reset the receive count */
					__HAL_DMA_SET_COUNTER(huart->hdmarx,SBUS_RX_BUF_NUM*2);

					if(Size == SBUS_RX_BUF_NUM)
					{
						/* Memory 1 to data update to remote_ctrl*/
						SBUS_TO_RC(SBUS_MultiRx_Buf[1],&remote_ctrl);
					}

			}

}

/**
  * @brief  USART1 接收完成回调：解析裁判系统(或图传)帧
  * @param  huart UART 句柄
  * @param  Size  本次接收到的数据长度
  * @retval 无
  * @note   与 USART5 一样的双缓冲切换逻辑，按 USART1_RX_Switch 决定
  *          解析成裁判系统还是图传数据。
  */



static void USER_USART1_RxHandler(UART_HandleTypeDef *huart,uint16_t Size){

#if USART1_RX_Switch
	 /* ---------- 开关=1：USART1 解析图传数据 ---------- */
  if(((((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR) & DMA_SxCR_CT ) == RESET){

					/* Disable DMA */
					__HAL_DMA_DISABLE(huart->hdmarx);

				  /* Switch Memory 0 to Memory 1*/
					((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR |= DMA_SxCR_CT;

				  /* Juge whether size is equal to the length of the received data */
					if(Size > 10)
					{

						/* Memory 0 data update to remote_ctrl*/
						Image_Transmission_Info_Update(Image_Trans_MultiRx_Buff[0]);

					}

					/* Reset the receive count */
				  __HAL_DMA_SET_COUNTER(huart->hdmarx,IMAGE_TRANS_RX_LENGTH*2);

			}
			/* Current memory buffer used is Memory 1 */
			else{
					/* Disable DMA */
					__HAL_DMA_DISABLE(huart->hdmarx);

				  /* Switch Memory 1 to Memory 0*/
					((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR &= ~(DMA_SxCR_CT);

					if(Size > 10)
					{
						/* Memory 1 to data update to remote_ctrl*/
						Image_Transmission_Info_Update(Image_Trans_MultiRx_Buff[1]);
					}
				  /* Reset the receive count */
					__HAL_DMA_SET_COUNTER(huart->hdmarx,IMAGE_TRANS_RX_LENGTH*2);
			}
#else
	 /* ---------- 开关=0：USART1 解析裁判系统数据 ---------- */
		if(((((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR) & DMA_SxCR_CT ) == RESET){

					__HAL_DMA_DISABLE(huart->hdmarx);

					((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR |= DMA_SxCR_CT;

					if(Size > 10 )
					{
						Referee_System_Frame_Update(Referee_System_Info_MultiRx_Buf[0]);
						memset(Referee_System_Info_MultiRx_Buf[0],0,REFEREE_RXFRAME_LENGTH);
					}
					__HAL_DMA_SET_COUNTER(huart->hdmarx,REFEREE_RXFRAME_LENGTH*2);
			}
			/* Current memory buffer used is Memory 1 */
			else{

					__HAL_DMA_DISABLE(huart->hdmarx);

					((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR &= ~(DMA_SxCR_CT);

					if(Size > 10)
					{
						Referee_System_Frame_Update(Referee_System_Info_MultiRx_Buf[1]);
						memset(Referee_System_Info_MultiRx_Buf[1],0,REFEREE_RXFRAME_LENGTH);

					}
					__HAL_DMA_SET_COUNTER(huart->hdmarx,REFEREE_RXFRAME_LENGTH*2);

			}


#endif
}

/**
  * @brief  USART10 接收完成回调(预留，当前未使用)
  * @param  huart UART 句柄
  * @param  Size  本次接收到的数据长度
  * @retval 无
  */
static void USER_USART10_RxHandler(UART_HandleTypeDef *huart,uint16_t Size){


}

/**
  * @brief  USART3 接收完成回调(预留，当前未使用)
  * @param  huart UART 句柄
  * @param  Size  本次接收到的数据长度
  * @retval 无
  */
static void USER_USART3_RxHandler(UART_HandleTypeDef *huart,uint16_t Size){


}

/**
  * @brief  USART2 接收完成回调(预留，当前未使用)
  * @param  huart UART 句柄
  * @param  Size  本次接收到的数据长度
  * @retval 无
  */
static void USER_USART2_RxHandler(UART_HandleTypeDef *huart,uint16_t Size){


}

/**
  * @brief  串口接收事件回调(HAL_UARTEx_RxEventCallback)：由 HAL 在空闲中断里调用
  * @param  huart UART 句柄
  * @param  Size  本次接收到的数据长度
  * @retval 无
  * @note   根据句柄把事件分发到对应串口的处理函数(USART5=SBUS, USART1=裁判/图传)，
  *          处理后重新使能空闲中断 + DMA，准备接收下一帧。
  */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart,uint16_t Size)
{
	 if(huart == &huart5){           /* USART5：SBUS 遥控器 */

		  USER_USART5_RxHandler(huart,Size);

	 }

	 if(huart == &huart1){           /* USART1：裁判系统/图传 */

      USER_USART1_RxHandler(huart,Size);

	 }

   huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;   /* 恢复接收方式 */

  /* Enalbe IDLE interrupt */
   __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);          /* 重新使能空闲中断 */

  /* Enable the DMA transfer for the receiver request */
   SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);      /* 重新使能 DMA 接收请求 */

  /* Enable DMA */
   __HAL_DMA_ENABLE(huart->hdmarx);                    /* 重新使能 DMA */
}

/**
  * @brief  向 VoFa+ 上位机发送 3 个 float 数据(JustFloat 协议)
  * @param  SendValue1/SendValue2/SendValue3  要发送的 3 个浮点数(如 PID 的电流/速度/位置)
  * @retval 无
  * @note   通过 USART7 以 DMA 发送，帧尾固定 0x00 0x00 0x80 0x7F 是 VoFa JustFloat
  *          协议的结束标志，VoFa 端选 JustFloat 引擎即可看到曲线。
  */
void USART_Vofa_Justfloat_Transmit(float SendValue1,float SendValue2,float SendValue3){

    __attribute__((section (".AXI_SRAM")))  static uint8_t Rx_Buf[16];   /* 发送缓冲(放 AXI SRAM 段, DMA 可访问) */

		uint8_t *SendValue1_Pointer,*SendValue2_Pointer,*SendValue3_Pointer;

		SendValue1_Pointer = (uint8_t *)&SendValue1;
		SendValue2_Pointer = (uint8_t *)&SendValue2;
		SendValue3_Pointer = (uint8_t *)&SendValue3;

		/* 依次把 3 个 float 的 4 字节(小端)拷贝到发送缓冲 */
		Rx_Buf[0] =  *SendValue1_Pointer;
		Rx_Buf[1] =  *(SendValue1_Pointer + 1);
		Rx_Buf[2] =  *(SendValue1_Pointer + 2);
		Rx_Buf[3] =  *(SendValue1_Pointer + 3);
		Rx_Buf[4] =  *SendValue2_Pointer;
		Rx_Buf[5] =  *(SendValue2_Pointer + 1);
		Rx_Buf[6] =  *(SendValue2_Pointer + 2);
		Rx_Buf[7] =  *(SendValue2_Pointer + 3);
		Rx_Buf[8] =  *SendValue3_Pointer;
		Rx_Buf[9] =  *(SendValue3_Pointer + 1);
		Rx_Buf[10] = *(SendValue3_Pointer + 2);
		Rx_Buf[11] = *(SendValue3_Pointer + 3);
		Rx_Buf[12] =  0x00;              /* 帧尾(JustFloat 结束标志) */
		Rx_Buf[13] =  0x00;
		Rx_Buf[14] =  0x80;
		Rx_Buf[15] =  0x7F;
		/* 注意: Rx_Buf 是 static 且 DMA 异步发送，若调用频率过高(前一帧未发完)会覆盖数据 */
		HAL_UART_Transmit_DMA(&huart7,Rx_Buf,sizeof(Rx_Buf));


}
