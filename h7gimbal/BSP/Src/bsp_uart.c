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

/* ================================================================================ */
/* 文件说明：BSP 层 UART（通用异步串口）驱动 —— 实现文件                             */
/* -------------------------------------------------------------------------------- */
/* 【本工程串口的用途】                                                              */
/*   - USART1 ：裁判系统(Referee_System) 或 图传(Image_Transmission)，二选一，        */
/*               由宏 USART1_RX_Switch 切换（1=图传，0=裁判系统）                    */
/*   - USART5 ：SBUS 遥控器接收机（remote_control）                                  */
/*   - USART7 ：VoFa+ 上位机（发送调试波形数据）                                      */
/*                                                                                  */
/* 【DMA 双缓冲(Double Buffer)接收原理】                                              */
/*   普通 DMA 收满一帧后要等 CPU 处理完才能继续收，期间新数据可能丢失。                 */
/*   双缓冲用两块内存(M0/M1)交替接收：DMA 写 M0 时 CPU 处理 M1，写满自动切到 M1，        */
/*   CPU 再处理 M0……如此乒乓交替，接收不中断、不丢数据。                               */
/*   配合串口「空闲中断(IDLE)」：检测到总线空闲说明一帧结束，立即切换缓冲并处理。       */
/*                                                                                  */
/* 【关键流程】                                                                     */
/*   1. BSP_USART_Init() 配置波特率并调用 USART_RxDMA_MultiBuffer_Init() 开启双缓冲；  */
/*   2. 数据收满/总线空闲 -> 触发 HAL_UARTEx_RxEventCallback()；                      */
/*   3. 回调里根据是哪个串口，分发到对应的 RxHandler；                                */
/*   4. Handler 判断当前用 M0 还是 M1，切到另一块，再把刚收满的这块交给上层解析。      */
/*                                                                                  */
/* 【被谁调用】                                                                     */
/*   - BSP_USART_Init()：MCU_Init()                                                  */
/*   - USART_Vofa_Justfloat_Transmit()：调试/PID 调参任务发送波形                     */
/* ================================================================================ */

#include "bsp_uart.h"
#include "usart.h"
#include "remote_control.h"
#include "Referee_System.h"
#include "Image_Transmission.h"

/* 各串口接收回调的私有实现（在 HAL_UARTEx_RxEventCallback 里分发调用） */
static void USER_USART5_RxHandler(UART_HandleTypeDef *huart,uint16_t Size);

static void USER_USART2_RxHandler(UART_HandleTypeDef *huart,uint16_t Size);

static void USER_USART3_RxHandler(UART_HandleTypeDef *huart,uint16_t Size);

/* 配置串口 DMA 双缓冲接收 */
static void USART_RxDMA_MultiBuffer_Init(UART_HandleTypeDef *, uint32_t *, uint32_t *, uint32_t );

PLL2_ClocksTypeDef PLL2_ClockFreq;  /* 保存 PLL2 各分频输出频率，用于算串口波特率 */

#define USART1_RX_Switch  1 //Referee_System 0  Image_Transmission 1
                            //USART1 用途开关：0=裁判系统  1=图传

/**
  * @brief  初始化各串口：配置 USART1、USART5 的波特率，并开启 DMA 双缓冲接收
  * @param  无
  * @retval 无
  * @note   1. 先读 PLL2 输出频率，算出 USART1 的时钟源频率
  *         2. USART1 由宏 USART1_RX_Switch 决定接图传(921600)还是裁判系统(115200)
  *         3. USART5 接 SBUS 遥控接收机（波特率在 CubeMX 已配好，这里只配 DMA 双缓冲）
  */
void BSP_USART_Init(void){
   
	 HAL_RCCEx_GetPLL2ClockFreq(&PLL2_ClockFreq);// 读取 PLL2 各分频输出频率
	 uint32_t USART1_ClockFreq = PLL2_ClockFreq.PLL2_Q_Frequency;//USART1 时钟源用 PLL2Q
	
	#if USART1_RX_Switch
	
	 USART1->CR1 &= ~USART_CR1_UE; //Disable USART1
   USART1->BRR = (uint32_t)(USART1_ClockFreq/921600);// Set baudrate 921600（BRR=时钟频率/波特率）
	 USART1->CR1 |= USART_CR1_UE;// Enable USART1
	 USART_RxDMA_MultiBuffer_Init(&huart1,(uint32_t *)Image_Trans_MultiRx_Buff[0],(uint32_t *)Image_Trans_MultiRx_Buff[1],IMAGE_TRANS_RX_LENGTH);
	
	#else 
   
	 USART1->CR1 &= ~USART_CR1_UE;
   USART1->BRR = (uint32_t)(USART1_ClockFreq/115200);// Set baudrate 115200（BRR=时钟频率/波特率）
	 USART1->CR1 |= USART_CR1_UE;
	 USART_RxDMA_MultiBuffer_Init(&huart1,(uint32_t *)Referee_System_Info_MultiRx_Buf[0],(uint32_t *)Referee_System_Info_MultiRx_Buf[1],REFEREE_RXFRAME_LENGTH);
	
	#endif
	
	//USART3 Init（注：这里实际初始化的是 USART5，接 SBUS 遥控接收机）
	 USART_RxDMA_MultiBuffer_Init(&huart5,(uint32_t *)SBUS_MultiRx_Buf[0],(uint32_t *)SBUS_MultiRx_Buf[1],SBUS_RX_BUF_NUM);

}

/**
  * @brief  配置串口的 DMA 双缓冲接收（带空闲中断）
  * @param  huart            串口句柄
  * @param  DstAddress       第一个接收缓冲地址（Memory 0）
  * @param  SecondMemAddress 第二个接收缓冲地址（Memory 1）
  * @param  DataLength       每块缓冲要接收的数据长度
  * @retval 无
  * @note   原理：使能 DMA 的「双缓冲模式(DBM)」，DMA 先写 M0，写满自动切到 M1，
  *         同时触发中断让 CPU 处理刚写满的 M0，如此乒乓交替，接收不中断。
  */
static void USART_RxDMA_MultiBuffer_Init(UART_HandleTypeDef *huart, uint32_t *DstAddress, uint32_t *SecondMemAddress, uint32_t DataLength){

 huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;  /* 接收模式：检测到空闲(IDLE)才认为一帧结束 */

 huart->RxXferSize    = DataLength * 2;             /* 记录本次接收总字节数（两块缓冲的容量） */

 SET_BIT(huart->Instance->CR3,USART_CR3_DMAR);      /* 打开串口接收 DMA 请求(DMAR) */

 __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);         /* 使能串口空闲中断 */

  do{
      __HAL_DMA_DISABLE(huart->hdmarx);             /* 先关 DMA */
  }while(((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR & DMA_SxCR_EN); /* 等它真正停掉 */

  /* Configure the source memory Buffer address  */
  /* 源地址 = 串口数据寄存器 RDR（收到的字节从这里被 DMA 搬走） */
  ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->PAR = (uint32_t)&huart->Instance->RDR;

  /* Configure the destination memory Buffer address */
  /* 目的地址 0 = 第一块缓冲 Memory 0 */
  ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->M0AR = (uint32_t)DstAddress;

  /* Configure DMA Stream destination address */
  /* 目的地址 1 = 第二块缓冲 Memory 1 */
  ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->M1AR = (uint32_t)SecondMemAddress;

  /* Configure the length of data to be transferred from source to destination */
  /* 每块缓冲接收的数据长度 */
  ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->NDTR = DataLength;

  /* Enable double memory buffer */
  /* 使能 DMA 双缓冲模式 */
  SET_BIT(((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR, DMA_SxCR_DBM);

  /* Enable DMA */
  __HAL_DMA_ENABLE(huart->hdmarx);                  /* 启动 DMA 开始接收 */

}



/**
  * @brief  USART5 接收回调（SBUS 遥控数据）
  * @param  huart UART handle
  * @param  Size  本次实际收到的字节数
  * @retval None
  * @note   判断当前 DMA 写满的是 Memory 0 还是 Memory 1，切到另一块继续收，
  *         再把刚写满的这块交给 SBUS_TO_RC() 解析成遥控数据 remote_ctrl
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
  * @brief  USART1 接收回调（裁判系统或图传，由宏 USART1_RX_Switch 决定）
  * @param  huart UART handle
  * @param  Size  本次实际收到的字节数
  * @retval None
  * @note   同样是「判断当前缓冲 -> 切到另一块 -> 处理刚收满的这块」的乒乓流程；
  *         #if 分支处理图传，数据交给 Image_Transmission_Info_Update()；
  *         #else 分支处理裁判系统，数据交给 Referee_System_Frame_Update()。
  */



static void USER_USART1_RxHandler(UART_HandleTypeDef *huart,uint16_t Size){

#if USART1_RX_Switch
	
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
  * @brief  USART10 接收回调（当前为空实现，预留）
  * @param  huart UART handle
  * @param  Size  本次实际收到的字节数
  * @retval None
  */
static void USER_USART10_RxHandler(UART_HandleTypeDef *huart,uint16_t Size){

	
}

/**
  * @brief  USART3 接收回调（当前为空实现，预留）
  * @param  huart UART handle
  * @param  Size  本次实际收到的字节数
  * @retval None
  */
static void USER_USART3_RxHandler(UART_HandleTypeDef *huart,uint16_t Size){

	
}

/**
  * @brief  USART2 接收回调（当前为空实现，预留）
  * @param  huart UART handle
  * @param  Size  本次实际收到的字节数
  * @retval None
  */
static void USER_USART2_RxHandler(UART_HandleTypeDef *huart,uint16_t Size){

	
}

/**
  * @brief  串口接收事件回调（HAL 库的弱函数，数据收满或总线空闲时自动调用）
  * @param  huart UART handle
  * @param  Size  本次实际收到的字节数
  * @retval None
  * @note   根据是哪个串口分发到对应的处理函数；处理完后重新使能空闲中断和 DMA，
  *         准备接收下一帧数据。
  */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart,uint16_t Size)
{
	 if(huart == &huart5){   /* 触发的是 USART5，交给 SBUS 遥控处理 */
	
		  USER_USART5_RxHandler(huart,Size);
			
	 } 
	 
	 if(huart == &huart1){   /* 触发的是 USART1，交给裁判系统/图传处理 */
	 
      USER_USART1_RxHandler(huart,Size);
		
	 }
	
   huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;
	
  /* Enalbe IDLE interrupt（重新使能空闲中断） */
   __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);

  /* Enable the DMA transfer for the receiver request（重新打开串口 DMA 接收请求） */
   SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);

  /* Enable DMA（重新启动 DMA 接收） */
   __HAL_DMA_ENABLE(huart->hdmarx);
}

/* ==================== 【VoFa 专用】3 通道发送函数 ==================== */
/**
 * @brief  向 VoFa+ 上位机发送 3 个 float 数据（JustFloat 协议）
 * @param  SendValue1/SendValue2/SendValue3  3 个浮点数（通常用于发送调试/PID 曲线）
 * @retval 无
 * @note   VoFa 的 JustFloat 协议：3 个 float（每 4 字节，共 12 字节）+ 4 字节帧尾。
 *         帧尾固定为 0x00 0x00 0x80 0x7F（浮点正无穷的字节序），上位机靠它识别帧边界。
 *         数据通过 USART7 以 DMA 方式发送。
 */
void USART_Vofa_Justfloat_Transmit(float SendValue1,float SendValue2,float SendValue3){

    /* 发送缓冲区：放在 AXI SRAM，方便 DMA 直接访问 */
    __attribute__((section (".AXI_SRAM")))  static uint8_t Rx_Buf[16];

		uint8_t *SendValue1_Pointer,*SendValue2_Pointer,*SendValue3_Pointer;

		/* 把 float 的地址强转成字节指针，方便按字节拆开 */
		SendValue1_Pointer = (uint8_t *)&SendValue1;
		SendValue2_Pointer = (uint8_t *)&SendValue2;
		SendValue3_Pointer = (uint8_t *)&SendValue3;


		/* 小端序：低字节在前。依次填入 float 1 的 4 个字节 */
		Rx_Buf[0] =  *SendValue1_Pointer;
		Rx_Buf[1] =  *(SendValue1_Pointer + 1);
		Rx_Buf[2] =  *(SendValue1_Pointer + 2);
		Rx_Buf[3] =  *(SendValue1_Pointer + 3);
		/* float 2 的 4 个字节 */
		Rx_Buf[4] =  *SendValue2_Pointer;
		Rx_Buf[5] =  *(SendValue2_Pointer + 1);
		Rx_Buf[6] =  *(SendValue2_Pointer + 2);
		Rx_Buf[7] =  *(SendValue2_Pointer + 3);
		/* float 3 的 4 个字节 */
		Rx_Buf[8] =  *SendValue3_Pointer;
		Rx_Buf[9] =  *(SendValue3_Pointer + 1);
		Rx_Buf[10] = *(SendValue3_Pointer + 2);
		Rx_Buf[11] = *(SendValue3_Pointer + 3);
		/* 4 字节帧尾：0x00 0x00 0x80 0x7F，VoFa 靠它识别帧边界 */
		Rx_Buf[12] =  0x00;
		Rx_Buf[13] =  0x00;
		Rx_Buf[14] =  0x80;
		Rx_Buf[15] =  0x7F;
		HAL_UART_Transmit_DMA(&huart7,Rx_Buf,sizeof(Rx_Buf)); /* 通过 USART7 DMA 发送整帧 */


}

/* ==================== 【VoFa 专用】4 通道发送函数（调 yaw 内外环用） ==================== */
/**
 * @brief  向 VoFa+ 上位机发送 4 个 float 数据（JustFloat 协议）
 * @param  SendValue1/SendValue2/SendValue3/SendValue4  4 个浮点数（用于 yaw 内外环等 4 通道波形）
 * @retval 无
 * @note   协议同 3 通道版：4 个 float(每 4 字节，共 16 字节) + 4 字节帧尾(0x00 0x00 0x80 0x7F)。
 *         数据通过 USART7 以 DMA 方式发送，VoFa 端通道数填 4。
 */
void USART_Vofa_Justfloat_Transmit4(float SendValue1,float SendValue2,float SendValue3,float SendValue4){

    /* 发送缓冲区：放在 AXI SRAM，方便 DMA 直接访问。4 float + 4 尾字节 = 20 字节 */
    __attribute__((section (".AXI_SRAM")))  static uint8_t Rx_Buf[20];

		uint8_t *SendValue1_Pointer,*SendValue2_Pointer,*SendValue3_Pointer,*SendValue4_Pointer;

		SendValue1_Pointer = (uint8_t *)&SendValue1;
		SendValue2_Pointer = (uint8_t *)&SendValue2;
		SendValue3_Pointer = (uint8_t *)&SendValue3;
		SendValue4_Pointer = (uint8_t *)&SendValue4;

		/* 小端序：低字节在前。依次填入 float 1 的 4 个字节 */
		Rx_Buf[0] =  *SendValue1_Pointer;
		Rx_Buf[1] =  *(SendValue1_Pointer + 1);
		Rx_Buf[2] =  *(SendValue1_Pointer + 2);
		Rx_Buf[3] =  *(SendValue1_Pointer + 3);
		/* float 2 的 4 个字节 */
		Rx_Buf[4] =  *SendValue2_Pointer;
		Rx_Buf[5] =  *(SendValue2_Pointer + 1);
		Rx_Buf[6] =  *(SendValue2_Pointer + 2);
		Rx_Buf[7] =  *(SendValue2_Pointer + 3);
		/* float 3 的 4 个字节 */
		Rx_Buf[8] =  *SendValue3_Pointer;
		Rx_Buf[9] =  *(SendValue3_Pointer + 1);
		Rx_Buf[10] = *(SendValue3_Pointer + 2);
		Rx_Buf[11] = *(SendValue3_Pointer + 3);
		/* float 4 的 4 个字节 */
		Rx_Buf[12] = *SendValue4_Pointer;
		Rx_Buf[13] = *(SendValue4_Pointer + 1);
		Rx_Buf[14] = *(SendValue4_Pointer + 2);
		Rx_Buf[15] = *(SendValue4_Pointer + 3);
		/* 4 字节帧尾：0x00 0x00 0x80 0x7F，VoFa 靠它识别帧边界 */
		Rx_Buf[16] = 0x00;
		Rx_Buf[17] = 0x00;
		Rx_Buf[18] = 0x80;
		Rx_Buf[19] = 0x7F;
		HAL_UART_Transmit_DMA(&huart7,Rx_Buf,sizeof(Rx_Buf)); /* 通过 USART7 DMA 发送整帧 */
}

