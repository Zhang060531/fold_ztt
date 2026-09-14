/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : MiniPC.c
  * @brief          : MiniPC interfaces functions 
  * @author         : GarssFan Wang
  * @date           : 2025/01/22
  * @version        : v1.0
  ******************************************************************************
  * @attention      : None
  ******************************************************************************
  */
/* USER CODE END Header */

/**
 ******************************************************************************
 * @file    MiniPC.c
 * @brief   上位机（妙算 MiniPC）USB CDC 通信实现
 *
 * 【本文件做什么】
 * 封装 USB 虚拟串口(CDC)的收发：
 *   - MiniPC_Transmit_Info：把数据通过 CDC_Transmit_HS 发送给上位机；
 *   - MiniPC_Recvive_Info：接收回调（由 usbd_cdc_if.c 的 CDC_Receive_HS 调用），
 *     目前是空实现，留给使用方补充协议解析。
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "MiniPC.h"
#include "usbd_cdc_if.h"


/**
 * @brief  通过 USB CDC 向上位机发送数据
 * @param  Buff：要发送的数据缓冲区指针
 * @retval 无
 * @note   注意：sizeof(*Buff) 实际等于 1（因为 Buff 是 uint8_t*，解引用后是 1 字节），
 *         所以这里实际只发了 1 字节。若要发送整块数据，应改成传入实际长度。
 */
void MiniPC_Transmit_Info(uint8_t *Buff){

    CDC_Transmit_HS(Buff,sizeof(*Buff));

}

//usbd_cdc_if.c -> CDC_Receive_HS
/**
 * @brief  接收上位机发来的数据（由 CDC_Receive_HS 回调调用）
 * @param  Buff：收到的数据缓冲区
 * @param  Len ：收到的数据长度
 * @retval 无
 * @note   目前为空实现，需要在这里补充对上位机协议数据的解析。
 */
void MiniPC_Recvive_Info(uint8_t* Buff, const uint32_t *Len){



}