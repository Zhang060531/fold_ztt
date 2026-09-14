/* ⚠️ 遗留未用：本模块走 USART1，但本工程 USART1 被裁判/图传占用，函数无人调用。
 *   调 VoFa 请用 bsp_uart.c 的 USART_Vofa_Justfloat_Transmit4（走 USART7）。 */
/**
  ******************************************************************************
  * @file       vofa.c
  * @brief      VoFa+ 上位机数据发送（支持 JustFloat / FireWater 两种协议）
  * @note
  *   1. 用 USART1（PA9 = TX）把数据发给电脑上的 VoFa+。
  *   2. 发送函数用「阻塞发送」HAL_UART_Transmit，无需配 DMA，最简单可靠。
  *      代价：发送期间 CPU 会等待（20 字节约 1.7ms@115200），调参够用。
  *   3. JustFloat：二进制浮点 + 帧尾，效率高，适合高频多通道（推荐）。
  *      FireWater：文本 "a,b,c\r\n"，可读性好，可用串口助手先验证通不通。
  ******************************************************************************
  */

#include "vofa.h"

/**
  * @brief  FireWater 文本协议发送：按 printf 格式写，最后必须带 \r\n
  * @note   例：Vofa_FireWater("%f,%f\r\n", f1, f2);
  */
void Vofa_FireWater(const char *format, ...)
{
    uint8_t txBuffer[100];
    int n;
    va_list args;

    va_start(args, format);
    n = vsnprintf((char *)txBuffer, sizeof(txBuffer), format, args);
    va_end(args);

    if (n > 0)
    {
        HAL_UART_Transmit(&huart1, txBuffer, (uint16_t)n, 1000);
    }
}

/**
  * @brief  JustFloat 二进制协议发送：发送 _num 个 float + 4 字节帧尾
  * @param  _data: float 数组首地址
  * @param  _num : 数据个数（VoFa+ 里「通道数」就填这个值，不含帧尾）
  * @note   帧尾固定为 00 00 80 7F（= +Inf），VoFa+ 靠它来分隔每一帧。
  *         例：float f[3] = {1.0f, 2.0f, 3.0f};  Vofa_JustFloat(f, 3);
  */
void Vofa_JustFloat(float *_data, uint8_t _num)
{
    uint8_t txBuffer[100];
    const uint8_t frame_tail[4] = {0x00, 0x00, 0x80, 0x7F};  /* +Inf 帧尾 */

    /* 数据（小端 float）+ 帧尾，拼成一整帧 */
    memcpy(txBuffer, (uint8_t *)_data, sizeof(float) * _num);
    memcpy(&txBuffer[_num * 4], frame_tail, 4);

    HAL_UART_Transmit(&huart1, txBuffer, (_num + 1) * 4, 1000);
}
