/**
 ******************************************************************************
 * @file    ws2812.c
 * @brief   WS2812 灯珠驱动实现（用 SPI 模拟单总线时序）
 *
 * 【驱动原理（新手必读）】
 * WS2812 用一根数据线按位发送颜色数据，通过高电平持续时间区分 0/1：
 *   - 0 码：高电平约 0.4us，低电平约 0.85us
 *   - 1 码：高电平约 0.8us，低电平约 0.45us
 * 每颗灯珠 24bit（G、R、B 各 8bit，先发高位），发完后数据线保持低电平超过
 * 约 50us 即「复位」，灯珠把收到的颜色锁存并点亮。
 *
 * 【用 SPI 模拟时序】
 * 本工程把 SPI 外设当作高速移位器：设好 SPI 波特率后，SPI 每发送 1bit 就对应
 * 一个固定宽度的高/低电平，于是用「1 个 SPI 字节」表示「WS2812 的 1bit」。
 *   - 0 码 = 0xC0 = 0b11000000（高电平占 2 个 SPI bit，其余为低）
 *   - 1 码 = 0xF0 = 0b11110000（高电平占 4 个 SPI bit，其余为低）
 * 24bit 颜色 => 24 个 SPI 字节。
 *
 * 【被谁调用】
 *   - WS2812_Ctrl(r,g,b)：设置灯珠颜色，参数为红/绿/蓝亮度(0~255)。
 ******************************************************************************
 */
#include "ws2812.h"

/* WS2812 单 bit 对应的 SPI 字节编码（见上方原理说明） */
#define WS2812_LowLevel    0xC0     // 0码：高电平 2 个 SPI bit
#define WS2812_HighLevel   0xF0     // 1码：高电平 4 个 SPI bit

/**
 * @brief  设置并点亮 WS2812 灯珠
 * @param  r：红色亮度 0~255
 * @param  g：绿色亮度 0~255
 * @param  b：蓝色亮度 0~255
 * @retval 无
 * @note   把 24bit 颜色按 WS2812 顺序（G、R、B，每字节高位在前）逐位转成 SPI 字节：
 *         txbuf[7-i]  对应 g 的第 i 位，txbuf[15-i] 对应 r，txbuf[23-i] 对应 b。
 *         先发一个空字节制造起始低电平，发完 24 字节后再发 100 个 0x00 字节
 *         保持数据线低电平 >50us 完成复位锁存。
 */
void WS2812_Ctrl(uint8_t r, uint8_t g, uint8_t b)
{
    uint8_t txbuf[24];
    uint8_t res = 0;
    /* 把 24bit 颜色逐位展开成 24 个 SPI 字节：
     * 对每个颜色字节，从最高位(bit7)到最低位(bit0)依次判断 0/1，
     * 0 -> WS2812_LowLevel，1 -> WS2812_HighLevel，结果再右移 1 位做时序微调
     * （实际发出的字节是 0x60 / 0x78）。txbuf 下标 7-i、15-i、23-i 表示先发高位。 */
    for (int i = 0; i < 8; i++)
    {
        txbuf[7-i]  = (((g>>i)&0x01) ? WS2812_HighLevel : WS2812_LowLevel)>>1;
        txbuf[15-i] = (((r>>i)&0x01) ? WS2812_HighLevel : WS2812_LowLevel)>>1;
        txbuf[23-i] = (((b>>i)&0x01) ? WS2812_HighLevel : WS2812_LowLevel)>>1;
    }
    /* 先发一个空字节(长度 0)把数据线拉低，制造起始低电平间隔 */
    HAL_SPI_Transmit(&WS2812_SPI_UNIT, &res, 0, 0xFFFF);
    while (WS2812_SPI_UNIT.State != HAL_SPI_STATE_READY);
    /* 发送 24 字节颜色数据 */
    HAL_SPI_Transmit(&WS2812_SPI_UNIT, txbuf, 24, 0xFFFF);
    /* 再发 100 个 0x00 字节，保持数据线低电平超过 50us，触发 WS2812 复位锁存 */
    for (int i = 0; i < 100; i++)
    {
        HAL_SPI_Transmit(&WS2812_SPI_UNIT, &res, 1, 0xFFFF);
    }
}