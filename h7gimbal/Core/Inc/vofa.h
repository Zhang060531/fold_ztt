#ifndef _VOFA_H
#define _VOFA_H

/* ⚠️ 遗留未用：本模块走 USART1（被裁判/图传占用），函数无人调用。
 *   调 VoFa 请用 bsp_uart.c 的 USART_Vofa_Justfloat_Transmit4（走 USART7）。 */

//....在此替换你的串口函数路径........
#include "usart.h"
//...................................
#include <stdio.h>
#include "stdint.h"
#include <string.h>
#include <stdarg.h>

void Vofa_FireWater(const char *format, ...);
void Vofa_JustFloat(float *_data, uint8_t _num);

#endif
