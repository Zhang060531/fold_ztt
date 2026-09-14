/* ===== 文件头 =====
 * 文件名  : ui_interface.c
 * 作用    : RMUI 图形库底层实现：CRC 校验、帧打包(帧头/SOF/length/CRC16)、
 *           以及"扫描脏图形并按 7/5/2/1 打包发送"的核心逻辑。
 * 结构    :
 *   1. 全局帧缓冲(_ui_1/2/5/7_frame、_ui_string_frame、ui_delete_frame)放在 .AXI_SRAM
 *   2. calc_crc8 / calc_crc16: 帧头 CRC8 与整帧 CRC16 校验
 *   3. ui_proc_*_frame: 填充 SOF/length/seq/crc/cmd_id/sub_id/send_id/recv_id
 *   4. ui_delete_layer: 删除图层
 *   5. ui_scan_and_send: 扫描脏图形 -> 打包 -> 串口发送
 * 调用关系:
 *   ui_g.c(ui_init_g/ui_update_g) -> ui_scan_and_send -> ui_proc_*_frame -> SEND_MESSAGE
 *   UI_Task.c -> ui_init_g / ui_update_g
 * ===== 调车关注 =====
 *   - 帧头结构见 ui_types.h 的 ui_frame_header_t 注释。
 *   - length = 6 + 15*num(图形帧) / 51(字符帧)，含义是"cmd_id 之后、CRC16 之前的字节数"
 *     (其中 6 = sub_id 2 + send_id 2 + recv_id 2)。
 *   - crc16 覆盖"13 字节帧头 + 数据段"，不含 crc16 自身。
 *   - 发送走串口1 DMA + osDelay(33) 限流(见 ui_interface.h 的 SEND_MESSAGE 宏)。
 */
//
// Created by bismarckkk on 2025/3/22.
// Dynamic Edition
//

#include <string.h>
#include <stdio.h>
#include "ui_interface.h"
#include "CRC.h"
#include "Referee_System.h"
extern Referee_System_Info_TypeDef Referee_System_Info;

/* 包序号: 每发一帧自增一次(可回绕)，接收端据此判断丢包/顺序 */
uint8_t seq = 0;
/* 自身 ID。本工程未实际用于打包(帧内 send_id 取裁判系统机器人 ID) */
int ui_self_id = 1;

/* 各帧的静态缓冲，放在 .AXI_SRAM 段(避开普通 RAM 且便于 DMA 直接访问) */
__attribute__((section (".AXI_SRAM"))) ui_string_frame_t _ui_string_frame; /* 字符帧缓冲 */
__attribute__((section (".AXI_SRAM"))) ui_1_frame_t _ui_1_frame;           /* 1 图形帧缓冲 */
__attribute__((section (".AXI_SRAM"))) ui_2_frame_t _ui_2_frame;           /* 2 图形帧缓冲 */
__attribute__((section (".AXI_SRAM"))) ui_5_frame_t _ui_5_frame;           /* 5 图形帧缓冲 */
__attribute__((section (".AXI_SRAM"))) ui_7_frame_t _ui_7_frame;           /* 7 图形帧缓冲 */

/**
 * @brief 以十六进制逐字节打印一帧内容，用于串口调试观察打包结果
 * @param message 待打印数据首地址
 * @param length  数据字节数
 * @note 依赖 printf 已重定向到调试串口
 */
void print_message(const uint8_t *message, const int length) {
    for (int i = 0; i < length; i++) {
        printf("%02x ", message[i]);
    }
    printf("\n\n");
}
/* CRC8 查找表由 CRC.c/CRC.h 提供，本文件只引用、不重复定义 */
extern const unsigned char CRC8_TAB[256]; 
/* CRC16 查找表由 CRC.c/CRC.h 提供 */
extern const uint16_t wCRC_Table[256]; 
//const unsigned char CRC8_TAB[256] = {
//    0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
//    0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e, 0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc, 0x23,
//    0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62, 0xbe, 0xe0,
//    0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff, 0x46, 0x18, 0xfa,
//    0xa4, 0x27, 0x79, 0x9b, 0xc5, 0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07, 0xdb, 0x85, 0x67,
//    0x39, 0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a, 0x65, 0x3b, 0xd9, 0x87,
//    0x04, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24, 0xf8, 0xa6, 0x44, 0x1a, 0x99,
//    0xc7, 0x25, 0x7b, 0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
//    0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd, 0x11,
//    0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50, 0xaf, 0xf1,
//    0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c, 0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee, 0x32, 0x6c, 0x8e,
//    0xd0, 0x53, 0x0d, 0xef, 0xb1, 0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73, 0xca, 0x94, 0x76, 0x28,
//    0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b, 0x57, 0x09, 0xeb, 0xb5,
//    0x36, 0x68, 0x8a, 0xd4, 0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16, 0xe9, 0xb7, 0x55, 0x0b, 0x88,
//    0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
//    0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35,
//};

/**
 * @brief 计算 CRC8 校验(初值 0xFF)，用于帧头前 4 字节(SOF+length+seq)
 * @param pchMessage 数据首地址
 * @param dwLength   数据字节数
 * @retval 8 位 CRC 校验值
 * @note 采用查表法，查找表 CRC8_TAB 由 CRC.c 提供
 */
unsigned char calc_crc8(unsigned char *pchMessage, unsigned int dwLength) {
    unsigned char ucCRC8 = 0xff;        /* CRC8 初值 */
    unsigned char ucIndex;
    while (dwLength--) {
        ucIndex = ucCRC8 ^ (*pchMessage++);
        ucCRC8 = CRC8_TAB[ucIndex];
    }
    return (ucCRC8);
}

//const uint16_t wCRC_Table[256] = {
//    0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
//    0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
//    0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
//    0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
//    0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
//    0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
//    0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
//    0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
//    0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
//    0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
//    0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
//    0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
//    0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
//    0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
//    0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
//    0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
//    0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
//    0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
//    0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
//    0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
//    0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
//    0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
//    0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
//    0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
//    0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
//    0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
//    0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
//    0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
//    0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
//    0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
//    0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
//    0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
//};

/**
 * @brief 计算 CRC16 校验(初值 0xFFFF)，用于整帧(帧头 13 字节 + 数据段，不含 crc16 自身)
 * @param pchMessage 数据首地址
 * @param dwLength   数据字节数
 * @retval 16 位 CRC 校验值；入参为空指针时返回 0xFFFF
 * @note 采用查表法，查找表 wCRC_Table 由 CRC.c 提供
 */
uint16_t calc_crc16(uint8_t *pchMessage, uint32_t dwLength)
{
    uint16_t wCRC = 0xffff;             /* CRC16 初值 */
    uint8_t chData;
    if (pchMessage == NULL)             /* 空指针保护 */
    {
        return 0xFFFF;
    }
    while(dwLength--)
    {
        chData = *pchMessage++;
        (wCRC) = ((uint16_t)(wCRC) >> 8) ^ wCRC_Table[((uint16_t)(wCRC) ^ (uint16_t)(chData)) & 0x00ff];
    }
    return wCRC;
}

/* ===== 图形帧打包宏 =====
 * 生成 ui_proc_1/2/5/7_frame 四个函数，填充帧头各字段:
 *   SOF    = 0xA5(帧头)
 *   length = 6 + 15*num(数据段长度 = sub_id 2 + send_id 2 + recv_id 2 + 图形数据 15*num)
 *   seq    = 自增序号
 *   crc8   = 对前 4 字节(SOF+length+seq)做 CRC8
 *   cmd_id = 0x0301(客户端绘制命令)
 *   sub_id = 0x0101~0x0104(分别对应 1/2/5/7 条图形)
 *   send_id= 机器人 ID, recv_id = 机器人 ID + 256(该机器人对应的客户端 ID)
 *   crc16  = 对 13 字节帧头 + 15*num 图形数据做 CRC16
 */
#define DEFINE_FRAME_PROC(num, id)                          \
void ui_proc_ ## num##_frame(ui_ ## num##_frame_t *msg) {   \
    msg->header.SOF = 0xA5;                                 \
    msg->header.length = 6 + 15 * num;                      \
    msg->header.seq = seq++;                                \
    msg->header.crc8 = calc_crc8((uint8_t*)msg, 4);        \
    msg->header.cmd_id = 0x0301;                            \
    msg->header.sub_id = id;                                \
    msg->header.send_id = Referee_System_Info.robot_status.robot_id;                       \
    msg->header.recv_id = Referee_System_Info.robot_status.robot_id + 256;                 \
    msg->crc16 = calc_crc16((uint8_t*)msg, 13 + 15 * num); \
}

DEFINE_FRAME_PROC(1, 0x0101)
DEFINE_FRAME_PROC(2, 0x0102)
DEFINE_FRAME_PROC(5, 0x0103)
DEFINE_FRAME_PROC(7, 0x0104)

/**
 * @brief 填充"字符帧"的帧头/长度/CRC 等字段
 * @param msg 字符帧指针(全局 _ui_string_frame)
 * @note length 固定 51 = 6 + 45(字符图形结构 45 字节)；
 *       crc16 覆盖 58 字节 = 13 帧头 + 45 字符图形。
 */
void ui_proc_string_frame(ui_string_frame_t *msg) {
    msg->header.SOF = 0xA5;                                       /* 帧头 */
    msg->header.length = 51;                                      /* 数据段长度 */
    msg->header.seq = seq++;                                      /* 包序号 */
    msg->header.crc8 = calc_crc8((uint8_t *) msg, 4);             /* 帧头 CRC8 */
    msg->header.cmd_id = 0x0301;                                  /* 客户端绘制命令 */
    msg->header.sub_id = 0x0110;                                  /* 0x0110 = 字符绘制 */
    msg->header.send_id = Referee_System_Info.robot_status.robot_id;              /* 机器人 ID */
    msg->header.recv_id = Referee_System_Info.robot_status.robot_id + 256;        /* 客户端 ID */
    msg->option.str_length = strlen(msg->option.string);          /* 按实际字符串长度填 str_length */
    msg->crc16 = calc_crc16((uint8_t *) msg, 58);                 /* 整帧 CRC16 */
}

/**
 * @brief 填充"删除图层帧"的帧头/长度/CRC 等字段
 * @param msg 删除帧指针
 * @note length = 8(数据段)，sub_id = 0x0100(删除图层)，crc16 覆盖 15 字节。
 */
void ui_proc_delete_frame(ui_delete_frame_t *msg) {
    msg->header.SOF = 0xA5;                                       /* 帧头 */
    msg->header.length = 8;                                       /* 数据段长度 */
    msg->header.seq = seq++;                                      /* 包序号 */
    msg->header.crc8 = calc_crc8((uint8_t *) msg, 4);             /* 帧头 CRC8 */
    msg->header.cmd_id = 0x0301;                                  /* 客户端绘制命令 */
    msg->header.sub_id = 0x0100;                                  /* 0x0100 = 删除图层 */
    msg->header.send_id = Referee_System_Info.robot_status.robot_id;              /* 机器人 ID */
    msg->header.recv_id = Referee_System_Info.robot_status.robot_id + 256;        /* 客户端 ID */
    msg->crc16 = calc_crc16((uint8_t *) msg, 15);                 /* 整帧 CRC16 */
}

/* 删除图层帧的缓冲，放 .AXI_SRAM 段 */
__attribute__((section (".AXI_SRAM"))) ui_delete_frame_t ui_delete_frame;

/**
 * @brief 删除指定/全部图层(把该图层上的所有图形清掉)
 * @param delete_type 删除方式(见裁判系统协议: 0=删除所有图层, 1=删除指定图层)
 * @param layer       要删除的图层号(delete_type=1 时有效)
 * @note 删除后客户端对应图层会被清空，需重新绘制
 */
void ui_delete_layer(const uint8_t delete_type, const uint8_t layer) {
    ui_delete_frame.delete_type = delete_type;
    ui_delete_frame.layer = layer;
    ui_proc_delete_frame(&ui_delete_frame);                            /* 填充帧头/CRC */
    SEND_MESSAGE((uint8_t *) &ui_delete_frame, sizeof(ui_delete_frame));/* 串口发送 */
}

/**
 * @brief RMUI 发送核心: 扫描所有脏图形/字符串，按 7/5/2/1 打包成帧并通过串口发送
 * @param ui_now_figures 当前图形数组
 * @param ui_dirty_figure 图形脏标记(每发送一次减 1，减到 0 停止发送)
 * @param ui_now_strings  当前字符串数组
 * @param ui_dirty_string 字符串脏标记
 * @param total_figures   图形总数
 * @param total_strings   字符串总数
 * @note 打包策略: 剩余待发图形 >5 用 7 图形帧, >2 用 5 图形帧, >1 用 2 图形帧,
 *       否则用 1 图形帧；不足一个包时把空位 operate_type 置 0(空操作)。
 */
void ui_scan_and_send(const ui_interface_figure_t *ui_now_figures, uint8_t *ui_dirty_figure,
                      const ui_interface_string_t *ui_now_strings, uint8_t *ui_dirty_string, 
                      const int total_figures, const int total_strings) {
    /* 先处理图形: 统计本周期内有多少条脏图形需要发送 */
    if (total_figures > 0) {
        int total_figure = 0;           /* 脏图形总数 */
        for (int i = 0; i < total_figures; i++) {
            if (ui_dirty_figure[i] > 0) {
                total_figure++;
            }
        }
        for (int i = 0, now_cap = 0, pack_size = 0; i < total_figures; i++) {
            if (ui_dirty_figure[i] > 0) {
                /* now_idx: 当前包内的槽位(0~6)；now_cap: 已处理的脏图形累计数 */
                const int now_idx = now_cap % 7;
                if (now_idx == 0) {
                    /* 每个新包开始(now_idx==0)时，按剩余图形数决定本包用几图形帧 */
                    const int remain_size = total_figure - now_cap;
                    if (remain_size > 5) {
                        pack_size = 7;
                    } else if (remain_size > 2) {
                        pack_size = 5;
                    } else if (remain_size > 1) {
                        pack_size = 2;
                    } else {
                        pack_size = 1;
                    }
                }
                if (pack_size == 7) {
                    _ui_7_frame.data[now_idx] = ui_now_figures[i];
                } else if (pack_size == 5) {
                    _ui_5_frame.data[now_idx] = ui_now_figures[i];
                } else if (pack_size == 2) {
                    _ui_2_frame.data[now_idx] = ui_now_figures[i];
                } else {
                    _ui_1_frame.data[now_idx] = ui_now_figures[i];
                }
                /* 本包已满(now_idx+1==pack_size)或已是最后一条(now_cap+1==total_figure)时发送 */
                if (now_idx + 1 == pack_size || now_cap + 1 == total_figure) {
                    /* 注意: 疑似bug —— 下面循环上界为 pack_size+1，
                     * 当最后一格已填充(now_idx==pack_size-1)时 j 会取到 pack_size，
                     * 写入 data[pack_size] 越界(数组有效下标 0..pack_size-1)，
                     * 会覆盖帧尾 crc16 字段乃至相邻内存(帧缓冲位于 .AXI_SRAM)。
                     * 疑似应改为 j < pack_size。 */
                    for (int j = now_idx + 1; j < pack_size + 1; j++) {
                        if (pack_size == 7) {
                            _ui_7_frame.data[j].operate_type = 0;
                        } else if (pack_size == 5) {
                            _ui_5_frame.data[j].operate_type = 0;
                        } else if (pack_size == 2) {
                            _ui_2_frame.data[j].operate_type = 0;
                        } else {
                            _ui_1_frame.data[j].operate_type = 0;
                        }
                    }
                    if (pack_size == 7) {
                        ui_proc_7_frame(&_ui_7_frame);
                        SEND_MESSAGE((uint8_t *) &_ui_7_frame, sizeof(_ui_7_frame));
                    } else if (pack_size == 5) {
                        ui_proc_5_frame(&_ui_5_frame);
                        SEND_MESSAGE((uint8_t *) &_ui_5_frame, sizeof(_ui_5_frame));
                    } else if (pack_size == 2) {
                        ui_proc_2_frame(&_ui_2_frame);
                        SEND_MESSAGE((uint8_t *) &_ui_2_frame, sizeof(_ui_2_frame));
                    } else {
                        ui_proc_1_frame(&_ui_1_frame);
                        SEND_MESSAGE((uint8_t *) &_ui_1_frame, sizeof(_ui_1_frame));
                    }
                }
                now_cap++;               /* 已处理脏图形计数 +1 */
                ui_dirty_figure[i]--;    /* 该图形剩余重发次数 -1，减到 0 停止发送 */
            }
        }
    }
    /* 再处理字符串: 逐条发送脏字符串(字符帧一条一发) */
    if (total_strings > 0) {
        for (int i = 0; i < total_strings; i++) {
            if (ui_dirty_string[i] > 0) {                                    /* 该字符串需要发送 */
                _ui_string_frame.option = ui_now_strings[i];                 /* 拷贝当前内容到发送缓冲 */
                ui_proc_string_frame(&_ui_string_frame);                     /* 填充帧头/CRC */
                SEND_MESSAGE((uint8_t *) &_ui_string_frame, sizeof(_ui_string_frame)); /* 串口发送 */
                ui_dirty_string[i]--;                                        /* 剩余重发次数 -1 */
            }
        }
    }
}
