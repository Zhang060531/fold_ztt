/* ===== 文件头 =====
 * 文件名  : ui_types.h
 * 作用    : RMUI(RoboMaster 裁判系统客户端绘制) 的核心数据类型定义。
 *           用 C 位域(bit-field)把一条"图形指令"打包成固定 15 字节的结构，
 *           再按裁判系统串口协议组装成完整帧发给裁判系统，由裁判系统转发到
 *           操作手客户端(1920x1080 屏幕)绘制。
 * 结构    :
 *   1. 编译器适配: MESSAGE_PACKED = __attribute__((packed))，1 字节对齐无填充。
 *   2. 宏 PRIMITIVE_CAT/CAT: 拼接标识符用。
 *   3. DEFINE_MESSAGE 宏: 批量生成 6 种图形结构(通用 figure/直线/矩形/圆/椭圆/圆弧)。
 *   4. ui_interface_number_t: 数字图形(浮点/整数)。
 *   5. ui_interface_string_t: 字符图形。
 *   6. ui_frame_header_t: 帧头(SOF/length/seq/crc8/cmd_id/sub_id/send_id/recv_id)。
 *   7. DEFINE_FIGURE_MESSAGE 宏: 生成 1/2/5/7 图形打包帧结构。
 *   8. ui_string_frame_t / ui_delete_frame_t: 字符帧 / 删除图层帧。
 * 调用关系:
 *   ui_interface.c -> 用这些结构打包发送
 *   ui_g.c         -> 用 ui_interface_figure_t / ui_interface_string_t 维护图形内容
 *   ui_g.h         -> 通过宏把数组元素强转成具体图形类型
 * ===== 调车关注 =====
 *   - 每条图形指令固定 15 字节(所有位域位宽之和 = 3*8 + 12*8 = 120 bit = 15 字节)。
 *   - operate_type(3bit): 0=删除(空操作) 1=添加(新建) 2=修改(更新)。
 *   - figure_type(3bit):  0=直线 1=矩形 2=圆 3=椭圆 4=圆弧 5=浮点数 6=整数 7=字符。
 *   - color(4bit) 颜色表:
 *       0=红蓝闪烁(阵营色) 1=粉色 2=绿色 3=橙色
 *       4=蓝绿色 5=粉紫色 6=黄色 7=白色 8=灰色
 *   - 坐标 start_x/start_y/end_x/end_y 是客户端 1920x1080 的像素坐标，
 *     原点在屏幕左上角，x 向右增大、y 向下增大(与数学坐标系 y 相反)。
 *   - 圆弧角度(arc): 本工程由 RM UI Designer 生成，按"0°=正上方、顺时针为正"
 *     (即 90°=正右、180°=正下、270°=正左)约定，调车时以屏幕实际显示为准。
 */
//
// Created by bismarckkk on 2025/3/22.
// Dynamic Edition
//

#ifndef UI_TYPES_H
#define UI_TYPES_H

// User Code Begin

// #define MANUAL_DIRTY

/* 依据编译器选择结构体"紧凑打包"关键字:
 *  GNU 与 ARMCC 都支持 __attribute__((packed))，保证结构体内存布局与协议逐字节一致。 */
#if defined(__GNUC__) || defined(__CC_ARM)
#define MESSAGE_PACKED __attribute__((packed))
#include <stdint.h>
#else
#error "MESSAGE_PACKED not defined for this compiler"
#endif

// User Code End

/* 标识符拼接宏: CAT(a,b) -> ab，用于 DEFINE_MESSAGE 里拼接字段名/类型名 */
#define PRIMITIVE_CAT(x, y) x ## y
#define CAT(x, y) PRIMITIVE_CAT(x, y)

/* ===== 图形指令通用结构生成宏 =====
 * 生成一个"一条图形指令"的位域结构，全部字段位宽相加恒等于 120 bit(15 字节):
 *
 *   figure_name[3]   : 图形名(3 字节，作为图形唯一标识，客户端靠它增删改)
 *   operate_type:3   : 操作类型 0=删除 1=添加 2=修改
 *   figure_type:3    : 图形类型 0线 1矩形 2圆 3椭圆 4圆弧 5浮点数 6整数 7字符
 *   layer:4          : 图层 0~9
 *   color:4          : 颜色(见文件头颜色表)
 *   p_a:9            : 类型相关参数(弧线为 start_angle，圆为占位)
 *   p_b:9            : 类型相关参数(弧线为 end_angle，圆为占位)
 *   width:10         : 线宽
 *   start_x:11       : 起点/圆心 x 像素坐标
 *   start_y:11       : 起点/圆心 y 像素坐标
 *   p_c:10           : 类型相关参数(圆为半径 r，椭圆为 rx)
 *   p_d:11           : 类型相关参数(直线/矩形为 end_x，椭圆为 ry)
 *   p_e:11           : 类型相关参数(直线/矩形为 end_y)
 */
#define DEFINE_MESSAGE(name, p_a, p_b, p_c, p_d, p_e)   \
typedef struct {                                        \
uint8_t figure_name[3];                                 \
uint32_t operate_type:3;                                \
uint32_t figure_type:3;                                 \
uint32_t layer:4;                                       \
uint32_t color:4;                                       \
uint32_t PRIMITIVE_CAT(,p_a) :9;                        \
uint32_t PRIMITIVE_CAT(,p_b):9;                         \
uint32_t width:10;                                      \
uint32_t start_x:11;                                    \
uint32_t start_y:11;                                    \
uint32_t PRIMITIVE_CAT(,p_c):10;                        \
uint32_t PRIMITIVE_CAT(,p_d):11;                        \
uint32_t PRIMITIVE_CAT(,p_e):11;                        \
} MESSAGE_PACKED ui_interface_ ## name ##_t

/* 通用 figure(全部为占位字段)；实际使用按图形类型强转成下面 5 种之一 */
DEFINE_MESSAGE(figure, _a, _b, _c, _d, _e);
/* 直线: p_d=end_x, p_e=end_y 为终点坐标 */
DEFINE_MESSAGE(line, _a, _b, _c, end_x, end_y);
/* 矩形: p_d=end_x, p_e=end_y 为对角顶点(另一角为 start_x/start_y) */
DEFINE_MESSAGE(rect, _a, _b, _c, end_x, end_y);
/* 圆: p_c=r 为半径 */
DEFINE_MESSAGE(round, _a, _b, r, _d, _e);
/* 椭圆: p_c=rx, p_d=ry 为 x/y 半轴 */
DEFINE_MESSAGE(ellipse, _a, _b, _c, rx, ry);
/* 圆弧: p_a=start_angle, p_b=end_angle 起止角，p_d=rx, p_e=ry 为 x/y 半轴 */
DEFINE_MESSAGE(arc, start_angle, end_angle, _c, rx, ry);

/* 数字图形(浮点/整数): 在通用字段基础上多一个 int32_t number 数值 */
typedef struct {
    uint8_t figure_name[3];    /* 图形名 */
    uint32_t operate_type: 3;  /* 操作类型 */
    uint32_t figure_type: 3;   /* 图形类型(5=浮点 6=整数) */
    uint32_t layer: 4;         /* 图层 */
    uint32_t color: 4;         /* 颜色 */
    uint32_t font_size: 9;     /* 字号 */
    uint32_t _b: 9;            /* 占位 */
    uint32_t width: 10;        /* 线宽 */
    uint32_t start_x: 11;      /* 起点 x */
    uint32_t start_y: 11;      /* 起点 y */
    int32_t number;            /* 要显示的数值 */
} MESSAGE_PACKED ui_interface_number_t;

/* 字符图形: 在通用字段基础上多 str_length 与 string[30] 字符串内容 */
typedef struct {
    uint8_t figure_name[3];    /* 图形名 */
    uint32_t operate_type: 3;  /* 操作类型 */
    uint32_t figure_type: 3;   /* 图形类型(7=字符) */
    uint32_t layer: 4;         /* 图层 */
    uint32_t color: 4;         /* 颜色 */
    uint32_t font_size: 9;     /* 字号 */
    uint32_t str_length: 9;    /* 字符串实际长度 */
    uint32_t width: 10;        /* 线宽 */
    uint32_t start_x: 11;      /* 起点 x */
    uint32_t start_y: 11;      /* 起点 y */
    uint32_t _c: 10;           /* 占位 */
    uint32_t _d: 11;           /* 占位 */
    uint32_t _e: 11;           /* 占位 */
    char string[30];           /* 字符串内容 */
} MESSAGE_PACKED ui_interface_string_t;

/* ===== 帧头 =====
 * 总长 13 字节:
 *   SOF(1)    帧头固定 0xA5
 *   length(2) 数据段长度(= cmd_id 之后的字节数，见 ui_interface.c 注释)
 *   seq(1)    包序号
 *   crc8(1)   帧头校验(对 SOF+length+seq 共 4 字节做 CRC8)
 *   cmd_id(2) 命令 ID，客户端绘制固定 0x0301
 *   sub_id(2) 子命令: 0x0100=删除图层 0x0101=1图形 0x0102=2图形
 *                     0x0103=5图形 0x0104=7图形 0x0110=字符
 *   send_id(2) 发送方 ID(机器人 ID)
 *   recv_id(2) 接收方 ID(客户端 ID = 机器人 ID + 256)
 */
typedef struct {
    uint8_t SOF;
    uint16_t length;
    uint8_t seq, crc8;
    uint16_t cmd_id, sub_id;
    uint16_t send_id, recv_id;
} MESSAGE_PACKED ui_frame_header_t;

/* ===== 图形打包帧生成宏 =====
 * 生成"帧头 + num 条图形 + crc16 尾校验"的完整帧结构。 */
#define DEFINE_FIGURE_MESSAGE(num)      \
typedef struct {                        \
ui_frame_header_t header;               \
ui_interface_figure_t data[num];        \
uint16_t crc16;                         \
} MESSAGE_PACKED ui_ ## num##_frame_t

DEFINE_FIGURE_MESSAGE(1);  /* 单图形帧 */
DEFINE_FIGURE_MESSAGE(2);  /* 2 图形帧 */
DEFINE_FIGURE_MESSAGE(5);  /* 5 图形帧 */
DEFINE_FIGURE_MESSAGE(7);  /* 7 图形帧 */

/* 字符帧: 帧头 + 1 条字符图形 + crc16 */
typedef struct {
    ui_frame_header_t header;
    ui_interface_string_t option;
    uint16_t crc16;
} MESSAGE_PACKED ui_string_frame_t;

/* 删除图层帧: 帧头 + 删除方式 + 图层 + crc16 */
typedef struct {
    ui_frame_header_t header;
    uint8_t delete_type;
    uint8_t layer;
    uint16_t crc16;
} MESSAGE_PACKED ui_delete_frame_t;

/* 各帧的全局缓冲(实现在 ui_interface.c，放在 .AXI_SRAM 段) */
extern ui_string_frame_t _ui_string_frame;
extern ui_1_frame_t _ui_1_frame;
extern ui_2_frame_t _ui_2_frame;
extern ui_5_frame_t _ui_5_frame;
extern ui_7_frame_t _ui_7_frame;

#endif //UI_TYPES_H
