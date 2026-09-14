/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : Remote_Control.c
  * @brief          : remote_control interfaces functions 
  * @author         : Yan Yuanbin
  * @date           : 2023/04/27
  * @version        : v1.0
  ******************************************************************************
  * @attention      : to be tested
  ******************************************************************************
  */
/* USER CODE END Header */

/**
 ******************************************************************************
 * @file    Remote_Control.h
 * @brief   大疆 DR16 遥控器接收解析的数据结构定义
 *
 * 【设备是什么】
 * DR16 是 RoboMaster 官方 2.4G 无线遥控器，车上通过 DR16 接收机接收信号。
 * 操作手用它控制机器人：左右摇杆（对应 4 个通道）、两个拨杆开关、鼠标（瞄准）、
 * 键盘（快捷键）。本文件定义解析结果的结构体与相关宏。
 *
 * 【通过什么接口连接】
 * DR16 接收机通过 SBUS 串口协议与 MCU 相连（一条 UART，100k 波特率，8E2 帧格式），
 * 底层用 DMA 双缓冲接收 18 字节的 SBUS 帧（SBUS_RX_BUF_NUM = 18）。
 *
 * 【数据协议大概长什么样】
 * SBUS 一帧 18 字节。标准 SBUS 是 16 个通道各 11bit 打包成 22 字节，大疆对后半段
 * 字节做了自定义扩展，用于传输鼠标、键盘数据。解析时逐字节拼出：
 *   通道 ch[0~3]（左右摇杆）+ 拨杆 s[0~1] + 鼠标 XYZ + 鼠标左右键 + 键盘键值。
 * 通道原始范围 0~1680（中位 1024），解析后减去 1024 变成 ±660 的对称值，方便控制。
 * 逐字节含义见 Remote_Control.c 中 SBUS_TO_RC 的注释。
 *
 * 【被谁调用】
 *   - SBUS_TO_RC()：串口中断/接收任务收到一帧后调用，把字节解析成结构体；
 *   - Remote_Message_Moniter()：周期任务调用，检测遥控器是否掉线并清零数据。
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "stdbool.h"
#include "stdlib.h"
#include "string.h"
#include "math.h"
#include <stdio.h>
#include <string.h>
/* Exported defines -----------------------------------------------------------*/
/**
 * @brief Length of SBUS received data
 */
#define SBUS_RX_BUF_NUM		18u
/**
 * @brief offset of remote control channel data
 */
#define RC_CH_VALUE_OFFSET		1024U

/**
 * @brief judgement keyboard set short time
 */
#define KEY_SET_SHORT_TIME		50U
/**
 * @brief judgement keyboard set long time
 */
#define KEY_SET_LONG_TIME		1000U

/**
 * @brief status of keyboard up
 */
#define KEY_UP                    0x00U
/**
 * @brief status of keyboard down
 */
#define KEY_DOWN                  0x01U

/**
 * @brief MAX speed of mouse speed
 */
#define MOUSE_SPEED_MAX		300U

/* Exported types ------------------------------------------------------------*/
/**
 * @brief typedef enum that contains the status of the keyboard.
 * @note  按键状态机：UP=松开；SHORT_DOWN=短按；DOWN=长按；PRESS=按下沿(0->1)；RELAX=释放沿(1->0)。
 *        通过计时区分「短按/长按」，通过边沿检测区分「按下瞬间/松开瞬间」。
 */
typedef enum
{
	UP,			/*!< up */
	SHORT_DOWN,	/*!< short time down */
	DOWN,		/*!< long time down */
	PRESS,		/*!< 0->1 */
	RELAX,		/*!< 1->0 */
	KeyBoard_Status_NUM,
}KeyBoard_Status_e;

typedef struct
{
	uint16_t Count;
	KeyBoard_Status_e Status;
	KeyBoard_Status_e last_Status;
	bool last_KEY_PRESS;
	bool KEY_PRESS;
}KeyBoard_Info_Typedef;

typedef struct
{
	KeyBoard_Info_Typedef press_l;
	KeyBoard_Info_Typedef press_r;
	KeyBoard_Info_Typedef W;
	KeyBoard_Info_Typedef S;
	KeyBoard_Info_Typedef A;
	KeyBoard_Info_Typedef D;
	KeyBoard_Info_Typedef SHIFT;
	KeyBoard_Info_Typedef CTRL;
	KeyBoard_Info_Typedef Q;
	KeyBoard_Info_Typedef E;
	KeyBoard_Info_Typedef R;
	KeyBoard_Info_Typedef F;
	KeyBoard_Info_Typedef G;
	KeyBoard_Info_Typedef Z;
	KeyBoard_Info_Typedef X;
	KeyBoard_Info_Typedef C;
	KeyBoard_Info_Typedef V;
	KeyBoard_Info_Typedef B;
}Remote_Pressed_Typedef;


/**
 * @brief typedef structure that contains the information for the remote control.
 * @note  遥控器解析结果总结构体，分三部分：
 *        rc(摇杆 ch[5] + 拨杆 s[2])、mouse(鼠标 xyz + 左右键)、key(键盘位域)。
 *        rc_lost 为掉线标志，online_cnt 为在线计数（收到帧会重置，周期任务递减）。
 */
typedef  struct
{
	/**
	 * @brief structure that contains the information for the lever/Switch.
	 */
	struct
	{
		int16_t ch[5];
		uint8_t s[2];
	} rc;
	
	/**
	 * @brief structure that contains the information for the mouse.
	 */
	struct
	{
		int16_t x;
		int16_t y;
		int16_t z;
		uint8_t press_l;
		uint8_t press_r;
	} mouse;

	/**
	 * @brief structure that contains the information for the keyboard.
	 */
	union
	{
		uint16_t v;
		struct
		{
			uint16_t W:1;
			uint16_t S:1;
			uint16_t A:1;
			uint16_t D:1;
			uint16_t SHIFT:1;
			uint16_t CTRL:1;
			uint16_t Q:1;
			uint16_t E:1;
			uint16_t R:1;
			uint16_t F:1;
			uint16_t G:1;
			uint16_t Z:1;
			uint16_t X:1;
			uint16_t C:1;
			uint16_t V:1;
			uint16_t B:1;
		} set;
	} key;

	bool rc_lost;   /*!< lost flag */
	uint8_t online_cnt;   /*!< online count */
} Remote_Info_Typedef;

/* Exported variables ---------------------------------------------------------*/
/**
 * @brief remote control structure variable
 */
extern Remote_Info_Typedef remote_ctrl;
/**
 * @brief remote control usart RxDMA MultiBuffer
 */
extern uint8_t SBUS_MultiRx_Buf[2][SBUS_RX_BUF_NUM];

/* Mouse Exported defines -----------------------------------------------------*/
#define MOUSE_X_MOVE_SPEED    (remote_ctrl.mouse.x )
#define MOUSE_Y_MOVE_SPEED    (remote_ctrl.mouse.y )
#define MOUSE_Z_MOVE_SPEED    (remote_ctrl.mouse.z )
#define MOUSE_PRESSED_LEFT    (remote_ctrl.mouse.press_l)
#define MOUSE_PRESSED_RIGHT   (remote_ctrl.mouse.press_r)

/* KeyBoard Exported defines --------------------------------------------------*/
#define KeyBoard_W            (remote_ctrl.key.set.W)
#define KeyBoard_S            (remote_ctrl.key.set.S)
#define KeyBoard_A            (remote_ctrl.key.set.A)
#define KeyBoard_D            (remote_ctrl.key.set.D)
#define KeyBoard_SHIFT        (remote_ctrl.key.set.SHIFT)
#define KeyBoard_CTRL         (remote_ctrl.key.set.CTRL)
#define KeyBoard_Q            (remote_ctrl.key.set.Q)
#define KeyBoard_E            (remote_ctrl.key.set.E)
#define KeyBoard_R            (remote_ctrl.key.set.R)
#define KeyBoard_F            (remote_ctrl.key.set.F)
#define KeyBoard_G            (remote_ctrl.key.set.G)
#define KeyBoard_Z            (remote_ctrl.key.set.Z)
#define KeyBoard_X            (remote_ctrl.key.set.X)
#define KeyBoard_C            (remote_ctrl.key.set.C)
#define KeyBoard_V            (remote_ctrl.key.set.V)
#define KeyBoard_B            (remote_ctrl.key.set.B)

/* Exported functions prototypes ---------------------------------------------*/
/**
  * @brief  convert the remote control received message
  */
extern void SBUS_TO_RC(volatile const uint8_t *sbus_buf, Remote_Info_Typedef *remote_ctrl);
/**
  * @brief  clear the remote control data while the device offline
  */
extern void Remote_Message_Moniter(Remote_Info_Typedef *remote_ctrl);

/**
  * @brief  report the cover status that acrroding the key_R swicthing
  */
extern bool Key_R(void);
/**
  * @brief  switch the shooter mode that acrroding the key_B
  */
extern bool Key_B(void);

/**
  * @brief  report the auto aim status that acrroding the mouse right swicthing
  */
extern bool Mouse_Pressed_Right(void);
/**
  * @brief  report the fire status that acrroding the mouse left swicthing
  */
extern bool Mouse_Pressed_Left(void);

#endif //REMOTE_CONTROL_H


