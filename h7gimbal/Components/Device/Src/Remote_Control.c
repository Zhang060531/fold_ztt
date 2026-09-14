/**
 ******************************************************************************
 * @file    Remote_Control.c
 * @brief   DR16 遥控器 SBUS 帧解析实现
 *
 * 【本文件做什么】
 * 把 SBUS 接收到的 18 字节帧解析成 Remote_Info_Typedef 结构体（SBUS_TO_RC），
 * 并提供掉线监测（Remote_Message_Moniter）。
 *
 * 【SBUS 位打包说明（新手必读）】
 * 标准 SBUS 把 16 个通道的 11bit 值紧密拼接成 22 字节，字节间没有对齐，
 * 所以解析时要跨字节取位（例如 ch[1] 由 buf[1] 低 3bit 拼 buf[2] 的 8bit 组成）。
 * 大疆把其中部分字节复用作鼠标/键盘数据。每个通道用 & 0x07FF 取出 11bit。
 * 通道中位值是 1024，解析后统一减去 1024 得到对称的 ± 范围。
 *
 * 【在线检测】
 * 每次收到帧会把 online_cnt 重置为 0xFA(250)；周期任务每次递减，
 * 降到阈值以下就认为遥控器掉线，把数据清零并置 rc_lost 标志。
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "Remote_Control.h"

/* Exported variables ---------------------------------------------------------*/
/**
 * @brief remote control structure variable
 */
 Remote_Info_Typedef remote_ctrl={
	.online_cnt = 0xFAU,
	.rc_lost = true,
};

/**
 * @brief remote control usart RxDMA MultiBuffer
 */
__attribute__((section (".AXI_SRAM"))) uint8_t SBUS_MultiRx_Buf[2][SBUS_RX_BUF_NUM];
 
/* Private variables ---------------------------------------------------------*/
/**
 * @brief structure that contains the information of keyboard
 */
KeyBoard_Info_Typedef KeyBoard_Info;

/* Private function prototypes -----------------------------------------------*/
/**
  * @brief  Update the status of keyboard
  */
static void Key_Status_Update(KeyBoard_Info_Typedef *KeyInfo,bool KeyBoard_Status);

/**
  * @brief  把接收到的 SBUS 18 字节帧解析成遥控器结构体
  * @param  sbus_buf: 指向收到的 SBUS 帧（18 字节）
  * @param  remote_ctrl: 解析结果写入的 Remote_Info_Typedef 结构体
  * @retval none
  * @note   18 字节布局（大疆扩展）：
  *         buf[0~5]    ：通道 ch0~ch3 的 11bit 位流（每通道 11bit，紧密拼接）；
  *         buf[5]      ：还包含两个拨杆 s0/s1；
  *         buf[6~11]   ：鼠标 X/Y/Z（各 2 字节）；
  *         buf[12~13]  ：鼠标左/右键按下状态；
  *         buf[14~15]  ：键盘按键位域（16 个键各占 1bit）；
  *         buf[16~17]  ：通道 ch4 的 11bit。
  *         解析完成后把 ch0~ch4 都减去 1024 中位值，得到对称范围。
  */
void SBUS_TO_RC(volatile const uint8_t *sbus_buf, Remote_Info_Typedef  *remote_ctrl)
{
    if (sbus_buf == NULL || remote_ctrl == NULL) return;

    /* Channel 0, 1, 2, 3
     * 四个通道各自取 11bit（& 0x07FF）。因为 11bit 不是字节对齐，需要跨字节移位拼接：
     *   ch0 = buf[0] 的 8bit + buf[1] 的低 3bit（左移 8）
     *   ch1 = buf[1] 的高 5bit（右移 3）+ buf[2] 的 8bit（左移 5）
     *   ch2 = buf[2] 的高 2bit（右移 6）+ buf[3] 的 8bit（左移 2）+ buf[4] 的低 2bit（左移 10）
     *   ch3 = buf[4] 的高 7bit（右移 1）+ buf[5] 的 8bit（左移 7）
     */
    remote_ctrl->rc.ch[0] = (  sbus_buf[0]       | (sbus_buf[1] << 8 ) ) & 0x07ff;                            //!< Channel 0
    remote_ctrl->rc.ch[1] = ( (sbus_buf[1] >> 3) | (sbus_buf[2] << 5 ) ) & 0x07ff;                            //!< Channel 1
    remote_ctrl->rc.ch[2] = ( (sbus_buf[2] >> 6) | (sbus_buf[3] << 2 ) | (sbus_buf[4] << 10) ) & 0x07ff;      //!< Channel 2
    remote_ctrl->rc.ch[3] = ( (sbus_buf[4] >> 1) | (sbus_buf[5] << 7 ) ) & 0x07ff;                            //!< Channel 3
    remote_ctrl->rc.ch[4] = (  sbus_buf[16] 	   | (sbus_buf[17] << 8) ) & 0x07ff;                 			      //!< Channel 4

    /* Switch left, right */
    remote_ctrl->rc.s[0] = ((sbus_buf[5] >> 4) & 0x0003);                  //!< Switch left
    remote_ctrl->rc.s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2;             //!< Switch right

    /* Mouse axis: X, Y, Z */
    remote_ctrl->mouse.x = sbus_buf[6]  | (sbus_buf[7] << 8);                    //!< Mouse X axis
    remote_ctrl->mouse.y = sbus_buf[8]  | (sbus_buf[9] << 8);                    //!< Mouse Y axis
    remote_ctrl->mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);                  //!< Mouse Z axis

    /* Mouse Left, Right Is Press  */
    remote_ctrl->mouse.press_l = sbus_buf[12];                                  //!< Mouse Left Is Press
    remote_ctrl->mouse.press_r = sbus_buf[13];                                  //!< Mouse Right Is Press

    /* KeyBoard value */
    remote_ctrl->key.v = sbus_buf[14] | (sbus_buf[15] << 8);                    //!< KeyBoard value

    remote_ctrl->rc.ch[0] -= RC_CH_VALUE_OFFSET;
    remote_ctrl->rc.ch[1] -= RC_CH_VALUE_OFFSET;
    remote_ctrl->rc.ch[2] -= RC_CH_VALUE_OFFSET;
    remote_ctrl->rc.ch[3] -= RC_CH_VALUE_OFFSET;
    remote_ctrl->rc.ch[4] -= RC_CH_VALUE_OFFSET;
    
		/* reset the online count */
		remote_ctrl->online_cnt = 0xFAU;
		
		/* reset the lost flag */
		remote_ctrl->rc_lost = false;
}
//------------------------------------------------------------------------------

/**
  * @brief  遥控器掉线监测（周期任务调用）
  * @param  remote_ctrl: 遥控器数据结构体
  * @retval none
  * @note   online_cnt 在每次收到帧时被重置为 0xFA(250)，本函数每次调用递减一次。
  *         当 online_cnt 降到阈值(0x32=50)以下，说明很久没收到新帧，判定掉线：
  *         把整个结构体清零并置 rc_lost=true，防止残留的旧数据被误用。
  */
void Remote_Message_Moniter(Remote_Info_Typedef  *remote_ctrl)
{
  /* Juege the device status */
  if(remote_ctrl->online_cnt <= 0x32U)
  {
    /* clear the data */
    memset(remote_ctrl,0,sizeof(Remote_Info_Typedef));

    /* reset the online count */
		
    /* set the lost flag */
		remote_ctrl->rc_lost = true;
		
  }
  else if(remote_ctrl->online_cnt > 0)
  {
    /* online count decrements which reseted in received interrupt  */
    remote_ctrl->online_cnt--;
  }
}
//------------------------------------------------------------------------------


