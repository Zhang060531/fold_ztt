




/* Includes ------------------------------------------------------------------*/
#include "Remote_Control.h"
#include "ramp.h"

/* ===== 文件说明：DR16 遥控器接收解析 (Remote_Control.c) =====
 * 作用：把 SBUS 串口收到的 18 字节原始帧解析成 Remote_Info_Typedef 结构体，
 *       并提供遥控器掉线检测。
 *
 * 结构：
 *   - SBUS_TO_RC()：从 sbus_buf 逐字节拼出摇杆通道、拨杆、鼠标、键盘数据。
 *   - Remote_Message_Moniter()：周期调用，判断遥控器是否掉线(online_cnt 递减到
 *     阈值就清空数据并置 rc_lost 标志)。
 *
 * 通道映射(DR16 遥控器)：
 *   - ch[0..3]：左右两个摇杆的 4 个通道(每个 11bit)，本工程在底盘控制里
 *     用 ch[2]=前后、ch[3]=横移、ch[0]=自旋(见 Control_Task.c)；
 *   - ch[4]：遥控器左上角拨轮；
 *   - s[0]：左拨杆开关(3 档)、s[1]：右拨杆开关(3 档)；
 *   - mouse.x/y/z：鼠标三轴移动量、press_l/r：鼠标左右键；
 *   - key.v：键盘 16 键位(bit 位域，W/S/A/D/SHIFT/CTRL/Q/E/R/F/G/Z/X/C/V/B)。
 *
 * ===== 调车关注 =====
 *  - 通道原始值范围 0~2047(11bit)，中位 1024，解析后减去 1024 变成 ±1024 的
 *    对称值，方便直接当速度/位置指令用。
 *  - 遥控器掉线后数据会被 memset 清零(摇杆回中、拨杆归 0)，避免飞车。
 * ===== ========================================================================= */

/* Exported variables ---------------------------------------------------------*/
/**
 * @brief 遥控器数据全局实例：初始在线计数 0xFA(250)、默认掉线
 */
 Remote_Info_Typedef remote_ctrl={
	.online_cnt = 0xFAU,   /*!< 在线计数初值，收到帧会重置 */
	.rc_lost = true,       /*!< 掉线标志，初始视为掉线 */
};

/**
 * @brief SBUS 接收 DMA 双缓冲(M0/M1 各 18 字节)
 */
__attribute__((section (".AXI_SRAM"))) uint8_t SBUS_MultiRx_Buf[2][SBUS_RX_BUF_NUM];

/* Private variables ---------------------------------------------------------*/
/**
 * @brief 键盘按键状态结构体(短按/长按/边沿检测用)
 */
KeyBoard_Info_Typedef KeyBoard_Info;

/* Private function prototypes -----------------------------------------------*/
/**
  * @brief  更新单个按键的状态(短按/长按/按下沿/释放沿)
  * @note   本文件未实现(见 Remote_Control.h 声明)，按键状态机相关。
  */
static void Key_Status_Update(KeyBoard_Info_Typedef *KeyInfo,bool KeyBoard_Status);

/**
  * @brief  解析一帧 SBUS 数据，更新遥控器结构体
  * @param  sbus_buf    指向 18 字节 SBUS 原始帧
  * @param  remote_ctrl 指向要更新的 Remote_Info_Typedef 结构体
  * @retval 无
  * @note   SBUS 16 个通道各 11bit 连续打包，前 4 个通道 + 拨杆 + 鼠标/键盘
  *          是大疆 DR16 的扩展格式，逐字节移位拼接取出。
  */
void SBUS_TO_RC(volatile const uint8_t *sbus_buf, Remote_Info_Typedef  *remote_ctrl)
{
    if (sbus_buf == NULL || remote_ctrl == NULL) return;   /* 空指针保护 */

    /* Channel 0, 1, 2, 3  解析 4 个摇杆通道(各 11bit, 逐字节移位拼接) */
    remote_ctrl->rc.ch[0] = (  sbus_buf[0]       | (sbus_buf[1] << 8 ) ) & 0x07ff;                            //!< Channel 0  右摇杆左右
    remote_ctrl->rc.ch[1] = ( (sbus_buf[1] >> 3) | (sbus_buf[2] << 5 ) ) & 0x07ff;                            //!< Channel 1  右摇杆上下
    remote_ctrl->rc.ch[2] = ( (sbus_buf[2] >> 6) | (sbus_buf[3] << 2 ) | (sbus_buf[4] << 10) ) & 0x07ff;      //!< Channel 2  左摇杆上下
    remote_ctrl->rc.ch[3] = ( (sbus_buf[4] >> 1) | (sbus_buf[5] << 7 ) ) & 0x07ff;                            //!< Channel 3  左摇杆左右
    remote_ctrl->rc.ch[4] = (  sbus_buf[16] 	   | (sbus_buf[17] << 8) ) & 0x07ff;                 			      //!< Channel 4  左上角拨轮

    /* Switch left, right  解析两个拨杆开关(左拨杆在 bit4~5, 右拨杆在 bit6~7) */
    remote_ctrl->rc.s[0] = ((sbus_buf[5] >> 4) & 0x0003);                  //!< Switch left  左拨杆(3 档: 0/1/2)
    remote_ctrl->rc.s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2;             //!< Switch right 右拨杆(3 档: 0/1/2)

    /* Mouse axis: X, Y, Z  解析鼠标三轴移动量(各 16bit) */
    remote_ctrl->mouse.x = sbus_buf[6]  | (sbus_buf[7] << 8);                    //!< Mouse X axis  鼠标 X 轴
    remote_ctrl->mouse.y = sbus_buf[8]  | (sbus_buf[9] << 8);                    //!< Mouse Y axis  鼠标 Y 轴
    remote_ctrl->mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);                  //!< Mouse Z axis  鼠标 Z 轴(滚轮)

    /* Mouse Left, Right Is Press  鼠标左右键(1=按下, 0=松开) */
    remote_ctrl->mouse.press_l = sbus_buf[12];                                  //!< Mouse Left Is Press  鼠标左键
    remote_ctrl->mouse.press_r = sbus_buf[13];                                  //!< Mouse Right Is Press 鼠标右键

    /* KeyBoard value  键盘 16 键位(bit 位域) */
    remote_ctrl->key.v = sbus_buf[14] | (sbus_buf[15] << 8);                    //!< KeyBoard value

    /* 通道值减去中位偏移 1024，变成以 0 为中心的对称值(-1024~+1023) */
    remote_ctrl->rc.ch[0] -= RC_CH_VALUE_OFFSET;
    remote_ctrl->rc.ch[1] -= RC_CH_VALUE_OFFSET;
    remote_ctrl->rc.ch[2] -= RC_CH_VALUE_OFFSET;
    remote_ctrl->rc.ch[3] -= RC_CH_VALUE_OFFSET;
    remote_ctrl->rc.ch[4] -= RC_CH_VALUE_OFFSET;

		/* reset the online count  重置在线计数(收到帧即视为在线) */
		remote_ctrl->online_cnt = 0xFAU;

		/* reset the lost flag  清除掉线标志 */
		remote_ctrl->rc_lost = false;
}
//------------------------------------------------------------------------------

/**
  * @brief  遥控器掉线监测：周期调用，递减在线计数，超时则清空数据
  * @param  remote_ctrl 指向 Remote_Info_Typedef 结构体
  * @retval 无
  * @note   online_cnt 在收到帧时被重置为 0xFA，本函数每次调用递减；
  *          递减到 0x32(50) 以下即判定掉线，memset 清空整个结构体并置掉线标志。
  */
void Remote_Message_Moniter(Remote_Info_Typedef  *remote_ctrl)
{
  /* Juege the device status  判断是否已掉线 */
  if(remote_ctrl->online_cnt <= 0x32U)
  {
    /* clear the data  掉线：清空遥控器数据(摇杆回中、拨杆归 0) */
    memset(remote_ctrl,0,sizeof(Remote_Info_Typedef));

    /* reset the online count  在线计数保持为 0(已掉线) */

    /* set the lost flag  置掉线标志 */
		remote_ctrl->rc_lost = true;

  }
  else if(remote_ctrl->online_cnt > 0)
  {
    /* online count decrements which reseted in received interrupt  未掉线：计数递减 */
    remote_ctrl->online_cnt--;
  }
}
//------------------------------------------------------------------------------


