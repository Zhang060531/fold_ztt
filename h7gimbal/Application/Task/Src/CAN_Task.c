/**
  ******************************************************************************
  * @file           : CAN_Task.c
  * @brief          : CAN task
  * @author         : GrassFam Wang
  * @date           : 2025/1/22
  * @version        : v1.1
  ******************************************************************************
  * @attention      : None
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "cmsis_os.h"
#include "CAN_Task.h"
#include "INS_Task.h"
#include "Motor.h"
#include "bsp_can.h"
#include "Remote_Control.h"
#include "Control_Task.h"
#include "Shoot_Task.h"
#include "Quaternion.h"
#include "vision.h"

/**
 * =====================================================================
 * 文件说明：CAN 发送任务 (CAN_Task.c) —— 云台所有 CAN 报文的出口
 * ---------------------------------------------------------------------
 * 这个任务每 1ms 跑一次，负责把云台算好的数据通过 CAN 发出去。发送内容：
 *
 *  1. 达妙 DM4310 电机使能/清错：上电时如果电机状态异常(state!=1)，
 *     先 Clear 再 Enable 把它拉起来
 *  2. TransmitChassis_Info → CAN 0x21 帧：底盘速度指令（发给底盘）
 *  3. TransmitGimbal_Info  → CAN 0x22 帧：云台状态（给底盘/上位机 UI）
 *  4. DM_Motor_CAN_TxMessage → 达妙电机 MIT 帧：把 PID 算出的力矩下发给
 *     pitch/yaw 两个 DM4310（pitch 额外加了重力补偿 front）
 *  5. TransmitFriMotorCurrent → CAN 0x200 帧：摩擦轮电机电流（发给大疆电调）
 *  6. SentToVision / SentToVisionangle → CAN3 帧：发给视觉(妙算)的数据
 *
 * ===== 调车关注 =====
 *  - front1 是 pitch 轴重力补偿系数：云台俯仰时重力分量会压电机，
 *    需要额外加一个随俯仰角余弦变化的力矩抵消(cos 表示水平时最大)
 *  - 若达妙电机一直不被使能，先查 DM_Motor_Command 里的 CAN ID 和状态字
 * =====================================================================
 */

extern Gimbal_t gimbal;
extern Shoot_t shoot;
/* USER CODE BEGIN Header_CAN_Task */
/**
* @brief Function implementing the StartCANTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CAN_Task */
float kp=0,speed=13.8;                    // 调试用变量(未参与实际控制)
float front1 = -1.26,front = 0;   //-1.11  // front1=pitch 重力补偿系数，front=当前补偿值
void CAN_Task(void const * argument)
{
    TickType_t CAN_Task_SysTick = 0;
    for (;;)
    {
	    front = front1 * cos(gimbal.angle.absolute_fdb[PITCH] * 2 * PI / 360);   // pitch 重力补偿：随俯仰角余弦变化，水平时最大

        CAN_Task_SysTick = osKernelSysTick();
        // CAN-FD
        if (gimbal.pitch_motor_fdb.Data.State != 1)
        {
 //         DM_Motor_Command(&FDCAN2_TxFrame, &DM_4310_Motor[0], Motor_Save_Zero_Position);
            DM_Motor_Command(&FDCAN2_TxFrame, &DM_4310_Motor[0], Motor_Clear);   // 清除俯仰电机错误
			DM_Motor_Command(&FDCAN2_TxFrame, &DM_4310_Motor[0], Motor_Enable);   // 使能俯仰电机

        }
        if (gimbal.yaw_motor_fdb.Data.State != 1)
        {
			DM_Motor_Command(&FDCAN2_TxFrame, &DM_4310_Motor[1], Motor_Clear);    // 清除偏航电机错误
            DM_Motor_Command(&FDCAN2_TxFrame, &DM_4310_Motor[1], Motor_Enable);  // 使能偏航电机
			//DM_Motor_Command(&FDCAN2_TxFrame, &DM_4310_Motor[1], Motor_Save_Zero_Position);


        }

				TransmitChassis_Info(&FDCAN2_TxFrame);   // 0x21 帧：底盘速度指令
        TransmitGimbal_Info(&FDCAN2_TxFrame);    // 0x22 帧：云台状态
        DM_Motor_CAN_TxMessage(&FDCAN2_TxFrame, &DM_4310_Motor[0], 0, 0, 0, 0, gimbal.motor_set[PITCH].Torque+front);   // pitch 力矩 + 重力补偿
        DM_Motor_CAN_TxMessage(&FDCAN2_TxFrame, &DM_4310_Motor[1], 0, 0, 0, 0, gimbal.motor_set[YAW].Torque);    // yaw 力矩
        TransmitFriMotorCurrent(&FDCAN1_TxFrame,
                                shoot.fri_motor_set[FR1].current_set,
                                shoot.fri_motor_set[FR2].current_set);   // CAN 0x200 帧：摩擦轮电流

		SentToVision(&FDCAN3_TxFrame,shoot.initial_speed, gimbal.vision_data.mode, 0);   // CAN3 帧：弹速+模式发给视觉
        SentToVisionangle(&FDCAN3_TxFrame, Quaternion_Info);   // CAN3 帧：四元数姿态发给视觉
   if(CAN_Task_SysTick % 2== 0){

   //500Hz发送 请保证所有任务osDelay(1)

   }
        osDelay(1);   // 延时 1ms，形成 1kHz 发送周期
    }

}


