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
/* ============================================================================
 * 文件说明：CAN 发送任务 (CAN_Task.c)
 * ----------------------------------------------------------------------------
 * 作用：底盘对外「发数据」的任务，每 1ms 循环一次，把 Control_Task 算好的
 *        控制量通过 FDCAN 发给电调和云台：
 *          - TransmitChassisMotorCurrent：4 个轮毂电机电流 → CAN 0x200（C620 电调）
 *          - Transmit_Shoot_Info        ：发射数据 → CAN 0x23（FDCAN2 给云台）
 *          - TransmitShootMotorCurrent  ：拨弹电机电流 → CAN 0x1ff
 *
 * 注意：底盘「收数据」不在这个任务里，而在 FDCAN 接收中断里（bsp_can.c），
 *       收到云台 0x21 速度设定、0x22 云台状态、0x211 超电数据后存到全局结构体，
 *       再由 Control_Task 的 FdbUpdate 取用。
 *
 * ===== 调车关注 =====
 *  - 本文件基本不用改，就是「把 Control_Task 算出的 current_set 发出去」。
 *  - 如果某路电机电流发不出去，先核对 CAN ID 和波特率是否与电调匹配。
 * ============================================================================
 */
#include "cmsis_os.h"
#include "CAN_Task.h"
#include "Control_Task.h"
#include "INS_Task.h"
#include "Motor.h"
#include "bsp_can.h"
#include "Remote_Control.h"
#include "Control_Task.h"
/* 底盘控制结构体、裁判系统数据（外部定义） */
extern Chassis_Info_Typedef chassis;
extern Referee_System_Info_TypeDef Referee_System_Info;
/* USER CODE BEGIN Header_CAN_Task */
/**
* @brief Function implementing the StartCANTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CAN_Task */
/**
 * @brief CAN 发送任务主函数（FreeRTOS 任务入口）
 * @note  每 1ms 循环一次，把底盘控制量发出去。先记 tick，再依次发送三帧：
 *        轮毂电机电流(0x200)、发射数据(0x23)、拨弹电机电流(0x1ff)。
 *        osDelay(1) 形成 1kHz 发送周期。
 * @param argument 未使用
 * @retval 无
 */
void CAN_Task(void const * argument)
{
    TickType_t CAN_Task_SysTick = 0;   /* 系统 tick 计数，用于分频 */
    for (;;)
    {

        CAN_Task_SysTick = osKernelSysTick();   /* 读取系统 tick */
        /* 1. 发送 4 个轮毂电机电流指令（CAN 0x200，给 C620 电调） */
        TransmitChassisMotorCurrent(&FDCAN1_TxFrame, chassis.chassis_motor_set[M1].current_set, chassis.chassis_motor_set[M2].current_set, chassis.chassis_motor_set[M3].current_set, chassis.chassis_motor_set[M4].current_set);
			  Transmit_Shoot_Info(&FDCAN2_TxFrame, Referee_System_Info.shoot_data.initial_speed, Referee_System_Info.power_heat_data.shooter_17mm_barrel_heat,Referee_System_Info.robot_status.shooter_barrel_heat_limit,Referee_System_Info.robot_status.shooter_barrel_cooling_value);   /* 2. 发送发射数据（CAN 0x23，给云台） */
         
        /* 3. 发送拨弹电机电流（CAN 0x1ff） */
        TransmitShootMotorCurrent(&FDCAN1_TxFrame, chassis.gimbal_data.current_set);

        /* 偶数 tick（2ms 一次）分支：预留 500Hz 的发送位，当前为空 */
        if (CAN_Task_SysTick % 2 == 0)
        {
            //500Hz发送 请保证所有任务osDelay(1)
        }
        osDelay(1);
    }

}


