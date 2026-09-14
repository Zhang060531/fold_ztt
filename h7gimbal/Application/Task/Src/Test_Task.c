#include "main.h"
#include "string.h"
#include "Test_Task.h"
#include "cmsis_os.h"
#include "Referee_System.h"
#include "Control_Task.h"
#include "bsp_uart.h"
#include "stdio.h"

#include "INS_Task.h"

/**
 * =====================================================================
 * 文件说明：【VoFa 专用】测试/调试任务 (Test_Task.c) —— 用 VoFa 调 yaw 串级 PID
 * ---------------------------------------------------------------------
 * 本任务周期性地把 yaw 轴「外环角度环 + 内环角速度环」的关键量通过
 * USART7 发给电脑上的 VoFa+ 上位机（JustFloat 协议，921600 波特率）。
 *
 * ===== yaw 串级 PID 结构（回顾 Control_Task.c 的 pidCalc）=====
 *   外环(角度环 gimbal_shell_pid[YAW])：角度误差  → 目标角速度 gyro.set[YAW]
 *   内环(角速度环 gimbal_core_pid[YAW])：角速度误差 → 力矩 motor_set[YAW].Torque
 *
 * ===== 调车关注 =====
 *  - 手动(普通)模式下 yaw 是「绝对模式」，角度用 absolute_* 变量；
 *    若你调的是相对模式，把下面 CH1/CH2 的 absolute_* 换成 relative_* 即可。
 *  - VoFa+ 里协议选 JustFloat，通道数填 4，波特率 921600（连 USART7）。
 *  - 想换别的量，按下方「可替换的变量」注释改对应参数即可。
 * =====================================================================
 */
void Test_Task(void const * argument)
{

	while(1)
	{
		/* ==================== 【VoFa 专用】发送 yaw 内外环 4 通道波形 ====================
		 * 通道1：yaw 角度设定    (外环输入，目标角度 °)
		 * 通道2：yaw 角度反馈    (外环反馈，IMU 绝对角 °) —— 看是否追上通道1
		 * 通道3：yaw 目标角速度  (内环设定 = 外环输出 °/s) —— 内外环的桥梁
		 * 通道4：yaw 角速度反馈  (内环反馈 °/s)           —— 看是否追上通道3
		 */
//		USART_Vofa_Justfloat_Transmit4(
//			gimbal.angle.absolute_set[YAW],   // CH1: yaw 角度设定(外环输入)
//			gimbal.angle.absolute_fdb[YAW],   // CH2: yaw 角度反馈(外环反馈)
//			gimbal.gyro.set[YAW],             // CH3: yaw 目标角速度(内环设定=外环输出)
//			gimbal.gyro.fdb[YAW]              // CH4: yaw 角速度反馈(内环反馈)
//		);

		/* ---- 可替换的变量（想换掉上面 4 个就用这些）----
		 * gimbal.motor_set[YAW].Torque   内环输出(力矩 N·m)，看是否顶到 ±2.5 限幅
		 * gimbal.angle.relative_set[YAW] 相对模式的角度设定(°)
		 * gimbal.angle.relative_fdb[YAW] 相对模式的角度反馈(°)
		 */
		
//		  USART_Vofa_Justfloat_Transmit4(
//			gimbal.angle.absolute_set[PITCH],   // CH1: yaw 角度设定(外环输入)
//			gimbal.angle.absolute_fdb[PITCH],   // CH2: yaw 角度反馈(外环反馈)
//			gimbal.gyro.set[PITCH],             // CH3: yaw 目标角速度(内环设定=外环输出)
//			gimbal.gyro.fdb[PITCH]              // CH4: yaw 角速度反馈(内环反馈)
//			
			USART_Vofa_Justfloat_Transmit4(
			INS_Info.Angle[0],  
			INS_Info.Angle[1],             // CH3: 
			INS_Info.Angle[2],
			INS_Info.Yaw_Angle
			);
			
//			USART_Vofa_Justfloat_Transmit4(
//			INS_Info.Angle[0],  
//			INS_Info.Angle[1],             // CH3: 
//			INS_Info.Angle[2],
//			INS_Info.Yaw_Angle
//		);
		osDelay(1);   /* 发送周期 1ms(1kHz)；若 VoFa 曲线抖动可改成 osDelay(2) */
	}
}
