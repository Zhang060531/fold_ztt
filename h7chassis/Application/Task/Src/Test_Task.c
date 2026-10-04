/* ============================================================================
 * 文件说明：测试任务 (Test_Task.c)
 * ----------------------------------------------------------------------------
 * 作用：预留的测试任务入口，当前为空（while(1) 里只有 osDelay(1) 空转）。
 *        调车时可以在这里临时放一些测试代码（如让某个电机转一下、打印调试信息），
 *        不影响主控制逻辑。
 *
 * 注意：底部 extern 的 ref_data 是个孤立声明（工程里并没有定义这个全局变量），
 *        属于遗留代码，实际裁判系统数据走 Control_Task.c 的 chassis.ref_data。
 *
 * ===== 调车关注 =====
 *  - 本任务基本没用，调车需要临时验证某个功能时可以借用这里。
 * ============================================================================
 */
#include "main.h"
#include "string.h"
#include "Test_Task.h"
#include "cmsis_os.h"
#include "Referee_System.h"
#include "Control_Task.h"
#include "stdio.h"

#include "bsp_uart.h"
#include "INS_Task.h"

/* 外部引用：chassis 在 Control_Task.c 定义。ref_data 为孤立声明，勿依赖 */
extern Chassis_Info_Typedef chassis;
extern Referee_System_Info_TypeDef ref_data;

/**
 * @brief 测试任务主函数（FreeRTOS 任务入口，当前为空）
 * @note  while(1) 里只有 osDelay(1)，没有任何测试逻辑，需要时自行在这里添加。
 * @param argument 未使用
 * @retval 无
 */
void Test_Task(void const * argument)
{

	while(1)   /* 功率拟合测试：发 9 通道给 VoFa+（4 电流 + 4 转速 + 1 实测功率） */
	{
//		float vofa_data[9];
//		vofa_data[0] = (float)chassis.chassis_motor_set[0].current_set;   // i1 电流指令
//		vofa_data[1] = (float)chassis.chassis_motor_set[1].current_set;   // i2
//		vofa_data[2] = (float)chassis.chassis_motor_set[2].current_set;   // i3
//		vofa_data[3] = (float)chassis.chassis_motor_set[3].current_set;   // i4
//		vofa_data[4] = (float)chassis.chassis_motor_fdb[0].Data.Velocity; // w1 转速 RPM
//		vofa_data[5] = (float)chassis.chassis_motor_fdb[1].Data.Velocity; // w2
//		vofa_data[6] = (float)chassis.chassis_motor_fdb[2].Data.Velocity; // w3
//		vofa_data[7] = (float)chassis.chassis_motor_fdb[3].Data.Velocity; // w4
//		vofa_data[8] = chassis.super_power.curren_chassis_power;          // P 底盘实测功率 W
//		USART_Vofa_Justfloat_Transmit_N(vofa_data, 9);
		
			
			USART_Vofa_Justfloat_Transmit(Shoot_Motor.Data.Velocity ,0,0);
    osDelay(1);		

	}
}
	




