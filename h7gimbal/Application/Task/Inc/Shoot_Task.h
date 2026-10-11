#ifndef __SHOOT_TASK_H
#define __SHOOT_TASK_H
#include "main.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "Motor.h"

/**
 * =====================================================================
 * 文件说明：发射机构任务头文件 (Shoot_Task.h)
 * ---------------------------------------------------------------------
 * 发射机构 = 2 个摩擦轮 M3508(FR1/FR2) + 1 个拨弹轮 M2006。
 * 定义发射相关的：PID 参数、摩擦轮下标、Shoot_Mode_e 发射状态机、Shoot_t 结构体。
 *
 * ===== 发射状态机 Shoot_Mode_e =====
 *  - SHOOT_STOP      停止：摩擦轮停转
 *  - SHOOT_READY     就绪：摩擦轮转起来，等开火指令
 *  - SHOOT_NORMAL    发射中：拨弹轮转一发子弹的角度
 *  - SHOOT_INVERSION 反转：拨弹轮反转(清卡弹用)
 *
 * ===== 调车关注 =====
 *  - FRICTION_MOTOR_PID_* 摩擦轮速度环 PID；SHOOT_MOTOR_SHELL/CORE 是拨弹轮串级 PID
 *  - 摩擦轮目标转速 FRICTION_MOTOR_SPEED_SET=6200 定义在 Shoot_Task.c
 * =====================================================================
 */
#define CHASSIS_SHOOT_ID 0x303
#define FR1 0	//friction motor 1 index
#define FR2 1	//friction motor 2 index

#define REDUCTION_RATIO 51.00f //拨弹盘电机减速比

#define FRICTION_MOTOR_PID_KP 19.1
#define FRICTION_MOTOR_PID_KI 0.1
#define FRICTION_MOTOR_PID_KD 5.1
#define FRICTION_MOTOR_PID_MAX_IOUT 6000
#define FRICTION_MOTOR_PID_MAX_OUT 25000

#define SHOOT_MOTOR_SHELL_PID_KP 200                           //调一下
#define SHOOT_MOTOR_SHELL_PID_KI 0
#define SHOOT_MOTOR_SHELL_PID_KD 2
#define SHOOT_MOTOR_SHELL_PID_MAX_IOUT 1000
#define SHOOT_MOTOR_SHELL_PID_MAX_OUT 20000

#define SHOOT_MOTOR_CORE_PID_KP 3
#define SHOOT_MOTOR_CORE_PID_KI 0
#define SHOOT_MOTOR_CORE_PID_KD 0
#define SHOOT_MOTOR_CORE_PID_MAX_IOUT 8000
#define SHOOT_MOTOR_CORE_PID_MAX_OUT 13500

#define FRI_FILTER 0.0624395339


typedef enum{
	SHOOT_STOP,
	SHOOT_READY,
	SHOOT_NORMAL,
	SHOOT_INVERSION
}Shoot_Mode_e;

typedef struct
{

	float continuous_angle_set;
	float continuous_angle_fdb;
}Shoot_Motor_t;

typedef struct
{
	DJI_Motor_Info_Typedef fri_motor_fdb[2];	//摩擦轮电机反馈
	DJI_Motor_Info_Typedef sht_motor_fdb;	 //拨盘电机反馈
	
	DJI_Motor_Contorl_Info_Typedef fri_motor_set[2];	//摩擦轮电机设置
	DJI_Motor_Contorl_Info_Typedef sht_motor_set;	//拨盘电机设置

	Shoot_Mode_e shoot_mode;
	Shoot_Motor_t shoot_motor; 
	
	int16_t fri_speed;   //其实不太理解这个变量有点多此一举
	int16_t shooter_barrel_heat;
	int16_t shooter_barrel_heat_limit;
	int16_t shooter_barrel_cooling_value;

	float initial_speed;
	
}Shoot_t;

extern Shoot_t shoot;
extern xQueueHandle CAN1_shoot_Queue;

void shoot_task(void const *pvParameters);
extern uint32_t GetMotorFR1Fdb(DJI_Motor_Info_Typedef * motor);
extern uint32_t GetMotorFR2Fdb(DJI_Motor_Info_Typedef * motor);

extern uint32_t GetShootMotorFdb(DJI_Motor_Info_Typedef* motor);
extern void Shoot_Info_Update(uint32_t *Identifier, uint8_t *Rx_Buf, Shoot_t *ShootData);

#endif
