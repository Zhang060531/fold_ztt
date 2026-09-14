/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : INS_Task.c
  * @brief          : INS task —— 惯性导航(IMU 姿态解算)任务
  * @author         : GrassFan Wang
  * @date           : 2025/01/22
  * @version        : v1.0
  ******************************************************************************
  * @attention      : None
  ******************************************************************************
  */
/* USER CODE END Header */

/* ============================================================================
 * INS_Task —— 惯性导航(IMU)任务：把 BMI088 陀螺/加速度计数据解算成云台姿态角
 * ----------------------------------------------------------------------------
 * 运行流程(1kHz 循环，每 1ms 一次)：
 *   1) BMI088_Info_Update() 读取陀螺仪(rad/s)与加速度计(g)原始数据；
 *   2) 三轴加速度做二阶低通滤波(去掉震动高频毛刺)；
 *   3) QuaternionEKF_Update() 用扩展卡尔曼滤波融合陀螺+加速度计，得到四元数姿态；
 *   4) 把四元数换算成欧拉角(pitch/yaw/roll)，弧度转角度，供云台控制使用；
 *   5) 处理 yaw 在 ±180° 的回绕，累加出"累计总角度"Yaw_TolAngle；
 *   6) 每 5ms 跑一次 BMI088 恒温控制(用加热片 + PID 把芯片维持在 40℃)。
 * 输出结构体：INS_Info(见 INS_Task.h)。
 * ========================================================================== */

/* Includes ------------------------------------------------------------------*/
#include "cmsis_os.h"        /* FreeRTOS 内核(任务/延时/系统 tick) */
#include "INS_Task.h"        /* 本任务头文件(INS_Info 结构体定义) */
#include "bmi088.h"          /* BMI088 六轴 IMU 芯片驱动(BMI088_Info 数据) */
#include "lpf.h"             /* 低通滤波器(二阶 LowPassFilter2p) */
#include "pid.h"             /* PID 控制器(用于 IMU 恒温) */
#include "config.h"          /* 工程宏定义(角度/角速度索引、RadiansToDegrees 等) */
#include "tim.h"             /* 定时器(加热 PWM 用到 htim3) */
#include "Quaternion.h"      /* 四元数 + 扩展卡尔曼滤波姿态解算 */
#include "bsp_pwm.h"         /* 板级 PWM(加热片功率控制) */

/* 下面 4 个 GetIMUxxx 是"对外读取接口"的前置声明(定义在本文件末尾)。
 * 其它任务(如云台控制)通过它们读取 IMU 解算出的角度/角速度。 */
void GetIMUPitchAngleFdb(float* angle);
void GetIMUYawAngleFdb(float* angle);
void GetIMUPitchGyroFdb(float* gyro);
void GetIMUYawGyroFdb(float* gyro);

/**
  * @brief the structure that contains the information for the INS.
  *        INS 姿态信息结构体的全局实例，全工程共享，保存最新一次解算结果。
  */
INS_Info_Typedef INS_Info;

/**
  * @brief the array that contains the data of LPF2p coefficients.
  *        二阶低通滤波器(巴特沃斯)的 3 个系数 {a0, a1, a2}：
  *        递推式  y(k) = a0·y(k-1) + a1·y(k-2) + a2·x(k)。
  *        三个值 a0+a1+a2 ≈ 1，保证直流增益 = 1(信号均值不放大不衰减)。
  */
static float INS_LPF2p_Alpha[3]={1.929454039488895f, -0.93178349823448126f, 0.002329458745586203f};

/**
  * @brief the structure that contains the Information of accel LPF2p.
  *        三轴加速度各自一个二阶低通滤波器实例(共 3 个)，用于滤除震动噪声。
  */
LowPassFilter2p_Info_TypeDef INS_AccelPF2p[3];

/**
  * @brief the Initialize data of state transition matrix.
  *        EKF 状态转移矩阵 A 的初值：6×6 单位阵。
  *        状态量共 6 维 = 四元数[4] + 陀螺零偏[2]，单位阵表示初始无耦合。
  */
static float QuaternionEKF_A_Data[36]={1, 0, 0, 0, 0, 0,
                                       0, 1, 0, 0, 0, 0,
                                       0, 0, 1, 0, 0, 0,
                                       0, 0, 0, 1, 0, 0,
                                       0, 0, 0, 0, 1, 0,
                                       0, 0, 0, 0, 0, 1};

/**
  * @brief the Initialize data of posteriori covariance matrix.
  *        EKF 后验协方差矩阵 P 的初值(6×6)。
  *        对角线代表对每个状态初始值"有多不确定"：
  *        四元数 4 项给 100000(很不确定→滤波器会快速收敛)，零偏 2 项给 100(相对较确定)。
  *        非对角 0.1 表示状态间有微小相关，作为数值稳定性兜底。
  */
static float QuaternionEKF_P_Data[36]= {100000, 0.1, 0.1, 0.1, 0.1, 0.1,
                                        0.1, 100000, 0.1, 0.1, 0.1, 0.1,
                                        0.1, 0.1, 100000, 0.1, 0.1, 0.1,
                                        0.1, 0.1, 0.1, 100000, 0.1, 0.1,
                                        0.1, 0.1, 0.1,   0.1,  100, 0.1,
                                        0.1, 0.1, 0.1,   0.1,  0.1, 100};

/**
  * @brief the Initialize data of Temperature Control PID.
  *        IMU 恒温 PID 参数数组，顺序对应 PID_Parameter_Typedef 字段：
  *        {KP=1200, KI=20, KD=0, Alpha=0, Deadband=0, LimitIntegral=0,
  *         LimitOutput=2000, KF=0, IntSeparationErr=0, BangBangErr=0,
  *         Fuzzy_ErrMax=0, Fuzzy_dErrMax=0}。
  *        即：只有 P=1200、I=20 起作用，输出限幅 ±2000(PWM 占空比上限)。
  */
static float TemCtrl_PID_Param[PID_PARAMETER_NUM]={1200,20,0,0,0,0,2000};

/**
  * @brief the structure that contains the Information of Temperature Control PID.
  *        IMU 恒温控制的 PID 控制器实例。
  */
PID_Info_TypeDef TempCtrl_PID;

/**
 * @brief Initializes the INS_Task.
 */
static void INS_Task_Init(void);

/**
  * @brief  Control the BMI088 temperature
  */
static void BMI088_Temp_Control(float temp);

/* USER CODE BEGIN Header_INS_Task */
/**
  * @brief  Function implementing the StartINSTask thread.
  *         惯性导航任务主函数，由 FreeRTOS 以 1kHz 周期调度。
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_INS_Task */
void INS_Task(void const * argument)
{
  /* USER CODE BEGIN INS_Task */
  TickType_t INS_Task_SysTick = 0;  /* 系统 tick 计数值，用于每 5ms 的恒温控制节拍 */


	/* Initializes the INS_Task. */
	INS_Task_Init();

  /* Infinite loop */
  for(;;)
  {
    INS_Task_SysTick = osKernelSysTick();   /* 读取当前系统 tick(1 tick = 1ms) */

		/* Update the BMI088 measurement —— 读取 BMI088 最新原始数据(陀螺 rad/s、加速度 g、温度) */
    BMI088_Info_Update(&BMI088_Info);

    /* Accel measurement LPF2p —— 三轴加速度分别做二阶低通滤波，滤除震动高频噪声 */
    INS_Info.Accel[0]   =   LowPassFilter2p_Update(&INS_AccelPF2p[0],BMI088_Info.Accel[0]);
    INS_Info.Accel[1]   =   LowPassFilter2p_Update(&INS_AccelPF2p[1],BMI088_Info.Accel[1]);
    INS_Info.Accel[2]   =   LowPassFilter2p_Update(&INS_AccelPF2p[2],BMI088_Info.Accel[2]);

    /* Update the INS gyro in radians —— 角速度原样拷贝(单位 rad/s)，陀螺一般不做滤波以免引入相位滞后 */
		INS_Info.Gyro[0]   =   BMI088_Info.Gyro[0];
    INS_Info.Gyro[1]   =   BMI088_Info.Gyro[1];
    INS_Info.Gyro[2]   =   BMI088_Info.Gyro[2];

		/* Update the QuaternionEKF —— 用扩展卡尔曼滤波融合陀螺+加速度计，解算姿态(四元数)
		 * dt=0.001f = 1ms，与本任务 1kHz 周期一致 */
    QuaternionEKF_Update(&Quaternion_Info,INS_Info.Gyro,INS_Info.Accel,0.001f);

    /* 把 EKF 解算出的欧拉角(弧度)整体拷到 INS_Info.Angle，顺序 [yaw,pitch,roll] */
    memcpy(INS_Info.Angle,Quaternion_Info.EulerAngle,sizeof(INS_Info.Angle));

		/* Update the Euler angle in degrees.
		 * 弧度转角度：×57.295779513(=180/π)。索引经 Config.h 重映射到云台实际轴系。 */
    INS_Info.Pitch_Angle = Quaternion_Info.EulerAngle[IMU_ANGLE_INDEX_PITCH]*57.295779513f;
    INS_Info.Yaw_Angle   = Quaternion_Info.EulerAngle[IMU_ANGLE_INDEX_YAW]   *57.295779513f;
    INS_Info.Roll_Angle  = Quaternion_Info.EulerAngle[IMU_ANGLE_INDEX_ROLL]*57.295779513f;

		/* Update the yaw total angle —— 处理 yaw 在 ±180° 的回绕跳变
		 * 相邻两次读数之差若 < -180°(说明正向越过了 +180 回绕到 -180)圈数 +1；
		 * 若 > +180°(反向回绕)圈数 -1。从而累加出连续不跳变的总角度。 */
		if(INS_Info.Yaw_Angle - INS_Info.Last_Yaw_Angle < -180.f)
		{
			INS_Info.YawRoundCount++;
		}
		else if(INS_Info.Yaw_Angle - INS_Info.Last_Yaw_Angle > 180.f)
		{
			INS_Info.YawRoundCount--;
		}
		INS_Info.Last_Yaw_Angle = INS_Info.Yaw_Angle;

		/* 累计总角度 = 当前(±180 内)角度 + 圈数×360°，可超出 ±180° 连续增长 */
		INS_Info.Yaw_TolAngle = INS_Info.Yaw_Angle + INS_Info.YawRoundCount*360.f;

    /* Update the INS gyro in degrees —— 角速度弧度转角度(度/秒)，供控制环使用 */
    INS_Info.Pitch_Gyro = INS_Info.Gyro[IMU_GYRO_INDEX_PITCH]*RadiansToDegrees;
    INS_Info.Yaw_Gyro   = INS_Info.Gyro[IMU_GYRO_INDEX_YAW]*RadiansToDegrees;
    INS_Info.Roll_Gyro  = INS_Info.Gyro[IMU_GYRO_INDEX_ROLL]*RadiansToDegrees;

		/* 每 5 个 tick(5ms)跑一次 IMU 恒温控制 */
		if(INS_Task_SysTick%5 == 0)
		{
			BMI088_Temp_Control(BMI088_Info.Temperature);
		}

    osDelay(1);   /* 延时 1ms，使本任务按 1kHz 周期运行 */

  }
  /* USER CODE END INS_Task */
}
//------------------------------------------------------------------------------
/**
 * @brief Initializes the INS_Task.
 *        初始化三轴加速度低通滤波器、IMU 恒温 PID、以及四元数 EKF。
 */
static void INS_Task_Init(void)
{
  /* Initializes the Second order lowpass filter —— 三轴加速度各自初始化一个二阶低通滤波器 */
  LowPassFilter2p_Init(&INS_AccelPF2p[0],INS_LPF2p_Alpha);
  LowPassFilter2p_Init(&INS_AccelPF2p[1],INS_LPF2p_Alpha);
  LowPassFilter2p_Init(&INS_AccelPF2p[2],INS_LPF2p_Alpha);

  /* Initializes the Temperature Control PID —— 位置式 PID，目标温度 40℃(见 BMI088_Temp_Control) */
	PID_Init(&TempCtrl_PID,PID_POSITION,TemCtrl_PID_Param);

  /* Initializes the Quaternion EKF —— 初始化四元数扩展卡尔曼滤波器
   * 参数：Process_Noise1=10(四元数过程噪声 Q1)、Process_Noise2=0.001(零偏过程噪声 Q2)、
   *       Measure_Noise=1000000(加速度计测量噪声 R，很大=不太信任加速度计修正，主要靠陀螺积分)、
   *       A/P 矩阵初值用上面定义的两个数组。 */
	QuaternionEKF_Init(&Quaternion_Info,10.f, 0.001f, 1000000.f,QuaternionEKF_A_Data,QuaternionEKF_P_Data);
}
//------------------------------------------------------------------------------
/**
  * @brief  Control the BMI088 temperature —— IMU 恒温控制
  *         用加热片(PWM)把 BMI088 芯片温度稳定在 40℃，减小温漂对陀螺零偏的影响。
  * @param  Temp  measure of the BMI088 temperature —— 当前测得的芯片温度(℃)
  * @retval none
  */
static void BMI088_Temp_Control(float Temp)
{
	/* 位置式 PID：目标 40℃ 减去实测温度，算出需要的加热功率 */
	PID_Calculate(&TempCtrl_PID,40.f,Temp);

	/* 对 PID 输出限幅(±2000，即 LimitOutput)，防止占空比越界 */
	VAL_LIMIT(TempCtrl_PID.Output,-TempCtrl_PID.Param.LimitOutput,TempCtrl_PID.Param.LimitOutput);

	/* 把输出量当作 PWM 比较值写入，调节加热片功率 */
	Heat_Power_Control((uint16_t)(TempCtrl_PID.Output));
}
//------------------------------------------------------------------------------
/* 下面 4 个 GetIMUxxx 是"对外读取接口"：其它模块不直接碰 INS_Info，而是通过它们取角度/角速度，
 * 好处是封装了数据来源、便于以后替换 IMU 或做单位换算时只改这里。 */

/* 读取俯仰角(度) */
void GetIMUPitchAngleFdb(float* angle)
{
	*angle = INS_Info.Pitch_Angle;
}
/* 读取偏航角(度)，注意是 ±180° 内会回绕的 Yaw_Angle */
void GetIMUYawAngleFdb(float* angle)
{
	*angle = INS_Info.Yaw_Angle;
}
/* 读取俯仰轴角速度(度/秒) */
void GetIMUPitchGyroFdb(float* gyro)
{
	*gyro = INS_Info.Pitch_Gyro;
}
/* 读取偏航轴角速度(度/秒) */
void GetIMUYawGyroFdb(float* gyro)
{
	*gyro = INS_Info.Yaw_Gyro;
}
