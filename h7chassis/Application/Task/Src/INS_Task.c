/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : INS_Task.c
  * @brief          : INS task
  * @author         : GrassFan Wang
  * @date           : 2025/01/22
  * @version        : v1.0
  ******************************************************************************
  * @attention      : None
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
/* ============================================================================
 * 文件说明：IMU 姿态解算任务 (INS_Task.c)
 * ----------------------------------------------------------------------------
 * 作用：读取 BMI088（陀螺仪 + 加速度计），用四元数 EKF 解算机体的 pitch/roll/yaw
 *        姿态角，供底盘控制使用（Control_Task 里读 Pitch/Roll 做坡度补偿）。
 *
 * 处理流程（每 1ms 循环一次）：
 *   1. BMI088_Info_Update：读原始加速度/角速度/温度
 *   2. 加速度过二阶低通滤波（去高频振动噪声）
 *   3. QuaternionEKF_Update：四元数 EKF 融合角速度 + 加速度，估计姿态
 *   4. 把欧拉角换算成角度(deg)，并做 yaw 的「跨圈累计」（±180° 环绕处理）
 *   5. 每 5ms 做一次 BMI088 温度控制（加热/保温，保证零偏稳定）
 *
 * ===== 调车关注 =====
 *  - 姿态漂/抖：先看 LPF2p 系数（INS_LPF2p_Alpha）和 EKF 参数（Q/P 矩阵）
 *  - BMI088 温控目标 40℃（BMI088_Temp_Control 里写死），温度不稳会影响零偏
 *  - yaw 角度是「累计圈数」的连续值，跨 ±180° 会累积 YawRoundCount
 * ============================================================================
 */
#include "cmsis_os.h"
#include "INS_Task.h"
#include "bmi088.h"
#include "lpf.h"
#include "pid.h"
#include "config.h"
#include "tim.h"
#include "Quaternion.h"
#include "bsp_pwm.h"

/**
  * @brief the structure that contains the information for the INS.
  * @note  IMU 姿态信息结构体（全局）。INS_Task 每 1ms 更新它的角度/角速度，
  *        Control_Task 通过外部引用读取 Pitch/Roll/Yaw 做坡度补偿。
  */
INS_Info_Typedef INS_Info; 

/**
  * @brief the array that contains the data of LPF2p coefficients.
  * @note  加速度二阶低通滤波系数 [b0, b1, b2]（对应截止频率的离散化系数），
  *        三个轴共用这一套。滤波越狠（截止越低）姿态越稳但越滞后。
  */
static float INS_LPF2p_Alpha[3]={1.929454039488895f, -0.93178349823448126f, 0.002329458745586203f};

/**
  * @brief the structure that contains the Information of accel LPF2p.
  * @note  三个轴（x/y/z）的加速度二阶低通滤波器句柄数组。
  */
LowPassFilter2p_Info_TypeDef INS_AccelPF2p[3];  

/**
  * @brief the Initialize data of state transition matrix.
  * @note  EKF 状态转移矩阵 A 的初值（6x6 单位阵），QuaternionEKF_Init 会使用。
  */
static float QuaternionEKF_A_Data[36]={1, 0, 0, 0, 0, 0,
                                       0, 1, 0, 0, 0, 0,
                                       0, 0, 1, 0, 0, 0,
                                       0, 0, 0, 1, 0, 0,
                                       0, 0, 0, 0, 1, 0,
                                       0, 0, 0, 0, 0, 1};

/**
  * @brief the Initialize data of posteriori covariance matrix.
  * @note  EKF 后验协方差矩阵 P 的初值（6x6）。对角元素大 = 初始估计不确定度高，
  *        让滤波器一开始更「相信」测量值，快速收敛。
  */																
static float QuaternionEKF_P_Data[36]= {100000, 0.1, 0.1, 0.1, 0.1, 0.1,
                                        0.1, 100000, 0.1, 0.1, 0.1, 0.1,
                                        0.1, 0.1, 100000, 0.1, 0.1, 0.1,
                                        0.1, 0.1, 0.1, 100000, 0.1, 0.1,
                                        0.1, 0.1, 0.1,   0.1,  100, 0.1,
                                        0.1, 0.1, 0.1,   0.1,  0.1, 100};

/**
  * @brief the Initialize data of Temperature Control PID.
  * @note  温控 PID 参数 {KP, KI, KD, ...}，目标温度 40℃（见 BMI088_Temp_Control）。
  */																				
static float TemCtrl_PID_Param[PID_PARAMETER_NUM]={1200,20,0,0,0,0,2000};   /* 温控 PID 参数：KP=1200 KI=20，其余 0，输出限幅 2000 */

/**
  * @brief the structure that contains the Information of Temperature Control PID.
  * @note  BMI088 温控 PID 句柄，PID_Calculate 的输出经限幅后送给加热电路。
  */
PID_Info_TypeDef TempCtrl_PID;

/**
 * @brief Initializes the INS_Task.
 * @note  INS_Task 初始化函数的前向声明（实现在下方）。
 */
static void INS_Task_Init(void);

/**
  * @brief  Control the BMI088 temperature
  */
static void BMI088_Temp_Control(float temp);

/* USER CODE BEGIN Header_INS_Task */
/**
  * @brief  Function implementing the StartINSTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_INS_Task */
/**
 * @brief IMU 姿态解算任务主函数（FreeRTOS 任务入口）
 * @note  每 1ms 循环一次：读 BMI088 → 加速度低通滤波 → EKF 解算 → 输出欧拉角。
 *        每 5ms 做一次温度控制。最后 osDelay(1) 形成 1kHz 解算周期。
 * @param argument 未使用
 * @retval 无
 */
void INS_Task(void const * argument)
{
  /* USER CODE BEGIN INS_Task */
  TickType_t INS_Task_SysTick = 0;


	/* 初始化 INS_Task：低通滤波 + 温控 PID + 四元数 EKF */
	INS_Task_Init();
	
  /* 无限循环（每 1ms 解算一次） */
  for(;;)
  {
    INS_Task_SysTick = osKernelSysTick();
		
		/* 更新 BMI088 原始测量数据（加速度/角速度/温度） */
    BMI088_Info_Update(&BMI088_Info);

    /* 加速度过二阶低通滤波（去高频振动） */
    INS_Info.Accel[0]   =   LowPassFilter2p_Update(&INS_AccelPF2p[0],BMI088_Info.Accel[0]);
    INS_Info.Accel[1]   =   LowPassFilter2p_Update(&INS_AccelPF2p[1],BMI088_Info.Accel[1]);
    INS_Info.Accel[2]   =   LowPassFilter2p_Update(&INS_AccelPF2p[2],BMI088_Info.Accel[2]);
		
    /* 更新陀螺仪角速度（单位 rad/s） */
		INS_Info.Gyro[0]   =   BMI088_Info.Gyro[0];
    INS_Info.Gyro[1]   =   BMI088_Info.Gyro[1];
    INS_Info.Gyro[2]   =   BMI088_Info.Gyro[2];
		
		/* 四元数 EKF 更新：融合角速度 + 加速度估计姿态（步长 1ms） */
    QuaternionEKF_Update(&Quaternion_Info,INS_Info.Gyro,INS_Info.Accel,0.001f);
		
    memcpy(INS_Info.Angle,Quaternion_Info.EulerAngle,sizeof(INS_Info.Angle));

		/* 把欧拉角换算成角度(deg) */
    INS_Info.Pitch_Angle = Quaternion_Info.EulerAngle[IMU_ANGLE_INDEX_PITCH]*57.295779513f;
    INS_Info.Yaw_Angle   = Quaternion_Info.EulerAngle[IMU_ANGLE_INDEX_YAW]   *57.295779513f;
    INS_Info.Roll_Angle  = Quaternion_Info.EulerAngle[IMU_ANGLE_INDEX_ROLL]*57.295779513f;
		
		/* 更新 yaw 总角度：跨过 ±180° 时圈数 ±1，得到连续的总角度 */
		if(INS_Info.Yaw_Angle - INS_Info.Last_Yaw_Angle < -180.f)
		{
			INS_Info.YawRoundCount++;
		}
		else if(INS_Info.Yaw_Angle - INS_Info.Last_Yaw_Angle > 180.f)
		{
			INS_Info.YawRoundCount--;
		}
		INS_Info.Last_Yaw_Angle = INS_Info.Yaw_Angle;
		
		INS_Info.Yaw_TolAngle = INS_Info.Yaw_Angle + INS_Info.YawRoundCount*360.f;
		
    /* 更新陀螺仪角速度（单位 deg/s） */
    INS_Info.Pitch_Gyro = INS_Info.Gyro[IMU_GYRO_INDEX_PITCH]*RadiansToDegrees;
    INS_Info.Yaw_Gyro   = INS_Info.Gyro[IMU_GYRO_INDEX_YAW]*RadiansToDegrees;
    INS_Info.Roll_Gyro  = INS_Info.Gyro[IMU_GYRO_INDEX_ROLL]*RadiansToDegrees;
		
		if(INS_Task_SysTick%5 == 0)   /* 每 5ms 做一次温度控制 */
		{
			BMI088_Temp_Control(BMI088_Info.Temperature);
		}

    osDelay(1);
		
  }
  /* USER CODE END INS_Task */
}
//------------------------------------------------------------------------------
/**
 * @brief 初始化 INS_Task 用到的滤波器和 EKF
 * @note  依次初始化：三个轴的加速度二阶低通滤波、温度控制 PID、四元数 EKF。
 *        EKF 参数（过程噪声/初始 P 等）见 QuaternionEKF_Init 调用。
 * @param 无
 * @retval 无
 */
static void INS_Task_Init(void)
{
  /* 初始化三轴加速度的二阶低通滤波器 */
  LowPassFilter2p_Init(&INS_AccelPF2p[0],INS_LPF2p_Alpha);
  LowPassFilter2p_Init(&INS_AccelPF2p[1],INS_LPF2p_Alpha);
  LowPassFilter2p_Init(&INS_AccelPF2p[2],INS_LPF2p_Alpha);
	
  /* 初始化温度控制 PID（位置式） */
	PID_Init(&TempCtrl_PID,PID_POSITION,TemCtrl_PID_Param);
	
  /* 初始化四元数 EKF（初始不确定度/噪声参数） */
	QuaternionEKF_Init(&Quaternion_Info,10.f, 0.001f, 1000000.f,QuaternionEKF_A_Data,QuaternionEKF_P_Data);
}
//------------------------------------------------------------------------------
/**
  * @brief  控制 BMI088 温度（加热/保温，保证陀螺零偏稳定）
  * @note   目标温度 40℃，PID 输出经限幅后送给加热电路。
  * @param  Temp BMI088 测得的当前温度 (℃)
  * @retval 无
  */
static void BMI088_Temp_Control(float Temp)
{
	PID_Calculate(&TempCtrl_PID,40.f,Temp);   /* 目标 40℃ 的温控 PID */
	
	VAL_LIMIT(TempCtrl_PID.Output,-TempCtrl_PID.Param.LimitOutput,TempCtrl_PID.Param.LimitOutput);   /* 输出限幅 */

	Heat_Power_Control((uint16_t)(TempCtrl_PID.Output));   /* 输出到加热电路（PWM） */
}
//------------------------------------------------------------------------------


