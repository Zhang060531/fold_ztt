/**
 ******************************************************************************
 * @file           : Control_Task.h
 * @brief          : Control task (Swerve Chassis Integrated Version with RLS & Offline Check)
 ******************************************************************************
 */

/* ============================================================================
 * 文件说明：底盘控制任务头文件 (Control_Task.h)
 * ----------------------------------------------------------------------------
 * 作用：声明底盘控制相关的结构体、宏常量、以及函数接口，是 Control_Task.c 的
 *        头文件，也被 CAN_Task.c 等引用。
 *
 * 核心数据结构：
 *  - Chassis_Info_Typedef：底盘控制核心结构体，存 4 个轮毂电机的反馈/设定、
 *    云台发来的数据、超级电容数据、裁判系统数据、功率控制中间量等。
 *  - Gimbal_Info_Typedef  ：云台通过 CAN 发来的状态（yaw/pitch/roll、模式标志等）。
 *  - Power_Ctrl_Typedef   ：功率控制用的系数与中间量。
 *  - Time_t               ：各数据源时间戳（用于超时/丢帧判断）。
 *
 * 与云台的关系：底盘不直接拿遥控器，而是接收云台通过 CAN 0x21 帧发来的
 *  X/Y/Z 三个速度设定（chassis_xyz_set），再由底盘做轮速解算 + 功率限制。
 *
 * ===== 调车关注 =====
 *  - 底盘速度环 PID 参数：CHASSIS_MOTOR_PID_* 系列宏，改轮速跟随的响应。
 *  - 麦轮速度解算：WHEEL_PERIMETER(轮周长mm)、CHASSIS_DECELE_RATIO(减速比)、
 *    Radius(轮距相关) 三个常量直接影响解算出的轮速目标值。
 *  - 功率限制：LAMBDA_*、POWER_TOLERANCE、MAX_CMD_CURRENT 等是拉格朗日功率
 *    优化分配的参数，一般不要动，除非在做功率管理调优。
 * ============================================================================
 */
#ifndef CONTROL_TASK_H
#define CONTROL_TASK_H

#include "stdint.h"
#include "stdbool.h"
#include "Motor.h"
#include "Referee_System.h"
#include "bsp_can.h"
#include "arm_math.h"
#include "RLS.h"


/* 电机转速(RPM) → 轮子线速度(m/s) 的换算系数。
 * 在 FdbUpdate 里用来把 M3508 反馈的 Velocity(RPM) 换算成 m/s 再做低通滤波，
 * 是经验标定值（含减速比换算，具体见 Mecanum_Calc 的 wheel_rpm_ratio 反推）。 */
#define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f


/* 缓冲能量相关阈值（单位 J，来自裁判系统 buffer_energy）。
 * 注意：这三个宏在 PowerLimit() 里没有直接引用，属于功率管理的备用/遗留定义。 */
#define MAX_POWER_BUFFER 60      /* 缓冲能量上限（单位：J） */ 
#define POWER_BUFFER_URGENT 18   /* 紧急缓冲能量阈值 */
#define MAX_TOTAL_CURRENT_SET 20000  /* 总电流设定上限 */


/* ===== 拉格朗日功率优化分配参数（PowerLimit 用）=====
 * 底盘预测功率超过上限时，用拉格朗日乘子法给 4 个轮毂电机分配电流，
 * 使总功率不超限。下面这些是迭代求解用的边界/容差参数：
 *  - MAX_CMD_CURRENT / MIN_CMD_CURRENT：单电机电流设定值的最大/最小限幅
 *    （即 ±16384，对应 C620 电调的电流指令满量程）
 *  - LAMBDA_INITIAL_UPPER_BOUND：λ 上界的初始猜测值
 *  - LAMBDA_UPPER_BOUND_MAX_ITER：λ 上界「扩张」的最大迭代次数
 *  - LAMBDA_UPPER_BOUND_STEP：λ 上界每次扩张的倍数
 *  - LAMBDA_MAX_ITER：二分法求 λ 的最大迭代次数
 *  - POWER_TOLERANCE：功率收敛容差 (W)，当前功率与目标差小于它就认为收敛
 */
#define MAX_CMD_CURRENT 16384.0f
#define MIN_CMD_CURRENT -16384.0f
#define LAMBDA_INITIAL_UPPER_BOUND 0.05f
#define LAMBDA_UPPER_BOUND_MAX_ITER 10
#define LAMBDA_UPPER_BOUND_STEP 10.0f
#define LAMBDA_MAX_ITER 30
#define POWER_TOLERANCE 0.8f


/* 底盘速度三个分量在数组里的下标：X=前后平移，Y=左右平移，Z=绕自身旋转 */
#define CHASSIS_X 0
#define CHASSIS_Y 1
#define CHASSIS_Z 2

/* 麦轮速度解算相关常量：
 *  - WHEEL_PERIMETER    ：轮子周长 (mm)
 *  - CHASSIS_DECELE_RATIO：底盘减速比（电机转速/轮子转速，3591/187≈19.2）
 *  - Radius             ：轮距相关的几何系数（用于旋转分量的折算） */
#define WHEEL_PERIMETER 180
#define CHASSIS_DECELE_RATIO 3591.f/187.f
#define Radius 2.3961f


/* 底盘 4 个轮毂电机共用的「速度环」PID 参数（PID_Calculate 用）：
 *  KP=比例  KI=积分  KD=微分；MAX_IOUT=积分限幅  MAX_OUT=输出限幅。
 *  注意：这是速度环（设定=目标转速 RPM，反馈=实际转速 RPM，输出=电流指令）。 */
#define CHASSIS_MOTOR_PID_KP 14
#define CHASSIS_MOTOR_PID_KI 0
#define CHASSIS_MOTOR_PID_KD 0
#define CHASSIS_MOTOR_PID_MAX_IOUT 100
#define CHASSIS_MOTOR_PID_MAX_OUT 18000

/**
 * @brief 底盘朝向枚举：表示当前「车头」朝哪个方向
 * @note  用于 INS_Angle_Calc 里把 IMU 的 pitch/roll 换算成底盘坐标系的俯仰/横滚。
 *        云台通过 CAN 帧下发 Chassis_Head，告诉底盘云台正对着车头还是车尾。
 *        LEFT/RIGHT 为预留值，本工程未使用。
 */
typedef enum
{
    HEAD,   /* 车头方向 */
    TAIL,   /* 车尾方向 */
    LEFT,   /* 左侧（预留，未使用） */
    RIGHT   /* 右侧（预留，未使用） */
}Head_fdb;

/**
 * @brief 时间戳结构体：记录各数据源最后一次收到数据的时间
 * @note  用「本次-上次」的时间差判断数据是否超时/丢帧（TimeErrorCalc 计算）。
 *        电机的离线检测在 Motor.c 的 Check_All_Motors_Offline 里做。
 */
typedef struct
{
    uint32_t motor[4];      /* 4 个底盘电机的反馈时间戳 */
    uint32_t yaw;           /* yaw 达妙电机反馈时间戳 */
    uint32_t speed_set[3];  /* X/Y/Z 速度设定帧时间戳 */
    uint32_t ree_power;     /* 超级电容功率数据时间戳 */
    uint32_t ree_statue;    /* 裁判系统状态时间戳 */
} Time_t;

/**
 * @brief 云台发来的状态数据（通过 CAN 0x22 帧接收，见 bsp_can.c 的 GiambalData）
 * @note  云台把它的工作状态/朝向打包发给底盘，底盘据此判断车头朝向、是否在
 *        视觉模式等。rotate_state 等是一组 bit 标志位（对应云台 TransmitGimbal_Info
 *        里 Data[0] 的各位）。
 */
typedef struct
{
    CANFrameInfo_typedef FDCANFrame;        /* CAN 收发 ID 信息 */
    uint8_t rotate_state;                   /* bit0：底盘旋转模式标志 */
    uint8_t follow_state;                   /* bit1：底盘跟随模式标志 */
    uint8_t zero_force_state;               /* bit2：云台零力模式标志 */
    uint8_t shoot_ready_state;              /* bit3：发射就绪标志 */
    uint8_t shoot_normal_state;             /* bit4：发射中标志 */
    uint8_t vision_robot_state;             /* bit5：视觉有目标标志 */
    uint8_t vision_shoot_state;             /* bit6：视觉开火标志 */
    uint8_t reset_flag;                     /* bit7：云台复位标志 */
    uint8_t CAP_Flag;                       /* 超级电容使能标志 */
    Head_fdb Chassis_Head;                  /* 云台朝向：车头/车尾 */

    float   pitch;                          /* 云台 pitch 角 (deg) */
    float   roll;                           /* 云台 roll 角 (deg) */
    float   yaw;                            /* 云台 yaw 角 (deg) */
    int16_t current_set;                    /* 云台要求的拨弹(发射)电机电流设定 */
    int16_t reserved_int;                   /* 预留 */
} Gimbal_Info_Typedef;

/**
 * @brief 功率控制结构体：存功率模型系数与拉格朗日分配的中间量
 * @note  PowerLimit() 里把每个轮子的功率近似为 i(电流) 和 w(转速) 的二次多项式：
 *        P = K1*i² + K2*w² + K3*i*w + K4/4，K1~K4 是实测拟合的功率模型系数。
 *        Err/Menbership/Sum 等是分配用的中间量（部分未使用）。
 */
typedef struct
{
    float K;            /* 系数 K（未使用） */
    float Err_Upper;    /* 误差上界 */
    float Err_Lower;    /* 误差下界 */
    float Power_Max;    /* 功率上限 */
    float Power_Allin;  /* 全力功率 */

    float K1;           /* 功率模型系数：电流二次项 (i²) */
    float K2;           /* 功率模型系数：转速二次项 (w²) */
    float K3;           /* 功率模型系数：电流×转速交叉项 (i*w) */
    float K4;           /* 功率模型常数项（4 轮空载损耗和，P=K1i²+K2w²+K3iw+K4/4） */

    float Err[4];         /* 各电机误差 */
    float Power_Limit[4]; /* 各电机功率限幅 */
    float Menbership[4];  /* 隶属度（模糊逻辑用，未使用） */

    struct {
        float Err_Sum;        /* 误差累加 */
        float Power_Sum;      /* 功率累加 */
        float I2_Sum;         /* i² 累加 */
        float W2_Sum;         /* w² 累加 */
        float Mech_Power_Sum; /* 机械功率累加 */
    } Sum;
} Power_Ctrl_Typedef;

/**
 * @brief 底盘控制核心结构体：存底盘所有反馈值/设定值/中间量
 * @note  这是底盘最核心的全局变量 chassis（定义在 Control_Task.c）。成员分几类：
 *         - 电机反馈：chassis_motor_fdb[4]（4 个 M3508 轮毂）、yaw_motor_fdb（达妙）、
 *           sht_motor_fdb（拨弹）
 *         - 电机设定：chassis_motor_set[4]（速度目标 speed_rpm_set、电流输出 current_set）
 *         - super_power：超级电容数据（电压/功率）
 *         - chassis_xyz_set[3]：云台发来的 X/Y/Z 速度设定（m/s 和 rad/s）
 *         - gimbal_data/ref_data：云台状态、裁判系统数据
 *         - Slope：坡度补偿用的三角函数与系数
 *         - PowerCtrl[2]：功率控制系数（只用 [1]）
 */
typedef struct
{
    CANFrameInfo_typedef FDCANFrame;    /* CAN 收发 ID 信息（接收 0x21 速度设定帧） */
    struct Angle_t
    {
        float Pitch;    /* 底盘等效俯仰角 (deg)，由 IMU 按车头朝向换算 */
        float Roll;     /* 底盘等效横滚角 (deg) */
    }Angle;
    DJI_Motor_Info_Typedef chassis_motor_fdb[4];   /* 4 个 M3508 轮毂电机反馈 */
    DM_Motor_Info_Typedef yaw_motor_fdb;           /* 达妙 DM4310 yaw 电机反馈 */
    DJI_Motor_Info_Typedef sht_motor_fdb;          /* 拨弹(发射)电机反馈 */

    DJI_Motor_Contorl_Info_Typedef chassis_motor_set[4];  /* 4 个轮毂电机设定 */
    SuperPower_Info_Typedef super_power;           /* 超级电容数据 */
    int super_power_flag;                          /* 超电状态标志 */
    int reset_flag;                                /* 复位标志 */
    uint32_t chassis_speed_set_time[3];            /* X/Y/Z 速度设定的时间戳 */
    float chassis_xyz_set[3];                      /* 云台发来的 X/Y/Z 速度设定 */
    float chassis_xyz_set_f[3];                    /* 坐标系旋转后的 X/Y/Z 设定（底盘坐标系） */

    Gimbal_Info_Typedef gimbal_data;               /* 云台状态数据 */
    Referee_System_Info_TypeDef ref_data;          /* 裁判系统数据 */

    float ChassisSpeedFdb[3];                      /* 底盘 X/Y/Z 实际速度反馈（滤波后） */

    Time_t time_now;       /* 本次时间戳 */
    Time_t time_last;      /* 上次时间戳 */
    Time_t time_error;     /* 时间差（用于判断丢帧） */

    struct {
        float Cos[2];      /* 坡度补偿：Pitch/Roll 的余弦 */
        float Sin[2];      /* 坡度补偿：Pitch/Roll 的正弦 */
        float K_angle[4];  /* 坡度补偿系数（分配到 4 个轮子） */
    } Slope;

    Power_Ctrl_Typedef PowerCtrl[2];   /* 功率控制系数（本工程只用 [1]） */
} Chassis_Info_Typedef;

/**
 * @brief 发射机构信息结构体（接收 0x1ff 拨弹电机反馈用）
 */
typedef struct
{
    CANFrameInfo_typedef FDCANFrame;                /* CAN 收发 ID 信息 */
    DJI_Motor_Contorl_Info_Typedef Shoot_Info;      /* 发射电机设定 */
} Shoot_Info_Typedef;

/* 底盘控制核心全局变量（Control_Task.c 中定义） */
extern Chassis_Info_Typedef chassis;

/* 各电机反馈获取函数：从 Motor.c 的全局电机结构体拷贝反馈，返回该电机的更新时间戳 */
extern uint32_t GetChassisMotor1Fdb(DJI_Motor_Info_Typedef * motor);
extern uint32_t GetChassisMotor2Fdb(DJI_Motor_Info_Typedef * motor);
extern uint32_t GetChassisMotor3Fdb(DJI_Motor_Info_Typedef * motor);
extern uint32_t GetChassisMotor4Fdb(DJI_Motor_Info_Typedef * motor);
extern uint32_t GetShootFdb(DJI_Motor_Info_Typedef * motor);

extern uint32_t GetYawMotorFdb(DM_Motor_Info_Typedef* motor);
/* 从云台发来的 0x21 帧读取 X/Y/Z 速度设定 */
extern uint32_t GetChassisXSpeedSet(float* set);
extern uint32_t GetChassisYSpeedSet(float* set);
extern uint32_t GetChassisZSpeedSet(float* set);
extern void SentShootToGimbal(FDCAN_TxFrame_TypeDef *FDCAN_TxFrame, uint8_t *Rxdata);

/* 云台数据/超电数据拷贝函数（bsp_can.c 中实现） */
void GetGimbalData(Gimbal_Info_Typedef* data);
void GetSuperData(SuperPower_Info_Typedef* data);
/* 把发射数据打包成 CAN 帧发给云台 */
extern void Transmit_Shoot_Info(FDCAN_TxFrame_TypeDef *FDCAN_TxFrame, float speed, int16_t heat,int16_t heatlimit,int16_t shooter_barrel_cooling_value);

void ChassisInit(void);                                              /* 底盘初始化：PID 参数、功率系数 */
void FdbUpdate(Chassis_Info_Typedef* chassis);                       /* 读取所有反馈数据 */
void Slope_Compensation_Update(Chassis_Info_Typedef* chassis);       /* 坡度补偿系数更新 */
void PowerLimit(Chassis_Info_Typedef* chassis_power_control);        /* 功率限幅核心算法 */
void TimeErrorCalc(Chassis_Info_Typedef* chassis);                   /* 计算各数据源时间差 */
float filter(float temp, float alpha);                               /* 一阶低通滤波 */
void Absolute_Calc(Chassis_Info_Typedef* absolute_speed, float angle);/* 坐标系旋转 + 麦轮解算入口 */
/* 注意：下面 AGV_* / Find_min_Angle / AngleLoop_* 是头文件里的遗留声明，
 * Control_Task.c 中并没有实现（实际用的是 Mecanum_Calc），属于死声明，勿依赖。 */
void AGV_angle_calc(Chassis_Info_Typedef *speed);
void AGV_calc(Chassis_Info_Typedef *speed);
void CHASSIS_Single_Loop_Out(void);                                  /* 底盘速度环 PID 输出电流 */
float32_t Find_min_Angle(float32_t angle1, float32_t angle2);
void AngleLoop_f(float *angle, float max_angle);
void AngleLoop_int(int32_t *angle, int32_t max_angle);

#endif //CONTROL_TASK_H
