
/**
 ******************************************************************************
 * @file    Bmi088.h
 * @brief   BMI088 六轴 IMU（惯性测量单元）驱动头文件
 *
 * 【设备是什么】
 * BMI088 是 Bosch 公司的一款高性能六轴惯性传感器，芯片内部集成了两个独立传感器：
 *   - 三轴加速度计（Accelerometer）：测量线性加速度，量程可选 ±3g / ±6g / ±12g / ±24g；
 *   - 三轴陀螺仪（Gyroscope）：测量旋转角速度（角速率），量程可选 ±125 ~ ±2000 dps。
 * 云台的姿态解算（俯仰/偏航/滚转角）正是靠「陀螺仪角速度积分 + 加速度计提供重力参考」实现，
 * 它是云台控制闭环里最核心的反馈传感器。
 *
 * 【通过什么接口连接】
 * SPI（4 线）：SCK + MOSI + MISO + 片选(CS/NSS)。
 * 加速度计和陀螺仪是芯片里两个独立的 SPI 从设备，共用同一组 SPI 总线，
 * 靠不同的片选引脚（BMI088_ACCEL_NS / BMI088_GYRO_NS，见 Bmi088.c）区分。
 *
 * 【数据协议大概长什么样】
 * BMI088 是「寄存器映射」型设备：向 SPI 总线发送寄存器地址，再读写数据。
 *   - 写寄存器：先发地址（最高位为 0），再发要写入的数据；
 *   - 读寄存器：先发「地址 | 0x80」（最高位置 1 表示读操作），随后连续发送无效字节(0x55)，
 *     同时从 MISO 引脚读回数据。
 * 三轴原始数据各占 16bit（有符号整数，小端：低字节在前），
 * 乘以灵敏度系数(SEN)后换算成 g（加速度）或 rad/s（角速度）。寄存器定义见 Bmi088_Reg.h。
 *
 * 【被谁调用】
 *   - BMI088_Init()：上电初始化时调用一次（配置量程/带宽/中断、使能传感器、标定零偏）；
 *   - BMI088_Info_Update()：由 1kHz 周期任务循环调用，读取并换算最新 IMU 数据。
 ******************************************************************************
 */
#ifndef DEVICE_BMI088_H
#define DEVICE_BMI088_H

#include "bmi088_reg.h"
#include "stdint.h"
#include "stdbool.h"

/* 温度换算：摄氏度 = 原始温度值 × 0.125 + 23.0，见 Bmi088.c 中的换算代码 */
#define BMI088_TEMP_FACTOR 0.125f
#define BMI088_TEMP_OFFSET 23.0f

#define BMI088_WRITE_ACCEL_REG_NUM 6
#define BMI088_WRITE_GYRO_REG_NUM 6

#define BMI088_GYRO_DATA_READY_BIT 0
#define BMI088_ACCEL_DATA_READY_BIT 1
#define BMI088_ACCEL_TEMP_DATA_READY_BIT 2

#define BMI088_LONG_DELAY_TIME 80
#define BMI088_COM_WAIT_SENSOR_TIME 150

#define BMI088_ACCEL_IIC_ADDRESSE (0x18 << 1)
#define BMI088_GYRO_IIC_ADDRESSE (0x68 << 1)

/* 加速度计灵敏度系数(SEN)：16bit 原始值 × SEN = 加速度(单位 g，1g ≈ 9.8m/s²)。
 * 例：±6g 量程时，满量程 32768 对应 6g，故 SEN = 6 / 32768 ≈ 0.0017944。
 * 量程越大 SEN 越大（同等原始值代表更大的加速度）。 */
#define BMI088_ACCEL_3G_SEN 0.0008974358974f
#define BMI088_ACCEL_6G_SEN 0.00179443359375f
#define BMI088_ACCEL_12G_SEN 0.0035888671875f
#define BMI088_ACCEL_24G_SEN 0.007177734375f

/* 陀螺仪灵敏度系数(SEN)：16bit 原始值 × SEN = 角速度(单位 rad/s)。
 * 例：±2000 dps 量程，先换算成弧度 2000 × π / 180 ≈ 34.9066 rad，除以 32768 ≈ 0.0010653。
 * 注意陀螺仪原始值单位是 dps(度/秒)，SEN 已把它折算成 rad/s。 */
#define BMI088_GYRO_2000_SEN 0.00106526443603169529841533860381f
#define BMI088_GYRO_1000_SEN 0.00053263221801584764920766930190693f
#define BMI088_GYRO_500_SEN 0.00026631610900792382460383465095346f
#define BMI088_GYRO_250_SEN 0.00013315805450396191230191732547673f
#define BMI088_GYRO_125_SEN 0.000066579027251980956150958662738366f

/* BMI088_Status_e：初始化/自检过程返回的状态码。
 * 初始化成功返回 0(BMI088_NO_ERROR)；某一步配置失败则返回对应错误码，
 * 方便定位是「量程配置失败」「带宽配置失败」还是「芯片没接好(BMI088_NO_SENSOR)」。 */
typedef enum
{
    BMI088_NO_ERROR                     = 0x00,
    BMI088_ACCEL_PWR_CTRL_ERROR           = 0x01,
    BMI088_ACCEL_PWR_CONF_ERROR           = 0x02,
    BMI088_ACCEL_CONF_ERROR               = 0x03,
    BMI088_ACCEL_SELF_TEST_ERROR          = 0x04,
    BMI088_ACCEL_RANGE_ERROR              = 0x05,
    BMI088_INT1_IO_CTRL_ERROR           = 0x06,
    BMI088_INT_MAP_DATA_ERROR           = 0x07,
    BMI088_GYRO_RANGE_ERROR             = 0x08,
    BMI088_GYRO_BANDWIDTH_ERROR         = 0x09,
    BMI088_GYRO_LPM1_ERROR              = 0x0A,
    BMI088_GYRO_CTRL_ERROR              = 0x0B,
    BMI088_GYRO_INT3_INT4_IO_CONF_ERROR = 0x0C,
    BMI088_GYRO_INT3_INT4_IO_MAP_ERROR  = 0x0D,

    BMI088_SELF_TEST_ACCEL_ERROR        = 0x80,
    BMI088_SELF_TEST_GYRO_ERROR         = 0x40,
    BMI088_NO_SENSOR                    = 0xFF,
}BMI088_Status_e;

typedef struct
{
	int16_t Accel_X;   
	int16_t Accel_Y;   
	int16_t Accel_Z;   

	int16_t Gyro_X;   
	int16_t Gyro_Y;  
	int16_t Gyro_Z;    

	int16_t Temperature; 
	
}MPU_Info_Typedef;

/* MPU_Info_Typedef：存放从寄存器直接读回的 16bit「原始值」。
 * 这些值还没有乘灵敏度系数、也没有减去零偏，是芯片输出的最原始数据。
 * Accel_X/Y/Z = 三轴加速度原始值；Gyro_X/Y/Z = 三轴角速度原始值；Temperature = 温度原始值。 */


typedef struct
{
    bool Offsets_Init;   

    float Accel[3];       
    float Gyro[3];       
    float Temperature;   

    MPU_Info_Typedef MPU_Info;

    float Offsets_Gyro_X;   
    float Offsets_Gyro_Y;   
    float Offsets_Gyro_Z;   

}BMI088_Info_Typedef;
/* BMI088_Info_Typedef：BMI088 最终对外输出的数据结构。
 *   - Offsets_Init：零偏是否已标定完成（标定后为 true）；
 *   - Accel[3] / Gyro[3]：换算成物理量(单位 g / rad/s)且已减去零偏的最终数据；
 *   - Temperature：摄氏温度；
 *   - MPU_Info：对应的原始 16bit 值（调试用）；
 *   - Offsets_Gyro_X/Y/Z：陀螺仪三轴零偏（静止时陀螺仪应输出 0，实际会有微小漂移，需标定补偿）。 */

extern BMI088_Info_Typedef BMI088_Info;

extern void BMI088_Init(void);

extern void BMI088_Info_Update(BMI088_Info_Typedef *BMI088_Info);


#endif