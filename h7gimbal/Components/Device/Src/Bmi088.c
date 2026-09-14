/**
 ******************************************************************************
 * @file    Bmi088.c
 * @brief   BMI088 六轴 IMU 驱动实现（SPI 通信）
 *
 * 【本文件做什么】
 * 实现 BMI088 的初始化（BMI088_Init）与数据读取换算（BMI088_Info_Update）。
 * 底层通过宏把「加速度计/陀螺仪各自拉低片选 -> 收发 SPI -> 拉高片选」封装成一次寄存器读写。
 *
 * 【SPI 访问时序（新手必读）】
 * BMI088 加速度计与陀螺仪是同一颗芯片里的两个 SPI 从设备，共用 SCK/MOSI/MISO，
 * 通过两个片选引脚区分：BMI088_ACCEL_NS(加速度计)、BMI088_GYRO_NS(陀螺仪)。
 * 每次访问流程：
 *   1) 拉低对应片选（选中设备）；
 *   2) 先发一个字节的寄存器地址（读操作时该字节最高位置 1，即 |0x80）；
 *   3) 写入时紧接着发数据字节；读取时连续发无效字节(0x55)，同时把 MISO 上的数据读回来；
 *   4) 拉高片选（结束本次访问）。
 * 详细字节协议见 Bmi088.h 头部的说明。
 *
 * 【数据换算】
 * 读回的 16bit 原始值 = 高字节<<8 | 低字节（小端）。再 × 灵敏度系数(SEN)得到物理量：
 *   加速度 → g；角速度 → rad/s；温度 → ℃。角速度还要减去标定得到的零偏(Offsets)。
 ******************************************************************************
 */
#include "Bmi088.h"
#include "bsp_gpio.h"
#include "bsp_spi.h"
#include "bsp_tick.h"

/* 底层 SPI 寄存器读写函数声明（均为静态函数，只在本文件内使用） */
static void BMI088_Write_Single_Reg(uint8_t, uint8_t);

static void BMI088_Read_Single_Reg(uint8_t, uint8_t *);

static void BMI088_Read_Multi_Reg(uint8_t, uint8_t *, uint8_t);

/* ==================== SPI 寄存器访问宏 ====================
 * 下面这些宏把「拉低片选 -> 读写 -> 拉高片选」打包成一行调用，避免重复代码。
 *   - Write_Single_Reg：写单个寄存器（加速度计/陀螺仪各一个）；
 *   - Read_Single_Reg ：读单个寄存器；
 *   - Read_Multi_Reg  ：连续读多个字节（用于一次读回 6/8 字节的三轴原始数据）。
 * 加速度计宏里的读操作手工拼了两次 0x55 无效字节 + 读回字节，原因见 BMI088_Read_Single_Reg。
 * 陀螺仪宏则直接调用通用静态函数 BMI088_Read_Single_Reg / BMI088_Read_Multi_Reg。
 * 注意：加速度计读多字节宏没有先发「地址|0x80」再读，而是交给 BMI088_Read_Multi_Reg 内部处理。 */
#define BMI088_Accel_Write_Single_Reg(reg, data) \
    {                                            \
        BMI088_ACCEL_NS_L();                     \
        BMI088_Write_Single_Reg((reg), (data));  \
        BMI088_ACCEL_NS_H();                     \
    }

#define BMI088_Accel_Read_Single_Reg(reg, data) \
    {                                           \
       BMI088_ACCEL_NS_L();                     \
       BMI088_Read_Write_Byte((reg) | 0x80);    \
       BMI088_Read_Write_Byte(0x55);            \
			 (data) = BMI088_Read_Write_Byte(0x55);   \
       BMI088_ACCEL_NS_H();                     \
    }
		
#define BMI088_Accel_Read_Multi_Reg(reg, data, len) \
    {                                               \
        BMI088_ACCEL_NS_L();                        \
        BMI088_Read_Write_Byte((reg) | 0x80);       \
        BMI088_Read_Multi_Reg(reg, data, len);      \
        BMI088_ACCEL_NS_H();                        \
    }

#define BMI088_Gyro_Write_Single_Reg(reg, data) \
    {                                           \
        BMI088_GYRO_NS_L();                     \
        BMI088_Write_Single_Reg((reg), (data)); \
        BMI088_GYRO_NS_H();                     \
    }

#define BMI088_Gyro_Read_Single_Reg(reg, data)  \
    {                                           \
        BMI088_GYRO_NS_L();                     \
        BMI088_Read_Single_Reg((reg), &(data)); \
        BMI088_GYRO_NS_H();                     \
    }

#define BMI088_Gyro_Read_Multi_Reg(reg, data, len)   \
    {                                                \
        BMI088_GYRO_NS_L();                          \
        BMI088_Read_Multi_Reg((reg), (data), (len)); \
        BMI088_GYRO_NS_H();                          \
    }

/* 当前实际使用的灵敏度系数：加速度计按 ±6g、陀螺仪按 ±2000 dps 量程。
 * 注意：这里必须与下面配置表里写入芯片的量程一致，否则换算出的物理量会错一个比例。 */
static float BMI088_ACCEL_SEN = BMI088_ACCEL_6G_SEN;

static float BMI088_GYRO_SEN = BMI088_GYRO_2000_SEN;
		
BMI088_Info_Typedef BMI088_Info;
		
/* 加速度计初始化配置表：每一行是 {寄存器地址, 写入值, 配置失败时的错误码}。
 * 依次完成：上电使能 -> 进入 Active 工作模式 -> 设采样率(1600Hz)与带宽 -> 设量程(±6g)
 *          -> 配置 INT1 中断引脚 -> 把「数据就绪」中断映射到 INT1 引脚。 */
static uint8_t Accel_Register_ConfigurationData_ErrorStatus[BMI088_WRITE_ACCEL_REG_NUM][3] =
{
    /* Turn on accelerometer */
    {BMI088_ACCEL_PWR_CTRL, BMI088_ACCEL_ENABLE_ACCEL_ON, BMI088_ACCEL_PWR_CTRL_ERROR},   

    /* Pause mode */
    {BMI088_ACCEL_PWR_CONF, BMI088_ACCEL_PWR_ACTIVE_MODE, BMI088_ACCEL_PWR_CONF_ERROR}, 

    /* ACCELeleration Configuration */
    {BMI088_ACCEL_CONF,  (BMI088_ACCEL_NORMAL| BMI088_ACCEL_1600_HZ | BMI088_ACCEL_CONF_MUST_Set), BMI088_ACCEL_CONF_ERROR}, 

    /* ACCELelerometer setting range */ 
    {BMI088_ACCEL_RANGE, BMI088_ACCEL_RANGE_6G, BMI088_ACCEL_RANGE_ERROR},  

    /* INT1 Configuration input and output pin */ 
    {BMI088_INT1_IO_CTRL, (BMI088_ACCEL_INT1_IO_ENABLE | BMI088_ACCEL_INT1_GPIO_PP | BMI088_ACCEL_INT1_GPIO_LOW), BMI088_INT1_IO_CTRL_ERROR}, 

    /* interrupt map pin */
    {BMI088_INT_MAP_DATA, BMI088_ACCEL_INT1_DRDY_INTERRUPT, BMI088_INT_MAP_DATA_ERROR}  
};		
		
/* 陀螺仪初始化配置表：每一行是 {寄存器地址, 写入值, 配置失败时的错误码}。
 * 依次完成：设量程(±2000dps) -> 设输出速率与带宽(2000dps/230Hz) -> 进入 Normal 工作模式
 *          -> 使能数据就绪中断 -> 配置 INT3 中断引脚 -> 把数据就绪中断映射到 INT3。 */
static uint8_t Gyro_Register_ConfigurationData_ErrorStatus[BMI088_WRITE_GYRO_REG_NUM][3] =
{
    /* Angular rate and resolution */
    {BMI088_GYRO_RANGE, BMI088_GYRO_2000, BMI088_GYRO_RANGE_ERROR}, 

    /* Data Transfer Rate and Bandwidth Settings */
    {BMI088_GYRO_BANDWIDTH, (BMI088_GYRO_2000_230_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set), BMI088_GYRO_BANDWIDTH_ERROR}, 

    /* Power Mode Selection Register */
    {BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE, BMI088_GYRO_LPM1_ERROR},   

    /* Data Interrupt Trigger Register */
    {BMI088_GYRO_CTRL, BMI088_DRDY_ON, BMI088_GYRO_CTRL_ERROR},   

    /* Interrupt Pin Trigger Register */
    {BMI088_GYRO_INT3_INT4_IO_CONF, (BMI088_GYRO_INT3_GPIO_PP | BMI088_GYRO_INT3_GPIO_LOW), BMI088_GYRO_INT3_INT4_IO_CONF_ERROR},  

    /* interrupt map register */
    {BMI088_GYRO_INT3_INT4_IO_MAP, BMI088_GYRO_DRDY_IO_INT3, BMI088_GYRO_INT3_INT4_IO_MAP_ERROR}   
};	

/**
 * @brief  初始化 BMI088 的加速度计子模块
 * @param  无
 * @retval BMI088_Status_e：成功返回 BMI088_NO_ERROR(0)，失败返回对应错误码
 * @note   流程：读芯片 ID 确认通信正常 -> 软件复位(写 0xB6 到 0x7E)并等待 80ms
 *         -> 再次读 ID 确认(加速度计 ID 应为 0x1E) -> 按配置表逐项写入寄存器，
 *         每写一项都回读校验，回读值不一致则返回该步的错误码。
 */
static BMI088_Status_e BMI088_Accel_Init(void)
{
    
    uint8_t res = 0;

    /* check the communication ------------------------------------------------*/
    /* read the accelerator ID address */
	  BMI088_Accel_Read_Single_Reg(BMI088_ACCEL_CHIP_ID, res);
    /* waiting 150us */
    Delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    /* read again */
    BMI088_Accel_Read_Single_Reg(BMI088_ACCEL_CHIP_ID, res);
    /* waiting 150us */
    Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    /* ACCELelerator software Reset ------------------------------------------------*/
    /* write 0xB6 to the register ACCEL_SOFTResET that addRess is 0x7E to reset software */
    BMI088_Accel_Write_Single_Reg(BMI088_ACCEL_SOFTRESET, BMI088_ACCEL_SOFTRESET_VALUE); 
    /* software reset waiting time, there is 80ms */
    Delay_ms(BMI088_LONG_DELAY_TIME);

    /* check the communication again ------------------------------------------------*/
    BMI088_Accel_Read_Single_Reg(BMI088_ACCEL_CHIP_ID, res);
    Delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_Accel_Read_Single_Reg(BMI088_ACCEL_CHIP_ID, res);
    Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    /* if the value of the 0x00 register is not 0x1E,reset failed */
    if (res != BMI088_ACCEL_CHIP_ID_VALUE)
    {
        return BMI088_NO_SENSOR;
    }

    /* config the accelerator sensor */
    for (uint8_t write_reg_num = 0; write_reg_num < BMI088_WRITE_ACCEL_REG_NUM; write_reg_num++)
    {

        BMI088_Accel_Write_Single_Reg(Accel_Register_ConfigurationData_ErrorStatus[write_reg_num][0], Accel_Register_ConfigurationData_ErrorStatus[write_reg_num][1]); 
        /* waiting 150us */
        Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        /* read the configuration */
        BMI088_Accel_Read_Single_Reg(Accel_Register_ConfigurationData_ErrorStatus[write_reg_num][0], res);
        /* waiting 150us */
        Delay_us(BMI088_COM_WAIT_SENSOR_TIME);
        
        /* check the configuration and return the specified error */
        if (res != Accel_Register_ConfigurationData_ErrorStatus[write_reg_num][1])
        {
            return (BMI088_Status_e)Accel_Register_ConfigurationData_ErrorStatus[write_reg_num][2];
        }
    }

    /* no error */
    return BMI088_NO_ERROR;  
}

/**
 * @brief  初始化 BMI088 的陀螺仪子模块
 * @param  无
 * @retval BMI088_Status_e：成功返回 BMI088_NO_ERROR(0)，失败返回对应错误码
 * @note   流程与加速度计类似：读 ID(陀螺仪 ID 应为 0x0F) -> 软件复位 -> 回读校验 -> 逐项写配置表。
 */
static BMI088_Status_e BMI088_Gyro_Init(void)
{
    uint8_t res = 0;

    /* check the communication ------------------------------------------------*/
    /* read the gyro ID address */
    BMI088_Gyro_Read_Single_Reg(BMI088_GYRO_CHIP_ID, res);
    /* waiting 150us */
    Delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    /* read again */
    BMI088_Gyro_Read_Single_Reg(BMI088_GYRO_CHIP_ID, res);
    /* waiting 150us */
    Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    /* gyro software reset ------------------------------------------------*/
    /* write 0xB6 to the register GYRO_SOFTRESET that address is 0x14 to reset software */
    BMI088_Gyro_Write_Single_Reg(BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
    /* software reset waiting time, there is 80ms */
    Delay_ms(BMI088_LONG_DELAY_TIME);

    /* check the communication again ------------------------------------------------*/
    BMI088_Gyro_Read_Single_Reg(BMI088_GYRO_CHIP_ID, res);
    Delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_Gyro_Read_Single_Reg(BMI088_GYRO_CHIP_ID, res);
    Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    /* if the value of the 0x00 register is not 0x0F,reset failed */
    if (res != BMI088_GYRO_CHIP_ID_VALUE)
    {
        return BMI088_NO_SENSOR;
    }

    /* config the gyro sensor */
    for (uint8_t write_reg_num = 0; write_reg_num < BMI088_WRITE_GYRO_REG_NUM; write_reg_num++)
    {
				
        BMI088_Gyro_Write_Single_Reg(Gyro_Register_ConfigurationData_ErrorStatus[write_reg_num][0], Gyro_Register_ConfigurationData_ErrorStatus[write_reg_num][1]);
        /* waiting 150us */
        Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        /* read the configuration */
        BMI088_Gyro_Read_Single_Reg(Gyro_Register_ConfigurationData_ErrorStatus[write_reg_num][0], res);
        /* waiting 150us */
        Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        /* check the configuration and return the specified error */
        if (res != Gyro_Register_ConfigurationData_ErrorStatus[write_reg_num][1])
        {
            return (BMI088_Status_e)Gyro_Register_ConfigurationData_ErrorStatus[write_reg_num][2];
        }
    }

    /* no error */
    return BMI088_NO_ERROR;
}

#define IMU_Calibration_ENABLE  1U

/**
 * @brief  标定陀螺仪零偏（Offset）
 * @param  BMI088_Info：要写入零偏结果的数据结构指针
 * @retval 无
 * @note   原理：陀螺仪静止时理论上应输出 0 rad/s，但实际会有微小的固定漂移（零偏）。
 *         本函数在 IMU 静止时采样 5000 次角速度取平均，把这个平均值作为零偏存下来，
 *         之后每次读到的角速度都减去它，就能消除静止漂移。
 *         宏 IMU_Calibration_ENABLE 为 0 时不做采样，直接使用事先测好写死的三个零偏值
 *         （每次上电都重新标定很耗时，比赛车上常用固定标定值）。
 */
static void BMI088_Offset_Update(BMI088_Info_Typedef *BMI088_Info)
{
#if IMU_Calibration_ENABLE /* ENABLE the BMI088 Calibration */
	
	uint8_t buf[8] = {0,};

  for(uint16_t i = 0; i < 5000; i++)
  {
    /* read the accelerator multi data */
    BMI088_Accel_Read_Multi_Reg(BMI088_ACCEL_XOUT_L, buf, 6);
    BMI088_Info->MPU_Info.Accel_X = (int16_t)((buf[1]) << 8) | buf[0];
    BMI088_Info->MPU_Info.Accel_Y = (int16_t)((buf[3]) << 8) | buf[2];
    BMI088_Info->MPU_Info.Accel_Z = (int16_t)((buf[5]) << 8) | buf[4];

    /* read the gyro multi data */
    BMI088_Gyro_Read_Multi_Reg(BMI088_GYRO_CHIP_ID, buf, 8);
    /* check the ID */
    if(buf[0] == BMI088_GYRO_CHIP_ID_VALUE)   
    {
      BMI088_Info->MPU_Info.Gyro_X = (int16_t)((buf[3]) << 8) | buf[2];
      BMI088_Info->MPU_Info.Gyro_Y = (int16_t)((buf[5]) << 8) | buf[4];
      BMI088_Info->MPU_Info.Gyro_Z = (int16_t)((buf[7]) << 8) | buf[6];

      /* update the gyro offsets */
      BMI088_Info->Offsets_Gyro_X += BMI088_GYRO_SEN * BMI088_Info->MPU_Info.Gyro_X;
      BMI088_Info->Offsets_Gyro_Y += BMI088_GYRO_SEN * BMI088_Info->MPU_Info.Gyro_Y;
      BMI088_Info->Offsets_Gyro_Z += BMI088_GYRO_SEN * BMI088_Info->MPU_Info.Gyro_Z;
    }
    /* waiting 1ms */
    Delay_ms(1);
  }

  BMI088_Info->Offsets_Gyro_X = BMI088_Info->Offsets_Gyro_X / 5000.f;
  BMI088_Info->Offsets_Gyro_Y = BMI088_Info->Offsets_Gyro_Y / 5000.f; 
  BMI088_Info->Offsets_Gyro_Z = BMI088_Info->Offsets_Gyro_Z / 5000.f;
	
#else /* DISABLE the BMI088 Calibration */
	/* store the previous offsets */
  BMI088_Info->Offsets_Gyro_X = -0.00125359884,
  BMI088_Info->Offsets_Gyro_Y = -0.00382664148;
  BMI088_Info->Offsets_Gyro_Z = 0.00183799898;
#endif

  /* set the offset init flag */
  BMI088_Info->Offsets_Init = true;
}

/**
 * @brief  BMI088 总初始化入口（对加速度计和陀螺仪分别初始化，再做零偏标定）
 * @param  无
 * @retval 无
 * @note   do-while 循环保证：只要加速度计或陀螺仪初始化失败(status 非 0)就反复重试，
 *         直到两者都成功为止（适用于上电时芯片尚未就绪的情况）。完成后标定零偏。
 */
void BMI088_Init(void)
{
  BMI088_Status_e status = BMI088_NO_ERROR;

  /* Initializes the BMI088 */
 
	do{	
    status |= BMI088_Accel_Init();
		
    status |= BMI088_Gyro_Init();

    Delay_ms(2);
	
	}while(status);
  
	BMI088_Offset_Update(&BMI088_Info);

}

/**
 * @brief  读取并换算 BMI088 的最新数据（由 1kHz 周期任务反复调用）
 * @param  BMI088_Info：数据输出到的结构体指针
 * @retval 无
 * @note   步骤：
 *         1) 连续读加速度计 6 字节(每轴 2 字节) -> 拼成 16bit 原始值 -> ×SEN 得到 g；
 *         2) 读 2 字节温度 -> 换算成 ℃；
 *         3) 连续读陀螺仪 8 字节：第 0 字节是芯片 ID(0x0F)，第 2~7 字节才是 XYZ 三轴数据，
 *            所以先校验 ID 再取数据 -> ×SEN 得到 rad/s -> 再减去零偏。
 *         拼字节方式为 (高字节<<8)|低字节，是小端格式。
 */
void BMI088_Info_Update(BMI088_Info_Typedef *BMI088_Info)
{
    uint8_t buf[8] = {0, 0, 0, 0, 0, 0};

    /* read the accelerator multi data */
		BMI088_Accel_Read_Multi_Reg(BMI088_ACCEL_XOUT_L, buf, 6);
    BMI088_Info->MPU_Info.Accel_X = (int16_t)((buf[1] << 8) | buf[0]);
    BMI088_Info->MPU_Info.Accel_Y = (int16_t)((buf[3] << 8) | buf[2]);
    BMI088_Info->MPU_Info.Accel_Z = (int16_t)((buf[5] << 8) | buf[4]);

    /* converts the accelerator data */
    BMI088_Info->Accel[0] =  BMI088_ACCEL_SEN * BMI088_Info->MPU_Info.Accel_X;
		BMI088_Info->Accel[1] =  BMI088_ACCEL_SEN * BMI088_Info->MPU_Info.Accel_Y;
		BMI088_Info->Accel[2] =  BMI088_ACCEL_SEN * BMI088_Info->MPU_Info.Accel_Z;

    /* read the temperature */
    BMI088_Accel_Read_Multi_Reg(BMI088_TEMP_M, buf, 2);
    BMI088_Info->MPU_Info.Temperature = (int16_t)((buf[0] << 3) | (buf[1] >> 5));
    if (BMI088_Info->MPU_Info.Temperature > 1023) BMI088_Info->MPU_Info.Temperature -= 2048;

    /* converts the temperature data */
    BMI088_Info->Temperature = BMI088_Info->MPU_Info.Temperature * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;

    /* read the gyro multi data */
		BMI088_Gyro_Read_Multi_Reg(BMI088_GYRO_CHIP_ID, buf, 8);
    /* check the ID */
    if(buf[0] == BMI088_GYRO_CHIP_ID_VALUE)   
    {
      BMI088_Info->MPU_Info.Gyro_X = (int16_t)((buf[3] << 8) | buf[2]);
      BMI088_Info->MPU_Info.Gyro_Y = (int16_t)((buf[5] << 8) | buf[4]);
      BMI088_Info->MPU_Info.Gyro_Z = (int16_t)((buf[7] << 8) | buf[6]);
    }

    /* converts the gyro data */
		BMI088_Info->Gyro[0] =   BMI088_GYRO_SEN * BMI088_Info->MPU_Info.Gyro_X - BMI088_Info->Offsets_Gyro_X;
	  BMI088_Info->Gyro[1] =   BMI088_GYRO_SEN * BMI088_Info->MPU_Info.Gyro_Y - BMI088_Info->Offsets_Gyro_Y;
	  BMI088_Info->Gyro[2] =   BMI088_GYRO_SEN * BMI088_Info->MPU_Info.Gyro_Z - BMI088_Info->Offsets_Gyro_Z;
}


/**
 * @brief  通过 SPI 写单个寄存器（通用底层，供加速度计/陀螺仪宏调用）
 * @param  reg  ：寄存器地址（写操作时最高位为 0）
 * @param  data ：要写入寄存器的值
 * @retval 无
 * @note   时序：先发地址字节，再发数据字节，两个字节由 SPI 依次送出。
 */
static void BMI088_Write_Single_Reg(uint8_t reg, uint8_t data)
{
    BMI088_Read_Write_Byte(reg);  

    BMI088_Read_Write_Byte(data);
}


/**
 * @brief  通过 SPI 读单个寄存器（通用底层）
 * @param  reg        ：寄存器地址，内部会 |0x80 表示读操作
 * @param  return_data：读到的数据写回到这个指针指向的变量
 * @retval 无
 * @note   时序：先发「地址|0x80」，再发一个无效字节(0x55)，同时从 MISO 读回寄存器值。
 */
static void BMI088_Read_Single_Reg(uint8_t reg, uint8_t *return_data)
{

    BMI088_Read_Write_Byte(reg | 0x80); 

    *return_data = BMI088_Read_Write_Byte(0x55);
}

/**
 * @brief  通过 SPI 连续读多个寄存器（通用底层）
 * @param  reg：起始寄存器地址（内部会 |0x80 表示读操作）
 * @param  buf：读到的数据依次存入这个缓冲区
 * @param  len：要读取的字节数
 * @retval 无
 * @note   时序：先发「地址|0x80」，随后每发一个无效字节(0x55)就回读一个字节，
 *         BMI088 支持地址自增，连续读能一次性读回整段寄存器。
 */
static void BMI088_Read_Multi_Reg(uint8_t reg, uint8_t *buf, uint8_t len)
{

  	BMI088_Read_Write_Byte(reg | 0x80);

    while (len != 0)
    {
     
        *buf = BMI088_Read_Write_Byte(0x55); 
        buf++;
        len--;
   
		}
}