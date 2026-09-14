#include "Bmi088.h"
#include "bsp_gpio.h"
#include "bsp_spi.h"
#include "bsp_tick.h"

/* ===== 文件说明：BMI088 六轴 IMU 驱动 (Bmi088.c) =====
 * 作用：通过 SPI 读写 BMI088 内部「加速度计 + 陀螺仪」两个独立从设备，
 *       完成初始化(量程/带宽/中断配置)与周期数据读取换算。
 *
 * 结构：
 *   - 底层：BMI088_Write_Single_Reg / Read_Single_Reg / Read_Multi_Reg 三个
 *           静态函数，封装「发寄存器地址 -> 读/写数据」的 SPI 时序。
 *   - 宏封装：BMI088_Accel/Gyro_xxx 宏在每次 SPI 前后拉低/拉高对应片选(CS)，
 *           因为加速度计和陀螺仪共用 SPI 总线，靠 CS 区分。
 *   - 初始化：BMI088_Accel_Init()/BMI088_Gyro_Init() 复位芯片、校验 ID、
 *           逐项写配置寄存器并回读校验；BMI088_Init() 组装调用。
 *   - 数据读取：BMI088_Info_Update() 周期读取原始值并换算成物理量。
 *
 * 量程/灵敏度：
 *   - 加速度计 ±6g，灵敏度 BMI088_ACCEL_6G_SEN = 6/32768，原始值×SEN = 加速度(g)；
 *   - 陀螺仪 ±2000 dps，灵敏度 BMI088_GYRO_2000_SEN = 2000*π/180/32768，
 *     原始值×SEN = 角速度(rad/s)。
 *
 * 坐标：芯片坐标系(右手系)，本工程未做转轴映射，直接按 X/Y/Z 原样输出。
 * 温度：加速度计内置温度传感器，原始 11bit 有符号值 × 0.125 + 23.0 = 摄氏度。
 *
 * ===== 调车关注 =====
 *  - BMI088_Init() 里是 do{...}while(status) 无限重试，若芯片没接好会卡死在初始化。
 *  - 陀螺仪零偏：默认用写死的 3 个 Offsets_Gyro_* 常量(IMU_Calibration_ENABLE=0)，
 *    如需上电自标定，把 IMU_Calibration_ENABLE 改成 1。
 * ===== ========================================================================== */

static void BMI088_Write_Single_Reg(uint8_t, uint8_t);        /* 写单寄存器(内部不发片选) */

static void BMI088_Read_Single_Reg(uint8_t, uint8_t *);       /* 读单寄存器(内部不发片选) */

static void BMI088_Read_Multi_Reg(uint8_t, uint8_t *, uint8_t); /* 读多寄存器(内部不发片选) */

/* 加速度计「写单寄存器」宏：拉低 CS -> 写 -> 拉高 CS */
#define BMI088_Accel_Write_Single_Reg(reg, data) \
    {                                            \
        BMI088_ACCEL_NS_L();                     \
        BMI088_Write_Single_Reg((reg), (data));  \
        BMI088_ACCEL_NS_H();                     \
    }

/* 加速度计「读单寄存器」宏。
 * 注意: 疑似bug —— 此处连发两个 0x55 哑字节、取第 3 个字节作数据，会读到 reg+1
 * 而非 reg(与下方 BMI088_Read_Single_Reg 只发 1 个哑字节的写法不一致)。 */
#define BMI088_Accel_Read_Single_Reg(reg, data) \
    {                                           \
       BMI088_ACCEL_NS_L();                     \
       BMI088_Read_Write_Byte((reg) | 0x80);    \
       BMI088_Read_Write_Byte(0x55);            \
			 (data) = BMI088_Read_Write_Byte(0x55);   \
       BMI088_ACCEL_NS_H();                     \
    }

/* 加速度计「读多寄存器」宏。
 * 注意: 疑似bug —— 宏里先发了一次 reg|0x80，紧接着 BMI088_Read_Multi_Reg 内部
 * 又发一次 reg|0x80，地址被连发两次，导致读回的数据整体后移 1 字节(错位)。 */
#define BMI088_Accel_Read_Multi_Reg(reg, data, len) \
    {                                               \
        BMI088_ACCEL_NS_L();                        \
        BMI088_Read_Write_Byte((reg) | 0x80);       \
        BMI088_Read_Multi_Reg(reg, data, len);      \
        BMI088_ACCEL_NS_H();                        \
    }

/* 陀螺仪「写单寄存器」宏：拉低 CS -> 写 -> 拉高 CS */
#define BMI088_Gyro_Write_Single_Reg(reg, data) \
    {                                           \
        BMI088_GYRO_NS_L();                     \
        BMI088_Write_Single_Reg((reg), (data)); \
        BMI088_GYRO_NS_H();                     \
    }

/* 陀螺仪「读单寄存器」宏：调用底层 Read_Single_Reg(发 1 个哑字节取数据，正确) */
#define BMI088_Gyro_Read_Single_Reg(reg, data)  \
    {                                           \
        BMI088_GYRO_NS_L();                     \
        BMI088_Read_Single_Reg((reg), &(data)); \
        BMI088_GYRO_NS_H();                     \
    }

/* 陀螺仪「读多寄存器」宏：地址只发一次(正确) */
#define BMI088_Gyro_Read_Multi_Reg(reg, data, len)   \
    {                                                \
        BMI088_GYRO_NS_L();                          \
        BMI088_Read_Multi_Reg((reg), (data), (len)); \
        BMI088_GYRO_NS_H();                          \
    }

/* 加速度计灵敏度系数：原始值(16bit) × SEN = 加速度(g)。当前用 ±6g 量程 */
static float BMI088_ACCEL_SEN = BMI088_ACCEL_6G_SEN;

/* 陀螺仪灵敏度系数：原始值(16bit) × SEN = 角速度(rad/s)。当前用 ±2000dps 量程 */
static float BMI088_GYRO_SEN = BMI088_GYRO_2000_SEN;

/* BMI088 数据全局实例(原始值 + 换算后物理量 + 零偏) */
BMI088_Info_Typedef BMI088_Info;

/* 加速度计初始化配置表：每行 = {寄存器地址, 写入值, 出错时返回的错误码} */
static uint8_t Accel_Register_ConfigurationData_ErrorStatus[BMI088_WRITE_ACCEL_REG_NUM][3] =
{
    /* Turn on accelerometer  使能加速度计 */
    {BMI088_ACCEL_PWR_CTRL, BMI088_ACCEL_ENABLE_ACCEL_ON, BMI088_ACCEL_PWR_CTRL_ERROR},

    /* Pause mode  电源模式：正常(非挂起) */
    {BMI088_ACCEL_PWR_CONF, BMI088_ACCEL_PWR_ACTIVE_MODE, BMI088_ACCEL_PWR_CONF_ERROR},

    /* ACCELeleration Configuration  输出数据率 ODR=1600Hz(必须写 1 的保留位) */
    {BMI088_ACCEL_CONF,  (BMI088_ACCEL_NORMAL| BMI088_ACCEL_1600_HZ | BMI088_ACCEL_CONF_MUST_Set), BMI088_ACCEL_CONF_ERROR},

    /* ACCELelerometer setting range  量程 ±6g */
    {BMI088_ACCEL_RANGE, BMI088_ACCEL_RANGE_6G, BMI088_ACCEL_RANGE_ERROR},

    /* INT1 Configuration input and output pin  INT1 引脚：使能/推挽/低有效 */
    {BMI088_INT1_IO_CTRL, (BMI088_ACCEL_INT1_IO_ENABLE | BMI088_ACCEL_INT1_GPIO_PP | BMI088_ACCEL_INT1_GPIO_LOW), BMI088_INT1_IO_CTRL_ERROR},

    /* interrupt map pin  数据就绪(DRDY)映射到 INT1 */
    {BMI088_INT_MAP_DATA, BMI088_ACCEL_INT1_DRDY_INTERRUPT, BMI088_INT_MAP_DATA_ERROR}
};

/* 陀螺仪初始化配置表：每行 = {寄存器地址, 写入值, 出错时返回的错误码} */
static uint8_t Gyro_Register_ConfigurationData_ErrorStatus[BMI088_WRITE_GYRO_REG_NUM][3] =
{
    /* Angular rate and resolution  量程 ±2000dps */
    {BMI088_GYRO_RANGE, BMI088_GYRO_2000, BMI088_GYRO_RANGE_ERROR},

    /* Data Transfer Rate and Bandwidth Settings  输出数据率 ODR=2kHz、带宽 230Hz */
    {BMI088_GYRO_BANDWIDTH, (BMI088_GYRO_2000_230_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set), BMI088_GYRO_BANDWIDTH_ERROR},

    /* Power Mode Selection Register  电源模式：正常模式 */
    {BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE, BMI088_GYRO_LPM1_ERROR},

    /* Data Interrupt Trigger Register  使能数据就绪中断 */
    {BMI088_GYRO_CTRL, BMI088_DRDY_ON, BMI088_GYRO_CTRL_ERROR},

    /* Interrupt Pin Trigger Register  INT3/INT4 引脚：推挽/低有效 */
    {BMI088_GYRO_INT3_INT4_IO_CONF, (BMI088_GYRO_INT3_GPIO_PP | BMI088_GYRO_INT3_GPIO_LOW), BMI088_GYRO_INT3_INT4_IO_CONF_ERROR},

    /* interrupt map register  数据就绪映射到 INT3 */
    {BMI088_GYRO_INT3_INT4_IO_MAP, BMI088_GYRO_DRDY_IO_INT3, BMI088_GYRO_INT3_INT4_IO_MAP_ERROR}
};

/**
  * @brief  初始化加速度计：复位 -> 校验芯片 ID -> 逐项写配置并回读校验
  * @param  无
  * @retval BMI088_Status_e  成功返回 BMI088_NO_ERROR，失败返回对应错误码
  */
static BMI088_Status_e BMI088_Accel_Init(void)
{

    uint8_t res = 0;

    /* check the communication ------------------------------------------------*/
    /* read the accelerator ID address  读芯片 ID(0x00, 应为 0x1E) */
	  BMI088_Accel_Read_Single_Reg(BMI088_ACCEL_CHIP_ID, res);
    /* waiting 150us */
    Delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    /* read again  再读一次确认通信正常 */
    BMI088_Accel_Read_Single_Reg(BMI088_ACCEL_CHIP_ID, res);
    /* waiting 150us */
    Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    /* ACCELelerator software Reset ------------------------------------------------*/
    /* write 0xB6 to the register ACCEL_SOFTResET that addRess is 0x7E to reset software  软件复位 */
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
        return BMI088_NO_SENSOR;   /* 芯片没接好或 ID 读错 */
    }

    /* config the accelerator sensor  逐项写配置寄存器并回读校验 */
    for (uint8_t write_reg_num = 0; write_reg_num < BMI088_WRITE_ACCEL_REG_NUM; write_reg_num++)
    {

        BMI088_Accel_Write_Single_Reg(Accel_Register_ConfigurationData_ErrorStatus[write_reg_num][0], Accel_Register_ConfigurationData_ErrorStatus[write_reg_num][1]);
        /* waiting 150us */
        Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        /* read the configuration  回读刚写入的寄存器 */
        BMI088_Accel_Read_Single_Reg(Accel_Register_ConfigurationData_ErrorStatus[write_reg_num][0], res);
        /* waiting 150us */
        Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        /* check the configuration and return the specified error  校验不一致则返回错误码 */
        if (res != Accel_Register_ConfigurationData_ErrorStatus[write_reg_num][1])
        {
            return (BMI088_Status_e)Accel_Register_ConfigurationData_ErrorStatus[write_reg_num][2];
        }
    }

    /* no error */
    return BMI088_NO_ERROR;
}

/**
  * @brief  初始化陀螺仪：复位 -> 校验芯片 ID -> 逐项写配置并回读校验
  * @param  无
  * @retval BMI088_Status_e  成功返回 BMI088_NO_ERROR，失败返回对应错误码
  */
static BMI088_Status_e BMI088_Gyro_Init(void)
{
    uint8_t res = 0;

    /* check the communication ------------------------------------------------*/
    /* read the gyro ID address  读芯片 ID(0x00, 应为 0x0F) */
    BMI088_Gyro_Read_Single_Reg(BMI088_GYRO_CHIP_ID, res);
    /* waiting 150us */
    Delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    /* read again  再读一次 */
    BMI088_Gyro_Read_Single_Reg(BMI088_GYRO_CHIP_ID, res);
    /* waiting 150us */
    Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    /* gyro software reset ------------------------------------------------*/
    /* write 0xB6 to the register GYRO_SOFTRESET that address is 0x14 to reset software  软件复位 */
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
        return BMI088_NO_SENSOR;   /* 芯片没接好或 ID 读错 */
    }

    /* config the gyro sensor  逐项写配置寄存器并回读校验 */
    for (uint8_t write_reg_num = 0; write_reg_num < BMI088_WRITE_GYRO_REG_NUM; write_reg_num++)
    {

        BMI088_Gyro_Write_Single_Reg(Gyro_Register_ConfigurationData_ErrorStatus[write_reg_num][0], Gyro_Register_ConfigurationData_ErrorStatus[write_reg_num][1]);
        /* waiting 150us */
        Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        /* read the configuration  回读刚写入的寄存器 */
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

/* 陀螺仪零偏标定开关：1=上电采样 5000 次求均值自标定；0=使用下面写死的零偏常量 */
#define IMU_Calibration_ENABLE  1U

/**
  * @brief  更新/设定陀螺仪零偏(Offsets_Gyro_X/Y/Z)
  * @param  BMI088_Info 指向 BMI088 数据结构体
  * @retval 无
  * @note   静止时陀螺仪理想输出为 0，实际会有微小漂移，零偏就是用来抵消这个漂移的。
  *         IMU_Calibration_ENABLE=0 时直接使用经验写死的零偏值(单位 rad/s)。
  */
static void BMI088_Offset_Update(BMI088_Info_Typedef *BMI088_Info)
{
#if IMU_Calibration_ENABLE /* ENABLE the BMI088 Calibration */

	uint8_t buf[8] = {0,};

  for(uint16_t i = 0; i < 5000; i++)   /* 采样 5000 次求平均 */
  {
    /* read the accelerator multi data */
    BMI088_Accel_Read_Multi_Reg(BMI088_ACCEL_XOUT_L, buf, 6);
    BMI088_Info->MPU_Info.Accel_X = (int16_t)((buf[1]) << 8) | buf[0];
    BMI088_Info->MPU_Info.Accel_Y = (int16_t)((buf[3]) << 8) | buf[2];
    BMI088_Info->MPU_Info.Accel_Z = (int16_t)((buf[5]) << 8) | buf[4];

    /* read the gyro multi data  从 0x00 开始读 8 字节(含 ID + 三轴角速度) */
    BMI088_Gyro_Read_Multi_Reg(BMI088_GYRO_CHIP_ID, buf, 8);
    /* check the ID */
    if(buf[0] == BMI088_GYRO_CHIP_ID_VALUE)
    {
      BMI088_Info->MPU_Info.Gyro_X = (int16_t)((buf[3]) << 8) | buf[2];
      BMI088_Info->MPU_Info.Gyro_Y = (int16_t)((buf[5]) << 8) | buf[4];
      BMI088_Info->MPU_Info.Gyro_Z = (int16_t)((buf[7]) << 8) | buf[6];

      /* update the gyro offsets  累加换算成 rad/s 的角速度 */
      BMI088_Info->Offsets_Gyro_X += BMI088_GYRO_SEN * BMI088_Info->MPU_Info.Gyro_X;
      BMI088_Info->Offsets_Gyro_Y += BMI088_GYRO_SEN * BMI088_Info->MPU_Info.Gyro_Y;
      BMI088_Info->Offsets_Gyro_Z += BMI088_GYRO_SEN * BMI088_Info->MPU_Info.Gyro_Z;
    }
    /* waiting 1ms */
    Delay_ms(1);
  }

  /* 除以采样次数求平均零偏 */
  BMI088_Info->Offsets_Gyro_X = BMI088_Info->Offsets_Gyro_X / 5000.f;
  BMI088_Info->Offsets_Gyro_Y = BMI088_Info->Offsets_Gyro_Y / 5000.f;
  BMI088_Info->Offsets_Gyro_Z = BMI088_Info->Offsets_Gyro_Z / 5000.f;

#else /* DISABLE the BMI088 Calibration */
	/* store the previous offsets  使用写死的经验零偏值(单位 rad/s) */
  BMI088_Info->Offsets_Gyro_X =  0.00407549459f;
  BMI088_Info->Offsets_Gyro_Y =  -0.00447348086f;
  BMI088_Info->Offsets_Gyro_Z =  -0.00070989196f;
#endif

  /* set the offset init flag  置零偏初始化完成标志 */
  BMI088_Info->Offsets_Init = true;
}

/**
  * @brief  BMI088 总初始化：加速度计 + 陀螺仪初始化 + 零偏设定
  * @param  无
  * @retval 无
  * @note   注意: do{...}while(status) 是「失败就无限重试」，若芯片未接好会卡死在这里。
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
  * @brief  读取并换算 BMI088 最新数据(加速度/角速度/温度)
  * @param  BMI088_Info 指向 BMI088 数据结构体
  * @retval 无
  * @note   由 1kHz 周期任务调用。原始值先存到 MPU_Info，再乘灵敏度系数换算成
  *         g(加速度) / rad/s(角速度)，陀螺仪数据会减去零偏。
  */
void BMI088_Info_Update(BMI088_Info_Typedef *BMI088_Info)
{
    uint8_t buf[8] = {0, 0, 0, 0, 0, 0};

    /* read the accelerator multi data  读加速度计三轴原始值(小端) */
		BMI088_Accel_Read_Multi_Reg(BMI088_ACCEL_XOUT_L, buf, 6);
    BMI088_Info->MPU_Info.Accel_X = (int16_t)((buf[1] << 8) | buf[0]);
    BMI088_Info->MPU_Info.Accel_Y = (int16_t)((buf[3] << 8) | buf[2]);
    BMI088_Info->MPU_Info.Accel_Z = (int16_t)((buf[5] << 8) | buf[4]);

    /* converts the accelerator data  原始值 × 灵敏度 = 加速度(g) */
    BMI088_Info->Accel[0] =  BMI088_ACCEL_SEN * BMI088_Info->MPU_Info.Accel_X;
		BMI088_Info->Accel[1] =  BMI088_ACCEL_SEN * BMI088_Info->MPU_Info.Accel_Y;
		BMI088_Info->Accel[2] =  BMI088_ACCEL_SEN * BMI088_Info->MPU_Info.Accel_Z;

    /* read the temperature  读温度(11bit 有符号, 存于两个寄存器) */
    BMI088_Accel_Read_Multi_Reg(BMI088_TEMP_M, buf, 2);
    BMI088_Info->MPU_Info.Temperature = (int16_t)((buf[0] << 3) | (buf[1] >> 5));
    if (BMI088_Info->MPU_Info.Temperature > 1023) BMI088_Info->MPU_Info.Temperature -= 2048;   /* 11bit 补码转有符号 */

    /* converts the temperature data  摄氏度 = 原始值 × 0.125 + 23.0 */
    BMI088_Info->Temperature = BMI088_Info->MPU_Info.Temperature * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;

    /* read the gyro multi data  从 0x00 读 8 字节(ID + 三轴角速度) */
		BMI088_Gyro_Read_Multi_Reg(BMI088_GYRO_CHIP_ID, buf, 8);
    /* check the ID */
    if(buf[0] == BMI088_GYRO_CHIP_ID_VALUE)
    {
      BMI088_Info->MPU_Info.Gyro_X = (int16_t)((buf[3] << 8) | buf[2]);
      BMI088_Info->MPU_Info.Gyro_Y = (int16_t)((buf[5] << 8) | buf[4]);
      BMI088_Info->MPU_Info.Gyro_Z = (int16_t)((buf[7] << 8) | buf[6]);
    }

    /* converts the gyro data  原始值 × 灵敏度 - 零偏 = 角速度(rad/s) */
		BMI088_Info->Gyro[0] =   BMI088_GYRO_SEN * BMI088_Info->MPU_Info.Gyro_X - BMI088_Info->Offsets_Gyro_X;
	  BMI088_Info->Gyro[1] =   BMI088_GYRO_SEN * BMI088_Info->MPU_Info.Gyro_Y - BMI088_Info->Offsets_Gyro_Y;
	  BMI088_Info->Gyro[2] =   BMI088_GYRO_SEN * BMI088_Info->MPU_Info.Gyro_Z - BMI088_Info->Offsets_Gyro_Z;
}


/**
  * @brief  写单寄存器(SPI)：发寄存器地址(最高位 0 表示写)，再发数据
  * @param  reg  寄存器地址
  * @param  data 要写入的值
  * @retval 无
  * @note   片选由外层宏控制，本函数只做 SPI 收发。
  */
static void BMI088_Write_Single_Reg(uint8_t reg, uint8_t data)
{
    BMI088_Read_Write_Byte(reg);     /* 写操作：地址最高位为 0 */

    BMI088_Read_Write_Byte(data);    /* 写入数据 */
}


/**
  * @brief  读单寄存器(SPI)：发寄存器地址(最高位 1 表示读)，再发 1 个哑字节换回数据
  * @param  reg         寄存器地址
  * @param  return_data 返回读到的值
  * @retval 无
  */
static void BMI088_Read_Single_Reg(uint8_t reg, uint8_t *return_data)
{

    BMI088_Read_Write_Byte(reg | 0x80);          /* 读操作：地址最高位置 1 */

    *return_data = BMI088_Read_Write_Byte(0x55); /* 发哑字节，同时从 MISO 收到 reg 的值 */
}

/**
  * @brief  读多寄存器(SPI)：发起始地址(读位)，连续读 len 个字节
  * @param  reg  起始寄存器地址
  * @param  buf  存放读回数据的缓冲区
  * @param  len  要读的字节数
  * @retval 无
  * @note   BMI088 支持地址自增连续读，每发一个 0x55 哑字节就能读到下一地址的数据。
  */
static void BMI088_Read_Multi_Reg(uint8_t reg, uint8_t *buf, uint8_t len)
{

  	BMI088_Read_Write_Byte(reg | 0x80);          /* 起始地址(读位) */

    while (len != 0)
    {

        *buf = BMI088_Read_Write_Byte(0x55);     /* 发哑字节，读回一个字节 */
        buf++;
        len--;

		}
}