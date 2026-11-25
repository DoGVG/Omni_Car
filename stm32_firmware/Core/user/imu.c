#include "imu.h"
#include "usart.h"
#include <stdio.h>
// 寄存器地址宏
#define MPU6050_WHO_AM_I 0x75
#define PWR_MGMT 0x6b
#define SMPLRT_DIV 0x19
#define CONFIG 0x1a
#define GYRO_CONFIG 0x1b
#define ACCEL_CONFIG 0x1c
#define MPU6050_return 0x68
#define MPU6500_return 0x70
//只在update里用 rad/s，其余均为°/s
#define GRYO_DPS 32.8f // 读数向度每秒转换系数
#define K_RAD (3.14159f/180.0f)//角度向弧度转换系数
uint8_t mpu_6050_addr = 0xffu,is_imu_ready;
int16_t gx_raw, gy_raw, gz_raw, temp_raw, ax_raw, ay_raw, az_raw;
float gz_basis,gz_cor;

/* 最近一次 I2C 操作的 HAL 状态，调试用
 *   0 = HAL_OK
 *   1 = HAL_ERROR    （从机 NACK：地址不对 / 芯片没供电 / 没接）
 *   2 = HAL_BUSY
 *   3 = HAL_TIMEOUT  （总线卡住：没上拉 / SDA 被拉低 / 引脚配错）
 */
static int8_t s_hal = -1;

int8_t imu_get_last_hal(void)
{
    return s_hal;
}

static int8_t imu_alive(uint16_t mpu_addr)
{
    uint8_t id = 0;
    HAL_StatusTypeDef st = HAL_I2C_Mem_Read(&hi2c1, mpu_addr, MPU6050_WHO_AM_I,
                                            I2C_MEMADD_SIZE_8BIT, &id, 1, 20);
    s_hal = (int8_t)st;
    return (st == HAL_OK && (id == MPU6050_return || id == MPU6500_return)) ? 1 : 0;
}

/* 扫描 I2C 总线，打印所有应答的 7 位地址
 * 用法：imu_init() 失败后调用一次，直接看出芯片到底在哪个地址 */
void imu_scan(void)
{
    char buf[40];
    uint8_t found = 0;

    HAL_UART_Transmit(&huart1, (uint8_t *)"I2C scan start\r\n", 16, 100);

    for (uint8_t a = 0x08; a < 0x78; a++)
    {
        if (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(a << 1), 2, 5) == HAL_OK)
        {
            int n = snprintf(buf, sizeof(buf), "  found 0x%02X\r\n", a);
            HAL_UART_Transmit(&huart1, (uint8_t *)buf, (uint16_t)n, 50);
            found++;
        }
    }

    if (found == 0)
    {
        HAL_UART_Transmit(&huart1, (uint8_t *)"  nothing on bus\r\n", 19, 100);
    }
    HAL_UART_Transmit(&huart1, (uint8_t *)"I2C scan end\r\n", 14, 100);
}

static inline int16_t byte_combine(uint8_t *p) // 拼接寄存器数据
{
    return (int16_t)(((uint16_t)p[0] << 8) | p[1]);
}
/*
只用于零偏标定使用
@return gz_real °/s
*/
static float read_gz(void) 
{
    uint8_t buf[2] = {0};
    HAL_I2C_Mem_Read(&hi2c1, mpu_6050_addr, 0x47, I2C_MEMADD_SIZE_8BIT, buf, 2, 20);
    int16_t gz_read=byte_combine(buf);
    float gz_real = gz_read / GRYO_DPS;
    return gz_real;
}
/*
    @return 0 正常 -1 寻址失败 -2 寄存器配置错误
*/
int8_t imu_init(void)
{
    HAL_Delay(100);
    // 地址探测
    if (imu_alive((0x68 << 1)))
    {
        mpu_6050_addr = 0xd0;
    }
    else if (imu_alive((0x69 << 1)))
    {
        mpu_6050_addr = 0xd2;
    }
    if((mpu_6050_addr!=0xd0)&&(mpu_6050_addr!=0xd2))   return -1;
    // 复位

    uint8_t buf = 0x80;
    if(HAL_I2C_Mem_Write(&hi2c1, mpu_6050_addr, PWR_MGMT, I2C_MEMADD_SIZE_8BIT, &buf, 1, 100)!=HAL_OK)      return -2;
    HAL_Delay(100);
    // 时钟源配置
    buf = 0x01;
    if(HAL_I2C_Mem_Write(&hi2c1, mpu_6050_addr, PWR_MGMT, I2C_MEMADD_SIZE_8BIT, &buf, 1, 100)!=HAL_OK)      return -2;
    // 采样，滤波配置
    // 1.采样率，1khz/(1+4)
    buf = 0x04;
    if(HAL_I2C_Mem_Write(&hi2c1, mpu_6050_addr, SMPLRT_DIV, I2C_MEMADD_SIZE_8BIT, &buf, 1, 20)!=HAL_OK)      return -2;
    // 2.滤波，频率44hz
    buf = 0x03;
    if(HAL_I2C_Mem_Write(&hi2c1, mpu_6050_addr, CONFIG, I2C_MEMADD_SIZE_8BIT, &buf, 1, 20)!=HAL_OK)      return -2;
    // 3.1 陀螺仪量程配置 1000°/s
    buf = 0x10;
    if(HAL_I2C_Mem_Write(&hi2c1, mpu_6050_addr, GYRO_CONFIG, I2C_MEMADD_SIZE_8BIT, &buf, 1, 20)!=HAL_OK)      return -2;
    // 3.2 加速度计配置 4g
    buf = 0x08;
    if(HAL_I2C_Mem_Write(&hi2c1, mpu_6050_addr, ACCEL_CONFIG, I2C_MEMADD_SIZE_8BIT, &buf, 1, 20)!=HAL_OK)      return -2;
    // 零偏校准
    float sum = 0.0f;
    for (int16_t i = 0; i <800; i++)
    {
        float temp_gz = read_gz();
        sum += temp_gz;
        HAL_Delay(2);
    }
    gz_basis =sum/400.0f;
    is_imu_ready=1;
    return 0;
}

void imu_update(void)
{
    if(!is_imu_ready)   return ;
    uint8_t buf[14] = {0};
    if(HAL_I2C_Mem_Read(&hi2c1, mpu_6050_addr, 0x3b,I2C_MEMADD_SIZE_8BIT,buf, 14, 5)!=HAL_OK)    return;//读取失败不做更新，保持上个周期
    ax_raw = byte_combine(&buf[0]);
    ay_raw = byte_combine(&buf[2]);
    az_raw = byte_combine(&buf[4]);
    temp_raw = byte_combine(&buf[6]);
    gx_raw = byte_combine(&buf[8]);
    gy_raw = byte_combine(&buf[10]);
    gz_raw = byte_combine(&buf[12]);
    gz_cor=(gz_raw/GRYO_DPS-gz_basis)*K_RAD;//转为rad/s，便于后续三角函数处理
}

float imu_get_gz(void)
{
    return gz_cor;
}

uint8_t imu_is_ready(void)
{
    return is_imu_ready;
}

int16_t imu_get_gz_raw(void)
{
    return gz_raw;
}

float imu_get_gz_bias(void)
{
    return gz_basis;
}