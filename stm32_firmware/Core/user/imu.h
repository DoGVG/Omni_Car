#ifndef IMU_H
#define IMU_H
#include<stdint.h>
#include"i2c.h"
#include"gpio.h"
int8_t  imu_init(void);
void    imu_update(void);
float   imu_get_gz(void);
uint8_t imu_is_ready(void);      /* 1 = 初始化成功 */
int16_t imu_get_gz_raw(void);    /* 原始 LSB，调试用 */
float   imu_get_gz_bias(void);   /* 上电校准的零偏，单位 °/s */
int8_t  imu_get_last_hal(void);  /* 最近一次 I2C 的 HAL 状态：0=OK 1=NACK 2=BUSY 3=TIMEOUT */
void    imu_scan(void);          /* 扫描 I2C 总线，串口打印所有应答地址 */


#endif