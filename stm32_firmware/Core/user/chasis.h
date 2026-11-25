#ifndef CHASIS_H
#define CHASIS_H
#include<stdint.h>
#include<math.h>
#include "encoder.h"
#include"pwm.h"
#include "imu.h"
//规定：向前vx+,向左vy+
/* 轮子编号 M_FL/M_FR/M_RL/M_RR 统一在 pwm.h 里定义（0~3） */
#define R_chasis 0.185f
#define K_body 0.7071f //sin45/2
#define R_wheel 0.035f
#define PI 3.1415926f
#define DT 0.001f
void chasis_init(void);
void chasis_targetrpm_cal(float vx,float vy,float wz);
void chasis_update(void);
float chasis_get_rpm_tar(uint8_t id);
float chasis_get_yaw(void);          /* 融合后的航向，单位 rad，范围 ±π */
float chasis_get_yaw_enc(void);
float chasis_get_wz(void);
float chasis_get_vx(void);
float chasis_get_vy(void);
float chasis_get_odom_x(void);
float chasis_get_odom_y(void);
void towards_test(void);
#endif