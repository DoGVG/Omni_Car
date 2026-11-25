#ifndef ENCODER_H
#define ENCODER_H
#include<stdint.h>
#include"tim.h"
#include "gpio.h"
#define K_RPM (60000.0f / 1320.0f)//11ppr encode3 30减速比,计数/ms转为rpm
/* 轮子编号统一在 pwm.h 里定义（M_FL=0 ~ M_RR=3），这里不再重复定义 */
void enc_init(void);
void enc_update(void);
int32_t enc_get_total(uint8_t id);
float enc_get_speed_rpm(uint8_t id);
int32_t enc_get_delta(uint8_t id);
#endif