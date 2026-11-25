#ifndef PID_H
#define PID_H

#include <stdint.h>
#include "pwm.h"
typedef struct {
    float kp;
    float ki;
    float kd;
    float dt;
    float integ;// 用于积分
    float err_last; //用于微分 
    float ff_dz; //死区
    float ff_k;  //前馈
} pid_t;


void  pid_init(pid_t *p);
void  pid_reset(pid_t *p);
float pid_calc(pid_t *p, float target_rpm, float actual_rpm);

#endif
