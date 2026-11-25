#ifndef PWM_H
#define PWM_H

#include <stdint.h>
#include "tim.h"

#define PWM_ARR      8399.0f    /* ARR，1 kHz 下 20 kHz PWM */
#define PWM_ARR_MAX  8400       /* ARR+1，真正的 100% */

/* 索引约定：与 encoder / pid 一致，0~3 */
#define M_FL  0
#define M_FR  1
#define M_RL  2
#define M_RR  3

void pwm_init(void);
void setpwm(uint8_t id, float duty);   /* duty: -1.0 ~ +1.0 */
void setbrake(void);                    /* 四轮刹车 */
void setcoast(void);                    /* 四轮滑行 */

#endif
