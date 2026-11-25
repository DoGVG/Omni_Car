#include "pid.h"
#include <math.h>
#define KP 0.0022f
#define KI 0.0073f
#define DT 0.001f
#define I_MAX 0.4f
/*打表初始化*/
static const pid_t pid_default[4] =
    {
        /*kp ki   kd   dt   itg   erl    ff_dz   ff_k*/
        {KP, KI, 0.0f, DT, 0.0f, 0.0f, 0.2496f, 0.0023f},
        {KP, KI, 0.0f, DT, 0.0f, 0.0f, 0.2898f, 0.0021f},
        {KP, KI, 0.0f, DT, 0.0f, 0.0f, 0.2195f, 0.0023f},
        {KP, KI, 0.0f, DT, 0.0f, 0.0f, 0.2314f, 0.0023f},
};
void pid_init(pid_t *p)
{
    for (uint8_t i = 0; i < 4; i++)
    {
        p[i] = pid_default[i];
    }
}
/*
清除积分
*/
void pid_reset(pid_t *p)
{
    if (p == 0)
        return;
    p->integ = 0.0f;
}

float pid_calc(pid_t *p, float target_rpm, float actual_rpm)
{
    if (p == 0)
        return 0.0f;

    float e = target_rpm - actual_rpm; // 误差
    // 1,积分
    if (target_rpm == 0.0f && fabsf(actual_rpm) < 5.0f)
    {
        p->integ = 0.0f;
    }
    p->integ += p->ki * e * (p->dt);
    if (p->integ > I_MAX)
        p->integ = I_MAX;
    if (p->integ < -I_MAX)
        p->integ = -I_MAX;
    // 2,加前馈
    float ff = 0.0f;
    if (target_rpm > 0.0f)
    {
        ff = p->ff_dz + p->ff_k * target_rpm;
    }
    else if (target_rpm < 0.0f)
    {
        ff = -(p->ff_dz + p->ff_k * (-target_rpm));
    }

    // 3,加总，电机不要微分项
    float u = ff + p->kp * e + p->integ;

    /* ---- 4. 输出限幅 ---- */
    if (u > 1.0f)
        u = 1.0f;
    if (u < -1.0f)
        u = -1.0f;

    return u;
}
