#include "pwm.h"

static const uint32_t ch[4] = {
    TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4
};

void pwm_init(void)
{
    for (uint32_t i = 0; i < 4; i++) {
        __HAL_TIM_SET_COMPARE(&htim1, ch[i], 0);
        __HAL_TIM_SET_COMPARE(&htim8, ch[i], 0);
        HAL_TIM_PWM_Start(&htim1, ch[i]);
        HAL_TIM_PWM_Start(&htim8, ch[i]);
    }
}

void setpwm(uint8_t id, float duty)
{
    if (id >= 4) return;

    if (duty >  1.0f) duty =  1.0f;
    if (duty < -1.0f) duty = -1.0f;

    if (duty >= 0.0f) {
        uint32_t ccr = (uint32_t)(duty * PWM_ARR);
        __HAL_TIM_SET_COMPARE(&htim1, ch[id], ccr);
        __HAL_TIM_SET_COMPARE(&htim8, ch[id], 0);
    } else {
        uint32_t ccr = (uint32_t)(-duty * PWM_ARR);
        __HAL_TIM_SET_COMPARE(&htim1, ch[id], 0);
        __HAL_TIM_SET_COMPARE(&htim8, ch[id], ccr);
    }
}

void setbrake(void)
{
    for (uint32_t i = 0; i < 4; i++) {
        __HAL_TIM_SET_COMPARE(&htim1, ch[i], PWM_ARR_MAX);
        __HAL_TIM_SET_COMPARE(&htim8, ch[i], PWM_ARR_MAX);
    }
}

void setcoast(void)
{
    for (uint32_t i = 0; i < 4; i++) {
        __HAL_TIM_SET_COMPARE(&htim1, ch[i], 0);
        __HAL_TIM_SET_COMPARE(&htim8, ch[i], 0);
    }
}
