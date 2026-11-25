#ifndef REMOTE_H
#define REMOTE_H

#include <stdint.h>

/* 1 = 用遥控器指令，0 = 用 test.c 的测试序列 */
#define REMOTE_ENABLE   1

void    remote_init(void);
void    remote_update(void);
uint8_t remote_is_alive(void);
uint8_t remote_get_flags(void);
void    remote_get_cmd(float *vx, float *vy, float *wz);

#endif
