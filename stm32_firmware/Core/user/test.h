#ifndef TEST_H
#define TEST_H

#include <stdint.h>

/* 0 = 速度环   1 = IMU   2 = 融合   3 = 底盘解算数据集 */
#define TEST_MODE   3

void  test_init(void);
float test_target_update(void);

/* ---- 1 kHz 中断里调用 ---- */
void  test_get_cmd(float *vx, float *vy, float *wz);
void  test_snapshot(void);

/* ---- 主循环里调用 ---- */
void  test_run_telemetry(const float duty[4]);

void  test_telemetry(const float duty[4]);
void  test_imu_telemetry(void);
void  test_fusion_telemetry(void);
void  test_chasis_telemetry(void);
void  test_send_pending(void);

#endif
