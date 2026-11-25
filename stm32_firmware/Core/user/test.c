#include "test.h"
#include "chasis.h"
#include "encoder.h"
#include "usart.h"
#include <stdio.h>

/* 暂时关闭串口发送：0 = 不发，1 = 发 */
#define TEST_TX_ENABLE   0

/* ============ 速度环测试序列 ============ */
typedef struct {
    float    rpm;
    uint16_t hold_ms;
} step_t;

static const step_t STEPS[] = {
    {   0.0f,  800 },
    { 100.0f, 3000 },
    {   0.0f, 2000 },
    { 200.0f, 3000 },
    {   0.0f, 2000 },
    { 100.0f, 3000 },
};
#define STEP_N   (sizeof(STEPS) / sizeof(STEPS[0]))

static uint8_t  s_idx;
static uint32_t s_t0;
static uint8_t  s_started;
static volatile float s_wz;

/* ============ 底盘解算数据集 ============
 * vx/vy 单位 m/s，wz 单位 rad/s，均为世界系
 */
typedef struct {
    float    vx;
    float    vy;
    float    wz;
    uint16_t ms;
} seq_t;

static const seq_t SEQ[] = {
    
    {  0.3f,  0.00f,  3.5f, 2000 },   /* 行进中转向  */
    {  0.3f,  0.00f,  3.5f, 2000 },   /* 行进中转向  */
    {  -0.3f,  0.00f,  3.5f, 2000 },   /* 行进中转向  */
    {  -0.3f,  0.00f,  3.5f, 2000 },   /* 行进中转向  */
    {  0.0f,  0.00f,  3.5f, 2000 },   /* 行进中转向  */
    
};
#define SEQ_N   (sizeof(SEQ) / sizeof(SEQ[0]))

static uint8_t  q_idx;
static uint32_t q_t0;
static uint8_t  q_started;
static volatile float q_vx, q_vy, q_wz;

/* 中断里采样、主循环里发送的快照 */
static volatile float   s_cmd_vx, s_cmd_vy, s_cmd_wz;
static volatile int16_t s_tx[8];
static volatile uint8_t s_tx_flag;
static uint8_t          s_div;

static float rpm2wz(float rpm)
{
    return rpm * 2.0f * PI * R_wheel / (60.0f * R_chasis);
}

#if TEST_TX_ENABLE
static float wz2rpm(float wz)
{
    return wz * 60.0f * R_chasis / (2.0f * PI * R_wheel);
}
#endif

static void test_seq_update(void)
{
    uint32_t now = HAL_GetTick();

    if (!q_started) { q_started = 1; q_t0 = now; }

    if ((now - q_t0) >= SEQ[q_idx].ms)
    {
        q_t0  = now;
        q_idx = (uint8_t)((q_idx + 1) % SEQ_N);
    }

    q_vx = SEQ[q_idx].vx;
    q_vy = SEQ[q_idx].vy;
    q_wz = SEQ[q_idx].wz;
}

void test_init(void)
{
    s_idx = 0; s_t0 = 0; s_started = 0; s_wz = 0.0f;

    q_idx = 0; q_t0 = 0; q_started = 0;
    q_vx = q_vy = q_wz = 0.0f;

    s_cmd_vx = s_cmd_vy = s_cmd_wz = 0.0f;
    s_tx_flag = 0;
    s_div = 0;
    for (uint8_t i = 0; i < 8; i++) s_tx[i] = 0;
}

float test_target_update(void)
{
    uint32_t now = HAL_GetTick();

    if (!s_started) { s_started = 1; s_t0 = now; }

    if ((now - s_t0) >= STEPS[s_idx].hold_ms)
    {
        s_t0  = now;
        s_idx = (uint8_t)((s_idx + 1) % STEP_N);
    }

    s_wz = rpm2wz(STEPS[s_idx].rpm);
    return s_wz;
}

/* 1 kHz 中断里调用：推进数据集，返回当前命令 */
void test_get_cmd(float *vx, float *vy, float *wz)
{
#if   (TEST_MODE == 1)
    s_cmd_vx = 0.0f;  s_cmd_vy = 0.0f;  s_cmd_wz = 0.0f;
#elif (TEST_MODE == 2)
    s_cmd_vx = 0.0f;  s_cmd_vy = 0.0f;  s_cmd_wz = 1.0f;
#elif (TEST_MODE == 3)
    test_seq_update();
    s_cmd_vx = q_vx;  s_cmd_vy = q_vy;  s_cmd_wz = q_wz;
#else
    s_cmd_vx = 0.0f;  s_cmd_vy = 0.0f;  s_cmd_wz = test_target_update();
#endif

    *vx = s_cmd_vx;
    *vy = s_cmd_vy;
    *wz = s_cmd_wz;
}

/* 1 kHz 中断里调用：每 10 拍把当前状态存进快照，不发送 */
void test_snapshot(void)
{
    if (++s_div < 10) return;
    s_div = 0;

#if   (TEST_MODE == 3)
    s_tx[0] = (int16_t)(s_cmd_vx * 1000.0f);
    s_tx[1] = (int16_t)(s_cmd_vy * 1000.0f);
    s_tx[2] = (int16_t)(s_cmd_wz * 1000.0f);
    s_tx[3] = (int16_t)(chasis_get_vx()     * 1000.0f);
    s_tx[4] = (int16_t)(chasis_get_vy()     * 1000.0f);
    s_tx[5] = (int16_t)(chasis_get_yaw()    * 1000.0f);
    s_tx[6] = (int16_t)(chasis_get_odom_x() * 1000.0f);
    s_tx[7] = (int16_t)(chasis_get_odom_y() * 1000.0f);
    s_tx_flag = 1;
#endif
}

/* 主循环里调用：把最近一份快照发出去 */
void test_send_pending(void)
{
#if TEST_TX_ENABLE
    if (!s_tx_flag) return;
    s_tx_flag = 0;

    char buf[96];
    int len = snprintf(buf, sizeof(buf), "%d,%d,%d,%d,%d,%d,%d,%d\n",
                       s_tx[0], s_tx[1], s_tx[2], s_tx[3],
                       s_tx[4], s_tx[5], s_tx[6], s_tx[7]);

    if (len > 0) {
        HAL_UART_Transmit(&huart1, (uint8_t *)buf, (uint16_t)len, 20);
    }
#else
    s_tx_flag = 0;
#endif
}

void test_telemetry(const float duty[4])
{
#if TEST_TX_ENABLE
    char buf[96];

    int len = snprintf(buf, sizeof(buf), "%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
                       (int)wz2rpm(s_wz),
                       (int)enc_get_speed_rpm(0),
                       (int)enc_get_speed_rpm(1),
                       (int)enc_get_speed_rpm(2),
                       (int)enc_get_speed_rpm(3),
                       (int)(duty[0] * 1000.0f),
                       (int)(duty[1] * 1000.0f),
                       (int)(duty[2] * 1000.0f),
                       (int)(duty[3] * 1000.0f));

    if (len > 0) {
        HAL_UART_Transmit(&huart1, (uint8_t *)buf, (uint16_t)len, 10);
    }
#else
    (void)duty;
#endif
}

void test_imu_telemetry(void)
{
#if TEST_TX_ENABLE
    char buf[80];

    int len = snprintf(buf, sizeof(buf), "%d,%d,%d,%d\n",
                       (int)imu_get_gz_raw(),
                       (int)(imu_get_gz_bias() * 100.0f),
                       (int)(imu_get_gz() * 1000.0f),
                       (int)(chasis_get_yaw() * 1000.0f));

    if (len > 0) {
        HAL_UART_Transmit(&huart1, (uint8_t *)buf, (uint16_t)len, 10);
    }
#endif
}

void test_fusion_telemetry(void)
{
#if TEST_TX_ENABLE
    char buf[96];

    int len = snprintf(buf, sizeof(buf), "%d,%d,%d,%d\n",
                       (int)(chasis_get_yaw()     * 1000.0f),
                       (int)(chasis_get_yaw_enc() * 1000.0f),
                       (int)(chasis_get_wz()      * 1000.0f),
                       (int)(imu_get_gz()         * 1000.0f));

    if (len > 0) {
        HAL_UART_Transmit(&huart1, (uint8_t *)buf, (uint16_t)len, 10);
    }
#endif
}

void test_chasis_telemetry(void)
{
#if TEST_TX_ENABLE
    char buf[128];

    int len = snprintf(buf, sizeof(buf), "%d,%d,%d,%d,%d,%d,%d,%d\n",
                       (int)(s_cmd_vx * 1000.0f),
                       (int)(s_cmd_vy * 1000.0f),
                       (int)(s_cmd_wz * 1000.0f),
                       (int)(chasis_get_vx()     * 1000.0f),
                       (int)(chasis_get_vy()     * 1000.0f),
                       (int)(chasis_get_yaw()    * 1000.0f),
                       (int)(chasis_get_odom_x() * 1000.0f),
                       (int)(chasis_get_odom_y() * 1000.0f));

    if (len > 0) {
        HAL_UART_Transmit(&huart1, (uint8_t *)buf, (uint16_t)len, 10);
    }
#endif
}

void test_run_telemetry(const float duty[4])
{
#if   (TEST_MODE == 1)
    test_imu_telemetry();
#elif (TEST_MODE == 2)
    test_fusion_telemetry();
#elif (TEST_MODE == 3)
    test_send_pending();
#else
    test_telemetry(duty);
#endif
}
