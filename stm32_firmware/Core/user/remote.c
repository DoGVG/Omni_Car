#include "remote.h"
#include "main.h"
#include <string.h>

/* 独立初始化 USART2（PA2=TX / PA3=RX），不依赖 CubeMX 生成
 * 这样只要 include 本文件、加进 CMakeLists 就能用
 *
 * 帧格式
 * [0] 0xAA  [1] 0x55
 * [2..3] vx   int16 LE  mm/s
 * [4..5] vy   int16 LE  mm/s
 * [6..7] wz   int16 LE  mrad/s
 * [8]    flags          bit0 = 使能
 * [9]    校验 = 前 9 字节累加取低 8 位
 */
#define FRAME_LEN       10
#define FRAME_HEAD0     0xAA
#define FRAME_HEAD1     0x55
#define REMOTE_LOST_MS  300
#define REMOTE_BAUD     115200

static UART_HandleTypeDef huart2_local;

static uint8_t  rx_byte;
static uint8_t  frame[FRAME_LEN];
static uint8_t  fidx;

static volatile int16_t  raw_vx = 0;
static volatile int16_t  raw_vy = 0;
static volatile int16_t  raw_wz = 0;
static volatile uint8_t  raw_flags = 0;
static volatile uint32_t last_rx_ms = 0;
static volatile uint8_t  alive = 0;

static void remote_uart2_init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_USART2_CLK_ENABLE();

    gpio.Pin       = GPIO_PIN_2 | GPIO_PIN_3;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_PULLUP;
    gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &gpio);

    huart2_local.Instance          = USART2;
    huart2_local.Init.BaudRate     = REMOTE_BAUD;
    huart2_local.Init.WordLength   = UART_WORDLENGTH_8B;
    huart2_local.Init.StopBits     = UART_STOPBITS_1;
    huart2_local.Init.Parity       = UART_PARITY_NONE;
    huart2_local.Init.Mode         = UART_MODE_TX_RX;
    huart2_local.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart2_local.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart2_local);

    HAL_NVIC_SetPriority(USART2_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
}

static void parse_byte(uint8_t b)
{
    if (fidx == 0)
    {
        if (b == FRAME_HEAD0) frame[fidx++] = b;
        return;
    }
    if (fidx == 1)
    {
        if (b == FRAME_HEAD1)      frame[fidx++] = b;
        else if (b != FRAME_HEAD0) fidx = 0;
        return;
    }

    frame[fidx++] = b;
    if (fidx < FRAME_LEN) return;

    uint8_t sum = 0;
    for (uint8_t i = 0; i < FRAME_LEN - 1; i++) sum += frame[i];

    if (sum == frame[FRAME_LEN - 1])
    {
        int16_t v, w, r;
        memcpy(&v, &frame[2], 2);
        memcpy(&w, &frame[4], 2);
        memcpy(&r, &frame[6], 2);

        raw_vx    = v;
        raw_vy    = w;
        raw_wz    = r;
        raw_flags = frame[8];
        last_rx_ms = HAL_GetTick();
        alive = 1;
    }
    fidx = 0;
}

void remote_init(void)
{
    fidx = 0;
    raw_vx = raw_vy = raw_wz = 0;
    raw_flags = 0;
    last_rx_ms = 0;
    alive = 0;

    remote_uart2_init();
    HAL_UART_Receive_IT(&huart2_local, &rx_byte, 1);
}

void USART2_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart2_local);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        parse_byte(rx_byte);
        HAL_UART_Receive_IT(&huart2_local, &rx_byte, 1);
    }
}

void remote_update(void)
{
    if (alive && (HAL_GetTick() - last_rx_ms) > REMOTE_LOST_MS)
    {
        alive = 0;
        raw_vx = raw_vy = raw_wz = 0;
        raw_flags = 0;
    }
}

uint8_t remote_is_alive(void)
{
    return alive;
}

uint8_t remote_get_flags(void)
{
    return raw_flags;
}

void remote_get_cmd(float *vx, float *vy, float *wz)
{
    if (!alive || !(raw_flags & 0x01))
    {
        *vx = 0.0f;
        *vy = 0.0f;
        *wz = 0.0f;
        return;
    }

    *vx = (float)raw_vx / 1000.0f;
    *vy = (float)raw_vy / 1000.0f;
    *wz = (float)raw_wz / 1000.0f;
}
