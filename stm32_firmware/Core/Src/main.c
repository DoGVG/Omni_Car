/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "pwm.h"
#include "encoder.h"
#include "chasis.h"
#include "imu.h"
#include "pid.h"
#include "test.h"
#include "remote.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
pid_t pid[4];
float pid_duty[4];      /* PID 输出占空比，供遥测显示 */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM8_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM6_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /* 1. 先让电机停住：pwm_init 内部先清 CCR 再启动定时器 */
  pwm_init();
  setcoast();

  /* 2. 启动四个编码器 */
  enc_init();

  /* 3. 底盘状态清零 */
  chasis_init();

  /* 4. PID 参数初始化（四个轮子一起，参数表在 pid.c 里） */
  pid_init(pid);

  /* 5. IMU 初始化：阻塞约 1 秒做零偏校准，返回 0 表示成功
   *    失败不在这里停机（速度环可以单独调试），只在串口提示一句 */
  if (imu_init() != 0)
  {
    HAL_UART_Transmit(&huart1, (uint8_t *)"IMU init failed\r\n", 17, 100);
  }

  /* 6. 遥控接收（USART2 + PA2/PA3） */
  remote_init();

  /* 7. 测试序列复位 */
  test_init();

  /* 8. 最后才开 1 kHz 中断，开始闭环控制 */
  HAL_TIM_Base_Start_IT(&htim6);

  uint32_t last_tx  = 0;
  uint32_t last_imu = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint32_t now = HAL_GetTick();

    /* 200 Hz：读 MPU6050（阻塞约 340us，放主循环安全） */
    if (now - last_imu >= 5)
    {
      last_imu = now;
      imu_update();
    }

    /* 100 Hz：遥测 */
    if (now - last_tx >= 10)
    {
      last_tx = now;
      test_run_telemetry(pid_duty);
    }

    /* 遥控超时检测 */
    remote_update();

    /* 遥控调试：500 ms 一次，CSV 便于 VOFA FireWater 画
     * 通道：alive, flags, vx(mm/s), vy(mm/s), wz(mrad/s)
     */
    {
      static uint32_t t_dbg = 0;
      if (now - t_dbg >= 500)
      {
        t_dbg = now;
        float dvx, dvy, dwz;
        remote_get_cmd(&dvx, &dvy, &dwz);
        char m[72];
        int n = snprintf(m, sizeof(m), "%d,%d,%d,%d,%d\n",
                         (int)remote_is_alive(),
                         (int)remote_get_flags(),
                         (int)(dvx * 1000.0f),
                         (int)(dvy * 1000.0f),
                         (int)(dwz * 1000.0f));
        HAL_UART_Transmit(&huart1, (uint8_t *)m, (uint16_t)n, 50);
      }
    }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /* 控制环在 TIM6 中断里，主循环留作遥控解析、保护、显示 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/*
  1khz中断，分频5置标志位用于主循环读mpu6050
  1khz积分解算yaw,enc,pid控制

*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM6)
  {
    /* 1. 读编码器 */
    enc_update();

    /* 2. 底盘正解 + 航向融合 + 里程计 */
    chasis_update();

    /* 3. 取速度指令并逆解 */
    float vx_cmd, vy_cmd, wz_cmd;
#if REMOTE_ENABLE
    remote_get_cmd(&vx_cmd, &vy_cmd, &wz_cmd);
#else
    test_get_cmd(&vx_cmd, &vy_cmd, &wz_cmd);
#endif
    chasis_targetrpm_cal(vx_cmd, vy_cmd, wz_cmd);

    /* 4. 四轮速度环 */
    for (uint8_t i = 0; i < 4; i++)
    {
      pid_duty[i] = pid_calc(&pid[i], chasis_get_rpm_tar(i), enc_get_speed_rpm(i));
      setpwm(i, pid_duty[i]);
    }

    /* 5. 采样快照（每 10 拍一次），主循环负责发送 */
    test_snapshot();
  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
