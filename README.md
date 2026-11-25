# 全向轮小车 / Omni-Wheel Robot Car

四轮 45° 全向轮底盘，STM32F407 做主控，ESP32-C3 做 WiFi 遥控。
世界系运动解算 + 陀螺仪航向融合 + 前馈 PI 速度环，1 kHz 闭环。

灵感来自 RoboMaster 步兵机器人，云台和发射机构暂未做（预留接口）。

![status](https://img.shields.io/badge/status-整车可遥控-brightgreen)
![mcu](https://img.shields.io/badge/MCU-STM32F407VET6-blue)
![wifi](https://img.shields.io/badge/WiFi-ESP32--C3-orange)

---

## 系统架构

```
            手机浏览器
          （网页双摇杆）
                │  WebSocket :81 / HTTP :80
                ▼
          ┌───────────┐
          │  ESP32-C3 │  WiFi AP  "omni-car"
          └─────┬─────┘
                │  UART 115200，10 字节定长帧
                ▼
          ┌───────────┐
          │ STM32F407 │  TIM6 1 kHz 闭环
          └─────┬─────┘
                │  PWM ×4
        ┌───────┴───────┐
        ▼       ▼       ▼       ▼
      DRV8871 ×4（左前/右前/左后/右后）
        │       │       │       │
      JGB37-520 减速电机 + 霍尔编码器 ×4
        │
      70 mm 全向轮 ×4，与车身成 45°
```

## 硬件配置

| 部件 | 型号 | 说明 |
|---|---|---|
| 主控 | STM32F407VET6 | 168 MHz，512 KB Flash |
| 电机 | JGB37-520-12100-EN ×4 | 12 V，1:30，空载 333 rpm，11 PPR 编码器 |
| 驱动 | DRV8871 ×4 | 单路 H 桥，PWM + 方向 |
| IMU | MPU6050 / MPU6500 | I2C1，只用陀螺仪 Z 轴 |
| WiFi | ESP32-C3 | AP 模式 + 网页遥控 |
| 轮子 | 70 mm 全向轮 ×4 | X 型布置，45° |
| 电源 | 3S 18650 | 经降压模块供 5V |

## 软件结构

```
omni_wheel_car/
├── stm32_firmware/              STM32 主控固件（CubeMX + CMake + Ninja）
│   ├── Core/
│   │   ├── user/
│   │   │   ├── pwm.c/h          四路 PWM，setpwm(id, duty)，duty ∈ [-1, 1]
│   │   │   ├── encoder.c/h      四路编码器，1 ms 采样 → rpm
│   │   │   ├── pid.c/h          前馈 + PI 速度环
│   │   │   ├── chasis.c/h       正解 / 逆解 / 航向融合 / 里程计
│   │   │   ├── imu.c/h          MPU6050 初始化 + 零偏校准
│   │   │   ├── remote.c/h       USART2 遥控接收 + 帧解析 + 超时保护
│   │   │   └── test.c/h         测试序列（速度环 / IMU / 融合 / 底盘数据集）
│   │   ├── Src/main.c           初始化 + 主循环 + TIM6 中断
│   │   └── Inc/
│   ├── CMakeLists.txt
│   ├── CMakePresets.json
│   └── allways_online.ioc       CubeMX 工程文件
└── esp32_remote/                ESP32-C3 遥控固件（PlatformIO + Arduino）
    ├── src/main.cpp             WiFi AP + 网页摇杆 + UART 发送
    ├── platformio.ini
    └── flash.ps1                一键烧录脚本
```

## 核心算法

### 运动学（世界系，右手系 x 前 / y 左 / z 上）

逆解，输入 `vx, vy (m/s)` 和 `wz (rad/s)`：

```
p_FL = K(−vx + vy) + R·wz
p_FR = K(+vx + vy) + R·wz
p_RL = K(−vx − vy) + R·wz
p_RR = K(+vx − vy) + R·wz

rpm_i = p_i · 60 / (2π·R_wheel)
```

其中 `K = 0.7071`（sin45°/2），`R_chasis = 0.185 m`，`R_wheel = 0.035 m`。

正解：

```
vx = ( −p_FL + p_FR − p_RL + p_RR ) / 4 / K
vy = (  p_FL + p_FR − p_RL − p_RR ) / 4 / K
wz = Σ p_i / 4 / R_chasis
```

### 速度环：前馈 + PI

```
u = sign(t)·(DZ + Kff·|t|) + Kp·e + I
```

- 前馈负责把工作点摆到位（实测误差 ±10%）
- Kp 改善动态响应，Ki 消除稳态误差
- 无 D 项（速度反馈噪声大，微分只会放大噪声）
- 目标为 0 且实际接近 0 时清积分，防止停车时积分残留

### 航向融合

以陀螺仪为主，编码器航向做慢速修正。**旋转时编码器打滑严重，修正必须关闭或大幅限幅**，否则会把准确的陀螺仪拖偏。

## 通信协议（STM32 ↔ ESP32）

10 字节定长帧，115200 8N1：

| 偏移 | 内容 | 类型 |
|---|---|---|
| 0 | `0xAA` | 帧头 |
| 1 | `0x55` | 帧头 |
| 2–3 | `vx` | int16 LE，mm/s |
| 4–5 | `vy` | int16 LE，mm/s |
| 6–7 | `wz` | int16 LE，mrad/s |
| 8 | flags | bit0 = 使能 |
| 9 | checksum | 前 9 字节累加取低 8 位 |

**四道安全机制**：网页需按"使能"；WebSocket 断开 300 ms 归零；STM32 收不到帧 300 ms 归零；收不到使能位直接归零。

## 编译与烧录

### STM32

需要 `arm-none-eabi-gcc` 和 CMake ≥ 3.20。

```bash
cd stm32_firmware
cmake --preset Debug
cmake --build --preset Debug
```

烧录用 `.vscode/tasks.json` 里的任务（ST-Link / DAP-Link），或 OpenOCD：

```bash
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg -c "program build/Debug/allways_online.elf verify reset exit"
```

### ESP32-C3

需要 [PlatformIO](https://platformio.org/)。首次编译会下载 ~800 MB 工具链。

```bash
cd esp32_remote
pio run                 # 编译
pio run -t upload       # 烧录
pio device monitor      # 串口监视
```

Windows 下可以直接用脚本（会先清理残留的 esptool 进程）：

```powershell
.\flash.ps1
```

如果 `platformio.ini` 里的 `upload_port` 和实际不符，改成设备管理器里的端口（ESP32 的 VID 是 `303A`，CH340 是 `1A86`）。

## 使用方法

1. 上电后 ESP32 开启热点 **omni-car**，密码 `12345678`
2. 手机连上热点，浏览器打开 `http://192.168.4.1`
3. 点 **"使能"** 按钮（不按车不会动）
4. 左摇杆控制平移，右摇杆控制转向

**遥控断开会自动停车**——页面关掉、WiFi 断开、STM32 掉线都会在 300 ms 内归零。

## 实测数据

| 项目 | 数值 |
|---|---|
| 编码器分辨率 | 1320 counts/rev（实测 ~1300） |
| 电机空载转速 | ~310 rpm @ 满占空比 |
| 静态增益 | 430 rpm / 单位占空比 |
| 死区 | 0.22 ~ 0.29（每个轮子不同） |
| 速度环阶跃上升（10–90%） | ~200 ms |
| 速度环超调 | < 5% |
| 稳态纹波 | ±4 rpm（编码器量化极限） |

## 已知限制

- **航向无绝对参考**：只有陀螺仪（漂移）和编码器（打滑）互补，长时间运行会有累积误差
- **低速抖动**：低于 ~20 rpm 时编码器分辨率不够（1 ms 采样只有 1 个计数），无法稳定闭环
- **无里程计校准**：`R_chasis` 未标定，长期里程计有比例误差
- **云台 / 发射机构未实现**：代码里预留了扩展位置

## 开发过程中踩过的坑

| 现象 | 真因 |
|---|---|
| 转速读数小 10 倍 | TIM6 配成 10 kHz，但 `K_RPM` 按 1 ms 算 |
| 闭环不收敛、duty 恒定 | PID 反馈值单位是 m/s，目标是 rpm，差 142 倍 |
| 一开闭环就飞车 | 编码器符号反了，正反馈 |
| 转圈时航向漂移 10°/轮 | 编码器航向修正把打滑误差当真实转动 |
| 超时后积分残留 | 停车时没清积分，输出卡在负占空比 |
| ESP32 热点搜不到且发烫 | PCB 天线离晶振太近，加 3.125 cm（λ/4）导线解决 |
| 串口高速通信丢字节 | 杜邦线在 460800 下信号完整性问题，降到 115200 |
| ESP32 烧录报 stub flasher 失败 | C3 原生 USB 通病，用 `--no-stub` 并清僵尸 esptool 进程 |

## 致谢

运动学符号约定参考 ROS REP-103 / ISO 8855，PID 整定参考 λ 整定（IMC）方法。

## 许可证

MIT
