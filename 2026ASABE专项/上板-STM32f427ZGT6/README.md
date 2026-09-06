# stm32-ASABE2026

ASABE 2026 机器人 STM32 主控固件。本工程负责执行机构电机与编码器控制、双侧视觉目标跟踪、播种动作流程，以及与树莓派、OpenMV、Arduino 等设备的串口通信。

## 硬件与开发环境

| 项目 | 配置 |
| --- | --- |
| MCU | STM32F427ZGT6，LQFP144 |
| 系统时钟 | 168 MHz |
| 外部晶振 | HSE 8 MHz |
| 开发工具 | STM32CubeIDE |
| 代码生成器 | STM32CubeMX 6.15.0 |
| STM32 固件包 | STM32Cube FW_F4 V1.28.3 |
| 编译器 | GNU Tools for STM32（GCC） |
| 操作系统 | FreeRTOS，CMSIS-RTOS v2 |
| 下载/调试 | ST-LINK，SWD |

## 主要功能

- 多路直流电机 PWM 控制、制动与软电机控制。
- 基于定时器输入捕获的正交编码器读取。
- 电机速度/位置 PID 控制。
- 左右两侧旋转、伸缩和视觉跟踪机构控制。
- 左右播种机构动作状态机。
- 接收树莓派双侧视觉目标，完成目标位置换算、滤波与跟踪。
- 接收左右 OpenMV 数据和 Arduino 工作指令。
- 使用 FreeRTOS 分离运动控制、通信解析和遥测发送。

## FreeRTOS 任务

| 任务 | 优先级 | 栈大小 | 用途 |
| --- | --- | --- | --- |
| `defaultTask` | Normal | 512 B | CubeMX 默认空闲任务 |
| `MotionTask` | Above Normal | 2048 B | 视觉滤波、运动控制和主业务逻辑 |
| `TelemetryTask` | Below Normal | 1024 B | 向树莓派发送统计/状态数据 |
| `CommTask` | Normal | 2048 B | 串口帧队列接收和协议解析 |

串口中断只负责收帧并投递消息，协议解析在 `CommTask` 中完成。运动任务按代码中的固定周期运行，修改系统 Tick 或任务周期后应重新检查控制参数。

## 外设分配

### 串口

| 串口 | 引脚 | 波特率 | 当前用途 |
| --- | --- | ---: | --- |
| UART4 | TX `PC10` / RX `PC11` | 9600 | 蓝牙调试接口；当前默认未启动接收 |
| UART7 | TX `PF7` / RX `PF6` | 115200 | 树莓派双侧视觉数据和状态回传 |
| USART1 | TX `PA9` / RX `PA10` | 115200 | Arduino 工作指令 |
| USART2 | TX `PD5` / RX `PD6` | 115200 | 左侧 OpenMV |
| USART3 | TX `PB10` / RX `PD9` | 115200 | 右侧 OpenMV |

所有接口均为 3.3 V TTL 电平。连接外部设备时必须共地，不要把 RS-232 电平直接接入 MCU。

### 定时器

| 定时器 | 用途 |
| --- | --- |
| TIM2、TIM3、TIM8 | 编码器输入捕获 |
| TIM4 | 4 路 PWM 输出 |
| TIM9 | 2 路 PWM 输出 |
| TIM6 | 控制逻辑计时 |
| TIM7 | 软件电机更新定时 |
| TIM1 | HAL 系统时基 |

具体 GPIO、定时器通道和中断配置以 [`stm32-ASABE2026.ioc`](stm32-ASABE2026.ioc) 为准。

## 串口协议摘要

多字节整数和浮点数按小端序传输。

### 树莓派 → STM32（UART7，12 字节）

| 字节 | 内容 |
| ---: | --- |
| 0 | 新帧标志位：bit0 左侧、bit1 右侧 |
| 1 | 左侧目标状态 |
| 2–3 | 左侧水平偏移，`int16` |
| 4–5 | 左侧水平速度，`int16` |
| 6 | 右侧目标状态 |
| 7–8 | 右侧水平偏移，`int16` |
| 9–10 | 右侧水平速度，`int16` |
| 11 | 帧尾 `0xAA` |

### STM32 → 树莓派（UART7，10 字节）

```text
0x55, total, empty, green, yellow, left_type, right_type, seq, xor, 0xAA
```

### OpenMV → STM32（USART2/USART3，6 字节）

```text
stable, offset_x(int16), offset_y(int8), in_window, 0xAA
```

### Arduino → STM32（USART1，6 字节）

```text
enable, car_speed(float32), 0xAA
```

协议的最终实现位于 `Core/Src/main.c`。修改发送端协议时，应同步修改长度、帧头/帧尾、字段顺序和解析逻辑。

## 工程目录

```text
stm32-ASABE2026/
├─ Core/
│  ├─ Inc/                 # 应用层和驱动头文件
│  └─ Src/                 # 应用层、外设和 FreeRTOS 源文件
├─ Drivers/                # STM32 HAL 与 CMSIS
├─ Middlewares/            # FreeRTOS/CMSIS-RTOS v2
├─ .settings/              # CubeIDE 工程设置
├─ stm32-ASABE2026.ioc     # CubeMX 硬件配置
├─ STM32F427ZGTX_FLASH.ld  # Flash 链接脚本
├─ STM32F427ZGTX_RAM.ld    # RAM 链接脚本
├─ .project
├─ .cproject
└─ README.md
```

## 导入与编译

1. 安装 STM32CubeIDE，并确保 STM32CubeF4 固件包可用。
2. 打开 STM32CubeIDE，选择 `File > Import...`。
3. 选择 `General > Existing Projects into Workspace`。
4. 将工程根目录选择为本目录，导入项目 `stm32-ASABE2026`。
5. 执行 `Project > Clean...`。
6. 执行 `Project > Build Project`。

正常构建后，主要产物位于 `Debug/`：

```text
Debug/stm32-ASABE2026.elf
Debug/stm32-ASABE2026.map
Debug/stm32-ASABE2026.list
```

如果工作区中仍保留旧工程名，建议先从 CubeIDE 工作区移除旧项目（不要勾选删除磁盘文件），再重新导入本目录。

## 下载与调试

1. 使用 ST-LINK 连接目标板的 `SWDIO`、`SWCLK`、`GND`，并确认目标板供电正常。
2. 在 CubeIDE 中选择 Debug 构建配置。
3. 使用 `stm32-ASABE2026 Debug` 调试配置，或让 CubeIDE 重新创建调试配置。
4. 首次下载前先断开电机动力电源或抬起运动机构，确认 PWM 极性、限位和急停逻辑后再带载测试。

调试配置中的日志路径可能包含本机绝对路径。工程移动到其他位置后，如调试启动失败，删除旧调试配置并由 CubeIDE 重新生成即可。

## 修改 CubeMX 配置时的注意事项

- 生成代码前先提交代码或创建备份。
- 保持自定义代码位于 `USER CODE BEGIN/END` 区域内。
- `Motor.c`、`MotorPidController.c`、`Car__.c`、`VisionSpeedTrack.c` 等自定义模块不会由 CubeMX 自动重建，但仍应在生成后检查工程差异。
- 重新生成代码后确认 FreeRTOS 源文件、头文件搜索路径和 `Middlewares` 目录仍在工程中。
- 不要手动长期维护 `Debug/makefile`；它应由 CubeIDE 根据 `.cproject` 重新生成。

## 打包与交付

共享可重新编译的源码工程时，应包含：

```text
.project
.cproject
.mxproject
.settings/
stm32-ASABE2026.ioc
STM32F427ZGTX_FLASH.ld
STM32F427ZGTX_RAM.ld
Core/
Drivers/
Middlewares/
README.md
```

以下内容通常不要放入源码压缩包：

```text
.git/
Debug/
Release/
Core.zip
*.o
*.d
*.su
*.cyclo
*.elf
*.map
*.list
__pycache__/
*.pyc
```

如果只交付可烧录固件，可在 CubeIDE 中启用 HEX/BIN 输出，然后提供 `.hex` 或 `.bin` 文件，并同时注明 MCU 型号和固件版本。

## 联调检查清单

- MCU 型号与链接脚本一致：STM32F427ZGT6。
- HSE 为 8 MHz，系统主频为 168 MHz。
- STM32 与所有串口设备共地，电平为 3.3 V TTL。
- 树莓派、OpenMV 和 Arduino 的波特率与字段顺序一致。
- 电机方向、编码器方向、PWM 通道和机械限位正确。
- FreeRTOS 已启动，`MotionTask`、`CommTask` 和 `TelemetryTask` 正常运行。
- 带载前已验证急停、复位和异常通信情况下的安全状态。
