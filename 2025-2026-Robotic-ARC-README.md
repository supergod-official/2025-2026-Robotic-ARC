# 2025-2026 Robotic ARC

本仓库用于整理 2025-2026 年机器人竞赛与培训代码，主要包含 **2025 CLaMe**、**2026 ASABE 专项**和 **2026 ARC 培训**相关内容。项目覆盖机器人底盘运动、编码器与 PID 控制、距离和姿态感知、执行机构控制、视觉识别，以及多控制器之间的串口通信。

## 系统组成

2026 ASABE 专项采用分层控制方案：

- **树莓派 + AI HAT+**：运行 YOLOv8 视觉模型，识别双株、单株和空株等目标。
- **STM32F427ZGT6 上板**：负责视觉目标跟踪、播种机构、电机与编码器控制，并与树莓派、OpenMV 和 Arduino 通信。
- **Arduino Mega 2560 Pro 下板**：负责底盘运动、路线状态机，以及 HWT101 姿态传感器、VL53L1X 测距传感器等设备的数据采集。

## 项目结构

```text
2025-2026-Robotic-ARC/
├─ 2026ASABE专项/
│  ├─ 树莓派+AIHAT/                 # YOLOv8 数据集、模型与部署说明
│  ├─ 上板-STM32f427ZGT6/           # STM32CubeIDE + FreeRTOS 主控工程
│  │  ├─ Core/Inc/                  # 应用模块与驱动头文件
│  │  ├─ Core/Src/                  # 电机、PID、编码器、通信和业务逻辑
│  │  ├─ Drivers/                   # STM32 HAL 与 CMSIS
│  │  ├─ Middlewares/               # FreeRTOS 中间件
│  │  └─ stm32-ASABE2026.ioc        # STM32CubeMX 硬件配置
│  └─ 下板-ARDUINOmega2560pro/      # Arduino 底盘控制与导航状态机
│     ├─ 2026-ASABE.ino             # Arduino 主程序入口
│     ├─ ASABE_StateMachine.*       # 路线任务与运动状态机
│     ├─ PC_carA.h                  # 底盘运动控制
│     └─ PCencoder_.*               # 编码器读取
├─ Arduino-mega-2560/
│  ├─ 2025-clame/                   # 2025 CLaMe 相关代码
│  ├─ 2026-ASABE/                   # ASABE Arduino 控制代码
│  ├─ 2026arc/                      # 2026 ARC 编码器训练/测试代码
│  ├─ HWT101_Test/                  # HWT101 姿态传感器测试
│  └─ libraries/                    # Arduino 所需的本地依赖库
└─ README.md
```

## 快速开始

- STM32 工程请使用 **STM32CubeIDE** 导入 `2026ASABE专项/上板-STM32f427ZGT6`，详细配置见该目录下的 README。
- Arduino 工程请使用 **Arduino IDE** 打开对应的 `.ino` 文件，并将 `Arduino-mega-2560/libraries` 中的依赖库加入开发环境。
- 视觉数据集、ONNX 模型和面向 AI HAT+ 的 HEF 模型获取方式，见 `2026ASABE专项/树莓派+AIHAT/README.md`。
- 上电联调前，请先确认控制器共地、串口电平与波特率正确，并在机构悬空或断开动力电源的条件下检查电机方向和限位逻辑。

> 作者寄语：Interest is all you need.
