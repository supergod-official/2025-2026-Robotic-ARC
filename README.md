# 2025-2026 Robotic ARC

机器人竞赛与培训项目仓库，汇总 **2025 CLaMe**、**2026 ASABE 专项**与 **2026 ARC 培训赛**的代码、规则和配套资料。内容涵盖底盘运动、编码器测速、PID 控制、距离与姿态感知、执行机构、视觉识别，以及多控制器串口通信。

## 2026 培训赛课程

> [!IMPORTANT]
> **Slides 课程网站：[https://slides.supergod.icu/2026arc/](https://slides.supergod.icu/2026arc/#/)**
>
> 课程代码与规则：[2026 夏 ARC 培训赛 - 代码部分 Lec 2](./2026%E5%A4%8FARC%E5%9F%B9%E8%AE%AD%E8%B5%9B-%E4%BB%A3%E7%A0%81%E9%83%A8%E5%88%86lec2/)

本节面向初学者，内容包括霍尔编码器、M/T 测速法和 Arduino Mega 2560 中断资源。培训赛规则由 2026 ASABE 规则简化而来。


## 2026 ASABE 系统架构

2026 ASABE 专项采用分层控制方案：

- **树莓派 + AI HAT+**：运行 YOLOv8 视觉模型，识别双株、单株和空株等目标。
- **STM32F427ZGT6 上板**：负责视觉目标跟踪、播种机构、电机与编码器控制，并与树莓派、OpenMV 和 Arduino 通信。
- **Arduino Mega 2560 Pro 下板**：负责底盘运动、路线状态机，以及 HWT101 姿态传感器、VL53L1X 测距传感器等设备的数据采集。

## 项目结构

```text
2025-2026-Robotic-ARC/
├─ 2026夏ARC培训赛-代码部分lec2/
│  ├─ README.md                     # 课程说明与 Slides 地址
│  └─ 2026arc/
│     ├─ 2026arc.ino                # 编码器计数与测速示例
│     ├─ encoder.h                  # 编码器宏定义与接口
│     └─ 2026_培训赛规则.pdf         # 基于 ASABE 规则的简化版规则
├─ 2026ASABE专项/
│  ├─ 树莓派+AIHAT/                 # YOLOv8 数据集、模型与部署
│  ├─ 上板-STM32f427ZGT6/           # STM32CubeIDE + FreeRTOS 工程
│  ├─ 下板-ARDUINOmega2560pro/      # Arduino 底盘控制与导航
│  └─ 2026ASABE比赛规则.pdf         # 2026ASABE规则最终版中翻
├─ Arduino-mega-2560/
│  ├─ 2025-clame/                   # 2025 CLaMe 代码
│  ├─ 2026-ASABE/                   # ASABE Arduino 控制代码
│  ├─ 2026arc/                      # ARC 编码器训练与测试代码
│  ├─ HWT101_Test/                  # HWT101 姿态传感器测试
│  └─ libraries/                    # Arduino 本地依赖库
└─ README.md
```

## 快速开始

1. **培训课程**：先打开 [Slides 课程网站](https://slides.supergod.icu/2026arc/#/)，再使用 Arduino IDE 打开 [`2026arc.ino`](./2026%E5%A4%8FARC%E5%9F%B9%E8%AE%AD%E8%B5%9B-%E4%BB%A3%E7%A0%81%E9%83%A8%E5%88%86lec2/2026arc/2026arc.ino)。
2. **STM32 工程**：使用 STM32CubeIDE 导入 [`2026ASABE专项/上板-STM32f427ZGT6`](./2026ASABE%E4%B8%93%E9%A1%B9/%E4%B8%8A%E6%9D%BF-STM32f427ZGT6/)，配置与编译方式见该目录的 README。
3. **Arduino 工程**：使用 Arduino IDE 打开对应的 `.ino` 文件，并将 [`Arduino-mega-2560/libraries`](./Arduino-mega-2560/libraries/) 中的依赖库加入开发环境。
4. **视觉部署**：数据集、ONNX 模型和面向 AI HAT+ 的 HEF 模型获取方式见 [树莓派 + AI HAT+ 说明](./2026ASABE%E4%B8%93%E9%A1%B9/%E6%A0%91%E8%8E%93%E6%B4%BE%2BAIHAT/README.md)。

> [!CAUTION]
> 本项目仅面向个人学习,缺少openmv等基础模块,并不能完全复刻the Robot。

> Interest is all you need.
