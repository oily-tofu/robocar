项目名称（Project Title）

一句话描述你的项目，例如：

基于 STM32 的直流电机测速与 PID 速度控制系统

📌 项目简介（Introduction）

用几句话介绍你的工程做什么、解决了什么问题，例如：

本项目使用 STM32F103 + 编码器测速 + 增量式 PID 实现电机速度闭环控制。
支持 PWM 输出、串口调试、速度显示，并提供可扩展的控制结构，适用于小车、机械臂底座等运动控制。

✨ 功能特性（Features）

✅ PWM 控制，最高频率可达 X kHz

✅ 编码器测速，单位支持 m/s、rpm

✅ 增量式 PID + 死区补偿

✅ 串口实时数据显示

✅ 可直接移植到机器人底盘或小车项目

✅ 清晰模块化结构（PWM/Encoder/PID/UART）

🛠 使用到的技术（Tech Stack）

MCU：STM32F103C8T6 / STM32F407（根据你的项目写）

编译器：Keil MDK 5

语言：C

外设：TIM、PWM、Encoder、USART

算法：增量式 PID、速度滤波（如用到）

📂 项目结构（Project Structure）
├── Core/
│   ├── main.c
│   └── ...
├── Hardware/
│   ├── pwm.c / pwm.h
│   ├── encoder.c / encoder.h
│   ├── pid.c / pid.h
│   └── usart.c / usart.h
├── README.md
└── ...

🚀 快速开始（Quick Start）

克隆仓库

git clone https://github.com/xxx/your_project.git


打开 Keil 工程

project.uvprojx


编译并下载到 STM32

串口监视器设置

波特率：115200

数据格式：8N1

上电后会连续输出当前速度与 PWM 值

⚙️ PID 参数说明（PID Parameters）

PID 使用增量式计算：

float delta_u =
    Kp * (Ek - Ek_1) +
    Ki * Ek +
    Kd * (Ek - 2*Ek_1 + Ek_2);


死区设置：

Out_Min 为电机能克服静摩擦力的最小 PWM

一般设定为 5%〜15% 最大 PWM

例如：

PID_Init(&speed_pid, 2.0f, 0.3f, 0.01f, 450, 900);

📈 控制效果（Results）

你可以放一些运行效果：

电机加速曲线图

串口输出示例

控制效果视频截图

例如：

目标速度：0.5 m/s
实际稳定：±0.02 m/s 内
响应时间：0.15 s

🔧 TODO（未来计划）

 支持 CAN 通讯

 增加速度低通滤波

 支持双电机差速控制

 支持 ROS2 接口

📄 许可证（License）

一般选 MIT：

本项目使用 MIT License，允许自由使用、修改和商用。

🤝 联系方式（Contact）

如果别人要联系你：

作者：oily_tofu

Email：xxx@example.com

Bilibili / GitHub / 微信交流群（可选）项目名称（Project Title）

一句话描述你的项目，例如：

基于 STM32 的直流电机测速与 PID 速度控制系统

📌 项目简介（Introduction）

用几句话介绍你的工程做什么、解决了什么问题，例如：

本项目使用 STM32F103 + 编码器测速 + 增量式 PID 实现电机速度闭环控制。
支持 PWM 输出、串口调试、速度显示，并提供可扩展的控制结构，适用于小车、机械臂底座等运动控制。

✨ 功能特性（Features）

✅ PWM 控制，最高频率可达 X kHz

✅ 编码器测速，单位支持 m/s、rpm

✅ 增量式 PID + 死区补偿

✅ 串口实时数据显示

✅ 可直接移植到机器人底盘或小车项目

✅ 清晰模块化结构（PWM/Encoder/PID/UART）

🛠 使用到的技术（Tech Stack）

MCU：STM32F103C8T6 / STM32F407（根据你的项目写）

编译器：Keil MDK 5

语言：C

外设：TIM、PWM、Encoder、USART

算法：增量式 PID、速度滤波（如用到）

📂 项目结构（Project Structure）
├── Core/
│   ├── main.c
│   └── ...
├── Hardware/
│   ├── pwm.c / pwm.h
│   ├── encoder.c / encoder.h
│   ├── pid.c / pid.h
│   └── usart.c / usart.h
├── README.md
└── ...

🚀 快速开始（Quick Start）

克隆仓库

git clone https://github.com/xxx/your_project.git


打开 Keil 工程

project.uvprojx


编译并下载到 STM32

串口监视器设置

波特率：115200

数据格式：8N1

上电后会连续输出当前速度与 PWM 值

⚙️ PID 参数说明（PID Parameters）

PID 使用增量式计算：

float delta_u =
    Kp * (Ek - Ek_1) +
    Ki * Ek +
    Kd * (Ek - 2*Ek_1 + Ek_2);


死区设置：

Out_Min 为电机能克服静摩擦力的最小 PWM

一般设定为 5%〜15% 最大 PWM

例如：

PID_Init(&speed_pid, 2.0f, 0.3f, 0.01f, 450, 900);

📈 控制效果（Results）

你可以放一些运行效果：

电机加速曲线图

串口输出示例

控制效果视频截图

例如：

目标速度：0.5 m/s
实际稳定：±0.02 m/s 内
响应时间：0.15 s

🔧 TODO（未来计划）

 支持 CAN 通讯

 增加速度低通滤波

 支持双电机差速控制

 支持 ROS2 接口

📄 许可证（License）

一般选 MIT：

本项目使用 MIT License，允许自由使用、修改和商用。

🤝 联系方式（Contact）

如果别人要联系你：

作者：oily_tofu

Email：xxx@example.com

Bilibili / GitHub / 微信交流群（可选）
