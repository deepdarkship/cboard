# C Board Firmware

STM32F407 电控基础功能 + 姿态与 GM6020 电机联动工程。

## 构建

```powershell
cd C:\Users\27767\Desktop\cboard
cmake --preset Debug
cmake --build --preset Debug --parallel
```

输出：

```text
build/Debug/cboard.elf
```

## 烧录

VS Code 中使用任务：

```text
OpenOCD: flash
```

或手动执行：

```powershell
openocd -f openocd.cfg -c "program build/Debug/cboard.elf verify reset exit"
```

## 外设与引脚

| 功能 | 外设 | 引脚 | 说明 |
| --- | --- | --- | --- |
| 蜂鸣器 | TIM4_CH3 | PD14 | 无源蜂鸣器，PWM |
| RGB LED | TIM5_CH1/2/3 | PH10/PH11/PH12 | 蓝/绿/红 |
| 调试串口 | USART1 | PA9 TX / PB7 RX | 115200 8N1 |
| DT7/DBUS | USART3 | PC11 RX | 100000 8E1，DMA1 Stream1 |
| BMI088 | SPI1 | PB3/PB4/PA7 | CS: PA4/PB0 |
| GM6020 CAN | CAN1 | PD0 RX / PD1 TX | 1 Mbps |

## 串口 Plotter 帧格式

`applications/plotter_task.cpp` 使用 `sp::Plotter` 发送 3 个 float：

```text
AA BB <长度字节> <float32 x 3>
```

默认发送加速度三轴：

```text
acc_x, acc_y, acc_z
```

如果需要改为角速度三轴，修改：

```text
applications/plotter_task.cpp
constexpr bool kPlotGyro = true;
```

SerialPlot 设置：

```text
波特率：115200
Data Format：Custom Frame
Frame Start：AA BB
# Channels：3
Number of Type：float
Endianness：Little Endian
First byte of the payload is size：勾选
```

## 控制模式

右拨杆 S2：

```text
DOWN -> 失能，电机输出为 0
MID  -> 姿态联动
UP   -> R 标复位
```

左拨杆 S1：

```text
DOWN -> B 电机比例 0.5
MID  -> B 电机比例 -1
UP   -> B 电机比例 3
```

## 基础功能调试开关

`applications/linkage_task.cpp`：

```text
kEnableMotorOutput : 是否允许电机输出
kEnableManualSource: 是否允许手动转动电机作为参考零点输入
```

`applications/imu_task.cpp`：

```text
kEnableImuPlot : 是否由 IMU 任务直接发送 Plotter 数据
```

`applications/plotter_task.cpp`：

```text
kPlotGyro : false 发送加速度三轴，true 发送角速度三轴
```

注意：USART1 只能有一个 Plotter 持续发送。当前由 `plotter_task.cpp` 发送，`imu_task.cpp` 中的 `kEnableImuPlot` 保持 `false`。

## CAN 接线

- CAN_H 接 CAN_H，CAN_L 接 CAN_L。
- GM6020 使用 24 V 电源。
- 总线两端需要 120 欧终端电阻。
- 当前 `applications/can_task.hpp` 中电机 ID：

```text
A 电机：4
B 电机：2
```

实车必须与 RoboMaster Assistant 中设置的电机 ID 一致。

## 目录说明

```text
applications/       任务和应用层
sp_middleware/      电机、传感器、通信中间件
Core/               CubeMX 初始化和中断
cmake/              CMake 工具链与生成配置
```
