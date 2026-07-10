# 征龙凌沧27赛季 工程机器人自定义控制器
## 暑期研发版本 - 6轴自定义控制器
### 关节定义以及对应电机
* Pitch: Unitree GO-8010-6 MIT
* Yaw: CubeMars AK80-6 MIT
* Yaw: CubeMars AK80-6 MIT
* Pitch: ZDT 42-40mm步进电机 力矩环
* Roll: ZDT 42-60mm步进电机 力矩环
* Pitch: ZDT 42-40mm步进电机 力矩环

### 仓库电机驱动说明
* 本仓库中新增了三种电机的驱动，均仿照ZLLC2026赛季代码框架中的电机驱动格式开发
  * `CubeMars AK80-6`，参考25年工程驱动，结合实验室现有AK电机进行修改开发，适配H723框架
  * `宇树 GO-8010-6电机`，RS485私有串口协议，使用DM-MC-02进行驱动时请注意开发板上RS485对应的引脚
  * `张大头 42步进电机`，EmmX固件私有CAN扩展帧协议，支持位置速度模式、速度模式（支持加速度参数）与力矩模式（电流斜率默认设置为20000mA/s，即20A/s）

#### `CubeMars AK80-6`: dvc_ak80_motor.cpp/h
* 该电机支持伺服模式（位置速度模式，速度模式）和MIT混控模式，伺服模式下可在上位中设置定时回传数据，MIT混控模式下一发一收。由于位置速度模式和速度模式使用CAN拓展帧进行控制，且考虑到自定义控制器主要使用MIT混控模式进行重力补偿和回传角度的跟踪功能，MIT控制模式通过直接发送前馈力矩也可以实现单片机跑PID闭环，相比于发送目标位置速度让电驱自动跑闭环更加灵活，因此**本驱动中只包含MIT控制模式的通信协议**，可通过自行补充`Task_PID_PeriodElapsedCallback()`并在`Task_Process_PeriodElapsedCallback()`中调用实现PID闭环算法
* 本驱动仿照26赛季达妙电机驱动编写，CAN发送函数在以下两个函数中被调用，不在drv_can中统一调用：
  * `Task_Process_PeriodElapsedCallback()`发送MIT控制指令
  * `Task_Alive_PeriodElapsedCallback()`根据当前AK80使能状态发送使能请求帧和失能请求帧
  * 补充说明：
    >当电机状态为失能时，`Task_Alive_PeriodElapsedCallback()`会不断发送失能请求帧，以达到电机保持失能且电机在接收到请求帧后进行应答以判断电机在线状态；当电机为使能模式时，仅会在电机从失能状态跳变到使能状态时发送使能请求帧，并通过检测电机返回的力矩是否保持在一个常数判断电机是否被成功使能，若未被成功使能则重发使能请求帧，原因如下：
    >> AK80电机与达妙电机不同，当电机已经处于使能状态时，继续发送使能帧会导致电机重新使能，表现在控制层面的效果则为电机在给定目标力矩的情况下，实际输出的力矩会上下震荡。注意到当电机为使能模式时，回传的力矩值会有小幅度的抖动，在失能模式时，回传的力矩值为一个不变的常数，此时对电机施加外力，回传的扭矩不变，仍然是常数，因此将这条现象作为使能成功的判据。

#### `Unitree GO-8010-6`: dvc_unitree_motor.cpp
* 该电机使用RS485总线串口通信，需要为电机分配一个串口，不同电机可以挂载在一个RS485总线上，但需要设置电机为不同ID，GO-8010-6电机只支持MIT控制模式，串口配置为8N1，波特率固定为4Mbps，无法更改，在配置CubeMX时请务必注意波特率的设置，推荐将相应串口的GPIO output速度设置为High或Very High，防止因为电平波形畸变导致通信受影响。
* 由于自定义控制器的项目去除了大部分无关的文件，因此未使用算法库中的数学库，驱动自身带有CRC校验函数与转整形的函数。
* PID闭环算法同上AK80电机
  
#### `张大头42步进电机`: dvc_zdt_motor.cpp/h
* 该驱动基于张大头官方 `STM32` 例程改造，使用 `EmmX` 固件私有 `CAN` 扩展帧协议，支持位置速度模式、速度模式和力矩模式。速度模式支持加速度参数，力矩模式本质上按电流指令工作；驱动默认限幅参数为 `Max_Torque = 0.43 N·m`、`Max_Current = 2.0 A`、`Max_Omega = 314.15927 rad/s`，默认电流斜率为 `20000 mA/s`，可在 `Class_ZDT_Motor::Init()` 中传入新参数覆盖。
* 该驱动发送依赖 `drv_can` 中补充的 `CAN_Send_Extended_Data()`，接收回调 `Class_ZDT_Motor::CAN_RxCpltCallback()` 处理完整 `Struct_CAN_Rx_Buffer *` 而不是裸 `uint8_t *`。由于 `drv_can` 是战队共用底层库，为保持最小改动和非侵入式接入，扩展帧全局过滤配置独立在上层初始化阶段补充；迁移该驱动时必须同步保留这部分配置，否则会出现只能发不能正常收的问题。
* 驱动的周期逻辑集中在 `TIM_Process_PeriodElapsedCallback()` 和 `TIM_Alive_PeriodElapsedCallback()` 中：前者负责使能/失能边沿指令、三种控制指令发送、位置查询以及可选的电流查询，后者通过查询回包更新电机在线状态。
* 位置模式下发送的是相对位置增量命令，而不是绝对位置写入；位置反馈来自位置查询，速度反馈由相邻位置查询结果差分估算，力矩反馈由电流查询结果结合转矩常数换算得到，属于估算值。当前实现以项目所需功能为主，不承诺已覆盖 `EmmX` 全部协议能力，新增协议功能前建议对照官方例程并结合实机验证。

### LCD 屏幕驱动说明

LCD 驱动位于 `User/Device/Inc/dvc_lcd.h` 和 `User/Device/Src/dvc_lcd.cpp`，参考 DM-MC02 的 LCD 例程适配 ST7789 控制器。当前使用竖屏 `240 x 280`、RGB565 色彩格式和 20 像素纵向偏移，已完成实机验证。

#### 接线与外设配置

| 信号 | STM32H723 引脚 | 配置 |
| --- | --- | --- |
| SCK | PB3 | SPI1_SCK |
| MOSI | PD7 | SPI1_MOSI |
| CS | PE15 | GPIO 推挽输出 |
| BLK | PB10 | GPIO 推挽输出 |
| RES | PB11 | GPIO 推挽输出 |
| DC | PD10 | GPIO 推挽输出 |

- SPI1 使用 TX-only 主机模式、8 位数据、CPOL High、第一边沿采样，实际速率为 `30 Mbit/s`。
- SPI1_TX 使用 `DMA2 Stream0`，DMA2 Stream0 与 SPI1 中断抢占优先级均为 `3`，低于 TIM5 控制中断的优先级 `1`。
- 当前工程未开启 D-Cache，LCD DMA 缓冲区位于 DMA 可访问的 AXI SRAM。后续若开启 D-Cache，需要在 DMA 启动前清理对应缓存行。
- IWDG1 当前在 CubeMX 配置和任务代码中关闭；重新启用时需要同时恢复初始化与周期喂狗逻辑。

#### 初始化与运行期刷新

- `Class_Controller::Init()` 通过 `LCD.Init(&hspi1)` 初始化屏幕。复位、寄存器配置、清屏和启动页绘制使用阻塞发送，但发生在 TIM4/TIM5 启动之前，不占用运行期控制周期。
- TIM5 的 1 ms 回调每 100 次只更新一次 LCD 刷新请求标记，不在中断中访问 SPI。
- `Task_Loop()` 以 10 Hz 读取左右机械臂 J0-J5 当前角度，调用 `Submit_Status()` 覆盖待显示快照，并持续调用 `Refresh()` 推进 DMA 状态机。
- `Refresh()` 每次最多启动一段 DMA 传输；DMA/SPI 回调只结束当前传输，不在中断中格式化文字或启动下一段数据。
- 驱动比较格式化后的字段内容，仅重绘发生变化的数值区域。新状态到来时覆盖旧的待显示状态，不堆积历史帧。

项目已完成默认状态页的数据提交。需要在其他任务中提交相同数据结构时，可使用以下调用方式：

```cpp
Struct_LCD_Status status = {};

for (uint8_t joint = 0; joint < LCD_JOINT_COUNT; joint++)
{
    status.Current_Joint_Angle[0][joint] = controller.Left_Arm.Get_Current_Joint_Angle(joint);
    status.Current_Joint_Angle[1][joint] = controller.Right_Arm.Get_Current_Joint_Angle(joint);
}

controller.LCD.Submit_Status(&status);
controller.LCD.Refresh();
```

`Submit_Status()` 和 `Refresh()` 必须在任务态或主循环中调用。当前只实现页面 `0`，其他页码不会触发 LCD 总线传输；背光可通过 `Set_Backlight(0/1)` 控制。内置字符集覆盖状态页所需英文、数字和常用符号，小写英文会转换为大写显示，浮点数固定显示两位小数。

## License

本仓库采用混合许可分发。

- `User/` 下自研代码默认采用 `AGPL-3.0-only`。
- `Drivers/STM32H7xx_HAL_Driver/` 保持 ST 的 `BSD-3-Clause` 许可。
- `Drivers/CMSIS/`、`DSP/`、`Src/arm_cortexM7*.lib` 与 `MDK-ARM/.cmsis/` 保持 ARM 上游 `Apache-2.0` 或包内原始许可。
- `User/Algorithm/Inc/Matrix.hpp` 保持其上游 `MIT` 归属与许可，不并入仓库根 `AGPL-3.0-only`。
- `Core/` 与 `MDK-ARM/startup_stm32h723xx.s` 保留 ST 原有头注释与分发条件，不改写为仓库根协议。

分发或修改本仓库时，请同时遵守根目录 [LICENSE](LICENSE) 与 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md) 中的边界说明。
