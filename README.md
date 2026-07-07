# 征龙凌沧27赛季 工程机器人自定义控制器
## 暑期研发版本 - 6轴自定义控制器
### 关节定义以及对应电机
* Pitch: Unitree GO-8010-6 MIT
* Yaw: CubeMars AK80-6 MIT
* Yaw: CubeMars AK80-6 MIT
* Pitch: ZDT 42-40mm步进电机 MIT
* Roll: ZDT 42-60mm步进电机 MIT
* Pitch: ZDT 42-40mm步进电机 MIT

## License

本仓库采用混合许可分发。

- `User/` 下自研代码默认采用 `AGPL-3.0-only`。
- `Drivers/STM32H7xx_HAL_Driver/` 保持 ST 的 `BSD-3-Clause` 许可。
- `Drivers/CMSIS/`、`DSP/`、`Src/arm_cortexM7*.lib` 与 `MDK-ARM/.cmsis/` 保持 ARM 上游 `Apache-2.0` 或包内原始许可。
- `User/Algorithm/Inc/Matrix.hpp` 保持其上游 `MIT` 归属与许可，不并入仓库根 `AGPL-3.0-only`。
- `Core/` 与 `MDK-ARM/startup_stm32h723xx.s` 保留 ST 原有头注释与分发条件，不改写为仓库根协议。

分发或修改本仓库时，请同时遵守根目录 [LICENSE](LICENSE) 与 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md) 中的边界说明。
