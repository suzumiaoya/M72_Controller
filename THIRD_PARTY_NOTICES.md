# Third-Party Notices

This repository is distributed as a mixed-license project.

- The root [LICENSE](LICENSE) file applies to self-authored source files unless a file or directory is explicitly covered by a different upstream notice.
- Third-party, vendor, generated, and upstream-derived files keep their original notices and licenses.
- Do not remove or replace upstream copyright, license, or attribution statements when redistributing modified versions.

## License boundaries

| Path | Origin | Effective license / notice |
| --- | --- | --- |
| `User/` | Project self-authored application, driver, device, controller, interaction, and task code | `AGPL-3.0-only`, except where noted below |
| `User/Algorithm/Inc/Matrix.hpp` | Upstream `SJTU-RoboMaster-Team/Matrix_and_Robotics_on_STM32` matrix library by Team JiaoLong-SJTU / Spoon Guan | Upstream `MIT`; excluded from the repository root `AGPL-3.0-only` relicensing |
| `Drivers/STM32H7xx_HAL_Driver/` | STMicroelectronics HAL package | `BSD-3-Clause` per [Drivers/STM32H7xx_HAL_Driver/LICENSE.txt](Drivers/STM32H7xx_HAL_Driver/LICENSE.txt) |
| `Drivers/CMSIS/` | ARM CMSIS and CMSIS-DSP source packages | `Apache-2.0` per [Drivers/CMSIS/LICENSE.txt](Drivers/CMSIS/LICENSE.txt) and package-local notices |
| `Drivers/CMSIS/Device/ST/STM32H7xx/` | ST device headers and templates | ST package terms; fallback `Apache-2.0` per [Drivers/CMSIS/Device/ST/STM32H7xx/LICENSE.txt](Drivers/CMSIS/Device/ST/STM32H7xx/LICENSE.txt) |
| `DSP/` | ARM CMSIS-DSP headers and static library | `Apache-2.0` |
| `Src/arm_cortexM7*.lib` and `DSP/libCMSISDSP.a` | ARM CMSIS-DSP prebuilt binaries | `Apache-2.0` |
| `Core/` | STM32CubeMX / ST generated board support and startup-adjacent application files | Preserve ST header notices; not relicensed by the repository root `LICENSE` |
| `MDK-ARM/startup_stm32h723xx.s` | ST startup template | Preserve ST header notice; not relicensed by the repository root `LICENSE` |
| `MDK-ARM/.cmsis/` | ARM CMSIS pack content committed with the IDE project | Preserve upstream ARM package notices, generally `Apache-2.0` |

## Matrix library note

The file `User/Algorithm/Inc/Matrix.hpp` is retained as a third-party component and is not covered by the repository root `AGPL-3.0-only` grant.

- Upstream project: <https://github.com/SJTU-RoboMaster-Team/Matrix_and_Robotics_on_STM32>
- Upstream organization: <https://github.com/SJTU-RoboMaster-Team>
- Local header authorship note is preserved for attribution.

## Redistribution guidance

- If you modify and redistribute self-authored files under `User/`, provide the corresponding source code under `AGPL-3.0-only`.
- If you modify third-party files, keep their upstream license notices intact and comply with their original terms.
- If you distribute firmware or binaries built from this repository, ensure the corresponding source and applicable third-party notices remain available to recipients.
