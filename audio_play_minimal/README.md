# STM32H750B-DK minimal audio play example

The following tutorial presents how to play audio through the built-in Line Out port (green TRS 3.5mm, marked as CN9) on STM32H750B-DK.
The project is created with STM32CubeMX v6.17.0 with STM32CubeH7 MCU Firmware Package v1.13.0; STM32CubeIDE v2.1.1 was used for building and flashing.

# 1. Create the project

- Create a new project in CubeMX.
- Select the STM32H750XBH MCU.
- When asked about configuring the MPU, it doesn't matter what you picked (I picked Yes in this project).

# 2. Configure basics in CubeMX

The BSP driver that we will import later requires the PDM2PCM library, even though we won't be using any of its functionalities in this project.
The PDM2PCM library is only needed for recording audio.

- Activate CRC under `Computing` -> `CRC`.
- Activate PDM2PCM under `Middleware and Software Packs` -> `PDM2PCM`.
- Set the toolchain in `Project Manager` tab to STM32CubeIDE and click `Generate Code`.

# 3. Add driver files

STM32CubeMX downloads the STM32CubeH7 MCU Firmware Package to `~/STM32Cube\Repository\STM32Cube_FW_H7_Vx.x.x`. If you do not have it, you
can download it from https://www.st.com/en/embedded-software/stm32cubeh7.html#get-software. The destinatino and source directories listed below are the same (destination relative to the project root and source relative to MCU FP root).

- Copy SAI driver files:
  - To `Drivers/STM32H7xx_HAL_Driver/Inc`: `stm32h7xx_hal_sai.h` and `stm32h7xx_ha_sai_ex.h`
  - To `Drivers/STM32H7xx_HAL_Driver/Src`: `stm32h7xx_hal_sai.c` and `stm32h7xx_ha_sai_ex.c`

- Copy BSP drivers:
  - To `Drivers/BSP/STM32H750B-DK`: `stm32h750b_discovery_audio.c`, `stm32h750b_discovery_audio.h`, `stm32h750b_discovery_bus.c`, `stm32h750b_discovery_bus.h`, `stm32h750b_discovery_errno.h`
  - Copy `Drivers/BSP/STM32H750B-DK/stm32h750b_discovery_conf_template.h` to `Drivers/BSP/STM32H750B-DK/stm32h750b_discovery_conf.h`

- Copy the `Drivers/BSP/Components/wm8994` folder.
- Copy the `Drivers/BSP/Components/Common/audio.h` file.

# Notes

Adjust your speaker/headphones volume before running this example. Volume can also be changed through `AudioOutInit.Volume` in `main.c` or by chaning the `AMPLITUDE` in `main.h`.