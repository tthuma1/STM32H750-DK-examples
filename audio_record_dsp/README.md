# STM32H750B-DK Digital Audio Processing Example

This project extends the `audio_record_minimal` project with digital signal processing of the captured microphone audio data. 
To learn how to the `audio_record_minimal` example, take a look at its [README.md](../audio_record_minimal/README.md).

# Project overview

# Setup tutorial

Below is the setup needed to reproduce this example, starting from `audio_record_minimal`.

# 1. Include the CMSIS DSP library

The STM32CubeH7 MCU Firmware Package (MCU FP), downloadable from https://www.st.com/en/embedded-software/stm32cubeh7.html#get-software
includes a version of the CMSIS DSP library (https://github.com/ARM-software/CMSIS-DSP). The MCU FP v1.13.0, used in this project,
comes with CMSIS DSP v1.9.0.

In the table below, each file lives at the **same relative path** in both places: copy it from
that path under the MCU FP root into the same path under your project root.

| Files to copy | Relative path (same in MCU FP and your project) |
|---------------|----------------------------------------|
| `arm_common_tables.h`<br> `arm_const_structs.h`<br> `arm_math.h` | `Drivers/CMSIS/DSP/Include` |
| `arm_biquad_cascade_df1_f32.c`<br> `arm_biquad_cascade_df1_init_f32.c`<br> `arm_fir_f32.c`<br> `arm_fir_init_f32.c` | `Drivers/CMSIS/DSP/Source` |

# 2. Update include paths in STM32CubeIDE

Open the project in STM32CubeIDE and navigate to `Project` → `Properties` → `C/C++ Build` → `Settings` → `MCU/MPU GCC Compiler` → `Include Paths`:
  - Add `../Drivers/CMSIS/DSP/Include` under `Include paths (-I)`

# 3. Copy source files from this example

Copy these files from this example project to your project:

- `Core/Src/dsp.c`
- `Core/Src/main.c`
- `Core/Inc/dsp.h`