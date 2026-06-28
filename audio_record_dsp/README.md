# STM32H750B-DK Digital Audio Processing Example

This project extends the `audio_record_minimal` project with digital signal processing of the captured microphone audio data. 
To learn how to the `audio_record_minimal` example, take a look at its [README.md](../audio_record_minimal/README.md).

Below is the setup needed to reproduce this example, starting from `audio_record_minimal`.

# 1. Include the CMSIS DSP library

The STM32CubeH7 MCU Firmware Package (MCU FP), downloadable from https://www.st.com/en/embedded-software/stm32cubeh7.html#get-software
includes a version of the CMSIS DSP library (https://github.com/ARM-software/CMSIS-DSP). The MCU FP v1.13.0, used in this project,
comes with CMSIS DSP v1.9.0

You must copy the following files from your MCU FP folder to this project (the MCU FP relative paths are the same as in this project):

- `<MCU_FP>/Drivers/CMSIS/DSP/Include`:
  - arm_common_tables.h, arm_const_structs.h, arm_math.h
- `<MCU_FP>/Drivers/CMSIS/DSP/Source`:
  - arm_biquad_cascade_df1_f32.c, arm_biquad_cascade_df1_init_f32.c, arm_fir_f32.c, arm_fir_init_f32.c,

# 2. Update include path in STM32CubeIDE

Open the project in STM32CubeIDE and navigate to `Project` → `Properties` → `C/C++ Build` → `Settings` → `MCU/MPU GCC Compiler` → `Include Paths`:
  - Add `../Drivers/CMSIS/DSP/Include` under `Include paths (-I)`

