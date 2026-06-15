# CLAUDE.md

## What this is

Bare-metal STM32 audio loopback demo on the **STM32H750B-DK** board
(STM32H750XBH6 MPU, Cortex-M7). It captures two-channel (stereo) PDM audio
from the on-board MEMS microphone (IMP34DT05TR, wired as a stereo PDM pair)
and plays it back in real time through the green line-out jack via the WM8994
codec. CubeMX-generated HAL project, built with STM32CubeIDE.

## Data path

```
mics ─PDM→ SAI4_A ─BDMA Ch1→ recordPDMBuf (D3 SRAM @0x38000000)
   ─[BDMA IRQ: CPU BSP_AUDIO_IN_PDMToPCM]→ RecPlayback ring (AXI SRAM, D1)
   ─DMA2_Stream1 (circular)→ SAI2_A ─I2S→ WM8994 ─→ line-out   (WM8994 ctrl via I2C4)
```

- **Audio in (instance 1):** `AUDIO_IN_DEVICE_DIGITAL_MIC` (= MIC1 | MIC2),
  `ChannelsNbr = 2`, SAI4_A in PDM mode (`MicPairsNbr = 2`), BDMA into
  `recordPDMBuf`. BDMA can *only* reach D3 SRAM, so `recordPDMBuf` is forced
  into the `.D3_SRAM` section (`0x38000000`).
- **PDM→PCM:** done by CPU in the BDMA half/complete callbacks
  (`BSP_AUDIO_IN_HalfTransfer_CallBack` / `..._TransferComplete_CallBack`).
  One PDM filter runs per channel and produces interleaved stereo PCM, written
  into `RecPlayback` at `playbackPtr`.
- **Audio out (instance 0):** WM8994 over SAI2_A + DMA2_Stream1, circular over
  `RecPlayback`, 2 channels. Configured/controlled over I2C4.
- Config: 16 kHz, 16-bit, stereo (`AUDIO_FREQUENCY`, sizes in `Core/Inc/main.h`).

## Where the logic lives

All application code is in the CubeMX `USER CODE` blocks of:

- **`Core/Src/main.c`** — init, buffer declarations, the two record callbacks
  (the actual loopback logic), `MPU_Config`, clock config.
- **`Core/Inc/main.h`** — `AUDIO_FREQUENCY`, `AUDIO_IN_PDM_BUFFER_SIZE`,
  `AUDIO_BUFF_SIZE`.

Everything else is generated or vendor code; regenerating from the `.ioc`
preserves only the `USER CODE` regions.

## File structure

```
audio_record_cubemx1.ioc      CubeMX project — edit peripherals/pins here, then regenerate
Core/
  Inc/   main.h, stm32h7xx_hal_conf.h, stm32h7xx_it.h
  Src/   main.c, stm32h7xx_hal_msp.c (peripheral MSP init), stm32h7xx_it.c (ISRs),
         system_stm32h7xx.c, sys{calls,mem}.c
  Startup/  startup_stm32h750xbhx.s
PDM2PCM/App/        pdm2pcm.c/.h — PDM2PCM middleware glue (MX_PDM2PCM_Init)
Drivers/
  BSP/Components/wm8994/         WM8994 codec driver
  BSP/STM32H750B-DK/             board BSP: stm32h750b_discovery_audio.{c,h} (BSP_AUDIO_* API),
                                 _bus.c (I2C4), _conf.h
  STM32H7xx_HAL_Driver/          ST HAL
  CMSIS/                         CMSIS core + device headers
Middlewares/ST/STM32_Audio/Addons/PDM/   PDM filter lib (libPDMFilter_CM7_GCC_wc32.a) + glo header
STM32H750XBHX_FLASH.ld          linker script — internal flash (defines RAM_D3 + .D3_SRAM)
STM32H750XBHX_RAM.ld            linker script — RAM-only variant
Debug/                          STM32CubeIDE build output (makefile + artifacts)
```

## Build / flash

Default build links against `STM32H750XBHX_FLASH.ld` (runs from internal flash).
Flash/debug with ST-LINK via the IDE or the `.launch` config.

## Gotchas

- **D3 SRAM is mandatory for the PDM buffer.** BDMA (the only DMA the SAI4
  domain uses) cannot access D1/D2 RAM. Don't move `recordPDMBuf`.
- **Cache coherency:** the MPU marks the relevant region non-cacheable
  (`MPU_Config`), so the callbacks don't do D-cache maintenance. If you change
  the MPU/region attributes, you must add `SCB_InvalidateDCache_by_Addr` around
  the PDM reads (see commented variants in `nastavitve.md`).
- After editing in CubeMX, only `USER CODE` blocks survive regeneration — keep
  app logic inside them.
