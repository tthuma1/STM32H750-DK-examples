## What this is

A bare-metal **STM32CubeMX** example for the **STM32H750B-DK** board (**STM32H750XBH6**,
Cortex-M7): it generates a sine wave in software and plays it out the line-out / headphone
jack over **SAI2** + the on-board **WM8994** codec. Playback only — no microphone capture.

## Audio parameters

Defined in `Core/Inc/main.h`:

- `SAMPLE_RATE` = 96000 Hz, stereo, 16-bit (`AUDIO_RESOLUTION_16B`)
- `TONE_FREQ`,`AMPLITUDE`
- `AUDIO_BUFFER_SIZE` = 2048 **bytes** → 1024 int16 samples → 512 stereo frames

## Data path

```
SRAM buffer (int16 stereo)
   ▼  DMA2_Stream1  (circular, mem→periph, request SAI2_A)
SAI2 Block A TX  ──I2S── PI4=MCLK, PI5=SCK, PI6=SD, PI7=FS (AF10) ──▶
WM8994 codec  (control registers are set over I2C4)
   ▼
green LINE-OUT / headphone jack
```

The buffer is a circular ping-pong double buffer: DMA half/complete interrupts trigger
regeneration of the just-played half (cleaned from D-Cache) while the other half plays; the
DMA never stops. SAI clock (PLL2) and SAI register setup live in the BSP driver
(`stm32h750b_discovery_audio.c`), not in `main.c`; `SystemClock_Config()` runs SYSCLK from HSI.

## File structure

| Path | Role |
|------|------|
| `Core/Src/main.c` | **Application.** Tone generation, double-buffer logic, DMA callbacks, clock/MPU/cache init. This is the file you edit. |
| `Core/Inc/main.h` | Audio parameter `#define`s (sample rate, tone freq, buffer size). |
| `Core/Inc/stm32h7xx_hal_conf.h` | HAL module enables (`HAL_SAI_MODULE_ENABLED` is on here). |
| `Core/Src/stm32h7xx_it.c`, `Core/Src/stm32h7xx_hal_msp.c` | ISRs and MSP init (CubeMX-generated). |
| `Drivers/BSP/STM32H750B-DK/stm32h750b_discovery_audio.{c,h}` | Board audio driver: `BSP_AUDIO_OUT_Init/Play`, SAI+DMA setup, pin/clock macros. |
| `Drivers/BSP/STM32H750B-DK/stm32h750b_discovery_bus.{c,h}` | I2C4 bus used to talk to the codec. |
| `Drivers/BSP/STM32H750B-DK/stm32h750b_discovery_conf.h` | BSP config (copied from `..._conf_template.h`). |
| `Drivers/BSP/Components/wm8994/` | WM8994 codec register driver. |
| `Drivers/BSP/Components/Common/audio.h` | Shared BSP audio types. |
| `Drivers/STM32H7xx_HAL_Driver/` | ST HAL (includes `stm32h7xx_hal_sai.c/.h` added manually). |
| `PDM2PCM/`, CRC init | Enabled for BSP include dependencies only; **unused** by the app. |
| `audio_play_cubemx1.ioc` | CubeMX project — regenerate code from here; keep edits inside `USER CODE` markers. |
| `STM32H750XBHX_FLASH.ld` / `_RAM.ld` | Linker scripts. |

## Working notes

- **D-Cache is enabled** (`SCB_EnableDCache()` in `main`). Any buffer the DMA reads must be
  `ALIGN_32BYTES` and `SCB_CleanDCache_by_Addr`'d after the CPU writes it. `buffer_ctl` already
  follows this.
- Do not try building or flashing the project.
