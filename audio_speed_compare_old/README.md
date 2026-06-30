# Audio Capture Speed Comparison — DMA vs. Interrupt vs. Polling

Bare-metal STM32H750B-DK (Cortex-M7) audio loopback that measures the **CPU
cycles spent per audio block** to capture PDM mic data and convert it to PCM,
under three SAI4 receive strategies: **DMA**, **interrupt (IT)**, and
**polling**. Same data path as the minimal record example; the only variable is
how the PDM bitstream gets from SAI4 into RAM.

## Data path

```
MEMS mics ─PDM→ SAI4_A ─[capture: DMA | IT | polling]→ recordPDMBuf (D3 SRAM @0x38000000)
   ─CPU BSP_AUDIO_IN_PDMToPCM()→ RecPlayback ring (AXI SRAM, D1)
   ─DMA2_Stream1 (circular)→ SAI2_A ─I2S→ WM8994 ─→ line-out   (codec ctrl via I2C4)
```

16 kHz, 16-bit, stereo. `AUDIO_IN_PDM_BUFFER_SIZE = 256` uint16 (two
half-buffers), `AUDIO_BUFF_SIZE = 4096`. SYSCLK 400 MHz (PLL1 from 25 MHz HSE),
HCLK 200 MHz, D/I-cache on. PDM→PCM is always done by the CPU; only the capture
transport changes.

## Selecting the mode

Set one macro in `Core/Inc/main.h`, rebuild, and read `g_work_cycles`:

```c
#define SAI_RX_MODE   SAI_RX_MODE_DMA     /* 0: BDMA circular, half + complete callbacks */
                    /* SAI_RX_MODE_IT       1: SAI FIFO interrupt, re-armed ping-pong   */
                    /* SAI_RX_MODE_POLLING  2: blocking HAL_SAI_Receive in main loop    */
```

`g_work_cycles` (extern, in `main.h`) holds the cycle count for the most recent
block — watch it live in the debugger. Measured with the **DWT cycle counter**
(enabled in `main()` after `MX_PDM2PCM_Init`).

## The three modes

| Mode | Transport | Where work runs | What `g_work_cycles` counts |
|------|-----------|-----------------|------------------------------|
| **DMA** | BDMA circular over the full buffer | half-/complete-transfer callbacks | CPU-active cycles only: `s_half_cycles` (half callback) + complete-callback work. Idle/DMA transfer excluded. |
| **IT** | SAI FIFO interrupt, manually re-armed ping-pong (`HAL_SAI_Receive_IT`) | every FIFO ISR (`SAI4_IRQHandler`) | CPU-active cycles only: `s_it_acc` accumulated across all FIFO ISRs for the block, gaps between ISRs excluded. Many ISRs per block. |
| **Polling** | blocking `HAL_SAI_Receive(HAL_MAX_DELAY)` in the `while(1)` loop | the main loop | Full block period — includes the busy-wait inside the blocking receive, since the CPU spins on the FIFO the whole time. |

Expected ordering (per `main.h`): **DMA < IT < Polling**. DMA offloads the
transfer entirely; IT pays per-FIFO ISR entry/exit overhead; polling burns the
CPU spinning on the FIFO for the whole block.

### Measurement details

- **DMA / IT** isolate *CPU-active* time only — idle stretches while the BDMA or
  SAI FIFO fills are excluded, so the numbers reflect real CPU cost.
- **IT** uses a ping-pong over the two half-buffers re-armed inside the complete
  callback; `SAI4_IRQHandler` brackets each `HAL_SAI_IRQHandler` call with DWT
  stamps and resets the accumulator at block start (`XferCount == XferSize`).
- **Polling** is wall-clock for the block by nature: the blocking receive does
  not return until the buffer is full, so the count includes that wait.
- Cache maintenance (`SCB_InvalidateDCache_by_Addr` before reading PDM,
  `SCB_CleanDCache_by_Addr` after writing PCM) runs inside the measured region in
  all modes; buffers are `ALIGN_32BYTES`.

## Where the logic lives

- **`Core/Src/main.c`** — buffers, DWT setup, mode-selected record start, the two
  BSP record callbacks (DMA/IT processing + instrumentation), polling loop.
- **`Core/Src/stm32h7xx_it.c`** — `SAI4_IRQHandler` with IT-mode cycle
  accumulation (no-op in DMA/polling since `SAI4_IRQn` stays disabled).
- **`Core/Inc/main.h`** — `SAI_RX_MODE` selector, audio config, `g_work_cycles`.
