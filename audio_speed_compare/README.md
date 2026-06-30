# STM32H750B-DK Audio Capture Speed Compariso: DMA vs. Interrupt vs. Pollingn

This project extends the `audio_record_minimal` project with comparing CPU time of different transfer modes of data between
the SAI peripheral and SRAM. 
To learn how the `audio_record_minimal` example works, take a look at its [README.md](../audio_record_minimal/README.md).

# Project overview

Measures the **CPU
cycles spent per audio block** to capture PDM mic data and convert it to PCM,
under three SAI4 receive strategies: **DMA**, **interrupt (IT)**, and
**polling**. Same data path is used as the minimal record example; the only variable is
how the PDM bitstream gets from SAI4 into RAM (the `audio_record_minimal` example uses DMA).

## Selecting the mode

Set one macro in `Core/Inc/main.h`, rebuild, and read `g_work_cycles`:

```c
#define SAI_RX_MODE   SAI_RX_MODE_DMA     /* 0: BDMA circular, half + complete callbacks */
                    /* SAI_RX_MODE_IT       1: SAI FIFO interrupt, restarted each half-buffer */
                    /* SAI_RX_MODE_POLLING  2: blocking HAL_SAI_Receive in main loop    */
```

`g_work_cycles` (extern, in `main.h`) holds the cycle count for the most recent
block — watch it live in the debugger. Measured with the **DWT cycle counter**
(enabled in `main()` after `MX_PDM2PCM_Init`).

## The three modes

| Mode | Transport | Where work runs | What is measured |
|------|-----------|-----------------|------------------------------|
| **DMA** | BDMA circular over the full buffer | half-/complete-transfer callbacks | CPU isn't used for transfering data, so only half-callback + complete-callback work is measured. |
| **IT** | SAI FIFO interrupt, restarted into the other half-buffer each block (`HAL_SAI_Receive_IT`) | every FIFO ISR (`SAI4_IRQHandler`) | Accumulates across all FIFO ISRs for the audio block, gaps between ISRs excluded. Many ISRs per block (one per word), where each ISR call moves a piece of data between SAI and RAM. PDM→PCM time is also counted after a half-buffer is filled. |
| **Polling** | blocking `HAL_SAI_Receive(HAL_MAX_DELAY)` in the `while(1)` loop | the main loop | Includes the busy-wait inside the blocking receive, since the CPU polls the FIFO continuously until the buffer is full. Measured is also the transfer time from SAI to RAM and PDM→PCM conversion |

For DMA, `g_work_cycles` will be just the PDM→PCM conversion. For IT, it will include the accumulated word-transfer time in ISR calls plus PDM→PCM. For polling, it captures the full blocking wait plus PDM→PCM.


Expected ordering (per `main.h`): **DMA < IT < Polling**. DMA offloads the
transfer entirely; IT pays per-FIFO ISR entry/exit overhead; polling burns the
CPU spinning on the FIFO for the whole block.


# Results

On my board, each of the modes uses the following amount of CPU cycles:

| Mode | CPU cycles |
|---------|-----------:|
| Polling | 380,000 |
| IT | 28,000 |
| DMA | 8,000 |

If cache is disabled, the numbers increase significantly:

| Mode | CPU cycles |
|---------|-----------:|
| Polling | 394,000 |
| IT | 86,000 |
| DMA | 38,000 |

