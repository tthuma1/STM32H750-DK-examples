# STM32H750B-DK Minimal Audio Record Example

The following tutorial presents how to record audio through the built-in PDM MEMS microphone (IMP34DT05TR, marked as U34) and play it back through the Line Out port (green TRS 3.5mm, PJ-3028, marked as CN9) on the STM32H750B-DK.
The project is created with STM32CubeMX v6.17.0 with STM32CubeH7 MCU Firmware Package v1.13.0; STM32CubeIDE v2.1.1 was used for building and flashing.

Enabling D- and I-cache is not necessary in this project, but is kept as a good practice for audio processing on embedded systems. The same is true for using the HSE clock as the source clock for SAI instead of HSI.

# Project overview

The on-board MEMS microphone outputs a 1-bit PDM stream that a serial-audio peripheral clocks
into RAM via DMA. The CPU converts the PDM to 16-bit stereo PCM in the DMA interrupts and writes
it into a playback buffer. A second DMA streams that PCM out through another serial-audio
peripheral to the external codec chip, which does the digital-to-analog conversion and drives the
line-out jack.

### Data flow:

```
MEMS mics ──PDM──▶ SAI4_A (PDM mode) ──BDMA Ch1──▶ recordPDMBuf (D3 SRAM @0x38000000)
                                                          │
                                       BDMA half/cplt IRQ │  CPU: BSP_AUDIO_IN_PDMToPCM()
                                                          ▼
                                              RecPlayback ring buffer (AXI SRAM, D1)
                                                          │
                                       DMA2_Stream1 ◀─────┘ (circular, mem→periph)
                                             ▼             
                                           SAI2_A ──I2S──▶ WM8994 ──▶ line-out jack
                                               (WM8994 registers are set over I2C4)
```

### Peripherals used

**MEMS microphone (IMP34DT05TR)**: on-board digital MEMS microphone. It outputs a
1-bit-per-sample **PDM** (Pulse-Density Modulation) bitstream on one data line. The driver treats
it as a stereo PDM pair (LR) and samples both the rising and falling edges of the mic clock, so the
left and right channels are both drawn from this one microphone.

**PDM (Pulse-Density Modulation)**: the 1-bit oversampled format the mic emits. It doesn't contain PCM samples 
directly, so the high-rate bitstream has to be low-pass filtered and downsampled into 16-bit PCM. This PDM→PCM
conversion is done by the CPU (`BSP_AUDIO_IN_PDMToPCM()`) inside the record DMA callbacks.

**SAI (Serial Audio Interface)**: the peripheral that serializes PCM samples onto an audio bus. It is
split into two independent blocks, *Block A* and *Block B*, each with its own clock, FIFO and
pins. This project uses two SAI instances:

- **SAI4 Block A** as the **receiver** in PDM mode: it supplies the mic clock and samples the
  incoming PDM bitstream from the microphone.
- **SAI2 Block A** as the **transmitter** to the codec. It generates four wires:
  - **SD** (serial data, PI6)
  - **SCK** (bit clock, PI5)
  - **FS** (frame sync / word select, PI7): indicates the start of the serial data (start of the frame)
  - **MCLK** (master clock, PI4): a high-frequency reference clock (multiple of the audio sample rate `Fs`) that synchronizes the audio data transfer and the codec

  All four SAI2 pins are configured for their respective functions with alternate-function **AF10**.

**I2S**: the framing protocol used on the SAI2-to-WM8994 link (Philips I²S
standard: MSB-first, data delayed one bit after FS, FS low = left channel; 32-bit slot (L+R), 16-bit data).

**WM8994**: Cirrus stereo codec that receives the I2S stream,
performs the D/A conversion and amplifies the result to the Line-Out / headphone jack. Its control registers (input routing, DAC path,
volume, power) are configured by the BSP driver over **I2C4**.

**DMA (Direct Memory Access)**: moves audio buffers between SRAM and the SAI data registers with
no CPU involvement. Two separate DMA controllers are used, one per direction:

- **BDMA Channel 1** (record): moves the PDM bitstream from `SAI4_A` into `recordPDMBuf`,
  **circular**, 16-bit. BDMA lives in the D3 domain and is the only DMA that can
  reach the SAI4 peripheral; it can also only access **D3 SRAM** (`0x38000000`), which is why
  `recordPDMBuf` is forced there.
- **DMA2 Stream 1** (playback): mapped to the `SAI2_A` request, memory→peripheral, **circular**,
  16-bit. It streams the PCM ring buffer out to the codec.

The DMA is organized into **streams**; each stream is wired to one
peripheral request.
**Circular** means that when the controller reaches the end of the buffer it wraps to the start
and keeps going forever. Both DMAs raise half-transfer and transfer-complete interrupts that
drive the buffer handling.

### Gapless audio transmission with double-buffering

Both DMAs in the loopback path use double-buffering, so neither ever has to stop and audio
flows without gaps:

- **Record (BDMA → `recordPDMBuf`):** the BDMA fills the PDM buffer continuously and raises an
  interrupt at the buffer midpoint (**half-transfer**) and at the end (**transfer-complete**).
  On half-transfer the BDMA is now filling the second half, so the CPU processes the first half;
  on transfer-complete it is filling the first half, so the CPU processes the second. Each
  callback runs the PDM→PCM conversion on the just-filled half and writes the result into the
  `RecPlayback` ring.
- **Playback (DMA2 → SAI2):** DMA2 streams `RecPlayback` circularly out to the codec. The
  PDM→PCM conversion in the record callbacks keeps writing fresh PCM ahead of the playback read
  pointer, so the half DMA2 is about to play is always refilled before it gets there.

Because the D-Cache is enabled, the callbacks do cache maintenance at both ends of the
conversion. Before reading the freshly captured PDM half they `SCB_InvalidateDCache_by_Addr()`
it, so the CPU sees the data the BDMA just wrote to SRAM rather than stale cache; after writing
the converted PCM samples they `SCB_CleanDCache_by_Addr()` that region, so DMA2 reads the fresh
data from SRAM and not stale cache. The buffers are `ALIGN_32BYTES` to match the 32-byte
cache-line granularity of those maintenance operations.

### Clocking

- **CPU / bus clocks** come from **HSI** (the 64 MHz internal RC oscillator).
- **The audio sample clock** must be exact, so it is derived from **HSE** (the board's external
  25 MHz crystal) through PLL2. Both serial-audio peripherals are fed from PLL2: the playback
  side (SAI2) is set up in the BSP driver's `MX_SAI2_ClockConfig()`, and the mic-capture side
  (SAI4) in `MX_SAI4_ClockConfig()`. For the 16 kHz used here PLL2 produces ≈49.14 MHz
  (HSE 25 MHz ÷ PLL2M 25 = 1 MHz VCO input, ×PLL2N 344 = 344 MHz, ÷PLL2P 7), which each SAI then
  divides down to the exact MCLK/SCK for the requested Fs.

### Schematics

It may be useful to take a look at the schematic, taken from https://www.st.com/resource/en/schematic_pack/mb1381-h750xb-b04.pdf (pages 3 and 13).

![Audio schematics overview](../assets/schem_min.png)
![Full audio schematics](../assets/schem_full.png)

# 1. Create the project

- Create a new project in CubeMX.
- Select the STM32H750XBH MCU.
- When asked about configuring the MPU, it doesn't matter what you pick (I picked Yes in this project).

# 2. Configure basics in CubeMX

The BSP driver that we will import later requires the PDM2PCM library, even though we won't be using any of its functionalities in this project.
The PDM2PCM library is only needed for recording audio.

- Activate CRC under `Computing` -> `CRC`.
- Activate PDM2PCM under `Middleware and Software Packs` -> `PDM2PCM`.
- Set the toolchain in `Project Manager` tab to STM32CubeIDE and click `Generate Code`.

# 3. Add driver files

CubeMX does not generate the audio drivers, so you must copy them
in by hand.

STM32CubeMX downloads the STM32CubeH7 MCU Firmware Package (MCU FP) to `~/STM32Cube/Repository/STM32Cube_FW_H7_Vx.x.x`.
If you do not have it, download it from
https://www.st.com/en/embedded-software/stm32cubeh7.html#get-software.

In the table below, each file lives at the **same relative path** in both places: copy it from
that path under the MCU FP root into the same path under your project root.

| Files to copy | Relative path (same in MCU FP and your project) |
|---------------|----------------------------------------|
| `stm32h7xx_hal_sai.h`<br> `stm32h7xx_hal_sai_ex.h` | `Drivers/STM32H7xx_HAL_Driver/Inc` |
| `stm32h7xx_hal_sai.c`<br> `stm32h7xx_hal_sai_ex.c` | `Drivers/STM32H7xx_HAL_Driver/Src` |
| `stm32h750b_discovery_audio.{c,h}`<br> `stm32h750b_discovery_bus.{c,h}`<br> `stm32h750b_discovery_errno.h` | `Drivers/BSP/STM32H750B-DK` |
| entire `wm8994` folder | `Drivers/BSP/Components/wm8994` |
| `audio.h` | `Drivers/BSP/Components/Common` |

Then create the BSP config header by copying the template and renaming it (in
`Drivers/BSP/STM32H750B-DK`):

- `stm32h750b_discovery_conf_template.h` → `stm32h750b_discovery_conf.h`

# 4. Add code

Add the following code to `STM32H750XBHX_FLASH.ld` inside the `SECTIONS` block (e.g. after the `.bss` block):
```
  .D3_SRAM  (NOLOAD) : { *(.D3_SRAM*)  } >RAM_D3
```

Uncomment the following line in `Core/Inc/stm32h7xx_hal_conf.h`:
```c
#define HAL_SAI_MODULE_ENABLED
```

Add the following code to `Core/Src/main.h`:
```c
/* USER CODE BEGIN Includes */
#include "stm32h750b_discovery_audio.h"
/* USER CODE END Includes */

// ...

/* USER CODE BEGIN Private defines */
#define AUDIO_FREQUENCY           16000U
/* Number of uint16 elements in the PDM record buffer (two DMA halves) */
#define AUDIO_IN_PDM_BUFFER_SIZE  (uint32_t)(128U*AUDIO_FREQUENCY/16000U*2U)
/* Number of uint16 elements in the PCM playback ring buffer */
#define AUDIO_BUFF_SIZE           4096U
/* USER CODE END Private defines */
```

Add the following code to `Core/Src/main.c`:
```c
/* USER CODE BEGIN PV */
/* PDM record buffer: SAI4 PDM uses BDMA, which can only access D3 SRAM
   (0x38000000). The .D3_SRAM section is mapped to RAM_D3 in the linker
   script. NOLOAD section: not zero-initialised at startup. */
ALIGN_32BYTES(static uint16_t recordPDMBuf[AUDIO_IN_PDM_BUFFER_SIZE]) __attribute__((section(".D3_SRAM")));

/* PCM playback ring buffer, lives in AXI SRAM (DMA2-accessible) */
ALIGN_32BYTES(static uint16_t RecPlayback[AUDIO_BUFF_SIZE]);

static uint32_t playbackPtr = 0;

BSP_AUDIO_Init_t AudioOutInit;
BSP_AUDIO_Init_t AudioInInit;
/* USER CODE END PV */

// ...

  /* USER CODE BEGIN 1 */
  SCB_EnableICache();
  SCB_EnableDCache();
  /* USER CODE END 1 */

// ...

  /* USER CODE BEGIN 2 */
  AudioOutInit.Device = AUDIO_OUT_DEVICE_HEADPHONE;
  AudioOutInit.ChannelsNbr = 2;
  AudioOutInit.SampleRate = AUDIO_FREQUENCY;
  AudioOutInit.BitsPerSample = AUDIO_RESOLUTION_16B;
  AudioOutInit.Volume = 80;

  AudioInInit.Device = AUDIO_IN_DEVICE_DIGITAL_MIC;
  AudioInInit.ChannelsNbr = 2;
  AudioInInit.SampleRate = AUDIO_FREQUENCY;
  AudioInInit.BitsPerSample = AUDIO_RESOLUTION_16B;
  AudioInInit.Volume = 80;

  /* Instance 1: MEMS microphones via SAI4 PDM interface + BDMA */
  if (BSP_AUDIO_IN_Init(1, &AudioInInit) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }

  /* Instance 0: WM8994 codec via SAI2 + DMA2, line out / headphone */
  if (BSP_AUDIO_OUT_Init(0, &AudioOutInit) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }

  /* Start recording: circular DMA over the whole PDM buffer */
  if (BSP_AUDIO_IN_RecordPDM(1, (uint8_t *)recordPDMBuf, 2U * AUDIO_IN_PDM_BUFFER_SIZE) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }

  /* Start playback of the PCM ring buffer that record callbacks fill */
  if (BSP_AUDIO_OUT_Play(0, (uint8_t *)RecPlayback, 2U * AUDIO_BUFF_SIZE) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }
  /* USER CODE END 2 */

// ...

/* USER CODE BEGIN 4 */
/**
  * @brief  Second half of the PDM record buffer is ready.
  * @param  Instance Audio in instance (1 = digital MEMS microphones)
  * @retval None
  */
void BSP_AUDIO_IN_TransferComplete_CallBack(uint32_t Instance)
{
  if (Instance == 1U)
  {
    /* Invalidate Data Cache to get the updated content of the SRAM */
    SCB_InvalidateDCache_by_Addr((uint32_t *)&recordPDMBuf[AUDIO_IN_PDM_BUFFER_SIZE / 2U],
                                 sizeof(recordPDMBuf) / 2U);

    BSP_AUDIO_IN_PDMToPCM(Instance,
                          (uint16_t *)&recordPDMBuf[AUDIO_IN_PDM_BUFFER_SIZE / 2U],
                          &RecPlayback[playbackPtr]);

    /* Clean Data Cache to update the content of the SRAM */
    SCB_CleanDCache_by_Addr((uint32_t *)&RecPlayback[playbackPtr],
                            AUDIO_IN_PDM_BUFFER_SIZE / 4U);

    playbackPtr += AUDIO_IN_PDM_BUFFER_SIZE / 4U / 2U;
    if (playbackPtr >= AUDIO_BUFF_SIZE)
    {
      playbackPtr = 0;
    }
  }
}

/**
  * @brief  First half of the PDM record buffer is ready.
  * @param  Instance Audio in instance (1 = digital MEMS microphones)
  * @retval None
  */
void BSP_AUDIO_IN_HalfTransfer_CallBack(uint32_t Instance)
{
  if (Instance == 1U)
  {
    /* Invalidate Data Cache to get the updated content of the SRAM */
    SCB_InvalidateDCache_by_Addr((uint32_t *)&recordPDMBuf[0],
                                 sizeof(recordPDMBuf) / 2U);

    BSP_AUDIO_IN_PDMToPCM(Instance,
                          (uint16_t *)&recordPDMBuf[0],
                          &RecPlayback[playbackPtr]);

    /* Clean Data Cache to update the content of the SRAM */
    SCB_CleanDCache_by_Addr((uint32_t *)&RecPlayback[playbackPtr],
                            AUDIO_IN_PDM_BUFFER_SIZE / 4U);

    playbackPtr += AUDIO_IN_PDM_BUFFER_SIZE / 4U / 2U;
    if (playbackPtr >= AUDIO_BUFF_SIZE)
    {
      playbackPtr = 0;
    }
  }
}

/**
  * @brief  Manages the DMA FIFO error event.
  * @param  Instance Audio out instance
  * @retval None
  */
void BSP_AUDIO_OUT_Error_CallBack(uint32_t Interface)
{
  Error_Handler();
}

/* USER CODE END 4 */

```

Using HSE as the source clock for SAI2 is recommended since it is more stable than HSI. Use the following `SystemClock_Config` function in `Core/Src/main.c`. Noted are the lines that need to be changed from the default CubeMX generated configuration:

```c
/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSE; // NOTE: changed from the default CubeMX project
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON; // NOTE: changed from the default CubeMX project
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSE);  // NOTE: changed from the default CubeMX project

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}
```

Add the following code to `Core/Src/stm32h7xx_it.c`:
```c
/* USER CODE BEGIN 1 */
/**
  * @brief  This function handles DMA2 Stream 1 interrupt request
  *         (SAI2 Block A: audio out to the WM8994 codec).
  */
void AUDIO_OUT_SAIx_DMAx_IRQHandler(void)
{
  BSP_AUDIO_OUT_IRQHandler(0);
}

/**
  * @brief  This function handles BDMA Channel 1 interrupt request
  *         (SAI4 PDM: digital MEMS microphone record).
  */
void AUDIO_IN_SAI_PDMx_DMAx_IRQHandler(void)
{
  BSP_AUDIO_IN_IRQHandler(1, AUDIO_IN_DEVICE_DIGITAL_MIC);
}
/* USER CODE END 1 */

```

# 5. Update include paths in STM32CubeIDE

Open the project in STM32CubeIDE and navigate to `Project` → `Properties` → `C/C++ Build` → `Settings` → `MCU/MPU GCC Compiler` → `Include Paths`:
  - Add `../Drivers/BSP/STM32H750B-DK` under `Include paths (-I)`

# 6. Build and flash the project

Build and flash the project from STM32CubeIDE.

# Notes

Adjust your speaker/headphones volume before running this example. Volume can also be changed through `AudioOutInit.Volume` and `AudioInInit.Volume` in `main.c`.