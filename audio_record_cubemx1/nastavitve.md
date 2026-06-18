Vse SAI nastavitve in PLL2 se nastavi v stm32_audio.c: ti rabiš samo narediit ist main.c, kot je bil prej
- MPU je vseeno ali enablas (jaz sem ga)
- Enable HSE crystal (pod System Core -> RCC)
- Pod Clock Configuration:
	- PLL Source Mux -> izberes HSE
- CRC:
	- Activated
- PDM2PCM: enabled

Dodaj SAI driverje v /Drivers/STM32H7xx_HAL_Driver v /Inc in /Src iz https://github.com/STMicroelectronics/stm32h7xx-hal-driver/tree/367a0a097b03a3d417dd6f6b02fd17e32b2a5742:
- sai.c, sai_ex.c, sai.h, sai_ex.h

v main.c lahko vklopiš cache, lahko pa tudi ne v USER CODE BEGIN 1:
```
  /* Enable I-Cache */
  SCB_EnableICache();

  /* Enable D-Cache */
  SCB_EnableDCache();
```
Če vklopiš cache, potem pazi, da sta recordPDMBuf in RecPlayback nastavljena na ALIGN_32BYTES, in pri DMA callbackih odkomentiraj `SCB_InvalidateDCache_by_Addr` in `SCB_CleanDCache_by_Addr` klice:

v main.c to dodaj pod USER CODE BEGIN 2:
```
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
```

Dodaj to v main.c v USER CODE BEGIN 4:
```
/**
  * @brief  Second half of the PDM record buffer is ready.
  * @param  Instance Audio in instance (1 = digital MEMS microphones)
  * @retval None
  */
void BSP_AUDIO_IN_TransferComplete_CallBack(uint32_t Instance)
{
  if (Instance == 1U)
  {
    // /* Invalidate Data Cache to get the updated content of the SRAM */
    // SCB_InvalidateDCache_by_Addr((uint32_t *)&recordPDMBuf[AUDIO_IN_PDM_BUFFER_SIZE / 2U],
    //                              sizeof(recordPDMBuf) / 2U);

    BSP_AUDIO_IN_PDMToPCM(Instance,
                          (uint16_t *)&recordPDMBuf[AUDIO_IN_PDM_BUFFER_SIZE / 2U],
                          &RecPlayback[playbackPtr]);

    // /* Clean Data Cache to update the content of the SRAM */
    // SCB_CleanDCache_by_Addr((uint32_t *)&RecPlayback[playbackPtr],
    //                         AUDIO_IN_PDM_BUFFER_SIZE / 4U);

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
    // /* Invalidate Data Cache to get the updated content of the SRAM */
    // SCB_InvalidateDCache_by_Addr((uint32_t *)&recordPDMBuf[0],
    //                              sizeof(recordPDMBuf) / 2U);

    BSP_AUDIO_IN_PDMToPCM(Instance,
                          (uint16_t *)&recordPDMBuf[0],
                          &RecPlayback[playbackPtr]);

    // /* Clean Data Cache to update the content of the SRAM */
    // SCB_CleanDCache_by_Addr((uint32_t *)&RecPlayback[playbackPtr],
    //                         AUDIO_IN_PDM_BUFFER_SIZE / 4U);

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

```

Dodaj to v main.c v USER CODE BEGIN PV:
```
/* PDM record buffer: SAI4 PDM uses BDMA, which can only access D3 SRAM
   (0x38000000). The .D3_SRAM section is mapped to RAM_D3 in the linker
   script. NOLOAD section: not zero-initialised at startup. */
ALIGN_32BYTES (static uint16_t recordPDMBuf[AUDIO_IN_PDM_BUFFER_SIZE]) __attribute__((section(".D3_SRAM")));

/* PCM playback ring buffer, lives in AXI SRAM (DMA2-accessible) */
ALIGN_32BYTES (static uint16_t RecPlayback[AUDIO_BUFF_SIZE]);

static uint32_t playbackPtr = 0;

BSP_AUDIO_Init_t AudioOutInit;
BSP_AUDIO_Init_t AudioInInit;

```

Dodaj BSP driverje v Drivers/BSP/STM32H750B-DK iz https://github.com/STMicroelectronics/stm32h750b-dk-bsp:
- audio, bus, errno
- kopiraj stm32h750b_discovery_conf_template.h v stm32h750b_discovery_conf.h

Dodaj BSP driverje v Drivers/BSP/Components/wm8994 iz https://github.com/STMicroelectronics/stm32-wm8994:

Dodaj BSP driverje v Drivers/BSP/Components/Common iz https://github.com/STMicroelectronics/stm32-bsp-common:
- samo audio.h

V main.h dodaj:
```
#include "stm32h750b_discovery_audio.h"
// ...
#define AUDIO_FREQUENCY           16000U
/* Number of uint16 elements in the PDM record buffer (two DMA halves) */
#define AUDIO_IN_PDM_BUFFER_SIZE  (uint32_t)(128U*AUDIO_FREQUENCY/16000U*2U)
/* Number of uint16 elements in the PCM playback ring buffer */
#define AUDIO_BUFF_SIZE           4096U

```

FLASH.ld, to daj v SECTIONS (npr. za .bss, ampak drugace je vseeno kam):
```
  .D3_SRAM  (NOLOAD) : { *(.D3_SRAM*)  } >RAM_D3
```

V stm32h7xx_hal_conf.h odkomentiraj:
```
#define HAL_SAI_MODULE_ENABLED
```

v Core/Src/stm32h7xx_it.c dodaj v USER CODE BEGIN 1:
```
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
```

Nastavi linker:
- Project -> Properties -> MCU/MPU GCC Compiler -> Include Paths
- dodaj `../Drivers/BSP/STM32H750B-DK`

Data flow:
MEMS mics ──PDM──▶ SAI4_A (PDM mode) ──BDMA Ch1──▶ recordPDMBuf (D3 SRAM @0x38000000)
                                                          │
                                       BDMA half/cplt IRQ │  CPU: BSP_AUDIO_IN_PDMToPCM()
                                                          ▼
                                              RecPlayback ring buffer (AXI SRAM, D1)
                                                          │
                                       DMA2_Stream1 ◀─────┘ (circular, mem→periph)
                                                          ▼
                                           SAI2_A ──I2S──▶ WM8994 ──▶ green line-out jack
                          (WM8994 registers are set over I2C4)
