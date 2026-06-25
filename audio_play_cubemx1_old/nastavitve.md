Vse SAI nastavitve in PLL2 se nastavi v stm32_audio.c: ti rabiš samo narediit ist main.c, kot je bil prej
- MPU je vseeno ali enablas (jaz sem ga)
- Enable HSE crystal (pod System Core -> RCC) - HSE nam da bolj stabilno uro kot HSI, kar je koristno za natančno vzorčenje in predvajanje zvoka
- Pod Clock Configuration:
	- PLL Source Mux -> izberes HSE
- CRC: (CRC in PDM2PCM ne bomo nič rabili v tem projektu, ampak jih rabimo, ker jih BSP audio driver include-a)
	- Activated
- PDM2PCM: enabled

Dodaj SAI driverje v /Drivers/STM32H7xx_HAL_Driver v /Inc in /Src iz https://github.com/STMicroelectronics/stm32h7xx-hal-driver/tree/367a0a097b03a3d417dd6f6b02fd17e32b2a5742:
- sai.c, sai_ex.c, sai.h, sai_ex.h

Dodaj BSP driverje v Drivers/BSP/STM32H750B-DK iz https://github.com/STMicroelectronics/stm32h750b-dk-bsp:
- audio, bus, errno
- kopiraj stm32h750b_discovery_conf_template.h v stm32h750b_discovery_conf.h

Dodaj BSP driverje v Drivers/BSP/Components/wm8994 iz https://github.com/STMicroelectronics/stm32-wm8994:

Dodaj BSP driverje v Drivers/BSP/Components/Common iz https://github.com/STMicroelectronics/stm32-bsp-common:
- samo audio.h

V main.h dodaj v USER CODE BEGIN Private defines:
```
#define AUDIO_BUFFER_SIZE 2048U
#define SAMPLE_RATE 96000.0f
#define TONE_FREQ   100.0f
#define AMPLITUDE   30000     // int16 max is 32767
#define TWO_PI      6.28318530718f
#define PHASE_INC   (TWO_PI * TONE_FREQ / SAMPLE_RATE)
```

V stm32h7xx_hal_conf.h odkomentiraj:
```
#define HAL_SAI_MODULE_ENABLED
```

Dodaj v main.c v USER CODE BEGIN Includes:
```
#include "stm32h750b_discovery_audio.h"
```

Dodaj to v main.c v USER CODE BEGIN PFP:
```
void AUDIO_Process(void);
void GenerateTone(int16_t *dst, uint32_t samples);
```

Dodaj to v main.c v USER CODE BEGIN PTD:
```
typedef enum {
  BUFFER_OFFSET_NONE = 0,
  BUFFER_OFFSET_HALF,
  BUFFER_OFFSET_FULL,
}BUFFER_StateTypeDef;

typedef struct {
  uint8_t buff[AUDIO_BUFFER_SIZE];
  BUFFER_StateTypeDef state;
}AUDIO_BufferTypeDef;
```

Dodaj to v main.c v USER CODE BEGIN PV:
```
ALIGN_32BYTES (static AUDIO_BufferTypeDef  buffer_ctl);
BSP_AUDIO_Init_t AudioOutInit;

static float phase = 0.0f;
```

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
  AudioOutInit.SampleRate = SAMPLE_RATE;
  AudioOutInit.BitsPerSample = AUDIO_RESOLUTION_16B;
  AudioOutInit.Volume = 40;

  /* Instance 0: WM8994 codec via SAI2 + DMA2, line out / headphone */
  if (BSP_AUDIO_OUT_Init(0, &AudioOutInit) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }

  GenerateTone((int16_t *)&buffer_ctl.buff[0], AUDIO_BUFFER_SIZE / 4);
  SCB_CleanDCache_by_Addr((uint32_t*)&buffer_ctl.buff[0], AUDIO_BUFFER_SIZE);
  BSP_AUDIO_OUT_Play(0, (uint8_t *)&buffer_ctl.buff[0], AUDIO_BUFFER_SIZE);

```

Dodaj to v main.c v while loop:
```
    AUDIO_Process();
```


Dodaj to v main.c v USER CODE BEGIN 4:
```
void AUDIO_Process(void)
{
  if (buffer_ctl.state == BUFFER_OFFSET_HALF)
  {
    GenerateTone((int16_t *)&buffer_ctl.buff[0], (AUDIO_BUFFER_SIZE / 2) / 4);
    buffer_ctl.state = BUFFER_OFFSET_NONE;
    SCB_CleanDCache_by_Addr((uint32_t*)&buffer_ctl.buff[0], AUDIO_BUFFER_SIZE / 2);
  }

  if (buffer_ctl.state == BUFFER_OFFSET_FULL)
  {
    GenerateTone((int16_t *)&buffer_ctl.buff[AUDIO_BUFFER_SIZE / 2], (AUDIO_BUFFER_SIZE / 2) / 4);
    buffer_ctl.state = BUFFER_OFFSET_NONE;
    SCB_CleanDCache_by_Addr((uint32_t*)&buffer_ctl.buff[AUDIO_BUFFER_SIZE / 2], AUDIO_BUFFER_SIZE / 2);
  }
}

/**
  * @brief  Manages the full Transfer complete event.
  * @param  None
  * @retval None
  */
void BSP_AUDIO_OUT_TransferComplete_CallBack(uint32_t Interface)
{
  /* allows AUDIO_Process() to refill 2nd part of the buffer  */
  buffer_ctl.state = BUFFER_OFFSET_FULL;
}

/**
  * @brief  Manages the DMA Half Transfer complete event.
  * @param  None
  * @retval None
  */
void BSP_AUDIO_OUT_HalfTransfer_CallBack(uint32_t Interface)
{
  /* allows AUDIO_Process() to refill 1st part of the buffer  */
  buffer_ctl.state = BUFFER_OFFSET_HALF;
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


void GenerateTone(int16_t *dst, uint32_t samples)
{
  for (uint32_t i = 0; i < samples; i++)
  {
    int16_t s = (int16_t)(AMPLITUDE * sinf(phase));
    phase += PHASE_INC;
    if (phase >= TWO_PI)
      phase -= TWO_PI;

    // stereo: L and R
    dst[2*i]     = s;
    dst[2*i + 1] = s;
  }
}
```

Nastavi linker:
- Project -> Properties -> C/C++ Build -> Settings -> MCU/MPU GCC Compiler -> Include Paths
- dodaj `../Drivers/BSP/STM32H750B-DK`

Data flow:
SRAM buffer (int16 stereo)
   ▼  DMA2_Stream1  (circular, mem→periph, request SAI2_A)
SAI2 Block A TX  ──I2S── PI4=MCLK, PI5=SCK, PI6=SD, PI7=FS (AF10) ──▶
WM8994 codec  (control registers are set over I2C4)
   ▼
green LINE-OUT / headphone jack
