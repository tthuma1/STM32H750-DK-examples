# STM32H750B-DK minimal audio play example

The following tutorial presents how to play audio through the built-in Line Out port (green TRS 3.5mm, marked as CN9) on STM32H750B-DK.
The project is created with STM32CubeMX v6.17.0 with STM32CubeH7 MCU Firmware Package v1.13.0; STM32CubeIDE v2.1.1 was used for building and flashing.

Enabling D- and I-cache is not necessary in this project, but is kept as a good practice for audio processing on embedded systems.

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

CubeMX does not generate the audio drivers, so you must copy them
in by hand.

STM32CubeMX downloads the STM32CubeH7 MCU Firmware Package (MCU FP) to `~/STM32Cube/Repository/STM32Cube_FW_H7_Vx.x.x`.
If you do not have it, download it from
https://www.st.com/en/embedded-software/stm32cubeh7.html#get-software.

In the table below, each file lives at the **same relative path** in both places: copy it from
that path under the MCU FP root into the same path under your project root.

| Files to copy | Relative path (same in MCU FP and your project) |
|---------------|----------------------------------------|
| `stm32h7xx_hal_sai.h`, `stm32h7xx_hal_sai_ex.h` | `Drivers/STM32H7xx_HAL_Driver/Inc` |
| `stm32h7xx_hal_sai.c`, `stm32h7xx_hal_sai_ex.c` | `Drivers/STM32H7xx_HAL_Driver/Src` |
| `stm32h750b_discovery_audio.{c,h}`, `stm32h750b_discovery_bus.{c,h}`, `stm32h750b_discovery_errno.h` | `Drivers/BSP/STM32H750B-DK` |
| entire `wm8994` folder | `Drivers/BSP/Components/wm8994` |
| `audio.h` | `Drivers/BSP/Components/Common` |

Then create the BSP config header by copying the template and renaming it (in
`Drivers/BSP/STM32H750B-DK`):

- `stm32h750b_discovery_conf_template.h` → `stm32h750b_discovery_conf.h`

# 4. Add code

Add the following include and constants to `Core/Src/main.h` (under `USER CODE BEGIN Includes` and `USER CODE BEGIN Private defines`):

```c
/* USER CODE BEGIN Includes */
#include "stm32h750b_discovery_audio.h"
/* USER CODE END Includes */

// ...

/* USER CODE BEGIN Private defines */
#define AUDIO_BUFFER_SIZE 2048U
#define SAMPLE_RATE 96000.0f
#define TONE_FREQ   200.0f
#define AMPLITUDE   3000
#define TWO_PI      6.28318530718f
#define PHASE_INC   (TWO_PI * TONE_FREQ / SAMPLE_RATE)
/* USER CODE END Private defines */
```

Uncomment the following line in `Core/Inc/stm32h7xx_hal_conf.h`:
```c
#define HAL_SAI_MODULE_ENABLED
```

Add the following code to `Core/Src/main.c`:
```c
/* USER CODE BEGIN PTD */
typedef enum {
  BUFFER_OFFSET_NONE = 0,
  BUFFER_OFFSET_HALF,
  BUFFER_OFFSET_FULL,
}BUFFER_StateTypeDef;

typedef struct {
  uint8_t buff[AUDIO_BUFFER_SIZE];
  BUFFER_StateTypeDef state;
}AUDIO_BufferTypeDef;
/* USER CODE END PTD */

// ...

/* USER CODE BEGIN PV */
ALIGN_32BYTES (static AUDIO_BufferTypeDef  buffer_ctl);
BSP_AUDIO_Init_t AudioOutInit;

static float phase = 0.0f;
/* USER CODE END PV */

// ...

/* USER CODE BEGIN PFP */
void AUDIO_Process(void);
void GenerateTone(int16_t *dst, uint32_t samples);
/* USER CODE END PFP */

// ...

  /* USER CODE BEGIN 1 */
  SCB_EnableICache();
  SCB_EnableDCache();
  /* USER CODE END 1 */

// ...

  /* USER CODE BEGIN 2 */
  AudioOutInit.Device = AUDIO_OUT_DEVICE_HEADPHONE;
  AudioOutInit.ChannelsNbr = 2;
  AudioOutInit.SampleRate = SAMPLE_RATE;
  AudioOutInit.BitsPerSample = AUDIO_RESOLUTION_16B;
  AudioOutInit.Volume = 50;

  /* Instance 0: WM8994 codec via SAI2 + DMA2, line out / headphone */
  if (BSP_AUDIO_OUT_Init(0, &AudioOutInit) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }

  GenerateTone((int16_t *)&buffer_ctl.buff[0], AUDIO_BUFFER_SIZE / 4);
  SCB_CleanDCache_by_Addr((uint32_t*)&buffer_ctl.buff[0], AUDIO_BUFFER_SIZE);
  BSP_AUDIO_OUT_Play(0, (uint8_t *)&buffer_ctl.buff[0], AUDIO_BUFFER_SIZE);
  /* USER CODE END 2 */

// ...

  /* USER CODE BEGIN WHILE */
  while (1)
  {
    AUDIO_Process();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */

// ...

/* USER CODE BEGIN 4 */
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
/* USER CODE END 4 */
```

Using HSE as the source clock for SAI2 is recommended since it is more stable than HSI. Use the following `SystemClock_Config` function in `Core/Src/main.c`:

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSE);

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
/* USER CODE END 1 */
```

# 5. Update include path in STM32CubeIDE

Open the project in STM32CubeIDE and navigate to `Project` → `Properties` → `C/C++ Build` → `Settings` → `MCU/MPU GCC Compiler` → `Include Paths`:
  - Add `../Drivers/BSP/STM32H750B-DK` under `Include paths (-I)`


# Notes

Adjust your speaker/headphones volume before running this example. Volume can also be changed through `AudioOutInit.Volume` in `main.c` or by chaning the `AMPLITUDE` in `main.h`.