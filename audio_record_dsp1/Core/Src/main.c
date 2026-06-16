/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "pdm2pcm.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "arm_math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* ---- DSP selection (independent, combinable; chained HPF -> LPF -> reverb -> conv) ---- */
#define DSP_ENABLE_HPF      0
#define DSP_ENABLE_LPF      1
#define DSP_ENABLE_REVERB   0
#define DSP_ENABLE_CONV     0          /* FIR convolution (single echo) */

/* ---- Tunables ---- */
#define DSP_HPF_CUTOFF_HZ   5000.0f
#define DSP_LPF_CUTOFF_HZ   5000.0f
#define DSP_REVERB_DELAY_MS 80
#define DSP_REVERB_FEEDBACK 0.40f      /* |g| < 1 for stability */
#define DSP_CONV_NTAPS      8          /* FIR smoothing: 8-tap moving average */

/* ---- Derived, compile-time-constant one-pole coefficients (RC model) ---- */
#define DSP_PI       3.14159265358979f
#define DSP_DT       (1.0f / (float)AUDIO_FREQUENCY)
#define DSP_LPF_RC   (1.0f / (2.0f * DSP_PI * DSP_LPF_CUTOFF_HZ))
#define DSP_LPF_A    (DSP_DT / (DSP_LPF_RC + DSP_DT))            /* low-pass alpha */
#define DSP_HPF_RC   (1.0f / (2.0f * DSP_PI * DSP_HPF_CUTOFF_HZ))
#define DSP_HPF_A    (DSP_HPF_RC / (DSP_HPF_RC + DSP_DT))        /* high-pass alpha */
#define DSP_REVERB_DELAY  (DSP_REVERB_DELAY_MS * AUDIO_FREQUENCY / 1000)

/* ---- CMSIS-DSP comparison switch ---- */
#define DSP_USE_CMSIS       0   /* 0 = custom hand-rolled, 1 = ARM CMSIS-DSP */
/* Stereo frames per DSP_Process call (matches callback formula) */
#define CMSIS_FRAMES        ((AUDIO_IN_PDM_BUFFER_SIZE / 4U / 2U) / 2U)
/* FIR state length = numTaps + blockSize - 1 */
#define CMSIS_FIR_STATE_LEN (DSP_CONV_NTAPS + CMSIS_FRAMES - 1)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

CRC_HandleTypeDef hcrc;

/* USER CODE BEGIN PV */
/* DWT cycle counts updated each DSP_Process call; read in debugger */
volatile uint32_t bench_custom_cycles;
volatile uint32_t bench_cmsis_cycles;

/* PDM record buffer: SAI4 PDM uses BDMA, which can only access D3 SRAM
   (0x38000000). The .D3_SRAM section is mapped to RAM_D3 in the linker
   script. NOLOAD section: not zero-initialised at startup. */
static uint16_t recordPDMBuf[AUDIO_IN_PDM_BUFFER_SIZE] __attribute__((section(".D3_SRAM")));

/* PCM playback ring buffer, lives in AXI SRAM (DMA2-accessible) */
static uint16_t RecPlayback[AUDIO_BUFF_SIZE];

static uint32_t playbackPtr = 0;

BSP_AUDIO_Init_t AudioOutInit;
BSP_AUDIO_Init_t AudioInInit;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_CRC_Init(void);
/* USER CODE BEGIN PFP */
static void DSP_Process(uint16_t *pcm, uint32_t frames);
#if DSP_USE_CMSIS
static void DSP_CMSIS_Init(void);
#endif
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* --- Per-channel DSP state (index 0 = L, 1 = R), zero-initialised ---
   Custom path and CMSIS reverb share these delay-line buffers.          */
#if DSP_ENABLE_HPF
  static float hpf_x1[2], hpf_y1[2];              /* prev input, prev output */
#endif
#if DSP_ENABLE_LPF
  static float lpf_y1[2];                         /* prev output */
#endif
#if DSP_ENABLE_REVERB
  static float reverb_buf[2][DSP_REVERB_DELAY];   /* one delay line per channel */
  static uint32_t reverb_idx[2];
#endif
#if DSP_ENABLE_CONV
  /* 8-tap moving-average (smoothing) FIR kernel: every tap = 1/8.
     y[n] = sum_k h[k] * x[n-k].  hist[0] = newest input. */
  static const float dsp_conv_kernel[DSP_CONV_NTAPS] = {
    0.125f, 0.125f, 0.125f, 0.125f,
    0.125f, 0.125f, 0.125f, 0.125f,
  };
  static float conv_hist[2][DSP_CONV_NTAPS];   /* last NTAPS inputs per channel */
#endif

/* --- CMSIS-DSP instances, state, and intermediate buffers --- */
#if DSP_USE_CMSIS
  /* Biquad state: 4 words per stage (x[n-1], x[n-2], y[n-1], y[n-2]) */
  #if DSP_ENABLE_HPF
    static arm_biquad_casd_df1_inst_f32 cmsis_hpf[2];
    static float32_t cmsis_hpf_state[2][4];
    static float32_t cmsis_hpf_coeffs[5];   /* filled by DSP_CMSIS_Init */
  #endif
  #if DSP_ENABLE_LPF
    static arm_biquad_casd_df1_inst_f32 cmsis_lpf[2];
    static float32_t cmsis_lpf_state[2][4];
    static float32_t cmsis_lpf_coeffs[5];
  #endif
  #if DSP_ENABLE_CONV
    static arm_fir_instance_f32 cmsis_fir[2];
    static float32_t cmsis_fir_state[2][CMSIS_FIR_STATE_LEN];
    static float32_t cmsis_fir_coeffs[DSP_CONV_NTAPS];
    static float32_t cmsis_fir_out_L[CMSIS_FRAMES];  /* arm_fir_f32 needs separate in/out */
    static float32_t cmsis_fir_out_R[CMSIS_FRAMES];
  #endif
  static float32_t cmsis_buf_L[CMSIS_FRAMES];  /* per-channel de-interleave scratch */
  static float32_t cmsis_buf_R[CMSIS_FRAMES];
#endif

/* Clamp a float sample to the signed 16-bit PCM range. */
static inline float dsp_clip(float v)
{
  if (v >  32767.0f) return  32767.0f;
  if (v < -32768.0f) return -32768.0f;
  return v;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CRC_Init();
  MX_PDM2PCM_Init();
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

  /* Initialise the CMSIS-DSP filter instances BEFORE starting the audio
     streams. The record DMA below enables the PDM callbacks, whose first
     interrupt can fire (and call DSP_Process -> arm_*) while the codec is
     still being configured over I2C - i.e. before the filters exist. Doing
     this here guarantees numStages/pCoeffs/pState are valid first. */
#if DSP_USE_CMSIS
  DSP_CMSIS_Init();
#endif

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

  /* Enable DWT cycle counter for DSP benchmarking */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL   |= DWT_CTRL_CYCCNTENA_Msk;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

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

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_CRC_DR_RESET(&hcrc);
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

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
    BSP_AUDIO_IN_PDMToPCM(Instance,
                          (uint16_t *)&recordPDMBuf[AUDIO_IN_PDM_BUFFER_SIZE / 2U],
                          &RecPlayback[playbackPtr]);

    DSP_Process(&RecPlayback[playbackPtr], (AUDIO_IN_PDM_BUFFER_SIZE / 4U / 2U) / 2U);

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
    BSP_AUDIO_IN_PDMToPCM(Instance,
                          (uint16_t *)&recordPDMBuf[0],
                          &RecPlayback[playbackPtr]);

    DSP_Process(&RecPlayback[playbackPtr], (AUDIO_IN_PDM_BUFFER_SIZE / 4U / 2U) / 2U);

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

/**
  * @brief  Apply the enabled DSP effects in place to a block of interleaved
  *         stereo 16-bit PCM. Effects are chained HPF -> LPF -> reverb -> conv.
  *         Two paths are available, selected by DSP_USE_CMSIS:
  *           0 = custom hand-rolled sample loop
  *           1 = ARM CMSIS-DSP block functions
  *         DWT cycle counts are written to bench_custom_cycles / bench_cmsis_cycles.
  * @param  pcm    Pointer to interleaved (L,R,L,R...) PCM, treated as signed.
  * @param  frames Number of stereo frames in the block.
  */
static void DSP_Process(uint16_t *pcm, uint32_t frames)
{
  uint32_t _t0 = DWT->CYCCNT;

#if DSP_USE_CMSIS
  /* ---- CMSIS-DSP path ----
     Convert int16 → float32 and de-interleave to per-channel buffers,
     apply block filters, then re-interleave back to int16. */
  for (uint32_t i = 0; i < frames; i++)
  {
    cmsis_buf_L[i] = (float32_t)(int16_t)pcm[2 * i + 0];
    cmsis_buf_R[i] = (float32_t)(int16_t)pcm[2 * i + 1];
  }

#if DSP_ENABLE_HPF
  /* 1-stage biquad HPF, in-place.
     CMSIS DF1: y[n] = b0*x[n]+b1*x[n-1]+b2*x[n-2] + a1*y[n-1]+a2*y[n-2]
     Coeffs: b0=α, b1=-α, b2=0, a1=α, a2=0  (α = DSP_HPF_A) */
  arm_biquad_cascade_df1_f32(&cmsis_hpf[0], cmsis_buf_L, cmsis_buf_L, frames);
  arm_biquad_cascade_df1_f32(&cmsis_hpf[1], cmsis_buf_R, cmsis_buf_R, frames);
#endif

#if DSP_ENABLE_LPF
  /* 1-stage biquad LPF, in-place.
     Coeffs: b0=α, b1=0, b2=0, a1=(1-α), a2=0  (α = DSP_LPF_A) */
  arm_biquad_cascade_df1_f32(&cmsis_lpf[0], cmsis_buf_L, cmsis_buf_L, frames);
  arm_biquad_cascade_df1_f32(&cmsis_lpf[1], cmsis_buf_R, cmsis_buf_R, frames);
#endif

#if DSP_ENABLE_REVERB
  /* No CMSIS equivalent for the delay-line reverb; use the same scalar loop
     but operating on the float buffers rather than int16. */
  for (uint32_t i = 0; i < frames; i++)
  {
    float32_t d;
    d = reverb_buf[0][reverb_idx[0]];
    cmsis_buf_L[i] += DSP_REVERB_FEEDBACK * d;
    reverb_buf[0][reverb_idx[0]] = cmsis_buf_L[i];
    if (++reverb_idx[0] >= DSP_REVERB_DELAY) reverb_idx[0] = 0;

    d = reverb_buf[1][reverb_idx[1]];
    cmsis_buf_R[i] += DSP_REVERB_FEEDBACK * d;
    reverb_buf[1][reverb_idx[1]] = cmsis_buf_R[i];
    if (++reverb_idx[1] >= DSP_REVERB_DELAY) reverb_idx[1] = 0;
  }
#endif

#if DSP_ENABLE_CONV
  /* arm_fir_f32 does not support in-place; output goes to separate scratch. */
  arm_fir_f32(&cmsis_fir[0], cmsis_buf_L, cmsis_fir_out_L, frames);
  arm_fir_f32(&cmsis_fir[1], cmsis_buf_R, cmsis_fir_out_R, frames);
  for (uint32_t i = 0; i < frames; i++)
  {
    pcm[2 * i + 0] = (uint16_t)(int16_t)dsp_clip(cmsis_fir_out_L[i]);
    pcm[2 * i + 1] = (uint16_t)(int16_t)dsp_clip(cmsis_fir_out_R[i]);
  }
#else
  for (uint32_t i = 0; i < frames; i++)
  {
    pcm[2 * i + 0] = (uint16_t)(int16_t)dsp_clip(cmsis_buf_L[i]);
    pcm[2 * i + 1] = (uint16_t)(int16_t)dsp_clip(cmsis_buf_R[i]);
  }
#endif

  bench_cmsis_cycles = DWT->CYCCNT - _t0;

#else  /* ---- Custom hand-rolled path ---- */

  for (uint32_t i = 0; i < frames; i++)
  {
    for (uint32_t ch = 0; ch < 2; ch++)
    {
      float x = (float)(int16_t)pcm[2 * i + ch];

#if DSP_ENABLE_HPF
      float y = DSP_HPF_A * (hpf_y1[ch] + x - hpf_x1[ch]);
      hpf_x1[ch] = x;
      hpf_y1[ch] = y;
      x = y;
#endif
#if DSP_ENABLE_LPF
      lpf_y1[ch] += DSP_LPF_A * (x - lpf_y1[ch]);
      x = lpf_y1[ch];
#endif
#if DSP_ENABLE_REVERB
      float d = reverb_buf[ch][reverb_idx[ch]];
      float out = x + DSP_REVERB_FEEDBACK * d;
      reverb_buf[ch][reverb_idx[ch]] = out;
      if (++reverb_idx[ch] >= DSP_REVERB_DELAY) reverb_idx[ch] = 0;
      x = out;
#endif
#if DSP_ENABLE_CONV
      for (uint32_t t = DSP_CONV_NTAPS - 1; t > 0; t--)
      {
        conv_hist[ch][t] = conv_hist[ch][t - 1];
      }
      conv_hist[ch][0] = x;
      float acc = 0.0f;
      for (uint32_t t = 0; t < DSP_CONV_NTAPS; t++)
      {
        acc += dsp_conv_kernel[t] * conv_hist[ch][t];
      }
      x = acc;
#endif
      pcm[2 * i + ch] = (uint16_t)(int16_t)dsp_clip(x);
    }
  }

  bench_custom_cycles = DWT->CYCCNT - _t0;
#endif /* DSP_USE_CMSIS */
}

#if DSP_USE_CMSIS
static void DSP_CMSIS_Init(void)
{
  /* Initialise biquad and FIR instances with coefficient/state arrays.
     CMSIS DF1 sign convention: a1, a2 are the POSITIVE feedback coefficients
     (y[n] = b0*x[n]+b1*x[n-1]+b2*x[n-2] + a1*y[n-1]+a2*y[n-2]).  */
#if DSP_ENABLE_HPF
  cmsis_hpf_coeffs[0] =  DSP_HPF_A;   /* b0 */
  cmsis_hpf_coeffs[1] = -DSP_HPF_A;   /* b1 */
  cmsis_hpf_coeffs[2] =  0.0f;        /* b2 */
  cmsis_hpf_coeffs[3] =  DSP_HPF_A;   /* a1 (positive in CMSIS) */
  cmsis_hpf_coeffs[4] =  0.0f;        /* a2 */
  arm_biquad_cascade_df1_init_f32(&cmsis_hpf[0], 1, cmsis_hpf_coeffs, cmsis_hpf_state[0]);
  arm_biquad_cascade_df1_init_f32(&cmsis_hpf[1], 1, cmsis_hpf_coeffs, cmsis_hpf_state[1]);
#endif
#if DSP_ENABLE_LPF
  cmsis_lpf_coeffs[0] =  DSP_LPF_A;          /* b0 */
  cmsis_lpf_coeffs[1] =  0.0f;               /* b1 */
  cmsis_lpf_coeffs[2] =  0.0f;               /* b2 */
  cmsis_lpf_coeffs[3] =  1.0f - DSP_LPF_A;  /* a1 */
  cmsis_lpf_coeffs[4] =  0.0f;               /* a2 */
  arm_biquad_cascade_df1_init_f32(&cmsis_lpf[0], 1, cmsis_lpf_coeffs, cmsis_lpf_state[0]);
  arm_biquad_cascade_df1_init_f32(&cmsis_lpf[1], 1, cmsis_lpf_coeffs, cmsis_lpf_state[1]);
#endif
#if DSP_ENABLE_CONV
  /* Coefficients in reverse order as required by arm_fir_f32.
     All taps are equal so the order does not matter here. */
  for (uint32_t i = 0; i < DSP_CONV_NTAPS; i++)
    cmsis_fir_coeffs[i] = 0.125f;
  arm_fir_init_f32(&cmsis_fir[0], DSP_CONV_NTAPS, cmsis_fir_coeffs, cmsis_fir_state[0], CMSIS_FRAMES);
  arm_fir_init_f32(&cmsis_fir[1], DSP_CONV_NTAPS, cmsis_fir_coeffs, cmsis_fir_state[1], CMSIS_FRAMES);
#endif
}
#endif /* DSP_USE_CMSIS */
/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
