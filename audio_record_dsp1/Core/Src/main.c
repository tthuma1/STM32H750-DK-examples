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
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* ---- DSP selection (independent, combinable; chained GATE -> HPF -> LPF -> reverb -> conv -> RIR) ---- */
#define DSP_ENABLE_GATE     0          /* noise gate / downward expander (runs first in the chain) */
#define DSP_ENABLE_HPF      0
#define DSP_ENABLE_LPF      0
#define DSP_ENABLE_REVERB   0
#define DSP_ENABLE_CONV     0          /* FIR convolution (single echo) */
#define DSP_ENABLE_RIR      1          /* Room Impulse Response convolution (room acoustics) */

/* ---- Tunables ---- */
#define DSP_HPF_CUTOFF_HZ   5000.0f
#define DSP_LPF_CUTOFF_HZ   2000.0f
#define DSP_REVERB_DELAY_MS 80
#define DSP_REVERB_FEEDBACK 0.40f      /* |g| < 1 for stability */
#define DSP_CONV_NTAPS      8          /* FIR smoothing: 8-tap moving average */
/* Noise gate / downward expander. The MEMS mic has a constant broadband
   self-noise floor; dry it is quiet, but the reverb integrates and sustains it
   into an audible wash. Gating the signal BEFORE the reverb means the noise
   floor never feeds the tail - during silence the input is muted, so no new
   reverb energy is produced (the existing tail still decays naturally).
   Channel-linked, soft-knee, with a fast attack (don't chop transients) and a
   slow release (smooth close). THRESH/KNEE are in int16 LSB and must be tuned
   to YOUR mic: read the |sample| level during silence (e.g. watch gate_env in
   the debugger with no signal) and set THRESH a little above it. */
#define DSP_GATE_THRESH     600.0f     /* env at/below this => gate fully closed (~noise floor) */
#define DSP_GATE_KNEE       500.0f     /* env this far above THRESH => fully open (soft ramp) */
#define DSP_GATE_ATK        0.40f      /* envelope attack coeff (fast: gate opens quickly) */
#define DSP_GATE_REL        0.0040f    /* envelope release coeff (faster: gate closes before noise smears) */
/* NOTE: the M7 runs at 64 MHz here (HSI, no PLL - see SystemClock_Config), so a
   DSP_Process callback has only ~64000 cycles (16 frames @ 16 kHz = 1 ms) for
   EVERYTHING (PDM->PCM + gate + RIR + cache). The RIR is a plain time-domain
   FIR whose cost scales linearly with the tap count (= LEN_MS * fs / 1000).
   Make sure that DSP_RIR_NTAPS will be small enough.
   */
#define DSP_RIR_RT60_MS     20         /* target reverb time of the synthetic room (RT60) */
#define DSP_RIR_LEN_MS      8          /* length of the impulse response actually convolved */
#define DSP_RIR_WET         0.25f      /* wet amount: 0 = dry only, larger = more reverb tail */

/* ---- Derived, compile-time-constant one-pole coefficients (RC model) ---- */
#define DSP_PI       3.14159265358979f
#define DSP_DT       (1.0f / (float)AUDIO_FREQUENCY)
#define DSP_LPF_RC   (1.0f / (2.0f * DSP_PI * DSP_LPF_CUTOFF_HZ))
#define DSP_LPF_A    (DSP_DT / (DSP_LPF_RC + DSP_DT))            /* low-pass alpha */
#define DSP_HPF_RC   (1.0f / (2.0f * DSP_PI * DSP_HPF_CUTOFF_HZ))
#define DSP_HPF_A    (DSP_HPF_RC / (DSP_HPF_RC + DSP_DT))        /* high-pass alpha */
#define DSP_REVERB_DELAY  (DSP_REVERB_DELAY_MS * AUDIO_FREQUENCY / 1000)
/* RIR length in taps (compile-time constant: ms * fs / 1000) */
#define DSP_RIR_NTAPS     (DSP_RIR_LEN_MS * AUDIO_FREQUENCY / 1000)
/* Output makeup gain for the RIR stage. The dry signal is already preserved at
   full scale (h[0] = 1), so adding the wet tail on top overflows ±32767 and
   clips. Scaling the wet+dry sum by 1/(1+WET) restores headroom so the reverb
   blends in instead of slamming the clip rails. */
#define DSP_RIR_MAKEUP    (1.0f / (1.0f + DSP_RIR_WET))

/* ---- Cache enables ---- */
#define ENABLE_ICACHE       1
#define ENABLE_DCACHE       1

/* ---- CMSIS-DSP comparison switch ---- */
#define DSP_USE_CMSIS       1   /* 0 = custom hand-rolled, 1 = ARM CMSIS-DSP */
/* Stereo frames per DSP_Process call (matches callback formula) */
#define CMSIS_FRAMES        ((AUDIO_IN_PDM_BUFFER_SIZE / 4U / 2U) / 2U)
/* FIR state length = numTaps + blockSize - 1 */
#define CMSIS_FIR_STATE_LEN (DSP_CONV_NTAPS + CMSIS_FRAMES - 1)
#define CMSIS_RIR_STATE_LEN (DSP_RIR_NTAPS + CMSIS_FRAMES - 1)
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
#if DSP_ENABLE_RIR
static void DSP_RIR_Build(void);
#endif
#if DSP_USE_CMSIS
static void DSP_CMSIS_Init(void);
#endif
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* --- Per-channel DSP state (index 0 = L, 1 = R), zero-initialised ---
   Custom path and CMSIS reverb share these delay-line buffers.          */
#if DSP_ENABLE_GATE
  /* Channel-linked noise-gate state, persisted across calls: a smoothed
     peak envelope of the (stereo-max) input, used to derive the gate gain. */
  static float gate_env = 0.0f;
#endif
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
#if DSP_ENABLE_RIR
  /* Synthetic room impulse response, h[0] = direct path. Built once at
     startup by DSP_RIR_Build(). The output is x convolved with h, i.e.
     y[n] = sum_k h[k] * x[n-k] - exactly room-acoustics convolution. */
  static float dsp_rir_kernel[DSP_RIR_NTAPS];
  #if !DSP_USE_CMSIS
    /* Custom path keeps a per-channel circular history of the last NTAPS
       inputs (newest at rir_idx) so each output is one dot product. */
    static float rir_hist[2][DSP_RIR_NTAPS];
    static uint32_t rir_idx[2];
  #endif
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
  #if DSP_ENABLE_RIR
    static arm_fir_instance_f32 cmsis_rir[2];
    static float32_t cmsis_rir_state[2][CMSIS_RIR_STATE_LEN];
    static float32_t cmsis_rir_coeffs[DSP_RIR_NTAPS];   /* time-reversed kernel for arm_fir */
    static float32_t cmsis_rir_out_L[CMSIS_FRAMES];     /* arm_fir_f32 needs separate in/out */
    static float32_t cmsis_rir_out_R[CMSIS_FRAMES];
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
#if ENABLE_ICACHE
  SCB_EnableICache();
#endif
#if ENABLE_DCACHE
  SCB_EnableDCache();
#endif
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
#if DSP_ENABLE_RIR
  DSP_RIR_Build();          /* generate the synthetic room response before init/streaming */
#endif
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
#if ENABLE_DCACHE
    SCB_InvalidateDCache_by_Addr((uint32_t *)&recordPDMBuf[AUDIO_IN_PDM_BUFFER_SIZE / 2U],
                                 sizeof(recordPDMBuf) / 2U);
#endif

    BSP_AUDIO_IN_PDMToPCM(Instance,
                          (uint16_t *)&recordPDMBuf[AUDIO_IN_PDM_BUFFER_SIZE / 2U],
                          &RecPlayback[playbackPtr]);

    DSP_Process(&RecPlayback[playbackPtr], (AUDIO_IN_PDM_BUFFER_SIZE / 4U / 2U) / 2U);

#if ENABLE_DCACHE
    SCB_CleanDCache_by_Addr((uint32_t *)&RecPlayback[playbackPtr],
                            AUDIO_IN_PDM_BUFFER_SIZE / 4U);
#endif

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
#if ENABLE_DCACHE
    SCB_InvalidateDCache_by_Addr((uint32_t *)&recordPDMBuf[0],
                                 sizeof(recordPDMBuf) / 2U);
#endif

    BSP_AUDIO_IN_PDMToPCM(Instance,
                          (uint16_t *)&recordPDMBuf[0],
                          &RecPlayback[playbackPtr]);

    DSP_Process(&RecPlayback[playbackPtr], (AUDIO_IN_PDM_BUFFER_SIZE / 4U / 2U) / 2U);

#if ENABLE_DCACHE
    SCB_CleanDCache_by_Addr((uint32_t *)&RecPlayback[playbackPtr],
                            AUDIO_IN_PDM_BUFFER_SIZE / 4U);
#endif

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

#if DSP_ENABLE_GATE
  /* Noise gate, applied before any reverb so the mic's idle hiss never feeds
     the tail. Channel-linked: one envelope/gain drives both L and R so the
     stereo image can't wander. Soft-knee gain ramps 0..1 over [THRESH, THRESH+KNEE]. */
  for (uint32_t i = 0; i < frames; i++)
  {
    float aL = cmsis_buf_L[i] < 0.0f ? -cmsis_buf_L[i] : cmsis_buf_L[i];
    float aR = cmsis_buf_R[i] < 0.0f ? -cmsis_buf_R[i] : cmsis_buf_R[i];
    float in = aL > aR ? aL : aR;
    float c  = (in > gate_env) ? DSP_GATE_ATK : DSP_GATE_REL;
    gate_env += c * (in - gate_env);                 /* fast-attack / slow-release follower */
    float g = (gate_env - DSP_GATE_THRESH) / DSP_GATE_KNEE;
    if (g < 0.0f) g = 0.0f; else if (g > 1.0f) g = 1.0f;
    cmsis_buf_L[i] *= g;
    cmsis_buf_R[i] *= g;
  }
#endif

#if DSP_ENABLE_HPF
  /* 2nd-order Butterworth HPF, in-place (bilinear-transform biquad). */
  arm_biquad_cascade_df1_f32(&cmsis_hpf[0], cmsis_buf_L, cmsis_buf_L, frames);
  arm_biquad_cascade_df1_f32(&cmsis_hpf[1], cmsis_buf_R, cmsis_buf_R, frames);
#endif

#if DSP_ENABLE_LPF
  /* 2nd-order Butterworth LPF, in-place (bilinear-transform biquad). */
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
  /* arm_fir_f32 does not support in-place; output goes to separate scratch,
     then copy back so later stages keep operating on cmsis_buf_*. */
  arm_fir_f32(&cmsis_fir[0], cmsis_buf_L, cmsis_fir_out_L, frames);
  arm_fir_f32(&cmsis_fir[1], cmsis_buf_R, cmsis_fir_out_R, frames);
  for (uint32_t i = 0; i < frames; i++)
  {
    cmsis_buf_L[i] = cmsis_fir_out_L[i];
    cmsis_buf_R[i] = cmsis_fir_out_R[i];
  }
#endif

#if DSP_ENABLE_RIR
  /* Room-acoustics convolution: filter each channel with the room impulse
     response (a long FIR), then wet/dry mix. Since h[0] = 1 the dry signal
     is preserved and the reverberant tail is added on top; the makeup gain
     keeps the dry+wet sum inside the 16-bit range so it doesn't clip. */
  arm_fir_f32(&cmsis_rir[0], cmsis_buf_L, cmsis_rir_out_L, frames);
  arm_fir_f32(&cmsis_rir[1], cmsis_buf_R, cmsis_rir_out_R, frames);
  for (uint32_t i = 0; i < frames; i++)
  {
    cmsis_buf_L[i] = DSP_RIR_MAKEUP * ((1.0f - DSP_RIR_WET) * cmsis_buf_L[i] + DSP_RIR_WET * cmsis_rir_out_L[i]);
    cmsis_buf_R[i] = DSP_RIR_MAKEUP * ((1.0f - DSP_RIR_WET) * cmsis_buf_R[i] + DSP_RIR_WET * cmsis_rir_out_R[i]);
  }
#endif

  for (uint32_t i = 0; i < frames; i++)
  {
    pcm[2 * i + 0] = (uint16_t)(int16_t)dsp_clip(cmsis_buf_L[i]);
    pcm[2 * i + 1] = (uint16_t)(int16_t)dsp_clip(cmsis_buf_R[i]);
  }

  bench_cmsis_cycles = DWT->CYCCNT - _t0;

#else  /* ---- Custom hand-rolled path ---- */

  for (uint32_t i = 0; i < frames; i++)
  {
#if DSP_ENABLE_GATE
    /* Channel-linked noise gate, computed once per frame from the stereo-max
       input level so both channels share one gain (see CMSIS path for notes). */
    float gate_g;
    {
      float l = (float)(int16_t)pcm[2 * i + 0];
      float r = (float)(int16_t)pcm[2 * i + 1];
      float aL = l < 0.0f ? -l : l;
      float aR = r < 0.0f ? -r : r;
      float in = aL > aR ? aL : aR;
      float c  = (in > gate_env) ? DSP_GATE_ATK : DSP_GATE_REL;
      gate_env += c * (in - gate_env);
      gate_g = (gate_env - DSP_GATE_THRESH) / DSP_GATE_KNEE;
      if (gate_g < 0.0f) gate_g = 0.0f; else if (gate_g > 1.0f) gate_g = 1.0f;
    }
#endif
    for (uint32_t ch = 0; ch < 2; ch++)
    {
      float x = (float)(int16_t)pcm[2 * i + ch];

#if DSP_ENABLE_GATE
      x *= gate_g;
#endif
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
#if DSP_ENABLE_RIR
      /* Room-acoustics convolution via a circular history buffer:
         y[n] = sum_k h[k] * x[n-k], newest input at rir_idx going backwards. */
      rir_hist[ch][rir_idx[ch]] = x;
      float racc = 0.0f;
      uint32_t p = rir_idx[ch];
      for (uint32_t k = 0; k < DSP_RIR_NTAPS; k++)
      {
        racc += dsp_rir_kernel[k] * rir_hist[ch][p];
        p = (p == 0) ? (DSP_RIR_NTAPS - 1) : (p - 1);
      }
      if (++rir_idx[ch] >= DSP_RIR_NTAPS) rir_idx[ch] = 0;
      x = DSP_RIR_MAKEUP * ((1.0f - DSP_RIR_WET) * x + DSP_RIR_WET * racc);
#endif
      pcm[2 * i + ch] = (uint16_t)(int16_t)dsp_clip(x);
    }
  }

  bench_custom_cycles = DWT->CYCCNT - _t0;
#endif /* DSP_USE_CMSIS */
}

#if DSP_ENABLE_RIR
/**
  * @brief  Generate a simple synthetic Room Impulse Response into
  *         dsp_rir_kernel[]. Statistical room model: a unity direct path at
  *         n = 0 followed by an exponentially-decaying white-noise tail whose
  *         decay constant is set by the target RT60. Convolving audio with
  *         this kernel reproduces the room's reverberation.
  *
  *         To use a *measured* RIR instead, drop your samples into
  *         dsp_rir_kernel[] (normalised, h[0] = direct path) and skip this.
  */
static void DSP_RIR_Build(void)
{
  const float fs  = (float)AUDIO_FREQUENCY;
  /* RT60 = time for the level to fall 60 dB. For env = exp(-n/tau),
     60 dB => n = ln(1000)*tau = 6.9078*tau, so tau = RT60_samples / 6.9078. */
  const float tau = (DSP_RIR_RT60_MS / 1000.0f) * fs / 6.9078f;
  uint32_t rng = 0x1234567u;          /* deterministic xorshift32 seed */
  float energy = 0.0f;

  for (uint32_t n = 0; n < DSP_RIR_NTAPS; n++)
  {
    rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5;
    float white = (float)(int32_t)rng / 2147483648.0f;   /* ~[-1, 1) */
    float env   = expf(-(float)n / tau);
    dsp_rir_kernel[n] = white * env;
    energy += dsp_rir_kernel[n] * dsp_rir_kernel[n];
  }

  /* Normalise the diffuse tail to a fixed energy so the wet signal stays at a
     sensible level and does not clip, then force a unity direct path. */
  float g = (energy > 0.0f) ? (1.0f / sqrtf(energy)) : 1.0f;
  for (uint32_t n = 0; n < DSP_RIR_NTAPS; n++)
    dsp_rir_kernel[n] *= g;
  dsp_rir_kernel[0] = 1.0f;
}
#endif

#if DSP_USE_CMSIS
static void DSP_CMSIS_Init(void)
{
  /* Initialise biquad and FIR instances with coefficient/state arrays.
     CMSIS DF1 sign convention: a1, a2 are the POSITIVE feedback coefficients
     (y[n] = b0*x[n]+b1*x[n-1]+b2*x[n-2] + a1*y[n-1]+a2*y[n-2]).  */
#if DSP_ENABLE_HPF
  /* 2nd-order Butterworth HPF via bilinear transform.
     K = tan(π·fc/fs);  norm = 1 + √2·K + K²
     b0 =  1/norm, b1 = -2/norm, b2 = 1/norm
     a1_std = 2(K²-1)/norm → CMSIS a1 = -a1_std
     a2_std = (1-√2·K+K²)/norm → CMSIS a2 = -a2_std */
  {
    float32_t K    = tanf(DSP_PI * DSP_HPF_CUTOFF_HZ / (float32_t)AUDIO_FREQUENCY);
    float32_t K2   = K * K;
    float32_t norm = 1.0f + 1.41421356f * K + K2;
    cmsis_hpf_coeffs[0] =  1.0f / norm;
    cmsis_hpf_coeffs[1] = -2.0f / norm;
    cmsis_hpf_coeffs[2] =  1.0f / norm;
    cmsis_hpf_coeffs[3] =  2.0f * (1.0f - K2) / norm;
    cmsis_hpf_coeffs[4] = -(1.0f - 1.41421356f * K + K2) / norm;
  }
  arm_biquad_cascade_df1_init_f32(&cmsis_hpf[0], 1, cmsis_hpf_coeffs, cmsis_hpf_state[0]);
  arm_biquad_cascade_df1_init_f32(&cmsis_hpf[1], 1, cmsis_hpf_coeffs, cmsis_hpf_state[1]);
#endif
#if DSP_ENABLE_LPF
  /* 2nd-order Butterworth LPF via bilinear transform.
     b0 = K²/norm, b1 = 2K²/norm, b2 = K²/norm  (same norm as HPF at same fc) */
  {
    float32_t K    = tanf(DSP_PI * DSP_LPF_CUTOFF_HZ / (float32_t)AUDIO_FREQUENCY);
    float32_t K2   = K * K;
    float32_t norm = 1.0f + 1.41421356f * K + K2;
    cmsis_lpf_coeffs[0] =  K2 / norm;
    cmsis_lpf_coeffs[1] =  2.0f * K2 / norm;
    cmsis_lpf_coeffs[2] =  K2 / norm;
    cmsis_lpf_coeffs[3] =  2.0f * (1.0f - K2) / norm;
    cmsis_lpf_coeffs[4] = -(1.0f - 1.41421356f * K + K2) / norm;
  }
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
#if DSP_ENABLE_RIR
  /* arm_fir_f32 expects coefficients in time-reversed order, so reverse the
     room impulse response (which DSP_RIR_Build built in natural order). */
  for (uint32_t i = 0; i < DSP_RIR_NTAPS; i++)
    cmsis_rir_coeffs[i] = dsp_rir_kernel[DSP_RIR_NTAPS - 1 - i];
  arm_fir_init_f32(&cmsis_rir[0], DSP_RIR_NTAPS, cmsis_rir_coeffs, cmsis_rir_state[0], CMSIS_FRAMES);
  arm_fir_init_f32(&cmsis_rir[1], DSP_RIR_NTAPS, cmsis_rir_coeffs, cmsis_rir_state[1], CMSIS_FRAMES);
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
