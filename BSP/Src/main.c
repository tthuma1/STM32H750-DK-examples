/**
  ******************************************************************************
  * @file    BSP/Src/audio_play.c
  * @author  MCD Application Team
  * @brief   This example code shows how to use the audio feature in the
  *          stm32h750b_discovery driver
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>

/** @addtogroup STM32H7xx_HAL_Examples
  * @{
  */

/** @addtogroup BSP
  * @{
  */

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/*Since SysTick is set to 1ms (unless to set it quicker) */
/* to run up to 48khz, a buffer around 1000 (or more) is requested*/
/* to run up to 96khz, a buffer around 2000 (or more) is requested*/
#define AUDIO_DEFAULT_VOLUME    70

/* Audio file size and start address are defined here since the audio file is
   stored in Flash memory as a constant table of 16-bit data */
#define AUDIO_START_OFFSET_ADDRESS    0            /* Offset relative to audio file header size */
#define AUDIO_BUFFER_SIZE            2048

/* Audio file size and start address are defined here since the audio file is
   stored in Flash memory as a constant table of 16-bit data */
#define AUDIO_START_OFFSET_ADDRESS    0            /* Offset relative to audio file header size */

#define SAMPLE_RATE     48000.0f
#define TONE_FREQ       100.0f
#define AMPLITUDE       30000     // int16 max is 32767
#define TWO_PI          6.28318530718f

/* Private typedef -----------------------------------------------------------*/
typedef enum {
  AUDIO_STATE_IDLE = 0,
  AUDIO_STATE_INIT,
  AUDIO_STATE_PLAYING,
}AUDIO_PLAYBACK_StateTypeDef;

typedef enum {
  BUFFER_OFFSET_NONE = 0,
  BUFFER_OFFSET_HALF,
  BUFFER_OFFSET_FULL,
}BUFFER_StateTypeDef;

typedef struct {
  uint8_t buff[AUDIO_BUFFER_SIZE];
  // uint32_t fptr;
  BUFFER_StateTypeDef state;
  // uint32_t AudioFileSize;
  // uint32_t *SrcAddress;
}AUDIO_BufferTypeDef;

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint32_t bytesread;
ALIGN_32BYTES (static AUDIO_BufferTypeDef  buffer_ctl);
static AUDIO_PLAYBACK_StateTypeDef  audio_state;
__IO uint32_t uwVolume = 20;
uint8_t ReadVol = 0;
__IO uint32_t uwPauseEnabledStatus = 0;
uint32_t updown = 1;

uint32_t AudioFreq[8] = {96000, 48000, 44100, 32000, 22050, 16000, 11025, 8000};

BSP_AUDIO_Init_t AudioPlayInit;

uint32_t OutputDevice = 0;

static float phase = 0.0f;
static float phase_inc = TWO_PI * TONE_FREQ / SAMPLE_RATE;

/* Private function prototypes -----------------------------------------------*/
// static uint32_t GetData(void *pdata, uint32_t offset, uint8_t *pbuf, uint32_t NbrOfData);
// AUDIO_ErrorTypeDef AUDIO_Start(uint32_t *psrc_address, uint32_t file_size);
AUDIO_ErrorTypeDef AUDIO_Start(void);
void GenerateTone(int16_t *dst, uint32_t samples);

// extern TS_Init_t hTS;

/* Private functions ---------------------------------------------------------*/
static void SystemClock_Config(void);
static void MPU_Config(void);
static void CPU_CACHE_Enable(void);

/**
  * @brief  Audio Play demo
  * @param  None
  * @retval None
  */
void main (void)
{
   MPU_Config();

  /* Enable the CPU Cache */
  CPU_CACHE_Enable();

  /* STM32H7xx HAL library initialization:
       - Systick timer is configured by default as source of time base, but user
         can eventually implement his proper time base source (a general purpose
         timer for example or other time source), keeping in mind that Time base
         duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and
         handled in milliseconds basis.
       - Set NVIC Group Priority to 4
       - Low Level Initialization
     */
  HAL_Init();

  /* Configure the system clock to 400 MHz */
  SystemClock_Config();

  /* When system initialization is finished, Cortex-M7 could wakeup (when needed) the Cortex-M4  by means of
     HSEM notification or by any D2 wakeup source (SEV,EXTI..)   */

  uint32_t *AudioFreq_ptr;
  // uint32_t y_size,x_size;
  // uint16_t x1, y1;
//  TS_State_t  TS_State;
  uint32_t AudioState;


  // BSP_LCD_GetXSize(0, &x_size);
  // BSP_LCD_GetYSize(0, &y_size);

  AudioFreq_ptr = &AudioFreq[0]; /*96K*/
  // ButtonState = 0;

   uint8_t VolStr[256] = {0};
  uint8_t FreqStr[256] = {0};
  uint8_t ts_status = BSP_ERROR_NONE;
  Point Points2[] = {{226, 196}, {265, 223}, {226, 248}};
  uwPauseEnabledStatus = 1; /* 0 when audio is running, 1 when Pause is on */
  uwVolume = 40;



  AudioPlayInit.Device = AUDIO_OUT_DEVICE_HEADPHONE;
  AudioPlayInit.ChannelsNbr = 2;
  AudioPlayInit.SampleRate = *AudioFreq_ptr;
  AudioPlayInit.BitsPerSample = AUDIO_RESOLUTION_16B;
  AudioPlayInit.Volume = uwVolume;


  if(BSP_AUDIO_OUT_Init(0, &AudioPlayInit) != 0)
  {
  }

  /*
  Start playing the file from a circular buffer, once the DMA is enabled, it is
  always in running state. Application has to fill the buffer with the audio data
  using Transfer complete and/or half transfer complete interrupts callbacks
  (BSP_AUDIO_OUT_TransferComplete_CallBack() or BSP_AUDIO_OUT_HalfTransfer_CallBack()...
  */
  // AUDIO_Start((uint32_t *)AUDIO_SRC_FILE_ADDRESS, (uint32_t)AUDIO_FILE_SIZE);
  AUDIO_Start();

  /* Display the state on the screen */

  /* IMPORTANT:
  AUDIO_Process() is called by the SysTick Handler, as it should be called
  within a periodic process */

  /* Infinite loop */
  while (1)
  {
    /* IMPORTANT: AUDIO_Process() should be called within a periodic process */
    AUDIO_Process();
    // if (CheckForUserInput() > 0)
    // {
    //   // ButtonState = 0;
    //   BSP_AUDIO_OUT_Stop(0);
    //   BSP_AUDIO_OUT_DeInit(0);
    //   return;
    // }
  }
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 400000000 (Cortex-M7 CPU Clock)
  *            HCLK(Hz)                       = 200000000 (Cortex-M4 CPU, Bus matrix Clocks)
  *            AHB Prescaler                  = 2
  *            D1 APB3 Prescaler              = 2 (APB3 Clock  100MHz)
  *            D2 APB1 Prescaler              = 2 (APB1 Clock  100MHz)
  *            D2 APB2 Prescaler              = 2 (APB2 Clock  100MHz)
  *            D3 APB4 Prescaler              = 2 (APB4 Clock  100MHz)
  *            HSE Frequency(Hz)              = 25000000
  *            PLL_M                          = 5
  *            PLL_N                          = 160
  *            PLL_P                          = 2
  *            PLL_Q                          = 4
  *            PLL_R                          = 2
  *            VDD(V)                         = 3.3
  *            Flash Latency(WS)              = 4
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  HAL_StatusTypeDef ret = HAL_OK;

  /* The voltage scaling allows optimizing the power consumption when the device is
     clocked below the maximum system frequency, to update the voltage scaling value
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_OFF;
  RCC_OscInitStruct.CSIState = RCC_CSI_OFF;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;

  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 160;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;

  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  ret = HAL_RCC_OscConfig(&RCC_OscInitStruct);
  // if(ret != HAL_OK)
  // {
  //   Error_Handler();
  // }

/* Select PLL as system clock source and configure  bus clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_D1PCLK1 | RCC_CLOCKTYPE_PCLK1 | \
                                 RCC_CLOCKTYPE_PCLK2  | RCC_CLOCKTYPE_D3PCLK1);

  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;
  ret = HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4);
  // if(ret != HAL_OK)
  // {
  //   Error_Handler();
  // }

 /*
  Note : The activation of the I/O Compensation Cell is recommended with communication  interfaces
          (GPIO, SPI, FMC, QSPI ...)  when  operating at  high frequencies(please refer to product datasheet)
          The I/O Compensation Cell activation  procedure requires :
        - The activation of the CSI clock
        - The activation of the SYSCFG clock
        - Enabling the I/O Compensation Cell : setting bit[0] of register SYSCFG_CCCSR
 */

  /*activate CSI clock mondatory for I/O Compensation Cell*/
  __HAL_RCC_CSI_ENABLE() ;

  /* Enable SYSCFG clock mondatory for I/O Compensation Cell */
  __HAL_RCC_SYSCFG_CLK_ENABLE() ;

  /* Enables the I/O Compensation Cell */
  HAL_EnableCompensationCell();
}

/**
  * @brief  Starts Audio streaming.
  * @param  None
  * @retval Audio error
  */
// AUDIO_ErrorTypeDef AUDIO_Start(uint32_t *psrc_address, uint32_t file_size)
// {
//   uint32_t bytesread;

//   buffer_ctl.state = BUFFER_OFFSET_NONE;
//   buffer_ctl.AudioFileSize = file_size;
//   buffer_ctl.SrcAddress = psrc_address;

//   bytesread = GetData( (void *)psrc_address,
//                        0,
//                        &buffer_ctl.buff[0],
//                        AUDIO_BUFFER_SIZE);
//   if(bytesread > 0)
//   {
//     BSP_AUDIO_OUT_Play(0,(uint8_t *)&buffer_ctl.buff[0], AUDIO_BUFFER_SIZE);
//     audio_state = AUDIO_STATE_PLAYING;
//     buffer_ctl.fptr = bytesread;
//     return AUDIO_ERROR_NONE;
//   }
//   return AUDIO_ERROR_IO;
// }
AUDIO_ErrorTypeDef AUDIO_Start(void)
{
  buffer_ctl.state = BUFFER_OFFSET_NONE;

  GenerateTone((int16_t *)&buffer_ctl.buff[0],
               AUDIO_BUFFER_SIZE / 4); // bytes → stereo samples

  SCB_CleanDCache_by_Addr((uint32_t*)&buffer_ctl.buff[0],
                          AUDIO_BUFFER_SIZE);

  BSP_AUDIO_OUT_Play(0,
                     (uint8_t *)&buffer_ctl.buff[0],
                     AUDIO_BUFFER_SIZE);

  audio_state = AUDIO_STATE_PLAYING;
  return AUDIO_ERROR_NONE;
}

/**
  * @brief  Gets Data from storage unit.
  * @param  None
  * @retval None
  */
// static uint32_t GetData(void *pdata, uint32_t offset, uint8_t *pbuf, uint32_t NbrOfData)
// {
//   uint8_t *lptr = pdata;
//   uint32_t ReadDataNbr;

//   ReadDataNbr = 0;
//   while(((offset + ReadDataNbr) < buffer_ctl.AudioFileSize) && (ReadDataNbr < NbrOfData))
//   {
//     pbuf[ReadDataNbr]= lptr [offset + ReadDataNbr];
//     ReadDataNbr++;
//   }

//   return ReadDataNbr;
// }

/**
  * @brief  Manages Audio process.
  * @param  None
  * @retval Audio error
  */
uint8_t AUDIO_Process(void)
{
  // uint32_t bytesread;
  AUDIO_ErrorTypeDef error_state = AUDIO_ERROR_NONE;

  switch(audio_state)
  {
  case AUDIO_STATE_PLAYING:

    // if(buffer_ctl.fptr >= buffer_ctl.AudioFileSize)
    // {
    //   /* Play audio sample again ... */
    //   buffer_ctl.fptr = 0;
    //   error_state = AUDIO_ERROR_EOF;
    // }

    /* 1st half buffer played; so fill it and continue playing from bottom*/
    // if(buffer_ctl.state == BUFFER_OFFSET_HALF)
    // {
    //   bytesread = GetData((void *)buffer_ctl.SrcAddress,
    //                       buffer_ctl.fptr,
    //                       &buffer_ctl.buff[0],
    //                       AUDIO_BUFFER_SIZE /2);

    //   if( bytesread >0)
    //   {
    //     buffer_ctl.state = BUFFER_OFFSET_NONE;
    //     buffer_ctl.fptr += bytesread;
    //           /* Clean Data Cache to update the content of the SRAM */
    //   SCB_CleanDCache_by_Addr((uint32_t*)&buffer_ctl.buff[0], AUDIO_BUFFER_SIZE/2);
    //   }
    // }
    if (buffer_ctl.state == BUFFER_OFFSET_HALF)
    {
      GenerateTone((int16_t *)&buffer_ctl.buff[0],
                  (AUDIO_BUFFER_SIZE / 2) / 4);

      buffer_ctl.state = BUFFER_OFFSET_NONE;
      SCB_CleanDCache_by_Addr((uint32_t*)&buffer_ctl.buff[0],
                              AUDIO_BUFFER_SIZE / 2);
    }


    /* 2nd half buffer played; so fill it and continue playing from top */
    // if(buffer_ctl.state == BUFFER_OFFSET_FULL)
    // {
    //   bytesread = GetData((void *)buffer_ctl.SrcAddress,
    //                       buffer_ctl.fptr,
    //                       &buffer_ctl.buff[AUDIO_BUFFER_SIZE /2],
    //                       AUDIO_BUFFER_SIZE /2);
    //   if( bytesread > 0)
    //   {
    //     buffer_ctl.state = BUFFER_OFFSET_NONE;
    //     buffer_ctl.fptr += bytesread;
    //   /* Clean Data Cache to update the content of the SRAM */
    //   SCB_CleanDCache_by_Addr((uint32_t*)&buffer_ctl.buff[AUDIO_BUFFER_SIZE/2], AUDIO_BUFFER_SIZE/2);
    //   }
    // }
    if (buffer_ctl.state == BUFFER_OFFSET_FULL)
    {
      GenerateTone((int16_t *)&buffer_ctl.buff[AUDIO_BUFFER_SIZE / 2],
                  (AUDIO_BUFFER_SIZE / 2) / 4);

      buffer_ctl.state = BUFFER_OFFSET_NONE;
      SCB_CleanDCache_by_Addr((uint32_t*)&buffer_ctl.buff[AUDIO_BUFFER_SIZE / 2],
                              AUDIO_BUFFER_SIZE / 2);
    }

    break;

  default:
    error_state = AUDIO_ERROR_NOTREADY;
    break;
  }
  return (uint8_t) error_state;
}

/*------------------------------------------------------------------------------
       Callbacks implementation:
           the callbacks API are defined __weak in the stm32769i_discovery_audio.c file
           and their implementation should be done the user code if they are needed.
           Below some examples of callback implementations.
  ----------------------------------------------------------------------------*/
/**
  * @brief  Manages the full Transfer complete event.
  * @param  None
  * @retval None
  */
void BSP_AUDIO_OUT_TransferComplete_CallBack(uint32_t Interface)
{
  if(audio_state == AUDIO_STATE_PLAYING)
  {
    /* allows AUDIO_Process() to refill 2nd part of the buffer  */
    buffer_ctl.state = BUFFER_OFFSET_FULL;
  }
}

/**
  * @brief  Manages the DMA Half Transfer complete event.
  * @param  None
  * @retval None
  */
void BSP_AUDIO_OUT_HalfTransfer_CallBack(uint32_t Interface)
{
  if(audio_state == AUDIO_STATE_PLAYING)
  {
    /* allows AUDIO_Process() to refill 1st part of the buffer  */
    buffer_ctl.state = BUFFER_OFFSET_HALF;
  }
}

/**
  * @brief  Manages the DMA FIFO error event.
  * @param  None
  * @retval None
  */
void BSP_AUDIO_OUT_Error_CallBack(uint32_t Interface)
{
  /* Display message on the LCD screen */
  UTIL_LCD_SetBackColor(UTIL_LCD_COLOR_RED);
  UTIL_LCD_DisplayStringAt(0, LINE(14), (uint8_t *)"       DMA  ERROR     ", CENTER_MODE);
  UTIL_LCD_SetBackColor(UTIL_LCD_COLOR_WHITE);

  /* Stop the program with an infinite loop */
  while (BSP_PB_GetState(BUTTON_USER) != RESET)
  {
    return;
  }

  /* could also generate a system reset to recover from the error */
  /* .... */
}
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

void GenerateTone(int16_t *dst, uint32_t samples)
{
  for (uint32_t i = 0; i < samples; i++)
  {
    int16_t s = (int16_t)(AMPLITUDE * sinf(phase));
    phase += phase_inc;
    if (phase >= TWO_PI)
      phase -= TWO_PI;

    // stereo: L and R
    dst[2*i]     = s;
    dst[2*i + 1] = s;
  }
}

/**
  * @brief  Configure the MPU attributes
  * @param  None
  * @retval None
  */
static void MPU_Config(void)
{
    MPU_Region_InitTypeDef MPU_InitStruct;

  /* Disable the MPU */
  HAL_MPU_Disable();

  /* Configure the MPU attributes as WT for SDRAM */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.BaseAddress = SDRAM_DEVICE_ADDR;
  MPU_InitStruct.Size = MPU_REGION_SIZE_16MB;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER3;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.SubRegionDisable = 0x00;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /* Enable the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}


/**
  * @brief  CPU L1-Cache enable.
  * @param  None
  * @retval None
  */
static void CPU_CACHE_Enable(void)
{
  /* Enable I-Cache */
  SCB_EnableICache();

  /* Enable D-Cache */
  SCB_EnableDCache();
}