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

/**
  * @brief  Audio Play demo
  * @param  None
  * @retval None
  */
void AudioPlay_demo (void)
{
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
