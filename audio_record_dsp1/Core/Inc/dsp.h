/**
  ******************************************************************************
  * @file           : dsp.h
  * @brief          : Software DSP stage (gate / HPF / LPF / reverb / conv / RIR)
  *                   configuration and public API for the audio loopback.
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DSP_H
#define __DSP_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"   /* AUDIO_FREQUENCY, AUDIO_IN_PDM_BUFFER_SIZE, AUDIO_BUFF_SIZE */

/* ---- CMSIS-DSP comparison switch ---- */
#define DSP_USE_CMSIS       1   /* 0 = custom hand-rolled, 1 = ARM CMSIS-DSP */

/* DSP configuration ---------------------------------------------------------*/
/* ---- DSP selection (independent, combinable; chained GATE -> HPF -> LPF -> reverb -> conv -> RIR) ---- */
#define DSP_ENABLE_GATE     0         /* noise gate / downward expander (runs first in the chain) */
#define DSP_ENABLE_HPF      0
#define DSP_ENABLE_LPF      1
#define DSP_ENABLE_REVERB   0
#define DSP_ENABLE_CONV     0          /* FIR convolution (single echo) */
#define DSP_ENABLE_RIR      0          /* Room Impulse Response convolution (room acoustics) */

/* ---- Tunables ---- */
#define DSP_HPF_CUTOFF_HZ   5000.0f
#define DSP_LPF_CUTOFF_HZ   5000.0f
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
#define DSP_RIR_RT60_MS     500         /* target reverb time of the synthetic room (RT60) */
#define DSP_RIR_LEN_MS      10         /* length of the impulse response actually convolved */
#define DSP_RIR_WET         0.85f      /* wet amount: 0 = dry only, larger = more reverb tail */

/* Exported functions prototypes ---------------------------------------------*/
void DSP_Process(uint16_t *pcm, uint32_t frames);
#if DSP_ENABLE_RIR
   void DSP_RIR_Build(void);
#endif
#if DSP_USE_CMSIS
   void DSP_CMSIS_Init(void);
#endif

/* DWT cycle counts updated each DSP_Process call; read in debugger */
extern volatile uint32_t bench_custom_cycles;
extern volatile uint32_t bench_cmsis_cycles;

#ifdef __cplusplus
}
#endif

#endif /* __DSP_H */
