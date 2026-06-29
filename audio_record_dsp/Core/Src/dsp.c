/**
  ******************************************************************************
  * @file           : dsp.c
  * @brief          : Software DSP stage for the audio loopback. Applies the
  *                   enabled effects (gate / HPF / LPF / reverb / conv / RIR)
  *                   in place to interleaved stereo 16-bit PCM. Two paths are
  *                   provided (custom hand-rolled vs. ARM CMSIS-DSP), selected
  *                   by DSP_USE_CMSIS and benchmarked via DWT cycle counts.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "dsp.h"
#include "arm_math.h"
#include <math.h>

/* Derived, compile-time-constant coefficients -------------------------------*/
#define DSP_PI       3.14159265358979f
/* ---- First-order bilinear-transform (Tustin) filter coefficients ----
   Prewarped cutoff  K = tan(pi*fc/fs).  The tanf() argument is a compile-time
   constant, so an optimising build folds these to literals (no runtime cost).
   Difference equations used in the custom path:
     LPF:  y[n] = b0*(x[n] + x[n-1]) - a1*y[n-1]   (b1 = +b0)
     HPF:  y[n] = b0*(x[n] - x[n-1]) - a1*y[n-1]   (b1 = -b0)
   with shared denominator a1 = (K - 1)/(K + 1). */
#define DSP_LPF_K    tanf(DSP_PI * DSP_LPF_CUTOFF_HZ / (float)AUDIO_FREQUENCY)
#define DSP_LPF_B0   (DSP_LPF_K / (1.0f + DSP_LPF_K))
#define DSP_LPF_A1   ((DSP_LPF_K - 1.0f) / (DSP_LPF_K + 1.0f))
#define DSP_HPF_K    tanf(DSP_PI * DSP_HPF_CUTOFF_HZ / (float)AUDIO_FREQUENCY)
#define DSP_HPF_B0   (1.0f / (1.0f + DSP_HPF_K))
#define DSP_HPF_A1   ((DSP_HPF_K - 1.0f) / (DSP_HPF_K + 1.0f))
/* 4th-order Butterworth = cascade of two 2nd-order sections. The two sections
   share the cutoff but use different pole-pair damping factors d = 1/Q:
     d_k = 2·sin((2k-1)·π/(2N)),  N = 4  →  d0 = 2·sin(π/8), d1 = 2·sin(3π/8). */
#define DSP_BUTTER4_D0    0.76536686f
#define DSP_BUTTER4_D1    1.84775907f
#define DSP_REVERB_DELAY  (DSP_REVERB_DELAY_MS * AUDIO_FREQUENCY / 1000)
/* RIR length in taps (compile-time constant: ms * fs / 1000) */
#define DSP_RIR_NTAPS     (DSP_RIR_LEN_MS * AUDIO_FREQUENCY / 1000)
/* Output makeup gain for the RIR stage. The dry signal is already preserved at
   full scale (h[0] = 1), so adding the wet tail on top overflows ±32767 and
   clips. Scaling the wet+dry sum by 1/(1+WET) restores headroom so the reverb
   blends in instead of slamming the clip rails. */
#define DSP_RIR_MAKEUP    (1.0f / (1.0f + DSP_RIR_WET))

/* Stereo frames per DSP_Process call (matches callback formula) */
#define CMSIS_FRAMES        ((AUDIO_IN_PDM_BUFFER_SIZE / 4U / 2U) / 2U)
/* FIR state length = numTaps + blockSize - 1 */
#define CMSIS_FIR_STATE_LEN (DSP_CONV_NTAPS + CMSIS_FRAMES - 1)
#define CMSIS_RIR_STATE_LEN (DSP_RIR_NTAPS + CMSIS_FRAMES - 1)

/* Benchmark counters --------------------------------------------------------*/
/* DWT cycle counts updated each DSP_Process call; read in debugger */
volatile uint32_t bench_custom_cycles;
volatile uint32_t bench_cmsis_cycles;

#if DSP_ENABLE_GATE
  /* Channel-linked noise-gate state, persisted across calls: a smoothed
     peak envelope of the (stereo-max) input, used to derive the gate gain. */
  static float gate_env = 0.0f;
#endif
#if DSP_ENABLE_HPF
  static float hpf_x1[2], hpf_y1[2];              /* prev input, prev output */
#endif
#if DSP_ENABLE_LPF
  static float lpf_x1[2], lpf_y1[2];              /* prev input, prev output */
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
  /* 4th-order Butterworth = 2 biquad stages. Biquad state: 4 words per stage
     (x[n-1], x[n-2], y[n-1], y[n-2]); coeffs: 5 per stage (b0,b1,b2,a1,a2). */
  #if DSP_ENABLE_HPF
    static arm_biquad_casd_df1_inst_f32 cmsis_hpf[2];
    static float32_t cmsis_hpf_state[2][8];
    static float32_t cmsis_hpf_coeffs[10];  /* filled by DSP_CMSIS_Init */
  #endif
  #if DSP_ENABLE_LPF
    static arm_biquad_casd_df1_inst_f32 cmsis_lpf[2];
    static float32_t cmsis_lpf_state[2][8];
    static float32_t cmsis_lpf_coeffs[10];
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

/**
  * @brief  Apply the enabled DSP effects in place to a block of interleaved
  *         stereo 16-bit PCM. Effects are chained Gate -> HPF -> LPF -> reverb -> conv.
  *         Two paths are available, selected by DSP_USE_CMSIS:
  *           0 = custom hand-rolled sample loop
  *           1 = ARM CMSIS-DSP block functions
  *         DWT cycle counts are written to bench_custom_cycles / bench_cmsis_cycles.
  * @param  pcm    Pointer to interleaved (L,R,L,R...) PCM, treated as signed.
  * @param  frames Number of stereo frames in the block.
  */
void DSP_Process(uint16_t *pcm, uint32_t frames)
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
  /* Noise gate. Channel-linked: one envelope/gain drives both L and R so the
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
  /* 4th-order Butterworth HPF, in-place (cascaded bilinear-transform biquads). */
  arm_biquad_cascade_df1_f32(&cmsis_hpf[0], cmsis_buf_L, cmsis_buf_L, frames);
  arm_biquad_cascade_df1_f32(&cmsis_hpf[1], cmsis_buf_R, cmsis_buf_R, frames);
#endif

#if DSP_ENABLE_LPF
  /* 4th-order Butterworth LPF, in-place (cascaded bilinear-transform biquads). */
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

#if DSP_ENABLE_HPF
  const float hpf_b0 = DSP_HPF_B0, hpf_a1 = DSP_HPF_A1;   /* b1 = -b0 */
#endif
#if DSP_ENABLE_LPF
  const float lpf_b0 = DSP_LPF_B0, lpf_a1 = DSP_LPF_A1;   /* b1 = +b0 */
#endif

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
      /* First-order Butterworth HPF (bilinear transform):
         y[n] = b0*(x[n] - x[n-1]) - a1*y[n-1]. */
      hpf_y1[ch] = hpf_b0 * (x - hpf_x1[ch]) - hpf_a1 * hpf_y1[ch];
      hpf_x1[ch] = x;
      x = hpf_y1[ch];
#endif
#if DSP_ENABLE_LPF
      /* First-order Butterworth LPF (bilinear transform):
         y[n] = b0*(x[n] + x[n-1]) - a1*y[n-1]. */
      lpf_y1[ch] = lpf_b0 * (x + lpf_x1[ch]) - lpf_a1 * lpf_y1[ch];
      lpf_x1[ch] = x;
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
         y[n] = sum_k h[k] * x[n-k], newest input at rir_idx going backwards. Wet/dry mix. */
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
void DSP_RIR_Build(void)
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
void DSP_CMSIS_Init(void)
{
  /* Initialise biquad and FIR instances with coefficient/state arrays.
     CMSIS DF1 sign convention: a1, a2 are the POSITIVE feedback coefficients
     (y[n] = b0*x[n]+b1*x[n-1]+b2*x[n-2] + a1*y[n-1]+a2*y[n-2]).  */
#if DSP_ENABLE_HPF
  /* 4th-order Butterworth HPF = two cascaded 2nd-order sections via bilinear
     transform. Both sections share the cutoff but use the Butterworth pole-pair
     damping d = 1/Q (DSP_BUTTER4_D0/_D1):
       K = tan(π·fc/fs);  norm = 1 + d·K + K²
       b0 =  1/norm, b1 = -2/norm, b2 = 1/norm
       a1_std = 2(K²-1)/norm → CMSIS a1 = -a1_std
       a2_std = (1-d·K+K²)/norm → CMSIS a2 = -a2_std */
  {
    static const float32_t d[2] = { DSP_BUTTER4_D0, DSP_BUTTER4_D1 };
    float32_t K  = tanf(DSP_PI * DSP_HPF_CUTOFF_HZ / (float32_t)AUDIO_FREQUENCY);
    float32_t K2 = K * K;
    for (uint32_t s = 0; s < 2; s++)
    {
      float32_t norm = 1.0f + d[s] * K + K2;
      cmsis_hpf_coeffs[5 * s + 0] =  1.0f / norm;
      cmsis_hpf_coeffs[5 * s + 1] = -2.0f / norm;
      cmsis_hpf_coeffs[5 * s + 2] =  1.0f / norm;
      cmsis_hpf_coeffs[5 * s + 3] =  2.0f * (1.0f - K2) / norm;
      cmsis_hpf_coeffs[5 * s + 4] = -(1.0f - d[s] * K + K2) / norm;
    }
  }
  arm_biquad_cascade_df1_init_f32(&cmsis_hpf[0], 2, cmsis_hpf_coeffs, cmsis_hpf_state[0]);
  arm_biquad_cascade_df1_init_f32(&cmsis_hpf[1], 2, cmsis_hpf_coeffs, cmsis_hpf_state[1]);
#endif
#if DSP_ENABLE_LPF
  /* 4th-order Butterworth LPF = two cascaded 2nd-order sections via bilinear
     transform (same damping pair as the HPF):
       b0 = K²/norm, b1 = 2K²/norm, b2 = K²/norm,  norm = 1 + d·K + K² */
  {
    static const float32_t d[2] = { DSP_BUTTER4_D0, DSP_BUTTER4_D1 };
    float32_t K  = tanf(DSP_PI * DSP_LPF_CUTOFF_HZ / (float32_t)AUDIO_FREQUENCY);
    float32_t K2 = K * K;
    for (uint32_t s = 0; s < 2; s++)
    {
      float32_t norm = 1.0f + d[s] * K + K2;
      cmsis_lpf_coeffs[5 * s + 0] =  K2 / norm;
      cmsis_lpf_coeffs[5 * s + 1] =  2.0f * K2 / norm;
      cmsis_lpf_coeffs[5 * s + 2] =  K2 / norm;
      cmsis_lpf_coeffs[5 * s + 3] =  2.0f * (1.0f - K2) / norm;
      cmsis_lpf_coeffs[5 * s + 4] = -(1.0f - d[s] * K + K2) / norm;
    }
  }
  arm_biquad_cascade_df1_init_f32(&cmsis_lpf[0], 2, cmsis_lpf_coeffs, cmsis_lpf_state[0]);
  arm_biquad_cascade_df1_init_f32(&cmsis_lpf[1], 2, cmsis_lpf_coeffs, cmsis_lpf_state[1]);
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
