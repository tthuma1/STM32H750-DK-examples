# STM32H750B-DK Digital Audio Processing Example

This project extends the `audio_record_minimal` project with digital signal processing of the captured microphone audio data. 
To learn how to the `audio_record_minimal` example, take a look at its [README.md](../audio_record_minimal/README.md).

# Project overview

The heart of this example is `Core/Src/dsp.c`, a software DSP stage that runs on
the Cortex-M7 every time a fresh block of PCM arrives from the PDM→PCM step.
`DSP_Process()` edits the interleaved stereo 16-bit PCM **in place**, so the
processed audio is what gets played back out of the line-out jack.

Six effects are available. They are independent and combinable (toggle each with
the `DSP_ENABLE_*` switches in `dsp.h`), and when more than one is enabled they
run in a **fixed chain**:

```
GATE → HPF → LPF → reverb → moving average → RIR
```

Every effect is implemented **twice**: a hand-rolled "custom" version written
from scratch, and a version built on the ARM **CMSIS-DSP** library. The
`DSP_USE_CMSIS` switch in `dsp.h` picks which path compiles. Both paths are
timed with the Cortex-M7 **DWT cycle counter** (`bench_custom_cycles` /
`bench_cmsis_cycles`), so you can read the two cycle counts in the debugger and
compare a naive C implementation against ARM's optimised one for the exact same
filter.

## Overview table

| Effect | What it does | Custom path | CMSIS path |
|--------|--------------|-------------|------------|
| **Gate** | Mutes the signal during silence so the mic's noise floor doesn't feed later effects | Channel-linked soft-knee noise gate; fast-attack / slow-release peak follower | Same scalar loop, operating on the de-interleaved float buffers |
| **HPF** | Removes low-frequency rumble / DC; keeps everything above the cutoff | 1st-order one-pole high-pass IIR (bilinear transform) | 4th-order Butterworth high-pass (cascade of two biquads) |
| **LPF** | Removes high-frequency hiss; keeps everything below the cutoff | 1st-order one-pole low-pass IIR (bilinear transform) | 4th-order Butterworth low-pass (cascade of two biquads) |
| **Reverb** | Adds a simple metallic echo/decay tail | IIR feedback delay-line (comb) | Same scalar loop (no CMSIS equivalent) |
| **Moving average** | Smooths / softens the signal (gentle treble roll-off) | 8-tap equal-weight moving-average FIR | 8-tap equal-weight FIR via `arm_fir_f32` |
| **RIR** | Convolves with a synthetic room response for realistic reverberation | Long FIR via circular history buffer | Long FIR via `arm_fir_f32` (time-reversed coeffs), wet/dry mix with makeup gain |

All cutoff frequencies, delay times, tap counts, gate thresholds and reverb
parameters live as `#define`s at the top of `dsp.h`. Because the filter
coefficients are derived from those defines with `tanf()` of a compile-time
constant argument, an optimising build folds them to literals — there is no
runtime cost to deriving the coefficients.

> **Why CMSIS can be faster.** The CMSIS-DSP block functions aren't just tidy
> wrappers — they are hand-optimised. They **unroll their inner loops** (e.g.
> processing 4 taps/samples per iteration) to cut loop overhead and keep the
> FPU pipeline fed, and on cores that have it they use **ARM NEON** SIMD to
> process several samples in a single instruction. *This board's Cortex-M7 has
> no NEON*, so here CMSIS wins only through loop unrolling and a well-scheduled
> FPU — but on a NEON-capable core (e.g. a Cortex-A application processor) the
> very same CMSIS calls can run several times faster with no code changes, which
> is a strong reason to prefer the library version when portability matters.

## Common processing model

The captured audio is **interleaved stereo 16-bit PCM** (`L, R, L, R, …`) at
`AUDIO_FREQUENCY` (16 kHz). One `DSP_Process()` call handles one block of
`frames` stereo frames.

- **Custom path** works directly on the `int16` samples, processing them sample
  by sample, with one set of filter state per channel (`[2]` arrays).
- **CMSIS path** first converts each `int16` to `float32` and **de-interleaves**
  the block into separate per-channel buffers (`cmsis_buf_L` / `cmsis_buf_R`),
  because the CMSIS block functions want contiguous mono streams. After all
  effects run it clips and **re-interleaves** back to `int16`.

Both paths finish by clamping each sample to the signed 16-bit range
`[-32768, +32767]` (`dsp_clip`) before writing it back, so an effect that
overshoots saturates cleanly instead of wrapping around.

## The effects in detail

### Gate (noise gate / downward expander)

The on-board MEMS microphone has a constant broadband self-noise floor. Dry,
this hiss is quiet and barely noticeable — but a reverb *integrates* it and
sustains it into an audible wash. The gate's job is to **mute the signal while
nothing is being said** so that no noise energy ever enters the reverb tail.
This is why it is placed first in the chain.

It works in three steps, computed once per frame:

1. **Level detection.** Take the larger of the two channels' magnitudes:
   `in = max(|L|, |R|)`. Using the stereo-max (rather than per-channel) makes
   the gate **channel-linked** — one gain is applied to both channels, so the
   stereo image can't wander as the gate opens and closes.

2. **Envelope follower** (a one-pole smoother with asymmetric time constants):

   ```
   env[n] = env[n-1] + c · (in − env[n-1])
   c = ATK  if in > env   (fast attack)
   c = REL  if in ≤ env   (slow release)
   ```

   The **fast attack** (`DSP_GATE_ATK = 0.40`) lets the gate open almost
   instantly so transients (the start of a word) aren't chopped off. The
   **slow release** (`DSP_GATE_REL = 0.004`) closes it gradually so the tail of
   a word isn't cut and the closing isn't audible as a click.

3. **Soft-knee gain.** Map the envelope to a gain in `[0, 1]` with a linear
   ramp instead of a hard switch:

   ```
   g = clamp( (env − THRESH) / KNEE , 0, 1 )
   ```

   Below `DSP_GATE_THRESH` (≈ noise floor) the gate is fully closed (`g = 0`);
   once the envelope is `DSP_GATE_KNEE` above threshold it is fully open
   (`g = 1`); in between it ramps smoothly. The two thresholds are expressed in
   raw int16 LSB and **must be tuned to your microphone** — watch `gate_env` in
   the debugger during silence and set `THRESH` a little above the idle level.

### HPF — high-pass filter

A high-pass filter removes low-frequency content (DC offset, handling rumble,
HVAC hum) below its cutoff `DSP_HPF_CUTOFF_HZ` and lets higher frequencies pass.

**Custom path — 1st-order one-pole IIR.** Designed with the bilinear (Tustin)
transform of an analog RC high-pass. The cutoff is *prewarped* so the digital
cutoff lands exactly on the analog one:

```
K  = tan(π · fc / fs)
b0 = 1 / (1 + K)
a1 = (K − 1) / (K + 1)
y[n] = b0 · (x[n] − x[n−1]) − a1 · y[n−1]
```

A first-order filter rolls off gently at **6 dB/octave**. It's one multiply-add
per sample per channel — extremely cheap, which is the point of the custom path.

**CMSIS path — 4th-order Butterworth.** Built as a cascade of **two** 2nd-order
sections (biquads) run by `arm_biquad_cascade_df1_f32`. Both sections share the
cutoff but use different pole-pair damping factors so that the overall response
is *maximally flat* in the passband (Butterworth):

```
d0 = 2·sin(π/8) ≈ 0.7654 ,  d1 = 2·sin(3π/8) ≈ 1.8478
```

Each section's coefficients (per `DSP_CMSIS_Init`):

```
K = tan(π · fc / fs) ,  norm = 1 + d·K + K²
b0 = 1/norm ,  b1 = −2/norm ,  b2 = 1/norm
a1 = 2(K²−1)/norm ,  a2 = (1 − d·K + K²)/norm     (CMSIS DF1 sign convention)
```

A 4th-order filter rolls off four times as steeply (**24 dB/octave**), giving a
much sharper transition than the custom one-pole at the cost of more arithmetic
— which is exactly the kind of trade-off the two-path benchmark is meant to
illustrate.

### LPF — low-pass filter

The mirror image of the HPF: it keeps content below `DSP_LPF_CUTOFF_HZ` and
attenuates higher frequencies (useful for taming hiss or band-limiting).

**Custom path — 1st-order one-pole IIR** (bilinear transform of an RC low-pass):

```
K  = tan(π · fc / fs)
b0 = K / (1 + K)
a1 = (K − 1) / (K + 1)
y[n] = b0 · (x[n] + x[n−1]) − a1 · y[n−1]
```

Note this shares the same denominator `a1` as the high-pass; only the feed-forward
sign differs (`x[n] + x[n−1]` for low-pass vs `x[n] − x[n−1]` for high-pass).

**CMSIS path — 4th-order Butterworth**, same two-biquad cascade and damping pair
as the HPF, with the low-pass numerator:

```
norm = 1 + d·K + K²
b0 = K²/norm ,  b1 = 2K²/norm ,  b2 = K²/norm
a1 = 2(K²−1)/norm ,  a2 = (1 − d·K + K²)/norm
```

### Reverb — feedback delay-line

A minimal, classic reverb: a single delay line fed back on itself (a **comb
filter**). Each output sample is the input plus a scaled copy of what came out
one delay-length ago:

```
y[n] = x[n] + g · y[n − D]
D = DSP_REVERB_DELAY_MS · fs / 1000   (delay in samples)
g = DSP_REVERB_FEEDBACK              (|g| < 1 for stability)
```

The feedback value is stored back into the circular `reverb_buf[ch]` so it
re-circulates, decaying by a factor `g` each pass. With `g = 0.40` and an
80 ms delay this produces a short, slightly metallic repeating echo. The
feedback gain **must satisfy `|g| < 1`**, otherwise the loop's energy grows
without bound and the output blows up.

There is no CMSIS block function for a feedback delay line, so the CMSIS path
runs the **same scalar loop**, just on the float buffers — a good illustration
that not every algorithm has a vectorised library equivalent.

### Moving average — FIR convolution smoothing

An 8-tap **finite impulse response** filter where every tap weight is equal
(`1/8`). This is a moving average:

```
y[n] = Σ_{k=0}^{7} h[k] · x[n−k] ,   h[k] = 1/8 for all k
```

A flat moving average is a simple low-pass: it smooths the waveform and gently
rolls off the high end. Being an FIR, it is **unconditionally stable** and has
linear phase. `DSP_CONV_NTAPS` sets the length.

**Custom path** keeps a per-channel history (`conv_hist`), shifts in the newest
sample, and computes the dot product by hand. **CMSIS path** uses
`arm_fir_f32`. Because `arm_fir_f32` cannot operate in place, its output goes to
a separate scratch buffer (`cmsis_fir_out_*`) which is then copied back so the
next stage in the chain still reads from `cmsis_buf_*`.

### RIR — Room Impulse Response convolution

This is the "real" reverb: instead of a single feedback echo, the signal is
convolved with a full **room impulse response** — the sound a room makes in
response to an instantaneous click. Convolving any audio with that response
makes it sound as though it were played in that room.

```
y[n] = Σ_{k=0}^{N−1} h[k] · x[n−k]
N = DSP_RIR_LEN_MS · fs / 1000   (number of taps)
```

**Building the RIR** (`DSP_RIR_Build`, run once at startup). The code uses a
statistical room model rather than a measured response:

- `h[0] = 1` is the **direct path** (the sound that reaches you straight from
  the source, undelayed and at full level).
- The rest of the taps are an **exponentially-decaying burst of white noise**,
  modelling the dense diffuse reflections of a real room:
  `h[n] = white(n) · exp(−n / τ)`.
- The decay constant `τ` is derived from the desired **RT60** — the time for the
  reverberation to fall by 60 dB. Since 60 dB is a factor of 1000 in amplitude
  and `exp(−n/τ)` reaches `1/1000` when `n = ln(1000)·τ ≈ 6.9078·τ`:

  ```
  τ = (RT60_ms / 1000) · fs / 6.9078
  ```

- The diffuse tail is normalised to a fixed energy (divide by `√Σh²`) so the wet
  level is predictable, then `h[0]` is forced back to `1`.

To use a **measured** RIR instead, you can simply drop your own normalised
samples into `dsp_rir_kernel[]` (with `h[0]` as the direct path) and skip the
synthetic builder.

**Wet/dry mix and makeup gain.** Because `h[0] = 1`, the convolution output
already contains the dry signal at full scale; adding the reverberant tail on
top would overflow ±32767 and clip. So the output is a blended, scaled mix:

```
y = MAKEUP · ( (1 − WET)·dry + WET·wet )
MAKEUP = 1 / (1 + WET)
```

`DSP_RIR_WET` sets how much tail is mixed in; `MAKEUP` restores headroom so the
reverb blends in instead of slamming the clip rails.

**Custom vs CMSIS.** The custom path keeps a per-channel **circular history
buffer** of the last `N` inputs and computes each output as one dot product
walking backwards through history. The CMSIS path uses `arm_fir_f32`, which
expects its coefficients **time-reversed**, so `DSP_CMSIS_Init` reverses
`dsp_rir_kernel[]` into `cmsis_rir_coeffs[]` before initialising the filter.

> ⚠️ **Cost warning.** The RIR is a plain time-domain FIR, so its cost scales
> linearly with the tap count `N = DSP_RIR_LEN_MS · fs / 1000`. In this project
> the M7 runs at only 64 MHz (HSI, no PLL), giving a `DSP_Process` callback just
> ~64000 cycles per 1 ms block to do *everything* (PDM→PCM, the effect chain,
> cache maintenance). Keep `DSP_RIR_LEN_MS` small enough that the convolution
> fits in the budget.

# Setup tutorial

Below is the setup needed to reproduce this example, starting from `audio_record_minimal`.

# 1. Include the CMSIS DSP library

The STM32CubeH7 MCU Firmware Package (MCU FP), downloadable from https://www.st.com/en/embedded-software/stm32cubeh7.html#get-software
includes a version of the CMSIS DSP library (https://github.com/ARM-software/CMSIS-DSP). The MCU FP v1.13.0, used in this project,
comes with CMSIS DSP v1.9.0.

In the table below, each file lives at the **same relative path** in both places: copy it from
that path under the MCU FP root into the same path under your project root.

| Files to copy | Relative path (same in MCU FP and your project) |
|---------------|----------------------------------------|
| `arm_common_tables.h`<br> `arm_const_structs.h`<br> `arm_math.h` | `Drivers/CMSIS/DSP/Include` |
| `arm_biquad_cascade_df1_f32.c`<br> `arm_biquad_cascade_df1_init_f32.c`<br> `arm_fir_f32.c`<br> `arm_fir_init_f32.c` | `Drivers/CMSIS/DSP/Source` |

# 2. Update include paths in STM32CubeIDE

Open the project in STM32CubeIDE and navigate to `Project` → `Properties` → `C/C++ Build` → `Settings` → `MCU/MPU GCC Compiler` → `Include Paths`:
  - Add `../Drivers/CMSIS/DSP/Include` under `Include paths (-I)`

# 3. Copy source files from this example

Copy these files from this example project to your project:

- `Core/Src/dsp.c`
- `Core/Src/main.c`
- `Core/Inc/dsp.h`