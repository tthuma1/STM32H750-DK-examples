# STM32H750B-DK Digital Audio Processing Example

This project extends the `audio_record_minimal` project with digital signal processing of the captured microphone audio data. 
To learn how the `audio_record_minimal` example works, take a look at its [README.md](../audio_record_minimal/README.md).

# Project overview

The `DSP_Process` function in `Core/Src/dsp.c` is a software DSP stage that runs on
-M7 every time a fresh block of PCM arrives from the PDM→PCM step.
`DSP_Process()` edits the interleaved stereo 16-bit PCM **in place**, so the
processed audio is what gets played back out of the line-out jack.

Six effects are available. They are independent and combinable (toggle each with
the `DSP_ENABLE_*` switches in `dsp.h`), and when more than one is enabled they
run in a **fixed chain**:

```
GATE → HPF → LPF → reverb → moving average → RIR
```

Every effect is implemented **twice**: a simple "custom" version written
from scratch, and a version built on the ARM **CMSIS-DSP** library. The
`DSP_USE_CMSIS` switch in `dsp.h` picks which path compiles. Both paths are
timed with the **DWT cycle counter** , so you can read the two cycle counts in the debugger and
compare a naive C implementation against ARM's optimised one for the exact same
filter.

## Overiview of the available effects

| Effect | What it does | Custom path | CMSIS path |
|--------|--------------|-------------|------------|
| **Amplitude gate** | Mutes the signal during silence so the mic's noise floor doesn't feed later effects | Channel-linked soft-knee noise gate; fast-attack / slow-release peak follower | Same loop (no CMSIS equivalent) |
| **High-pass filter** | Removes low-frequency rumble / DC; keeps everything above the cutoff | 1st-order one-pole high-pass IIR (Butterworth, bilinear transform) | 4th-order Butterworth high-pass (cascade of two biquads) |
| **Low-pass filter** | Removes high-frequency hiss; keeps everything below the cutoff | 1st-order one-pole low-pass IIR (Butterworth, bilinear transform) | 4th-order Butterworth low-pass (cascade of two biquads) |
| **Reverb** | Adds a simple metallic echo/decay tail | IIR feedback delay-line (comb) | Same loop (no CMSIS equivalent) |
| **Moving average** | Smooths the signal (treble reduction) | 8-tap equal-weight moving-average FIR | 8-tap equal-weight FIR via `arm_fir_f32` |
| **RIR** | Convolves with a synthetic room response for realistic reverberation, with wet/dry mix and makeup gain | Long FIR via circular history buffer | Long FIR via `arm_fir_f32` (time-reversed coeffs) over a linear state buffer with block shifting |

All cutoff frequencies, delay times, tap counts, gate thresholds and reverb
parameters live as `#define`s at the top of `dsp.h`. Filter
coefficients are derived from those at compile-time.

> In this project, the CMSIS implementations are faster, because they **unroll their inner loops** (e.g.
> processing 4 taps/samples per iteration). On cores that have it, CMSIS uses **ARM NEON** SIMD to
> process several samples in a single instruction. This board's Cortex-M7 has
> no NEON, so here CMSIS wins only through loop unrolling. On a NEON-capable core (e.g. a Cortex-A application processor) the
> same CMSIS calls would be much faster.

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
`[-32768, +32767]` (in `dsp_clip`) before writing it back, so an effect that
overshoots saturates cleanly instead of wrapping around.

## The effects in detail

### Gate (noise gate / downward expander)

The on-board MEMS microphone has a constant broadband self-noise floor. The gate's job is to mute the signal while
no meaningful data is being captured.

It works in three steps, computed once per frame:

1. **Level detection.** Take the larger of the two channels' magnitudes:
   `in = max(|L|, |R|)`. Driving the gate from the louder of the two channels
   instead of gating each one on its own keeps it **channel-linked**: both
   channels always get the same gain, so the stereo image stays put as the gate
   opens and closes.

2. **Envelope follower** (a one-pole smoother with asymmetric time constants):

$$
\begin{aligned}
\mathrm{env}[n] &= \mathrm{env}[n-1] + c \cdot \big(\mathrm{in} - \mathrm{env}[n-1]\big) \\
c &= \mathrm{ATK} \quad \text{if } \mathrm{in} > \mathrm{env} \quad (\text{fast attack}) \\
c &= \mathrm{REL} \quad \text{if } \mathrm{in} \le \mathrm{env} \quad (\text{slow release})
\end{aligned}
$$

   The **fast attack** (`DSP_GATE_ATK = 0.40`) lets the gate open almost
   instantly so transients (the start of a word) aren't chopped off. The
   **slow release** (`DSP_GATE_REL = 0.004`) closes it gradually so the tail of
   a word isn't cut and the closing isn't audible as a click.

3. **Soft-knee gain.** Map the envelope to a gain in `[0, 1]` with a linear
   ramp instead of a hard switch:

$$
g = \mathrm{clamp}\!\left( \frac{\mathrm{env} - \mathrm{THRESH}}{\mathrm{KNEE}},\; 0,\; 1 \right)
$$

   Below `DSP_GATE_THRESH` (≈ noise floor) the gate is fully closed (`g = 0`);
   once the envelope is `DSP_GATE_KNEE` above threshold it is fully open
   (`g = 1`); in between it ramps smoothly. The two thresholds are expressed in
   raw int16 LSB and **must be tuned to your microphone**. Watch `gate_env` in
   the debugger during silence and set `THRESH` a little above the idle level.

### High-pass filter

A high-pass filter removes low-frequency content (e.g. DC offset, 
bass) below its cutoff `DSP_HPF_CUTOFF_HZ` and lets higher frequencies pass.

**Custom path (1st-order one-pole IIR (Butterworth)):** Designed with the bilinear (Tustin)
transform of an analog RC high-pass. The cutoff is prewarped so the digital
cutoff lands exactly on the analog one:

$$
\begin{aligned}
K &= \tan(\pi \cdot f_c / f_s) \\
b_0 &= \frac{1}{1 + K} \\
a_1 &= \frac{K - 1}{K + 1} \\
y[n] &= b_0 \cdot \big(x[n] - x[n-1]\big) - a_1 \cdot y[n-1]
\end{aligned}
$$

A first-order filter rolls off at **6 dB/octave**.

**CMSIS path (4th-order Butterworth):** Built as a cascade of two 2nd-order
sections (biquads) run by `arm_biquad_cascade_df1_f32`. Both sections share the
cutoff but use different pole-pair damping factors so that the overall response
is *maximally flat* in the passband (Butterworth):

$$
d_0 = 2\sin(\pi/8) \approx 0.7654, \qquad d_1 = 2\sin(3\pi/8) \approx 1.8478
$$

Each section's coefficients:

$$
\begin{aligned}
K &= \tan(\pi \cdot f_c / f_s), \qquad \mathrm{norm} = 1 + d K + K^2 \\
b_0 &= \frac{1}{\mathrm{norm}}, \qquad b_1 = \frac{-2}{\mathrm{norm}}, \qquad b_2 = \frac{1}{\mathrm{norm}} \\
a_1 &= \frac{2(1-K^2)}{\mathrm{norm}}, \qquad a_2 = \frac{-(1 - d K + K^2)}{\mathrm{norm}} \quad (\text{CMSIS DF1 sign convention})
\end{aligned}
$$

The 4th-order output is the two biquads run back-to-back: the input feeds
section 1 (damping $d_0$), and its output feeds section 2 (damping $d_1$). With
the stored DF1 coefficients (feedback terms *added*), each section $s$ computes

$$
y_s[n] = b_0\,x_s[n] + b_1\,x_s[n-1] + b_2\,x_s[n-2] + a_1\,y_s[n-1] + a_2\,y_s[n-2]
$$

$$
x_1[n] = x[n], \qquad x_2[n] = y_1[n], \qquad y[n] = y_2[n]
$$

> **Note on CMSIS DF1 vs. standard biquad form:** The `a_1`/`a_2` above are the
> values out code actually stores, which differ in sign from the textbook
> biquad form. The standard difference equation *subtracts* the feedback terms:
> `y[n] = b0·x[n] + b1·x[n-1] + b2·x[n-2] − a1·y[n-1] − a2·y[n-2]`, giving
> `a1 = 2(K²−1)/norm` and `a2 = (1−dK+K²)/norm`. `arm_biquad_cascade_df1_f32`
> instead *adds* its feedback terms, so it expects those coefficients **already
> negated**, hence the flipped signs shown here.

A 4th-order filter rolls off four times as steeply (**24 dB/octave**), giving a
much sharper transition than the one-pole at the cost of more arithmetic.

### Low-pass filter

The mirror image of the HPF: it keeps content below `DSP_LPF_CUTOFF_HZ` and
attenuates higher frequencies (useful for taming hiss or band-limiting).

**Custom path (1st-order one-pole IIR (Butterworth)):** bilinear transform of an RC low-pass:

$$
\begin{aligned}
K &= \tan(\pi \cdot f_c / f_s) \\
b_0 &= \frac{K}{1 + K} \\
a_1 &= \frac{K - 1}{K + 1} \\
y[n] &= b_0 \cdot \big(x[n] + x[n-1]\big) - a_1 \cdot y[n-1]
\end{aligned}
$$

Note this shares the same denominator `a1` as the high-pass; only the feed-forward
sign differs (`x[n] + x[n−1]` for low-pass vs `x[n] − x[n−1]` for high-pass).

**CMSIS path (4th-order Butterworth):** same two-biquad cascade and damping pair
as the HPF, with the low-pass numerator:

$$
\begin{aligned}
\mathrm{norm} &= 1 + d K + K^2 \\
b_0 &= \frac{K^2}{\mathrm{norm}}, \qquad b_1 = \frac{2K^2}{\mathrm{norm}}, \qquad b_2 = \frac{K^2}{\mathrm{norm}} \\
a_1 &= \frac{2(1-K^2)}{\mathrm{norm}}, \qquad a_2 = \frac{-(1 - d K + K^2)}{\mathrm{norm}} \quad (\text{CMSIS DF1 sign convention})
\end{aligned}
$$

> **Note:** As with the HPF, these `a_1`/`a_2` are in the CMSIS DF1 sign
> convention (negated relative to the standard biquad form.

### Reverb (feedback delay-line)

A single delay line fed back on itself (a **comb
filter**). Each output sample is the input plus a scaled copy of what came out
one delay-length ago:

$$
\begin{aligned}
y[n] &= x[n] + g \cdot y[n - D] \\
D &= \mathrm{DSP\_REVERB\_DELAY\_MS} \cdot f_s / 1000 \quad (\text{delay in samples}) \\
g &= \mathrm{DSP\_REVERB\_FEEDBACK} \quad (|g| < 1 \text{ for stability})
\end{aligned}
$$

The feedback value is stored back into the circular `reverb_buf[ch]` so it
re-circulates, decaying by a factor `g` each pass. With `g = 0.40` and an
80 ms delay this produces a short, slightly metallic repeating echo. The
feedback gain **must satisfy `|g| < 1`**, otherwise the loop's energy grows
without bound and the output blows up.

There is no CMSIS block function for a feedback delay line, so the CMSIS path
runs the same scalar loop.

### Moving average (FIR convolution smoothing)

An 8-tap finite impulse response filter where every tap weight is equal
(`1/8`). This is a moving average:

$$
y[n] = \sum_{k=0}^{7} h[k] \cdot x[n-k], \qquad h[k] = \tfrac{1}{8} \text{ for all } k
$$

A flat moving average is a simple low-pass: it smooths the waveform and gently
rolls off the high end. Being an FIR, it is unconditionally stable and has
linear phase. `DSP_CONV_NTAPS` sets the length.

**Custom path** keeps a per-channel history (`conv_hist`), shifts in the newest
sample, and computes the dot product by hand. **CMSIS path** uses
`arm_fir_f32`. Because `arm_fir_f32` cannot operate in place, its output goes to
a separate scratch buffer (`cmsis_fir_out_*`) which is then copied back so the
next stage in the chain still reads from `cmsis_buf_*`.

### RIR (Room Impulse Response convolution)

Simulates a real reverb. The signal is
convolved with a full **room impulse response** (the sound a room makes in
response to an instantaneous click). Convolving any audio with that response
makes it sound as though it were played in that room.

$$
\begin{aligned}
y[n] &= \sum_{k=0}^{N-1} h[k] \cdot x[n-k] \\
N &= \mathrm{DSP\_RIR\_LEN\_MS} \cdot f_s / 1000 \quad (\text{number of taps})
\end{aligned}
$$

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

$$
\tau = (\mathrm{RT60_{ms}} / 1000) \cdot f_s / 6.9078
$$

- The diffuse tail is normalised to a fixed energy (divide by `√Σh²`) so the wet
  level is predictable, then `h[0]` is forced back to `1`.

To use a **measured** RIR instead, you can simply drop your own normalised
samples into `dsp_rir_kernel[]` (with `h[0]` as the direct path) and skip the
synthetic builder.

**Wet/dry mix and makeup gain.** The **dry**
signal is the original, unprocessed input (here the clean mic sample), while the
**wet** signal is the fully processed output of the effect (in this case the
reverberant tail produced by convolving the input with the room response). A
wet/dry mix lets you dial in how much of the effect you hear: `WET = 0` is
bypass (dry only), `WET = 1` is the full reverb, and values in between blend the
two. Because `h[0] = 1`, the convolution output already contains the dry signal
at full scale; adding the reverberant tail on top would overflow ±32767 and
clip. So the output is a blended, scaled mix:

$$
\begin{aligned}
y &= \mathrm{MAKEUP} \cdot \big( (1 - \mathrm{WET}) \cdot \mathrm{dry} + \mathrm{WET} \cdot \mathrm{wet} \big) \\
\mathrm{MAKEUP} &= \frac{1}{1 + \mathrm{WET}}
\end{aligned}
$$

`DSP_RIR_WET` sets how much tail is mixed in; `MAKEUP` restores headroom so the
reverb blends in instead of slamming the clip rails.

**Custom vs CMSIS.** The custom path keeps a per-channel **circular history
buffer** (`rir_hist`) of the last `N` inputs: the newest sample is written at a
moving write index and each output is one dot product walking backwards through
history with a modular wrap. The CMSIS path does **not** use a circular buffer.
`arm_fir_f32` keeps a **linear state buffer** of length `N + frames − 1`
(`cmsis_rir_state`, see `CMSIS_RIR_STATE_LEN`): each call appends the new block
to the tail, runs a straight (non-modular) multiply-accumulate, and at the end
`memmove`s the trailing `N − 1` history samples back to the front, which produces
a sliding delay line rather than a wrapping index. That is why its state array is `frames
− 1` longer than the tap count, and why it expects its coefficients
**time-reversed**, so `DSP_CMSIS_Init` reverses `dsp_rir_kernel[]` into
`cmsis_rir_coeffs[]` before initialising the filter.

> ⚠️ **Cost warning.** The RIR is a plain time-domain FIR, so its cost scales
> linearly with the tap count `N = DSP_RIR_LEN_MS · fs / 1000`. In this project
> the M7 runs at only 64 MHz (HSI, no PLL), giving a `DSP_Process` callback just
> ~64000 cycles per 1 ms block to do everything (PDM→PCM, the effect chain,
> cache maintenance). Keep `DSP_RIR_LEN_MS` small enough that the convolution
> fits in the budget.

# D-cache toggling

The cache can be disabled be setting `ENABLE_DCACHE` to `0`. If you disable D-cache, you will
notice the `DSP_Process` function takes significantly more time and may run out of time
often (refer to the note at the bottom). This toggle is only meant to show the importance of
caching in real-time DSP applications.

# Setup tutorial

Below is the setup needed to reproduce this example, starting from `audio_record_minimal`.

## 1. Include the CMSIS DSP library

The STM32CubeH7 MCU Firmware Package (MCU FP), downloadable from https://www.st.com/en/embedded-software/stm32cubeh7.html#get-software
includes a version of the CMSIS DSP library (https://github.com/ARM-software/CMSIS-DSP). The MCU FP v1.13.0, used in this project,
comes with CMSIS DSP v1.9.0.

In the table below, each file lives at the **same relative path** in both places: copy it from
that path under the MCU FP root into the same path under your project root.

| Files to copy | Relative path (same in MCU FP and your project) |
|---------------|----------------------------------------|
| `arm_common_tables.h`<br> `arm_const_structs.h`<br> `arm_math.h` | `Drivers/CMSIS/DSP/Include` |
| `arm_biquad_cascade_df1_f32.c`<br> `arm_biquad_cascade_df1_init_f32.c`<br> `arm_fir_f32.c`<br> `arm_fir_init_f32.c` | `Drivers/CMSIS/DSP/Source` |

## 2. Update include paths in STM32CubeIDE

Open the project in STM32CubeIDE and navigate to `Project` → `Properties` → `C/C++ Build` → `Settings` → `MCU/MPU GCC Compiler` → `Include Paths`:
  - Add `../Drivers/CMSIS/DSP/Include` under `Include paths (-I)`

## 3. Copy source files from this example

Copy these files from this example project to your project:

- `Core/Src/dsp.c`
- `Core/Src/main.c`
- `Core/Inc/dsp.h`

# Notes

If you hear wierd electric buzzing when running DSP processing, it is very likely that
your code is too slow and DMA starts reading data before it has finished processing.
In this case, try to run the SysClock at a faster frequency or optimize your code.
The M7 core is not capable of very complex real-time DSP operations.
