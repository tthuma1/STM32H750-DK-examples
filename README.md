# STM32H750B-DK Audio Examples

This repository contains examples for the STM32H750B-DK board (STM32H750XBH6 MCU).
There is a README in each folder that explains each example in further detail.
Boilerplate code was generated with STM32CubeMX v6.17.0 with STM32CubeH7 MCU Firmware Package v1.13.0; STM32CubeIDE v2.1.1 was used for building and flashing.

The examples included are:
- `audio_play_minimal`: A tutorial on playing audio through the built-in Line Out port (green TRS 3.5mm, marked as CN9) using a simple sine wave.
- `audio_record_minimal`: A tutorial on how to capture audio from the built-in MEMS microphone (IMP34DT05TR, marked as U34) and play it back via the Line Out port.
- `audio_dsp_process`: Demonstrates basic DSP processing of MEMS microphone data. Includes low- and high-pass filters, an amplitude gate, FIR smoothing, and reverb via IIR and RIR (long FIR) filters.
- `audio_speed_compare`: Compares CPU time required to capture and process MEMS microphone data using DMA, interrupt-based SAI, and SAI polling.
