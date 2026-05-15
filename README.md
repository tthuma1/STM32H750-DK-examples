# STM32H750-DK-examples

### 15/05/2036 Instruction for BSP folder

The `BSP` folder currently contains minimal working audio record/playback demo using the BSP project.

### Startup

- Load ExtMem_Boot (https://github.com/STMicroelectronics/STM32CubeH7/tree/master/Projects/STM32H750B-DK/Templates/ExtMem_Boot) into internal flash.
- Build the project in the `BSP` folder.
- Open STM32CubeProgrammer and upload `BSP\STM32CubeIDE\Debug` to address `0x90000000`
- Done. Sound from the embedded MEMS microphone goes into the speaker connected to Line Out (green 3.5mm)

### DMA to INT and polling conversion for microphone data

Search for `BSP_AUDIO_IN_RecordPDM` in `BSP/Src/main.c`. This method is defined in `BSP\Drivers\BSP\STM32H750B-DK\stm32h750b_discovery_audio.c`, which uses `HAL_SAI_Receive_DMA`. Replace the `HAL_SAI_Receive_DMA` call with `HAL_SAI_Receive` (polling) or `HAL_SAI_Receive_IT` (interrupt based) calls and write interrupt callbacks as needed.

Currently this callback is used for `HAL_SAI_Receive_DMA`. Something similar should work with `HAL_SAI_Receive_IT`:

```c
void HAL_SAI_RxCpltCallback(SAI_HandleTypeDef *hsai)
{
  /* Call the record update function to get the second half */
  if (hsai->Instance == AUDIO_IN_SAIx)
  {
    BSP_AUDIO_IN_TransferComplete_CallBack(0);
  }
  else
  {
    BSP_AUDIO_IN_TransferComplete_CallBack(1);
  }
}
```

