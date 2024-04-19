/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "arm_math.h"  // Include CMSIS-DSP header
#include <math.h>
#include "fatfs.h"  //audio stuff
#include "stm32h7xx_hal.h" //for sd card stuff
#include <math.h> //for mathematical operations


I2C_HandleTypeDef hi2s2;
DMA_HandleTypeDef hdma_spi2_tx;
DMA_HandleTypeDef hdma_i2s2_ext_rx;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_DAC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI4_Init(void);


DAC_HandleTypeDef hdac1; //We have to declare these for some reason
I2C_HandleTypeDef hi2c1;
SPI_HandleTypeDef hspi4;

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
#define DRIVE 10.0f //distortion drive
#define WET 0.5f  //mix ratio of processed and original for distortion
#define FILTER_TAP_NUM 17
#define BLOCK_SIZE_FLOAT 512
#define BLOCK_SIZE_U16 2048
#define REVERB_BUFFER_SIZE 4800 // 100ms delay at 48kHz
#define REVERB_FEEDBACK 0.5f // Feedback coefficient
#define REVERB_MIX 0.3f // Mix between original and reverb signal

 float lpf_taps[FILTER_TAP_NUM] = {
  -0.0021834891907904987,
  0.023133081888390004,
  0.03440125360693663,
  0.054016706019288735,
  0.07610902012650608,
  0.09772535709704201,
  0.11593264129629442,
  0.12810228628568973,
  0.13238343618749146,
  0.12810228628568973,
  0.11593264129629442,
  0.09772535709704201,
  0.07610902012650608,
  0.054016706019288735,
  0.03440125360693663,
  0.023133081888390004,
  -0.0021834891907904987
};

 float hpf_taps[FILTER_TAP_NUM] = {
     // high-pass filter coefficients here
 };



// Global buffers for I2S DMA
uint16_t rxBuf[BLOCK_SIZE_U16 * 2];
uint16_t txBuf[BLOCK_SIZE_U16 * 2];
uint8_t useGainUp = 0; // 0 means gain up effect is off, 1 means on
uint8_t useGainDown = 0; // 0 means gain down effect is off, 1 means on
uint8_t callback_state = 0;
uint8_t useReverb = 0; // Control flag for reverb effect
uint8_t useLPF = 0; // Control flag for low-pass filter effect
uint8_t useHPF = 0; // Control flag for high-pass filter effect
uint8_t useDistortion = 0;

SPI_HandleTypeDef hsd;
HAL_SD_CardCIDTypedef SDCardInfo;
FATFS SDFatFs;  // File system object for SD card logical drive
FIL MyFile;     // File object
char SDPath[4]; // SD card logical drive path
float gainFactorDown = 0.8; // Example gain factor for decreasing volume
float gainFactorUp = 1.2; // Example gain factor for increasing volume
float reverbBuffer[REVERB_BUFFER_SIZE];
float l_buf_in [BLOCK_SIZE_FLOAT*2];
float r_buf_in [BLOCK_SIZE_FLOAT*2];
float l_buf_out [BLOCK_SIZE_FLOAT*2];
float r_buf_out [BLOCK_SIZE_FLOAT*2];



int reverbWriteIndex = 0;


float ApplyGain(float sample, float gainFactor) {
    return sample * gainFactor;
}



float ApplyReverb(float inputSample) {
    // Calculate the read index
    int readIndex = reverbWriteIndex - REVERB_BUFFER_SIZE / 2;
    if (readIndex < 0) readIndex += REVERB_BUFFER_SIZE;

    // Read the delayed sample and apply feedback
    float delayedSample = reverbBuffer[readIndex] * REVERB_FEEDBACK;

    // Write the new sample to the buffer
    reverbBuffer[reverbWriteIndex++] = inputSample + delayedSample;

    // Wrap the write index
    if (reverbWriteIndex >= REVERB_BUFFER_SIZE) reverbWriteIndex = 0;

    // Mix the original and delayed samples
    return inputSample * (1.0f - REVERB_MIX) + delayedSample * REVERB_MIX;
}


float applySoftClipping(float sample) {
    float threshold = 0.8f;  // Clipping threshold
    if (sample > threshold) {
        sample = threshold + (sample - threshold) / 3;
    } else if (sample < -threshold) {
        sample = -threshold + (sample + threshold) / 3;
    }
    return sample;
}
int main(void)
{

  HAL_Init();


  /* Configure the system clock */
  SystemClock_Config();


  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_DAC1_Init();
  
  /* Mount the filesystem */
    if (f_mount(&SDFatFs, SDPath, 1) == FR_OK) {
        /* Open the audio file */
        if (f_open(&MyFile, "audiofile.wav", FA_READ) == FR_OK) {
            UINT br;  // Bytes read
            while (!f_eof(&MyFile)) {
                BYTE buffer[4096];  // Temporary buffer for audio data
                if (f_read(&MyFile, buffer, sizeof(buffer), &br) == FR_OK) {
                    ProcessAudioAndEffects(buffer, br);
                }
            }
            f_close(&MyFile);
        }
        f_mount(NULL, SDPath, 1);  // Unmount the filesystem
    }

    while (1) {
        HAL_Delay(10);  // Main loop delay
    }
}


void Process_LPF(float* input, float* output, int blockSize) {
    for (int n = 0; n < blockSize; ++n) {
        float outSample = 0.0f;
        for (int i = 0; i < FILTER_TAP_NUM; ++i) {
            int index = (n - i + blockSize) % blockSize; // Circular buffer indexing
            outSample += input[index] * lpf_taps[i];
        }
        output[n] = outSample;
    }
}

void PlayRegularAudio(uint8_t *data, UINT size) {
    int numSamples = size / 2; // Assuming 16-bit audio samples
    uint16_t *samples = (uint16_t *)data;

    // Output directly to DAC or whatever output method is being used
    if (HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t *)samples, numSamples, DAC_ALIGN_12B_R) != HAL_OK) {
        // Handle possible error
        Error_Handler();
    }
}

void Process_HPF(float* input, float* output, int blockSize) {
    for (int n = 0; n < blockSize; ++n) {
        float outSample = 0.0f;
        for (int i = 0; i < FILTER_TAP_NUM; ++i) {
            int index = (n - i + blockSize) % blockSize; // Circular buffer indexing
            outSample += input[index] * hpf_taps[i];
        }
        output[n] = outSample;
    }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */

void HAL_I2SEx_TxRxHalfCpltCallback(I2C_HandleTypeDef *hi2s) {
    callback_state = 1; // Indicate half transfer complete
    ProcessAudioEffects(); // Function to decide and call the appropriate effect
}

void HAL_I2SEx_TxRxCpltCallback(I2C_HandleTypeDef *hi2s) {
    callback_state = 2; // Indicate full transfer complete
    ProcessAudioEffects(); // Function to decide and call the appropriate effect
}

// Convert 16-bit PCM data to floating point for processing
void PCM16_to_Float(int16_t *input, float *output, size_t numSamples) {
    for (size_t i = 0; i < numSamples; i++) {
        output[i] = (float)input[i] / 32768.0f; // Convert to range [-1, 1]
    }
}

// Convert processed floating point audio back to 16-bit PCM
void Float_to_PCM16(float *input, int16_t *output, size_t numSamples) {
    for (size_t i = 0; i < numSamples; i++) {
        if (input[i] < -1.0f) input[i] = -1.0f;
        else if (input[i] > 1.0f) input[i] = 1.0f;
        output[i] = (int16_t)(input[i] * 32767.0f);
    }
}

void ReadGesture(void) {
    uint8_t gesture = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) << 2) | // Bit 2
                      (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) << 1) | // Bit 1
                      HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2);         // Bit 0
    // Reset all effects first
    uint8_t useNothing = 0;  // Flag to bypass audio effects
    useLPF = 0;
    useHPF = 0;
    useGainUp = 0;
    useGainDown = 0;
    useReverb = 0;
    useDistortion = 0;

    switch(gesture) {
        case 0: // Closed Hand
        	useNothing = 1;
        	
            break;
        case 1: // Rock & Roll
            useLPF = 1;
            break;
        case 2: // Peace
            useHPF = 1;
            break;
        case 3: // Index Up
            useGainUp = 1;
            break;
        case 4: // All Fingers Up
            useGainDown = 1;
            break;
        case 5: // Pinky
            useReverb = 1;
            break;
        case 6: // Custom gesture for Distortion
            useDistortion = 1;
            break;
        case 7: // Another custom gesture (Reserved for future use)
            // Activate any other effect or combination here
            break;
        default:
            // Handle unexpected values if necessary
            break;
    }
    return gesture; // For debugging or further processing
}

void ReadAudioFile() {
    UINT br;  // Variable to store the number of bytes read
    BYTE buffer[2048];  // Buffer to store read data

    // Open the file with read access
    if(f_open(&MyFile, "audiofile.wav", FA_READ) != FR_OK) {
        // Error handling
        Error_Handler();
    }

    while(!f_eof(&MyFile)) {
        // Read data in blocks of 2048 bytes
        if(f_read(&MyFile, buffer, sizeof(buffer), &br) != FR_OK) {
            // Error handling
            break;
        }
        // Process and/or transmit data
        ProcessAudio(buffer, br);
    }

    // Close the file
    f_close(&MyFile);
}

void ProcessAudioAndEffects(uint8_t *data, UINT size) {
    int numSamples = size / 4; // Assuming 16-bit stereo data
    int16_t *samples = (int16_t*)data;
    int16_t *pcmData = (int16_t *)data;
    static float l_buf_in[BLOCK_SIZE_FLOAT];
    static float r_buf_in[BLOCK_SIZE_FLOAT];
    static float l_buf_out[BLOCK_SIZE_FLOAT];
    static float r_buf_out[BLOCK_SIZE_FLOAT];

    // Convert 16-bit PCM to float
    for (int i = 0; i < numSamples; i += 2) {
        l_buf_in[i / 2] = (float)pcmData[i] / 32768.0f;
        r_buf_in[i / 2] = (float)pcmData[i + 1] / 32768.0f;
    }

    // Reset output buffers
    memset(l_buf_out, 0, sizeof(l_buf_out));
    memset(r_buf_out, 0, sizeof(r_buf_out));

    // Apply effects based on flags
    if (useNothing) {
        PlayRegularAudio(data, size);  // Just output the audio without effects
    }
    
    if (useLPF) {
        Process_LPF(l_buf_in, l_buf_out, numSamples / 2);
        Process_LPF(r_buf_in, r_buf_out, numSamples / 2);
    }
    if (useHPF) {
        Process_HPF(l_buf_in, l_buf_out, numSamples / 2);
        Process_HPF(r_buf_in, r_buf_out, numSamples / 2);
    }
    if (useReverb) {
        for (int i = 0; i < numSamples / 2; i++) {
            l_buf_out[i] = ApplyReverb(l_buf_in[i]);
            r_buf_out[i] = ApplyReverb(r_buf_in[i]);
        }
    }
    if (useGainUp) {
        for (int i = 0; i < numSamples / 2; i++) {
            l_buf_out[i] = ApplyGain(l_buf_out[i], gainFactorUp);
            r_buf_out[i] = ApplyGain(r_buf_out[i], gainFactorUp);
        }
    }
    if (useGainDown) {
        for (int i = 0; i < numSamples / 2; i++) {
            l_buf_out[i] = ApplyGain(l_buf_out[i], gainFactorDown);
            r_buf_out[i] = ApplyGain(r_buf_out[i], gainFactorDown);
        }
    }
    if (useDistortion) {
        for (int i = 0; i < numSamples / 2; i++) {
            l_buf_out[i] = applySoftClipping(l_buf_in[i]);
            r_buf_out[i] = applySoftClipping(r_buf_in[i]);
        }
    }

    // Convert float back to 16-bit PCM
    for (int i = 0; i < numSamples; i += 2) {
        pcmData[i] = (int16_t)(l_buf_out[i / 2] * 32767.0f);
        pcmData[i + 1] = (int16_t)(r_buf_out[i / 2] * 32767.0f);
    }
    // Output the processed (or unprocessed) audio
    HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t*)samples, numSamples, DAC_ALIGN_12B_R);
}


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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00707CBB;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analog filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI4_Init(void)
{

  /* USER CODE BEGIN SPI4_Init 0 */

  /* USER CODE END SPI4_Init 0 */

  /* USER CODE BEGIN SPI4_Init 1 */

  /* USER CODE END SPI4_Init 1 */
  /* SPI4 parameter configuration*/
  hspi4.Instance = SPI4;
  hspi4.Init.Mode = SPI_MODE_MASTER;
  hspi4.Init.Direction = SPI_DIRECTION_2LINES;
  hspi4.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi4.Init.NSS = SPI_NSS_HARD_INPUT;
  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi4.Init.CRCPolynomial = 0x0;
  hspi4.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi4.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi4.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi4.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi4.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi4.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi4.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi4.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi4.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi4.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI4_Init 2 */

  /* USER CODE END SPI4_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : CARD_DET_Pin */
  GPIO_InitStruct.Pin = CARD_DET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(CARD_DET_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : STATE2_Pin STATE1_Pin STATE0_Pin */
  GPIO_InitStruct.Pin = STATE2_Pin|STATE1_Pin|STATE0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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

#ifdef  USE_FULL_ASSERT
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
