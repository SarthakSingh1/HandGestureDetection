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
#include "usb_device.h"
#include "arm_math.h"  // Include CMSIS-DSP header
#include <math.h>
#include "fatfs.h"  //audio stuff
#include "stm32f4xx_hal.h" //for sd card stuff
#include <math.h> //for mathematical operations
/* Private variables ---------------------------------------------------------*/


I2S_HandleTypeDef hi2s2;
DMA_HandleTypeDef hdma_spi2_tx;
DMA_HandleTypeDef hdma_i2s2_ext_rx;



/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C2_Init(void);
static void MX_I2S2_Init(void);
static void MX_SDIO_SD_Init(void);


/*

FIR filter designed with
http://t-filter.appspot.com

sampling frequency: 48000 Hz

* 0 Hz - 2000 Hz
  gain = 1
  desired ripple = 5 dB
  actual ripple = 3.284632065621469 dB

* 5000 Hz - 24000 Hz
  gain = 0
  desired attenuation = -30 dB
  actual attenuation = -32.02295267957132 dB

*/

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

SD_HandleTypeDef hsd;
HAL_SD_CardInfoTypedef SDCardInfo;
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


static float firData[FILTER_TAP_NUM];
static int firPtr[FILTER_TAP_NUM];
static int firWPtr = 0;
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


float Do_Distortion (float insample) {

	float threshold_noise = 2000000.0f;


	float threshold_lower = 10000000.0f;
	float gain_lower = 2.0f;

	float threshold_higher = 60000000.0f;
	float gain_higher = 0.5f;

	float outgain = 2.0f;


	if (fabs(insample) < threshold_lower && fabs(insample) > threshold_noise ) return outgain*(insample*gain_lower);
	if (fabs(insample) > threshold_higher) return outgain*(insample*gain_higher);
	return outgain*insample;
}

int main(void)
{

  HAL_Init();


  /* Configure the system clock */
  SystemClock_Config();


  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C2_Init();
  MX_I2S2_Init();
  MX_USB_DEVICE_Init();
  MX_I2C2_Init();


  HAL_I2SEx_TransmitReceive_DMA (&hi2s2, txBuf, rxBuf, 4);


  while (1)
  {
      ReadGesture(); // Call this at regular intervals
          // Other tasks...
      HAL_Delay(100); // Delay for 100 ms
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

void HAL_I2SEx_TxRxHalfCpltCallback(I2S_HandleTypeDef *hi2s) {
    callback_state = 1; // Indicate half transfer complete
    ProcessAudioEffects(); // Function to decide and call the appropriate effect
}

void HAL_I2SEx_TxRxCpltCallback(I2S_HandleTypeDef *hi2s) {
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
    useLPF = 0;
    useHPF = 0;
    useGainUp = 0;
    useGainDown = 0;
    useReverb = 0;
    useDistortion = 0;

    switch(gesture) {
        case 0: // Closed Hand
            // All effects off, already reset above
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
            l_buf_out[i] = Do_Distortion(l_buf_in[i]);
            r_buf_out[i] = Do_Distortion(r_buf_in[i]);
        }
    }

    // Convert float back to 16-bit PCM
    for (int i = 0; i < numSamples; i += 2) {
        pcmData[i] = (int16_t)(l_buf_out[i / 2] * 32767.0f);
        pcmData[i + 1] = (int16_t)(r_buf_out[i / 2] * 32767.0f);
    }
}


void EXTI0_IRQHandler(void) {
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0); // Handle EXTI line 0 interrupt
}

void EXTI1_IRQHandler(void) {
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1); // Handle EXTI line 1 interrupt
}

void EXTI2_IRQHandler(void) {
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2); // Handle EXTI line 2 interrupt
}

// Callback function called by HAL_GPIO_EXTI_IRQHandler
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    ReadGesture(); // Process the gesture
}


void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */



/**
  * @brief I2S2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S2_Init(void)
{

  /* USER CODE BEGIN I2S2_Init 0 */

  /* USER CODE END I2S2_Init 0 */

  /* USER CODE BEGIN I2S2_Init 1 */

  /* USER CODE END I2S2_Init 1 */
  hi2s2.Instance = SPI2;
  hi2s2.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_48K;
  hi2s2.Init.CPOL = I2S_CPOL_LOW;
  hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_ENABLE;
  if (HAL_I2S_Init(&hi2s2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S2_Init 2 */

  /* USER CODE END I2S2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);

}



void MX_SDIO_SD_Init(void)
{
    // Initialize SDIO peripheral
    hsd.Instance = SDIO;
    hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
    hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
    hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
    hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
    hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
    hsd.Init.ClockDiv = SDIO_TRANSFER_CLK_DIV; // Set according to your clock configuration needs

    // Initialize the SD card
    if (HAL_SD_Init(&hsd) != HAL_OK)
    {
        Error_Handler();
    }

    // Configure the SD Bus width (1 bit or 4 bits)
    if (HAL_SD_ConfigWideBusOperation(&hsd, SDIO_BUS_WIDE_4B) != HAL_OK)
    {
        Error_Handler();
    }

    // Get Card Info
    HAL_SD_GetCardInfo(&hsd, &SDCardInfo);
}



void MX_FATFS_Init(void) {
    // Link the SD card driver
    if(FATFS_LinkDriver(&SD_Driver, SDPath) == 0) {
        // Mount the file system
        if(f_mount(&SDFatFs, (TCHAR const*)SDPath, 0) != FR_OK) {
            // Error handling
            Error_Handler();
        }
    } else {
        // Driver linking error handling
        Error_Handler();
    }
}


/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE(); // Ensure GPIO port A's clock is enabled
//
//    /*Configure GPIO pin Output Level */
//    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
//
//    /*Configure GPIO pin : LED_Pin */
//    GPIO_InitStruct.Pin = LED_Pin;
//    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//    GPIO_InitStruct.Pull = GPIO_NOPULL;
//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//    HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

    // Additional code for configuring GPIOA pins as input
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING; // Interrupt Mode
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // Enable and set EXTI Line[0:2] Interrupt to the given priority
    HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);
    HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI1_IRQn);
    HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI2_IRQn);
}


/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
