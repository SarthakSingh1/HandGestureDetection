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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32h7xx_hal.h" //Needed for I2C

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */
static const uint8_t ST7036_I2C_ADDR = 0x78 << 1; //Slave address for LCD

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
void init_LCD(void); //Initializes LCD
void Case1_LCD(void); //Initializes LCD
void Case2_LCD(void); //Initializes LCD
void Case3_LCD(void); //Initializes LCD
void Case4_LCD(void); //Initializes LCD
void Case5_LCD(void); //Initializes LCD
void Case6_LCD(void); //Initializes LCD
void Case7_LCD(void); //Initializes LCD

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  uint8_t STATE2;
  uint8_t STATE1;
  uint8_t STATE0;
  uint8_t sum;

  HAL_StatusTypeDef ret;
  uint8_t dataInit[2] = {0x00, 0x80};


  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  init_LCD();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	/* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  	  	STATE2 = HAL_GPIO_ReadPin(GPIOD, STATE2_Pin);
	      	STATE1 = HAL_GPIO_ReadPin(GPIOD, STATE1_Pin);
	      	STATE0 = HAL_GPIO_ReadPin(GPIOD, STATE0_Pin);
	      	sum = (STATE2 << 2) | (STATE1 << 1)  |  STATE0;

	      	ret = HAL_I2C_Master_Transmit(&hi2c1, ST7036_I2C_ADDR, dataInit, 2, 100);
	        if(ret != HAL_OK)
	        {
	      		Error_Handler();
	      	}

	        HAL_Delay(1);

	      	switch (sum) {
	      	   case 1:
	      	    	Case1_LCD();
	      	    	break;
	      	   case 2:
	      		    Case2_LCD();
	      	    	break;
	      	   case 3:
	      		    Case3_LCD();
	      	    	break;
	      	   case 4:
	      		    Case4_LCD();
	      	    	break;
	      	   case 5:
	      		    Case5_LCD();
	      	    	break;
	      	   case 6:
	      		    Case6_LCD();
	      	    	break;
	      	   default:
	      		    Case7_LCD();
	      	    	break;
	      	}

  }

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
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
  hi2c1.Init.Timing = 0x20A09DEA;
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

  /** Configure Analogue filter
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : STATE2_Pin STATE1_Pin STATE0_Pin */
  GPIO_InitStruct.Pin = STATE2_Pin|STATE1_Pin|STATE0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/****************************************************
* Initialization For ST7036i *
*****************************************************/
void init_LCD(void)
{
	HAL_Delay(100);

	uint8_t instruct1[2] = {0x00, 0x38};
	HAL_StatusTypeDef ret1;
	ret1 = HAL_I2C_Master_Transmit(&hi2c1, ST7036_I2C_ADDR, instruct1, 2, 100);
	if(ret1 != HAL_OK)
	{
		Error_Handler();
	}

	HAL_Delay(1);

	uint8_t instruct2[2] = {0x00, 0x39};
	HAL_StatusTypeDef ret2;
	ret2 = HAL_I2C_Master_Transmit(&hi2c1, ST7036_I2C_ADDR, instruct2, 2, 100);
	if(ret2 != HAL_OK)
	{
		Error_Handler();
	}

	HAL_Delay(1);

	uint8_t instruct3[2] = {0x00, 0x14};
	HAL_StatusTypeDef ret3;
	ret3 = HAL_I2C_Master_Transmit(&hi2c1, ST7036_I2C_ADDR, instruct3, 2, 100);
	if(ret3 != HAL_OK)
	{
		Error_Handler();
	}

	HAL_Delay(1);

	uint8_t instruct4[2] = {0x00, 0x78};
	HAL_StatusTypeDef ret4;
	ret4 = HAL_I2C_Master_Transmit(&hi2c1, ST7036_I2C_ADDR, instruct4, 2, 100);
	if(ret4 != HAL_OK)
	{
		Error_Handler();
	}

	HAL_Delay(1);

	uint8_t instruct5[2] = {0x00, 0x5e};
	HAL_StatusTypeDef ret5;
	ret5 = HAL_I2C_Master_Transmit(&hi2c1, ST7036_I2C_ADDR, instruct5, 2, 100);
	if(ret5 != HAL_OK)
	{
		Error_Handler();
	}

	HAL_Delay(1);

	uint8_t instruct6[2] = {0x00, 0x6d};
	HAL_StatusTypeDef ret6;
	ret6 = HAL_I2C_Master_Transmit(&hi2c1, ST7036_I2C_ADDR, instruct6, 2, 600);
	if(ret6 != HAL_OK)
	{
		Error_Handler();
	}

	HAL_Delay(500);

	uint8_t instruct7[2] = {0x00, 0x0c};
	HAL_StatusTypeDef ret7;
	ret7 = HAL_I2C_Master_Transmit(&hi2c1, ST7036_I2C_ADDR, instruct7, 2, 100);
	if(ret7 != HAL_OK)
	{
		Error_Handler();
	}

	HAL_Delay(1);

	uint8_t instruct8[2] = {0x00, 0x01};
	HAL_StatusTypeDef ret8;
	ret8 = HAL_I2C_Master_Transmit(&hi2c1, ST7036_I2C_ADDR, instruct8, 2, 100);
	if(ret8 != HAL_OK)
	{
		Error_Handler();
	}

	HAL_Delay(10);

	uint8_t instruct9[2] = {0x00, 0x06};
	HAL_StatusTypeDef ret9;
	ret9 = HAL_I2C_Master_Transmit(&hi2c1, ST7036_I2C_ADDR, instruct9, 2, 100);
	if(ret9 != HAL_OK)
	{
		Error_Handler();
	}

	HAL_Delay(10);

}

void Case1_LCD(void)
{
	uint8_t dataBuffer11[2] = {0x40, 0x4c}; //LO_PASS
	uint8_t dataBuffer12[2] = {0x40, 0x4f}; //LO_PASS
	uint8_t dataBuffer13[2] = {0x40, 0xb0}; //LO_PASS
	uint8_t dataBuffer14[2] = {0x40, 0x50}; //LO_PASS
	uint8_t dataBuffer15[2] = {0x40, 0x41}; //LO_PASS
	uint8_t dataBuffer16[2] = {0x40, 0x53}; //LO_PASS
	uint8_t dataBuffer17[2] = {0x40, 0x53}; //LO_PASS

    HAL_StatusTypeDef ret1;
	ret1 = HAL_I2C_Master_Transmit(&hi2c1, ST7036_I2C_ADDR, dataBuffer11, 2, 100);
	if(ret1 != HAL_OK)
	{
		Error_Handler();
	}

	HAL_Delay(1);

	HAL_StatusTypeDef ret2;
	ret2 = HAL_I2C_Master_Transmit(&hi2c1, ST7036_I2C_ADDR, dataBuffer12, 2, 100);
	if(ret2 != HAL_OK)
	{
		Error_Handler();
	}

	HAL_Delay(1);

	HAL_StatusTypeDef ret3;
	ret3 = HAL_I2C_Master_Transmit(&hi2c1, ST7036_I2C_ADDR, dataBuffer13, 2, 100);
	if(ret3 != HAL_OK)
	{
		Error_Handler();
	}

	HAL_Delay(1);

	HAL_StatusTypeDef ret4;
	ret4 = HAL_I2C_Master_Transmit(&hi2c1, ST7036_I2C_ADDR, dataBuffer14, 2, 100);
	if(ret4 != HAL_OK)
	{
		Error_Handler();
	}

	HAL_Delay(1);

	HAL_StatusTypeDef ret5;
	ret5 = HAL_I2C_Master_Transmit(&hi2c1, ST7036_I2C_ADDR, dataBuffer15, 2, 100);
	if(ret5 != HAL_OK)
	{
		Error_Handler();
	}

	HAL_Delay(1);

	HAL_StatusTypeDef ret6;
	ret6 = HAL_I2C_Master_Transmit(&hi2c1, ST7036_I2C_ADDR, dataBuffer16, 2, 100);
	if(ret6 != HAL_OK)
	{
		Error_Handler();
	}

	HAL_Delay(1);

	HAL_StatusTypeDef ret7;
	ret7 = HAL_I2C_Master_Transmit(&hi2c1, ST7036_I2C_ADDR, dataBuffer17, 2, 100);
	if(ret7 != HAL_OK)
	{
		Error_Handler();
	}

	HAL_Delay(1);
}

void Case2_LCD(void)
{
	uint8_t dataBuffer21[2] = {0x40, 0x48}; //HI_PASS
	uint8_t dataBuffer22[2] = {0x40, 0x49}; //HI_PASS
	uint8_t dataBuffer23[2] = {0x40, 0xb0}; //HI_PASS
	uint8_t dataBuffer24[2] = {0x40, 0x50}; //HI_PASS
	uint8_t dataBuffer25[2] = {0x40, 0x41}; //HI_PASS
	uint8_t dataBuffer26[2] = {0x40, 0x53}; //HI_PASS
	uint8_t dataBuffer27[2] = {0x40, 0x53}; //HI_PASS

    HAL_StatusTypeDef ret1;
	ret1 = HAL_I2C_Master_Transmit(&hi2c1, ST7036_I2C_ADDR, dataBuffer21, 2, 100);
	if(ret1 != HAL_OK)
	{
		Error_Handler();
	}

	HAL_Delay(1);

	HAL_StatusTypeDef ret2;
	ret2 = HAL_I2C_Master_Transmit(&hi2c1, ST7036_I2C_ADDR, dataBuffer22, 2, 100);
	if(ret2 != HAL_OK)
	{
		Error_Handler();
	}

	HAL_Delay(1);

	HAL_StatusTypeDef ret3;
	ret3 = HAL_I2C_Master_Transmit(&hi2c1, ST7036_I2C_ADDR, dataBuffer23, 2, 100);
	if(ret3 != HAL_OK)
	{
		Error_Handler();
	}

	HAL_Delay(1);

	HAL_StatusTypeDef ret4;
	ret4 = HAL_I2C_Master_Transmit(&hi2c1, ST7036_I2C_ADDR, dataBuffer24, 2, 100);
	if(ret4 != HAL_OK)
	{
		Error_Handler();
	}

	HAL_Delay(1);

	HAL_StatusTypeDef ret5;
	ret5 = HAL_I2C_Master_Transmit(&hi2c1, ST7036_I2C_ADDR, dataBuffer25, 2, 100);
	if(ret5 != HAL_OK)
	{
		Error_Handler();
	}

	HAL_Delay(1);

	HAL_StatusTypeDef ret6;
	ret6 = HAL_I2C_Master_Transmit(&hi2c1, ST7036_I2C_ADDR, dataBuffer26, 2, 100);
	if(ret6 != HAL_OK)
	{
		Error_Handler();
	}

	HAL_Delay(1);

	HAL_StatusTypeDef ret7;
	ret7 = HAL_I2C_Master_Transmit(&hi2c1, ST7036_I2C_ADDR, dataBuffer27, 2, 100);
	if(ret7 != HAL_OK)
	{
		Error_Handler();
	}

	HAL_Delay(1);
}

void Case3_LCD(void)
{
	uint8_t dataBuffer31[2] = {0x40, 0x47}; //GAIN_UP
	uint8_t dataBuffer32[2] = {0x40, 0x41}; //GAIN_UP
	uint8_t dataBuffer33[2] = {0x40, 0x49}; //GAIN_UP
	uint8_t dataBuffer34[2] = {0x40, 0x4e}; //GAIN_UP
	uint8_t dataBuffer35[2] = {0x40, 0xb0}; //GAIN_UP
	uint8_t dataBuffer36[2] = {0x40, 0x55}; //GAIN_UP
	uint8_t dataBuffer37[2] = {0x40, 0x50}; //GAIN_UP

    HAL_StatusTypeDef ret1;
	ret1 = HAL_I2C_Master_Transmit(&hi2c1, ST7036_I2C_ADDR, dataBuffer31, 2, 100);
	if(ret1 != HAL_OK)
	{
		Error_Handler();
	}

	HAL_Delay(1);

	HAL_StatusTypeDef ret2;
	ret2 = HAL_I2C_Master_Transmit(&hi2c1, ST7036_I2C_ADDR, dataBuffer32, 2, 100);
	if(ret2 != HAL_OK)
	{
		Error_Handler();
	}

	HAL_Delay(1);

	HAL_StatusTypeDef ret3;
	ret3 = HAL_I2C_Master_Transmit(&hi2c1, ST7036_I2C_ADDR, dataBuffer33, 2, 100);
	if(ret3 != HAL_OK)
	{
		Error_Handler();
	}

	HAL_Delay(1);

	HAL_StatusTypeDef ret4;
	ret4 = HAL_I2C_Master_Transmit(&hi2c1, ST7036_I2C_ADDR, dataBuffer34, 2, 100);
	if(ret4 != HAL_OK)
	{
		Error_Handler();
	}

	HAL_Delay(1);

	HAL_StatusTypeDef ret5;
	ret5 = HAL_I2C_Master_Transmit(&hi2c1, ST7036_I2C_ADDR, dataBuffer35, 2, 100);
	if(ret5 != HAL_OK)
	{
		Error_Handler();
	}

	HAL_Delay(1);

	HAL_StatusTypeDef ret6;
	ret6 = HAL_I2C_Master_Transmit(&hi2c1, ST7036_I2C_ADDR, dataBuffer36, 2, 100);
	if(ret6 != HAL_OK)
	{
		Error_Handler();
	}

	HAL_Delay(1);

	HAL_StatusTypeDef ret7;
	ret7 = HAL_I2C_Master_Transmit(&hi2c1, ST7036_I2C_ADDR, dataBuffer37, 2, 100);
	if(ret7 != HAL_OK)
	{
		Error_Handler();
	}

	HAL_Delay(1);
}

void Case4_LCD(void)
{
	uint8_t dataBuffer41[2] = {0x40, 0x47}; //GAIN_DN
	uint8_t dataBuffer42[2] = {0x40, 0x41}; //GAIN_DN
	uint8_t dataBuffer43[2] = {0x40, 0x49}; //GAIN_DN
	uint8_t dataBuffer44[2] = {0x40, 0x4e}; //GAIN_DN
	uint8_t dataBuffer45[2] = {0x40, 0xb0}; //GAIN_DN
	uint8_t dataBuffer46[2] = {0x40, 0x44}; //GAIN_DN
	uint8_t dataBuffer47[2] = {0x40, 0x4e}; //GAIN_DN

    HAL_StatusTypeDef ret1;
	ret1 = HAL_I2C_Master_Transmit(&hi2c1, ST7036_I2C_ADDR, dataBuffer41, 2, 100);
	if(ret1 != HAL_OK)
	{
		Error_Handler();
	}

	HAL_Delay(1);

	HAL_StatusTypeDef ret2;
	ret2 = HAL_I2C_Master_Transmit(&hi2c1, ST7036_I2C_ADDR, dataBuffer42, 2, 100);
	if(ret2 != HAL_OK)
	{
		Error_Handler();
	}

	HAL_Delay(1);

	HAL_StatusTypeDef ret3;
	ret3 = HAL_I2C_Master_Transmit(&hi2c1, ST7036_I2C_ADDR, dataBuffer43, 2, 100);
	if(ret3 != HAL_OK)
	{
		Error_Handler();
	}

	HAL_Delay(1);

	HAL_StatusTypeDef ret4;
	ret4 = HAL_I2C_Master_Transmit(&hi2c1, ST7036_I2C_ADDR, dataBuffer44, 2, 100);
	if(ret4 != HAL_OK)
	{
		Error_Handler();
	}

	HAL_Delay(1);

	HAL_StatusTypeDef ret5;
	ret5 = HAL_I2C_Master_Transmit(&hi2c1, ST7036_I2C_ADDR, dataBuffer45, 2, 100);
	if(ret5 != HAL_OK)
	{
		Error_Handler();
	}

	HAL_Delay(1);

	HAL_StatusTypeDef ret6;
	ret6 = HAL_I2C_Master_Transmit(&hi2c1, ST7036_I2C_ADDR, dataBuffer46, 2, 100);
	if(ret6 != HAL_OK)
	{
		Error_Handler();
	}

	HAL_Delay(1);

	HAL_StatusTypeDef ret7;
	ret7 = HAL_I2C_Master_Transmit(&hi2c1, ST7036_I2C_ADDR, dataBuffer47, 2, 100);
	if(ret7 != HAL_OK)
	{
		Error_Handler();
	}

	HAL_Delay(1);
}

void Case5_LCD(void)
{
	uint8_t dataBuffer51[2] = {0x40, 0x52}; //REVERB!
	uint8_t dataBuffer52[2] = {0x40, 0x45}; //REVERB!
	uint8_t dataBuffer53[2] = {0x40, 0x56}; //REVERB!
	uint8_t dataBuffer54[2] = {0x40, 0x45}; //REVERB!
	uint8_t dataBuffer55[2] = {0x40, 0x52}; //REVERB!
	uint8_t dataBuffer56[2] = {0x40, 0x42}; //REVERB!
	uint8_t dataBuffer57[2] = {0x40, 0x21}; //REVERB!

    HAL_StatusTypeDef ret1;
	ret1 = HAL_I2C_Master_Transmit(&hi2c1, ST7036_I2C_ADDR, dataBuffer51, 2, 100);
	if(ret1 != HAL_OK)
	{
		Error_Handler();
	}

	HAL_Delay(1);

	HAL_StatusTypeDef ret2;
	ret2 = HAL_I2C_Master_Transmit(&hi2c1, ST7036_I2C_ADDR, dataBuffer52, 2, 100);
	if(ret2 != HAL_OK)
	{
		Error_Handler();
	}

	HAL_Delay(1);

	HAL_StatusTypeDef ret3;
	ret3 = HAL_I2C_Master_Transmit(&hi2c1, ST7036_I2C_ADDR, dataBuffer53, 2, 100);
	if(ret3 != HAL_OK)
	{
		Error_Handler();
	}

	HAL_Delay(1);

	HAL_StatusTypeDef ret4;
	ret4 = HAL_I2C_Master_Transmit(&hi2c1, ST7036_I2C_ADDR, dataBuffer54, 2, 100);
	if(ret4 != HAL_OK)
	{
		Error_Handler();
	}

	HAL_Delay(1);

	HAL_StatusTypeDef ret5;
	ret5 = HAL_I2C_Master_Transmit(&hi2c1, ST7036_I2C_ADDR, dataBuffer55, 2, 100);
	if(ret5 != HAL_OK)
	{
		Error_Handler();
	}

	HAL_Delay(1);

	HAL_StatusTypeDef ret6;
	ret6 = HAL_I2C_Master_Transmit(&hi2c1, ST7036_I2C_ADDR, dataBuffer56, 2, 100);
	if(ret6 != HAL_OK)
	{
		Error_Handler();
	}

	HAL_Delay(1);

	HAL_StatusTypeDef ret7;
	ret7 = HAL_I2C_Master_Transmit(&hi2c1, ST7036_I2C_ADDR, dataBuffer57, 2, 100);
	if(ret7 != HAL_OK)
	{
		Error_Handler();
	}

	HAL_Delay(1);
}

void Case6_LCD(void)
{
	uint8_t dataBuffer61[2] = {0x40, 0x44}; //DISTORT
	uint8_t dataBuffer62[2] = {0x40, 0x49}; //DISTORT
	uint8_t dataBuffer63[2] = {0x40, 0x53}; //DISTORT
	uint8_t dataBuffer64[2] = {0x40, 0x54}; //DISTORT
	uint8_t dataBuffer65[2] = {0x40, 0x4f}; //DISTORT
	uint8_t dataBuffer66[2] = {0x40, 0x52}; //DISTORT
	uint8_t dataBuffer67[2] = {0x40, 0x54}; //DISTORT

    HAL_StatusTypeDef ret1;
	ret1 = HAL_I2C_Master_Transmit(&hi2c1, ST7036_I2C_ADDR, dataBuffer61, 2, 100);
	if(ret1 != HAL_OK)
	{
		Error_Handler();
	}

	HAL_Delay(1);

	HAL_StatusTypeDef ret2;
	ret2 = HAL_I2C_Master_Transmit(&hi2c1, ST7036_I2C_ADDR, dataBuffer62, 2, 100);
	if(ret2 != HAL_OK)
	{
		Error_Handler();
	}

	HAL_Delay(1);

	HAL_StatusTypeDef ret3;
	ret3 = HAL_I2C_Master_Transmit(&hi2c1, ST7036_I2C_ADDR, dataBuffer63, 2, 100);
	if(ret3 != HAL_OK)
	{
		Error_Handler();
	}

	HAL_Delay(1);

	HAL_StatusTypeDef ret4;
	ret4 = HAL_I2C_Master_Transmit(&hi2c1, ST7036_I2C_ADDR, dataBuffer64, 2, 100);
	if(ret4 != HAL_OK)
	{
		Error_Handler();
	}

	HAL_Delay(1);

	HAL_StatusTypeDef ret5;
	ret5 = HAL_I2C_Master_Transmit(&hi2c1, ST7036_I2C_ADDR, dataBuffer65, 2, 100);
	if(ret5 != HAL_OK)
	{
		Error_Handler();
	}

	HAL_Delay(1);

	HAL_StatusTypeDef ret6;
	ret6 = HAL_I2C_Master_Transmit(&hi2c1, ST7036_I2C_ADDR, dataBuffer66, 2, 100);
	if(ret6 != HAL_OK)
	{
		Error_Handler();
	}

	HAL_Delay(1);

	HAL_StatusTypeDef ret7;
	ret7 = HAL_I2C_Master_Transmit(&hi2c1, ST7036_I2C_ADDR, dataBuffer67, 2, 100);
	if(ret7 != HAL_OK)
	{
		Error_Handler();
	}

	HAL_Delay(1);
}

void Case4_LCD(void)
{
	uint8_t dataBuffer71[2] = {0x40, 0x4e}; //NO_HAND
	uint8_t dataBuffer72[2] = {0x40, 0x4f}; //NO_HAND
	uint8_t dataBuffer73[2] = {0x40, 0xb0}; //NO_HAND
	uint8_t dataBuffer74[2] = {0x40, 0x48}; //NO_HAND
	uint8_t dataBuffer75[2] = {0x40, 0x41}; //NO_HAND
	uint8_t dataBuffer76[2] = {0x40, 0x4e}; //NO_HAND
	uint8_t dataBuffer77[2] = {0x40, 0x44}; //NO_HAND

    HAL_StatusTypeDef ret1;
	ret1 = HAL_I2C_Master_Transmit(&hi2c1, ST7036_I2C_ADDR, dataBuffer71, 2, 100);
	if(ret1 != HAL_OK)
	{
		Error_Handler();
	}

	HAL_Delay(1);

	HAL_StatusTypeDef ret2;
	ret2 = HAL_I2C_Master_Transmit(&hi2c1, ST7036_I2C_ADDR, dataBuffer72, 2, 100);
	if(ret2 != HAL_OK)
	{
		Error_Handler();
	}

	HAL_Delay(1);

	HAL_StatusTypeDef ret3;
	ret3 = HAL_I2C_Master_Transmit(&hi2c1, ST7036_I2C_ADDR, dataBuffer73, 2, 100);
	if(ret3 != HAL_OK)
	{
		Error_Handler();
	}

	HAL_Delay(1);

	HAL_StatusTypeDef ret4;
	ret4 = HAL_I2C_Master_Transmit(&hi2c1, ST7036_I2C_ADDR, dataBuffer74, 2, 100);
	if(ret4 != HAL_OK)
	{
		Error_Handler();
	}

	HAL_Delay(1);

	HAL_StatusTypeDef ret5;
	ret5 = HAL_I2C_Master_Transmit(&hi2c1, ST7036_I2C_ADDR, dataBuffer75, 2, 100);
	if(ret5 != HAL_OK)
	{
		Error_Handler();
	}

	HAL_Delay(1);

	HAL_StatusTypeDef ret6;
	ret6 = HAL_I2C_Master_Transmit(&hi2c1, ST7036_I2C_ADDR, dataBuffer76, 2, 100);
	if(ret6 != HAL_OK)
	{
		Error_Handler();
	}

	HAL_Delay(1);

	HAL_StatusTypeDef ret7;
	ret7 = HAL_I2C_Master_Transmit(&hi2c1, ST7036_I2C_ADDR, dataBuffer77, 2, 100);
	if(ret7 != HAL_OK)
	{
		Error_Handler();
	}

	HAL_Delay(1);
}

/*****************************************************/
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
