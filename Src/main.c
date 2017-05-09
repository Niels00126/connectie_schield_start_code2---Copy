/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "bmi160_support.h"


/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
SD_HandleTypeDef hsd;
HAL_SD_CardInfoTypedef SDCardInfo;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;
SPI_HandleTypeDef hspi4;
SPI_HandleTypeDef hspi5;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI3_Init(void);
static void MX_SPI4_Init(void);
static void MX_SPI5_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
#define MASK_DATA1	0xFF
#define MASK_DATA2	0x80
#define MASK_DATA3	0x7F
#define SPI_BUFFER_LEN  0x1
s8 bmi160_spi_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
   s32 ierror = BMI160_INIT_VALUE; // BMI160_INIT_VALUE is 0

   u8 array[SPI_BUFFER_LEN] = {MASK_DATA1};
   u8 stringpos;

   array[BMI160_INIT_VALUE] = reg_addr|MASK_DATA2; // MASK data 2 is 0x80 waardoor er gelzen wordt. //
                                                       //De reg add moet maar 7 bits groot zijn of met andere woorden er worden er 7 gebruikt
	
        HAL_GPIO_WritePin(GPIOB,SPI5_CS_Pin, GPIO_PIN_RESET);
        
        HAL_SPI_Transmit(&hspi5, array, cnt, 100);
        HAL_SPI_Receive(&hspi5, reg_data, cnt, 100);
        
        HAL_GPIO_WritePin(GPIOB,SPI5_CS_Pin, GPIO_PIN_SET);
        HAL_Delay(100);
	
	for (stringpos = BMI160_INIT_VALUE; stringpos< cnt; stringpos++){
		*(reg_data + stringpos) = array[stringpos+ BMI160_GEN_READ_WRITE_DATA_LENGTH];
        }
	return (s8)ierror;
}



s8 bmi160_spi_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	s32 ierror = BMI160_INIT_VALUE;
	

	u8 array[SPI_BUFFER_LEN * C_BMI160_BYTE_COUNT];
	u8 stringpos = BMI160_INIT_VALUE;

	for (stringpos = BMI160_INIT_VALUE; stringpos < cnt; stringpos++) {
          
            array[stringpos * C_BMI160_BYTE_COUNT] =(reg_addr++) & MASK_DATA3;
            array[stringpos * C_BMI160_BYTE_COUNT + BMI160_GEN_READ_WRITE_DATA_LENGTH] = *(reg_data + stringpos);
	}
		
        HAL_GPIO_WritePin(GPIOB,SPI5_CS_Pin, GPIO_PIN_RESET);
 
        HAL_SPI_Transmit(&hspi5, array, 1, 100);
        HAL_SPI_Transmit(&hspi5, reg_data, 1, 100);
 
        HAL_GPIO_WritePin(GPIOB,SPI5_CS_Pin, GPIO_PIN_SET);
        HAL_Delay(100);
  
	return (s8)ierror;
}



/* USER CODE END 0 */



int main(void)
{ 
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_SPI5_Init();


  uint8_t outBuffer[16];
  outBuffer[0]=0x65;
  outBuffer[1]=0x33;
  outBuffer[2]=0x22;
  outBuffer[3] = 0x66;
  
  uint8_t Buffer_ID = 0x65;
  uint8_t Buffer_Send = 0x16;
  uint8_t Buffer_Rec = 0x11;
  
  
 
 HAL_GPIO_WritePin(GPIOB,SPI5_CS_Pin, GPIO_PIN_SET);
 HAL_Delay(100);

 //HAL_GPIO_WritePin(GPIOB,SPI5_CS_Pin, GPIO_PIN_RESET);
 
 //HAL_SPI_Transmit(&hspi5, &Buffer_ID, 1, 100);
 //HAL_SPI_Transmit(&hspi5, &Buffer_Send, 1, 100);
 
 
 
 bmi160_spi_bus_write(outBuffer[3], Buffer_ID, &Buffer_Send, 1);
  
 HAL_GPIO_WritePin(GPIOB,SPI5_CS_Pin, GPIO_PIN_SET);
//Buffer_ID = 0xE5;
 HAL_Delay(100);
  HAL_GPIO_WritePin(GPIOB,SPI5_CS_Pin, GPIO_PIN_RESET);
  
  bmi160_spi_bus_read(outBuffer[3], Buffer_ID, &Buffer_Rec, 1);
 
 
 HAL_Delay(100);
 
 //bmi160_spi_bus_write(outBuffer[3], outBuffer[0], &outBuffer[1], 1);
 HAL_Delay(100);
 //bmi160_spi_bus_read(outBuffer[3], outBuffer[0], &outBuffer[2], 1);
 HAL_Delay(100);
 
 /* 
 HAL_GPIO_WritePin(GPIOB,SPI5_CS_Pin, GPIO_PIN_RESET);
// HAL_SPI_TransmitReceive(&hspi5, outBuffer, inBuffer, 2, 100);
  
 HAL_SPI_Transmit(&hspi5, &outBuffer[0], 1, 100);
 HAL_SPI_Transmit(&hspi5, &outBuffer[1], 1, 100);
  HAL_GPIO_WritePin(GPIOB,SPI5_CS_Pin, GPIO_PIN_SET);
  HAL_Delay(100);
  HAL_GPIO_WritePin(GPIOB,SPI5_CS_Pin, GPIO_PIN_RESET);
 HAL_SPI_Transmit(&hspi5, &Buffer_send, 1, 100);
 HAL_SPI_Receive(&hspi5, &Buffer, 1, 100);
 //HAL_GPIO_WritePin(GPIOB,SPI5_CS_Pin, GPIO_PIN_SET);
  */
 
 ///HAL_SPI_Transmit(&hspi5, &Buffer_send, 2, 100);
 //HAL_SPI_Receive(&hspi5, &Buffer, 2, 100);
 
 
 
 //HAL_GPIO_WritePin(GPIOB,SPI5_CS_Pin, GPIO_PIN_SET);
  //HAL_SPI_TransmitReceive(&hspi5, outBuffer, inBuffer, 16, 100);
//HAL_SPI_Transmit(&hspi5, outBuffer, 16, 100);
//HAL_SPI_Receive(&hspi5, inBuffer, 16, 100);
  /* Infinite loop */
 //Buffer_send = Buffer_send | 0x01; /* USER CODE BEGIN WHILE */
 //HAL_GPIO_WritePin(GPIOB,SPI5_CS_Pin, GPIO_PIN_RESET);
 //HAL_SPI_TransmitReceive(&hspi5, &Buffer_send, &Buffer, 1, 100);
 //HAL_GPIO_WritePin(GPIOB,SPI5_CS_Pin, GPIO_PIN_SET);
 HAL_Delay(100);

  HAL_Delay(100);
   uint8_t Buffer_read = 0x00;
 uint8_t Buffer_received = 0x11;
 uint8_t devadd= 0x66;
//bmi160_spi_bus_read(devadd, Buffer_read, &Buffer_received, 1);
 HAL_Delay(100);
 
  while (1)
  {
  /* USER CODE END WHILE */
 

  /* USER CODE BEGIN 3 */

  }
  
     //bmi160_initialize_sensor();
  
  /* USER CODE BEGIN 2 */
 /* for (int i=0;i<16;i++){
     outBuffer[i]='A'+i;
    inBuffer[i]=0;
  }*/
  /* USER CODE END 2 */

  
//bmi160_initialize_sensor();    
 //u8 count = 1;
//bmi160_spi_bus_write(&hspi5, 0x80, 0x00, count );//(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);

//HAL_SPI_TransmitReceive(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size, uint32_t Timeout
 //outBuffer[0]=0x80;
 //outBuffer[1]=0x0;

 //HAL_GPIO_WritePin(GPIOB,SPI5_CS_Pin, GPIO_PIN_RESET);
 //HAL_SPI_TransmitReceive(&hspi5, outBuffer, inBuffer, 2, 100);
 //HAL_GPIO_WritePin(GPIOB,SPI5_CS_Pin, GPIO_PIN_SET);
 
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* SDIO init function */
static void MX_SDIO_SD_Init(void)
{

  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 0;
  if (HAL_SD_Init(&hsd, &SDCardInfo) != SD_OK)
  {
    Error_Handler();
  }

}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }

}

/* SPI2 init function */
static void MX_SPI2_Init(void)
{

  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }

}

/* SPI3 init function */
static void MX_SPI3_Init(void)
{

  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }

}

/* SPI4 init function */
static void MX_SPI4_Init(void)
{

  hspi4.Instance = SPI4;
  hspi4.Init.Mode = SPI_MODE_MASTER;
  hspi4.Init.Direction = SPI_DIRECTION_2LINES;
  hspi4.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi4.Init.NSS = SPI_NSS_SOFT;
  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi4.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi4) != HAL_OK)
  {
    Error_Handler();
  }

}

/* SPI5 init function */
static void MX_SPI5_Init(void)
{

  hspi5.Instance = SPI5;
  hspi5.Init.Mode = SPI_MODE_MASTER;
  hspi5.Init.Direction = SPI_DIRECTION_2LINES;
  hspi5.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi5.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi5.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi5.Init.NSS = SPI_NSS_SOFT;
  hspi5.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi5.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi5.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi5.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi5.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi5) != HAL_OK)
  {
    Error_Handler();
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, User_Button_Pin|Green_Led_Pin|Blue_Led_Pin|BMI_INT1_Pin 
                          |BMI_INT2_Pin|LSM_INT1_Pin|ICM_FSYNC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LISS_INT1_Pin|LISS_INT2_Pin|SPI5_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, ICM_INT1_Pin|SPI4_CS_Pin|SPI3_CS_Pin|SPI2_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : User_Button_Pin Green_Led_Pin Blue_Led_Pin BMI_INT1_Pin 
                           BMI_INT2_Pin LSM_INT1_Pin ICM_FSYNC_Pin */
  GPIO_InitStruct.Pin = User_Button_Pin|Green_Led_Pin|Blue_Led_Pin|BMI_INT1_Pin 
                          |BMI_INT2_Pin|LSM_INT1_Pin|ICM_FSYNC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : Push_Button_Pin */
  GPIO_InitStruct.Pin = Push_Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Push_Button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LISS_INT1_Pin LISS_INT2_Pin SPI5_CS_Pin */
  GPIO_InitStruct.Pin = LISS_INT1_Pin|LISS_INT2_Pin|SPI5_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : ICM_INT1_Pin SPI4_CS_Pin SPI3_CS_Pin SPI2_CS_Pin */
  GPIO_InitStruct.Pin = ICM_INT1_Pin;//ICM_INT1_Pin|SPI4_CS_Pin|SPI3_CS_Pin|SPI2_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI1_CS_Pin */
  GPIO_InitStruct.Pin = SPI1_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI1_CS_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
