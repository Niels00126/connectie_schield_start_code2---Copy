/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define User_Button_Pin GPIO_PIN_13
#define User_Button_GPIO_Port GPIOC
#define Green_Led_Pin GPIO_PIN_0
#define Green_Led_GPIO_Port GPIOC
#define Blue_Led_Pin GPIO_PIN_1
#define Blue_Led_GPIO_Port GPIOC
#define Push_Button_Pin GPIO_PIN_2
#define Push_Button_GPIO_Port GPIOA
#define BMI_INT1_Pin GPIO_PIN_4
#define BMI_INT1_GPIO_Port GPIOC
#define BMI_INT2_Pin GPIO_PIN_5
#define BMI_INT2_GPIO_Port GPIOC
#define LISS_INT1_Pin GPIO_PIN_1
#define LISS_INT1_GPIO_Port GPIOB
#define LISS_INT2_Pin GPIO_PIN_2
#define LISS_INT2_GPIO_Port GPIOB
#define SPI5_CS_Pin GPIO_PIN_14
#define SPI5_CS_GPIO_Port GPIOB
#define LSM_INT1_Pin GPIO_PIN_6
#define LSM_INT1_GPIO_Port GPIOC
#define ICM_FSYNC_Pin GPIO_PIN_7
#define ICM_FSYNC_GPIO_Port GPIOC
#define ICM_INT1_Pin GPIO_PIN_8
#define ICM_INT1_GPIO_Port GPIOA
#define SPI4_CS_Pin GPIO_PIN_13
#define SPI4_CS_GPIO_Port GPIOA
#define SPI3_CS_Pin GPIO_PIN_14
#define SPI3_CS_GPIO_Port GPIOA
#define SPI2_CS_Pin GPIO_PIN_15
#define SPI2_CS_GPIO_Port GPIOA
#define SPI1_CS_Pin GPIO_PIN_2
#define SPI1_CS_GPIO_Port GPIOD
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
