/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <stm32l0xx_nucleo_32.h>
#include <stdlib.h>
#include <ctype.h>
#include <stdint.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/*!
 *  Structure définissant le buffer circulaire  
 */
typedef struct circ_bbuf_t
{
  uint8_t *const buffer;
  int head;
  int tail;
  const int maxlen;
} circ_bbuf_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/*!
 *  1 pour usage multi uart sinon 0
 */
#define USE_MULTIPLE_UART 1

/*!
 *  Nombres de ports uart connectés
 */
#define MAX_UART_PORT 2

/*!
 *  Taille des|du buffer(s) circulaire(s)
 */
#if USE_MULTIPLE_UART == 1
#define CIRC_BUFFER_MAX_SIZE    550
#else
#define CIRC_BUFFER_MAX_SIZE    1024
#endif

/*!
 *  Adresse i2c de la middle board (inutile)
 */
#define I2C_ADDRESS 0x08

/*!
 *  @brief   Maccro d'initialisation du buffer circ
 *  @param   x   label du buffer circ
 *  @param   y   taille
 */
#define CIRC_BBUF_DEF(x, y)      \
  uint8_t x##_data_space[y + 1]; \
  circ_bbuf_t x = {              \
      .buffer = x##_data_space,  \
      .head = 0,                 \
      .tail = 0,                 \
      .maxlen = y + 1}

/*!
 *  réinitialiser le buffer à zero
 */
#define CIRC_BBUF_RESET(x) \
  do                       \
  {                        \
    x.head = 0;            \
    x.tail = 0;            \
  } while (0)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef hlpuart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

//Init circular buffer
#if USE_MULTIPLE_UART == 1
CIRC_BBUF_DEF(cbuf1, CIRC_BUFFER_MAX_SIZE);    // un buffer circulaire pour chaque port uart
CIRC_BBUF_DEF(cbuf2, CIRC_BUFFER_MAX_SIZE);
#else
CIRC_BBUF_DEF(cbuf, CIRC_BUFFER_MAX_SIZE);
#endif

/*!
 *  Buffers de réception par uart
 */
#if USE_MULTIPLE_UART == 1
uint8_t rxBuff1 = 0;
uint8_t rxBuff2 = 0;
#else
uint8_t rxBuff = 0;
#endif


#if USE_MULTIPLE_UART == 1
/*!
 *  Tableau contenant les différentes adresses de buffer circ
 */
circ_bbuf_t *bbuf_table[MAX_UART_PORT] = {
  &cbuf1,
  &cbuf2
};
#endif

/*!
 *  Port uart : utilisé comme index, on peut imagine une évolution 
 *              avec envoi du port uart en plus du caractère pour
 *              de meilleurs logs
 */
uint8_t uartPort = 1;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_LPUART1_UART_Init(void);
/* USER CODE BEGIN PFP */



//Circular buffer functions
int circ_bbuf_push(circ_bbuf_t *c, uint8_t data);
int circ_bbuf_pop(circ_bbuf_t *c, uint8_t *data);
int circ_bbuf_free_space(circ_bbuf_t *c);

/*!
 *  Fonction basique pour convertir un entier compris entre 0 et 9 en caractère
 */
char itoch(int i);

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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_LPUART1_UART_Init();
  /* USER CODE BEGIN 2 */

  BSP_LED_Init(LED3);

  BSP_LED_Off(LED3);
  
  //buffer d'envoi en i2c
  uint8_t txbuff[2] = {0};
  uint8_t buff = 0;

#if USE_MULTIPLE_UART == 1
  HAL_UART_Receive_IT(&hlpuart1, &rxBuff1, 1);
  HAL_UART_Receive_IT(&huart2, &rxBuff2, 1);
  /*mettre toutes les fonctions d'interupt*/
#else
  HAL_UART_Receive_IT(&huart2, &rxBuff, 1);
#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    //on tente de récup un caractère du buffer circ
#if USE_MULTIPLE_UART == 1
    if(circ_bbuf_pop(bbuf_table[uartPort-1], &buff) != -1)
#else
    if(circ_bbuf_pop(&cbuf, &txbuff[1]) != -1)
#endif
    {
      txbuff[0] = buff;
      txbuff[1] = itoch(uartPort);        //on convertie l'entier en caractere correspondant 
 

      //envoi du caractère + le port uart par i2c 
      if (HAL_I2C_Slave_Transmit(&hi2c1, txbuff, 2, 0xFF) != HAL_OK)
      {
        BSP_LED_On(LED3);
      }

    }

#if USE_MULTIPLE_UART == 1
      //si un des 2 uart n'a toujours pas reçu de données en uart, on ne change pas de port
      if(rxBuff1 != 0 && rxBuff2 != 0)
      {
        //changement du port uart 
        if(txbuff[0] == '\r' || txbuff[0] == '\n')
        {
          if(uartPort == MAX_UART_PORT)
          {
            uartPort = 1;
          }
          else
          {
            uartPort++;
          }
        }
      }
#endif  
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /**Configure the main internal regulator output voltage 
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_LPUART1
                              |RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

  //l'adresse est shift de 1 bit vers la gauche pour suivre le formalisme imposé par ST
  //hi2c1.Init.OwnAddress1 = (I2C_ADDRESS << 1);

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00100713;
  hi2c1.Init.OwnAddress1 = 16;
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
  /**Configure Analogue filter 
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /**Configure Digital filter 
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /**I2C Fast mode Plus enable 
  */
  HAL_I2CEx_EnableFastModePlus(I2C_FASTMODEPLUS_I2C1);
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200 ;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/*!
 * @brief   Permet de mettre une valeur dans le buffer circulaire
 * @param   c           circular buffer address
 * @param   data        données à mettre dans le buffer circulaire
 * @retval  int         0 - Success
 *                     -1 - Out of space
 */
int circ_bbuf_push(circ_bbuf_t *c, uint8_t data)
{
  int next;

  next = c->head + 1; // next is where head will point to after this write.
  if (next >= c->maxlen)
    next = 0;

  // if the head + 1 == tail, circular buffer is full. Notice that one slot
  // is always left empty to differentiate empty vs full condition
  if (next == c->tail)
    return -1;

  c->buffer[c->head] = data; // Load data and then move
  c->head = next;            // head to next data offset.
  return 0;                  // return success to indicate successful push.
}

/*!
 * @brief   Permet de récupérer la valeur à lire dans le buffer circulaire
 * @param   c           circular buffer address
 * @param   data        variable pour acceuillir la valeur pop
 * @retval  int         0 - Success
 *                     -1 - Empty
 */
int circ_bbuf_pop(circ_bbuf_t *c, uint8_t *data)
{
  int next;

  if (c->head == c->tail) // if the head == tail, we don't have any data
    return -1;

  next = c->tail + 1; // next is where tail will point to after this read.
  if (next >= c->maxlen)
    next = 0;

  *data = c->buffer[c->tail]; // Read data and then move
  c->tail = next;             // tail to next offset.
  return 0;                   // return success to indicate successful push.
}

/*!
 * @brief   Permet de récupérer la taille de l'espace libre dans le buffer circulaire
 * @param   c           circular buffer address
 * @retval  int         number of bytes available
 */
int circ_bbuf_free_space(circ_bbuf_t *c)
{
  int freeSpace;
  freeSpace = c->tail - c->head;
  if (freeSpace <= 0)
    freeSpace += c->maxlen;
  return freeSpace - 1; // -1 to account for the always-empty slot.
}


/*!
 * @brief   Callback de la fonction d'interuption HAL_UART_Receive_IT(...)
 * @param   huart       pointeur vers une structure de l'uart
 * @retval  none
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
#if USE_MULTIPLE_UART == 1
  if(huart == &huart2)
  {
    circ_bbuf_push(&cbuf2, rxBuff2);
    HAL_UART_Receive_IT(huart, &rxBuff2, 1);
  }
  else if(huart == &hlpuart1)
  {
    circ_bbuf_push(&cbuf1, rxBuff1);
    HAL_UART_Receive_IT(huart, &rxBuff1, 1);
  }
       
#else
  circ_bbuf_push(&cbuf, rxBuff);
  HAL_UART_Receive_IT(&huart2, &rxBuff, 1);
#endif
}

char itoch(int i)
{
  return (i + '0') ;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
