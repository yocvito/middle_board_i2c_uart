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

typedef struct {
    void * const buffer;
    uint16_t push_count;
    uint16_t pop_count;
    uint16_t const size;
    uint16_t const element_size;
} circ_gbuf_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//taille du buffer d'émission
#define TXBUFFERSIZE 140

//si l'on utilise un ou plusieurs ports uart
#define USE_MULTIPLE_UART 0

//mettre à zero si l'on veut faire des tests avec l'UART par exemple
#define USE_I2C 1

//nombre de port uart connectés
#define MAX_UART_PORT 2

#define I2C_ADDRESS 0x08

// Macro Switches

// uncomment to zero element space in XX_circ_gbuf_data after a pop.
// #define CRICBUF_CLEAN_ON_POP

/*!
 *  Define init macro
 */
#define CIRC_GBUF_V_DEF(t,x,s)                 \
    t x ## _circ_gbuf_data[s];                 \
    circ_gbuf_t x = {                          \
        .buffer = x ## _circ_gbuf_data,        \
        .push_count = 0,                       \
        .pop_count = 0,                        \
        .size = s ,                            \
        .element_size = sizeof(t)              \
    };
/*!
 *  define init macro + buffer manipulation macros
 */
#define CIRC_GBUF_DEF(t,x,s)                   \
    CIRC_GBUF_V_DEF(t,x,s)                     \
    inline int x##_push_refd(t * pt)           \
    {                                          \
        return circ_gbuf_push(&x,pt);          \
    }                                          \
    inline int x##_pop_refd(t * pt)            \
    {                                          \
        return circ_gbuf_pop(&x,pt,0);         \
    }                                          \
    inline int x##_peek_refd(t * pt)           \
    {                                          \
        return circ_gbuf_pop(&x,pt,1);         \
    }


/*!
 *  Reset the buffer to 0
 */
#define CIRC_GBUF_RESET(x)                     \
    do {                                       \
        x.push_count = 0;                      \
        x.pop_count = 0;                       \
    } while(0)

/*!
 *  Push an element into the buffer
 */
#define CIRC_GBUF_PUSH(x,y) x ## _push_refd(y)
/*!
 *  Peek the last element of the buffer without delete it
 */
#define CIRC_GBUF_PEEK(x,y) x ## _peek_refd(y)
/*!
 *  Peek the last element of the buffer
 */
#define CIRC_GBUF_POP(x,y) x ## _pop_refd(y)
/*!
 *  Get the size of the free space
 */
#define CIRC_GBUF_FS(x) circ_gbuf_free_space(&x)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

//Init circular buffer
CIRC_GBUF_DEF(uint8_t, cbuf, 1024);

uint8_t txBuff = 0;
uint8_t rxBuff[2] = {NULL};

#if USE_MULTIPLE_UART == 1
uint8_t deveui_table[MAX_UART_PORT][8] = {0};
#else
uint8_t devEui[8] = {0};
#endif

#if USE_MULTIPLE_UART == 1
UART_HandleTypeDef *uart_table[MAX_UART_PORT] = {
    0,
    &huart2
};
#endif

uint8_t uartPort = 2;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

//Circular buffer functions
int circ_gbuf_push(circ_gbuf_t *circ_gbuf, void *elem);
int circ_gbuf_pop (circ_gbuf_t *circ_gbuf, void *elem, int read_only);
int circ_gbuf_free_space(circ_gbuf_t *circ_gbuf);


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
  /* USER CODE BEGIN 2 */

  BSP_LED_Init(LED3);

  BSP_LED_Off(LED3);



  uint8_t cpybf[10] = {0};

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    while (HAL_UART_GetState(&huart2) != 32)
      {
      }

    if (HAL_UART_Receive_IT(&huart2, rxBuff, 1) == HAL_OK)
    {
      
      CIRC_GBUF_PUSH(cbuf,rxBuff[0]);

      CIRC_GBUF_POP(cbuf,&txBuff);

      if (HAL_I2C_Slave_Transmit_IT(&hi2c1, &txBuff, 1) != HAL_OK)
      {
        BSP_LED_On(LED3);
      }

      while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)
      {
      }    
    }

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
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

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00100413;
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
  huart2.Init.BaudRate = 115200;
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
 *  @brief  Analyse des différents charactères des données de la trame pour reconnaitre ou non le DevEui
 *  @param  str      Données reçu par UART
 *  @retval boolean
 */
static bool isDevEui(char *str)
{
  bool ret = false;

  if ((strlen(str) - 1) >= 37)
  {
    for (int i = 15, k = i + 2; i < 37; i++)
    {
      if (k != i)
      {
        if (isxdigit(str[i]))
          ret = true;
        else
          ret = false;
      }
      else
      {
        k += 3;
      }
    }
  }
  return ret;
}

/*!
 *  @brief  Vérifie si le devEui n'est pas initialisé
 *  @retval boolean
 */
static bool isEmptyDevEui()
{
  bool ret = false;
  for (int i = 0; i < 7; i++)
  {
#if USE_MULTIPLE_UART == 1
    ret = (deveui_table[uartPort - 1][i] == 0) ? true : false;
#else
    ret = (devEui[i] == 0) ? true : false;
#endif
  }
  return ret;
}

/*!
 *  @brief  Permet de reconstituer le DevEui au format envoyer par i2c
 *  @param  str_deveui   chaine de charatère contenant le DevEui non formaté
 *  @retval none
 */
static void devEuiFormat(uint8_t *str_deveui)
{
  char buff[3];
  int ibuff = 0;
  for (int i = 0, k = 0; i < strlen((char *)str_deveui); i++)
  {
    if (i <= 15)
      continue;
    if (str_deveui[i] != '-' && str_deveui[i + 1] != '-' && i != 37)
    {
      //on utilise un for au lieu d'une simple copie de valeur de str_deveui à buff car problème lors de la copie des valeurs de txBuff
      for (int j = '0'; j <= 'F'; j++)
      {
        if (isxdigit(j))
        {
          if (j == str_deveui[i])
            buff[0] = j;
          if (j == str_deveui[i + 1])
            buff[1] = j;
        }
      }
      buff[2] = '\0';
      //conversion en base 16 du buffer dans un entier
      ibuff = (int)strtol(buff, NULL, 16);
#if USE_MULTIPLE_UART == 1
      deveui_table[uartPort - 1][k] = ibuff;
#else
      devEui[k] = ibuff;
#endif
      k++;
    }
    if (i == 37)
      break;
  }
}

#if USE_I2C == 1
/*!
 *  @brief  Permet de formater la chaine selon le format suivant : 
 *    - i=0             i2c addr
 *    - i=1             uart port
 *    - i=2 à i=10      device unique id
 *    - i>10            payload
 *  @param  str_deveui   chaine de charatère contenant le DevEui non formaté
 *  @retval none
 */
/*
static void buildI2cFrame(char *payload)
{
  //on remet la chaine à 0
  iTxData[0] = '\0';
  for (int i = 0, k = 0; i < 10; i++)
  {
    //i2c addr
    if (i == 0)
    {
      iTxData[i] = I2C_ADDRESS;
    }
    //uart port
    else if (i == 1)
    {
      iTxData[i] = uartPort;
    }
    //DevEui
    else
    {
#if USE_MULTIPLE_UART == 1
      iTxData[i] = deveui_table[uartPort - 1][k];
#else
      iTxData[i] = devEui[k];
#endif
      k++;
    }
  }
  iTxData[10] = '\0';

  strcat((char *)iTxData, payload);
}
*/
#endif

/*!
 * @brief   Permet de mettre une valeur dans le buffer circulaire
 * @param   circ_buf    circular buffer address
 * @param   elem        données à mettre dans le buffer circulaire
 * @retval  int         0 - Success
 *                     -1 - Out of space
 */
int circ_gbuf_push(circ_gbuf_t *circ_buf, void *elem)
{
    int total;
    char *head;

    total = circ_buf->push_count - circ_buf->pop_count;
    if (total < 0)
        total += (2 * circ_buf->size);

    if (total >=  circ_buf->size)
        return -1; // Full

    head = circ_buf->buffer + ( (circ_buf->push_count % circ_buf->size)
                                * circ_buf->element_size );
    memcpy(head, elem, circ_buf->element_size);
    circ_buf->push_count++;
    if (circ_buf->push_count >= (2*circ_buf->size))
        circ_buf->push_count = 0;
    return 0;
}

/*!
 * @brief   Permet de récupérer la valeur à lire dans le buffer circulaire
 * @param   circ_buf    circular buffer address
 * @param   elem        variable pour acceuillir la valeur sortie
 * @retval  int         0 - Success
 *                     -1 - Empty
 */
int circ_gbuf_pop(circ_gbuf_t *circ_buf, void *elem, int read_only)
{
    int total;
    char *tail;

    total = circ_buf->push_count - circ_buf->pop_count;
    if (total < 0)
        total += (2 * circ_buf->size);

    if (total == 0)
        return -1; // Empty

    tail = circ_buf->buffer + ((circ_buf->pop_count % circ_buf->size)
                                            * circ_buf->element_size);

    if (elem)
        memcpy(elem, tail, circ_buf->element_size);

    if (!read_only) {
#ifdef CRICBUF_CLEAN_ON_POP
        memset(tail, 0, circ_buf->element_size);
#endif
        circ_buf->pop_count++;
        if (circ_buf->pop_count >= (2*circ_buf->size))
            circ_buf->pop_count = 0;
    }
    return 0;
}

/*!
 * @brief   Permet de récupérer la taille de l'espace libre dans le buffer circulaire
 * @param   circ_buf    circular buffer address
 * @retval  int         number of bytes available
 */
int circ_gbuf_free_space(circ_gbuf_t *circ_buf)
{
    int total;

    total = circ_buf->push_count - circ_buf->pop_count;
    if (total < 0)
        total += (2 * circ_buf->size);

    return circ_buf->size - total;
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
