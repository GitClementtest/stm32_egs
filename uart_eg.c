//In this example the microcontroller echos back the received bytes to the sender using UART RX interrupt.
//This example used an STM32F4 Discovery (STM32F407VG), GPIO and alternate function values should be changed according to the STM32 microcontroller in use.

#include "stm32f4xx.h"

UART_HandleTypeDef huart2;

/* Single byte to store input */
uint8_t byte;

void SystemClock_Config(void);

/* UART2 Interrupt Service Routine */
void USART2_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart2);
}

/* This callback is called by the HAL_UART_IRQHandler when the given number of bytes are received */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    /* Transmit one byte with 100 ms timeout */
    HAL_UART_Transmit(&huart2, &byte, 1, 100);

    /* Receive one byte in interrupt mode */ 
    HAL_UART_Receive_IT(&huart2, &byte, 1);
  }
}

void uart_gpio_init()
{
  GPIO_InitTypeDef GPIO_InitStruct;

  __GPIOA_CLK_ENABLE();

  /**USART2 GPIO Configuration
  PA2     ------> USART2_TX
  PA3     ------> USART2_RX
  */
  GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void uart_init()
{
  __USART2_CLK_ENABLE();

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart2);

  /* Peripheral interrupt init*/
  HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
}

int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
             
  uart_gpio_init();
  uart_init();

  HAL_UART_Receive_IT(&huart2, &byte, 1);

  while(1)
  {

  }
}
