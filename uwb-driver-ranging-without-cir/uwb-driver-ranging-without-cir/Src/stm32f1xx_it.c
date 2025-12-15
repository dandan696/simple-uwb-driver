/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_it.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "port.h"
#include "instance.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
/* USER CODE BEGIN EV */
extern uint8_t UART_RX_BUF[200];
uint8_t USART_RX_BUF[200];
extern __IO uint32_t uwTick;
extern  uint32_t range_time;                                    //测距产生时间，串口打包发送
uint32_t global_time = 0;
uint32_t global_ms = 0;
uint32_t pps_time = 0;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel6 global interrupt.
  */
void DMA1_Channel6_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel6_IRQn 0 */

  /* USER CODE END DMA1_Channel6_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_rx);
  /* USER CODE BEGIN DMA1_Channel6_IRQn 1 */

  /* USER CODE END DMA1_Channel6_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel7 global interrupt.
  */
void DMA1_Channel7_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel7_IRQn 0 */

  /* USER CODE END DMA1_Channel7_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_tx);
  /* USER CODE BEGIN DMA1_Channel7_IRQn 1 */

  /* USER CODE END DMA1_Channel7_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[9:5] interrupts.
  */
void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */

  /* USER CODE END EXTI9_5_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_5);
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */

  /* USER CODE END EXTI9_5_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
  /* USER CODE END USART1_IRQn 0 */	

  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */
		
 /* USER CODE END USART1_IRQn 1 */
}
#include "instance.h"
uint64_t GetSysRunTimeUs(void)
{
		return uwTick * 1000U + (SysTick->LOAD - SysTick->VAL) * 1000U / SysTick->LOAD;
}
uint64_t GetGlobal_Us(void)
{
		return (uint64_t)global_time * 1000U +(uint64_t)(uwTick - global_ms)*1000U + (SysTick->LOAD - SysTick->VAL) * 1000U / SysTick->LOAD;
}
void fresh_time_s(uint8_t receivedData)
{
  uint8_t res;
  res = receivedData;
  static char my_char[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  static uint8_t j = 0, k = 0;
  static uint8_t myGlobalutc[15] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  {
    if (res == 'G')
      my_char[0] = 'G';
    if (res == 'P' && my_char[0] == 'G')
      my_char[1] = 'P';
    if (res == 'C' && my_char[1] == 'P')
      my_char[4] = 'C';
  }
  if (my_char[4] == 'C')
  {
    if (res == ',')
      k++;
    switch (k)
    {
      case 1:
        if (res != ',' && res != '.')
        {
          myGlobalutc[j] = res - 48;
          j++;
        }
        break;
      case 9:
        if (res != ',' && res != '.')
        {
          myGlobalutc[j] = res - 48;
          j++;
        }
        break;
    }
  }
  if (my_char[4] == 'C' && k == 2)
  {
    for (int p = 0; p < 10; p++)
    {
      my_char[p] = 0;
    }
    k = 0;
    j = 0;
    uint32_t uu1 = 0, uu2 = 0;
    if (!(myGlobalutc[2] == 0 && myGlobalutc[3] == 0 && myGlobalutc[4] == 0 && myGlobalutc[5] == 0))
    {
      if (myGlobalutc[0] < 10 && myGlobalutc[1] < 10 && myGlobalutc[2] < 10 && myGlobalutc[3] < 10 && myGlobalutc[4] < 10 && myGlobalutc[5] < 10)
      {
        {
          uu2 = myGlobalutc[0] * 36000 + myGlobalutc[1] * 3600 + myGlobalutc[2] * 600 + myGlobalutc[3] * 60 + myGlobalutc[4] * 10 + myGlobalutc[5];
          uu2 *= 1000;
          uu1 = uwTick - pps_time;
          global_ms = uwTick;
          uu1 += uu2;
          global_time = uu1;
        }

        /*
      printf("%d.", myGlobalutc[0]);
      printf("%d.", myGlobalutc[1]);
      printf("%d.", myGlobalutc[2]);
      printf("%d.", myGlobalutc[3]);
      printf("%d.", myGlobalutc[4]);
      printf("%d", myGlobalutc[5]);
      printf("___%d", global_time);
      printf("___%d", uwTick-global_ms);
      printf("___%lld", GetGlobal_Us());
      printf("\n");
      */
      }
    }
    for (int p = 0; p < 10; p++)
    {
      myGlobalutc[p] = 0;
    }
  }
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart==&huart1)
	{
		HAL_UART_Receive_IT(&huart1,&USART_RX_BUF[0],1);
		fresh_time_s(USART_RX_BUF[0]);
	}
}


void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */

  __HAL_UART_CLEAR_IDLEFLAG(&huart2);                   //清除中断标记
  HAL_UART_DMAStop(&huart2);                            //停止DMA接收

  
  HAL_UART_IdleCpltCallback(&huart2);
  HAL_UART_Receive_DMA(&huart2, &UART_RX_BUF[0], sizeof(UART_RX_BUF));
  
  

  /* USER CODE END USART2_IRQn 1 */
}


void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */

  /* USER CODE END EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_11);
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */

  /* USER CODE END EXTI15_10_IRQn 1 */
}

/* USER CODE BEGIN 1 */
void EXTI0_IRQHandler(void)
{

  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);

}
/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
