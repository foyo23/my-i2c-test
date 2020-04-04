/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "adc.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "stdarg.h"
#include "string.h"
#include <stdlib.h>

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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
char TxBuff[512];
void USART1_printf( char *fmt, ... )
{
	unsigned int i = 0;
  int tempvalue;
  va_list arg_ptr;
  va_start( arg_ptr, fmt );
  vsnprintf( TxBuff, 512, fmt, arg_ptr );
  tempvalue = strlen( TxBuff );
  while( i< tempvalue ) {
    LL_USART_TransmitData8( USART1, TxBuff[i++] );
    while( !( USART1->ISR & LL_USART_ISR_TXE ) );
  }
  va_end( arg_ptr );
}

void delay_ms( short time ) 
{
	short i = 0;
	while( time-- )
	{
		i = 5000;
		while( i-- );
	}
}

//uint8_t I2C_ReadChar( I2C_TypeDef *I2Cx, uint8_t Addr, uint8_t Reg, uint8_t *Value, uint8_t size )
//{
//   LL_I2C_HandleTransfer(I2Cx, (Addr<<1), LL_I2C_ADDRSLAVE_7BIT, 1, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);
//   while( !LL_I2C_IsActiveFlag_STOP(I2Cx) ) // wait for stop flag
//   {
//      /* Indicate the status of Transmit data register empty flag */
//      if( LL_I2C_IsActiveFlag_TXE(I2Cx) )
//      {
//         /* Write data in Transmit Data register. */
//         LL_I2C_TransmitData8(I2Cx, Reg);
//      }
//   }
//   LL_I2C_ClearFlag_STOP(I2Cx);

//	 uint8_t u8_rxIndex = 0;
//   LL_I2C_HandleTransfer( I2Cx, (Addr<<1), LL_I2C_ADDRSLAVE_7BIT, size, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_READ );
//   while( !LL_I2C_IsActiveFlag_STOP(I2Cx) ) // wait for stop flag
//   {
//      /*Check RXNE flag value in ISR register*/
//      if( LL_I2C_IsActiveFlag_RXNE(I2Cx) )
//      {
//        /*Read character in Receive Data register.*/
//        //*Value = LL_I2C_ReceiveData8( I2Cx );
//				Value[u8_rxIndex] = LL_I2C_ReceiveData8( I2Cx );
//      }
//   }

//   LL_I2C_ClearFlag_STOP(I2Cx);
//   /* all is good, return 0 */
//   return 0;
//}

#define ADDR_24LCxx_Write 0xA0
#define ADDR_24LCxx_Read 0xA1
#define BufferSize 0x100
uint8_t WriteBuffer[BufferSize],ReadBuffer[BufferSize];
uint16_t i;

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
  MX_ADC_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_TIM14_Init();
  MX_TIM16_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  USART1_printf("\r\n***************I2C Example*******************************\r\n");
  for (i = 0; i < 256; i++)
    WriteBuffer[i] = i;//255-i; /* WriteBuffer init */

  // /* wrinte date to EEPROM */
  // for (i = 0; i < 32; i++)
  // {
  //   if (HAL_I2C_Mem_Write(&hi2c1, ADDR_24LCxx_Write, i*8,
  //                         I2C_MEMADD_SIZE_8BIT, WriteBuffer+i*8, 8, 0xff) == HAL_OK)

  //   {
  //     USART1_printf("\r\n EEPROM 24C02 %d. Write Test OK \r\n", i + 1);
  //     HAL_Delay(5);
  //   }
  //   else
  //   {
  //     HAL_Delay(5);
  //     USART1_printf("\r\n EEPROM 24C02 %d. Write Test False \r\n", i + 1);
  //   }
  // }
  
  /* read date from EEPROM */
  HAL_I2C_Mem_Read(&hi2c1, ADDR_24LCxx_Read, 0, I2C_MEMADD_SIZE_8BIT, ReadBuffer, BufferSize, 0x20);
  for (i = 0; i < 256; i++)
    USART1_printf("0x%02X  ", ReadBuffer[i]);

  if (memcmp(WriteBuffer, ReadBuffer, BufferSize) == 0) /* check data */
    USART1_printf("\r\n EEPROM 24C02 Read Test OK\r\n");
  else
    USART1_printf("\r\n EEPROM 24C02 Read Test False\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    //USART1_printf( "...\r\n" );
    delay_ms( 500 );
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);

  if(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_1)
  {
  Error_Handler();  
  }
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {
    
  }
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_HSI14_Enable();

   /* Wait till HSI14 is ready */
  while(LL_RCC_HSI14_IsReady() != 1)
  {
    
  }
  LL_RCC_HSI14_SetCalibTrimming(16);
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI_DIV_2, LL_RCC_PLL_MUL_12);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {
    
  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
  
  }
  LL_SetSystemCoreClock(48000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();  
  };
  LL_RCC_HSI14_EnableADCControl();
  LL_RCC_SetUSARTClockSource(LL_RCC_USART1_CLKSOURCE_PCLK1);
  LL_RCC_SetI2CClockSource(LL_RCC_I2C1_CLKSOURCE_HSI);
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
