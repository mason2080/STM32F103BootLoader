
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
  * COPYRIGHT(c) 2018 STMicroelectronics
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
#include "stm32f3xx_hal.h"
#include "BootLoader.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
int  cnt_Timer2=0;

//int value __attribute__((section(".ARM.__at_0x08003000")))=0xAAAAAA;
	
uint8_t aTxBuffer[10];
uint8_t aRxBuffer[1];
uint8_t aRxTempBuffer[26];
_Bool usartRecvFlag=RESET;
_Bool firstJumpToFlag=RESET;
_Bool recvUpdateCmd=RESET;

uint32_t AppAddressBase=0;
uint16_t RecvCmdLine=0;
uint16_t Status=0;

uint8_t tempDebug=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */


#define ApplicationAddress 0x08004000 //BootLoader 16K
typedef void (*pFunction)(void);

pFunction Jump_To_Application;
uint32_t JumpAddress;

void Jump()
{
		if (((*(__IO uint32_t*)ApplicationAddress) & 0x2FFE0000 ) == 0x20000000)
    { 
      JumpAddress = *(__IO uint32_t*) (ApplicationAddress + 4);
      Jump_To_Application = (pFunction) JumpAddress;
      __set_MSP(*(__IO uint32_t*) ApplicationAddress);
      Jump_To_Application();
    }
}

uint8_t Flash_CRC8(uint8_t data[],uint32_t startIndex,uint32_t len)
{
	uint16_t crc=0;
	for(uint8_t i=0;i<len;i++)
	{
		crc+=data[startIndex+i];
	}
	return (uint8_t)crc;
}
void CheckBootStatus(uint16_t Status)
{
	if(Status==0x5555)
	{
				aTxBuffer[0]=0x5A ;
				aTxBuffer[1]=0xA5 ;
				aTxBuffer[2]=0x00 ;
				aTxBuffer[3]=0xAA ;
				aTxBuffer[4]=0x55 ;
				aTxBuffer[5]=Flash_CRC8(aTxBuffer,0,5) ;
			  
				
				for(uint8_t i=0;i<4;i++)
				{
			  	aRxTempBuffer[i]=0;
				}
				HAL_UART_Transmit(&huart1,aTxBuffer,6,100);		
  }
}
void update()
{
	uint8_t tempCrc=0;
	if(usartRecvFlag==SET)
	{
	  usartRecvFlag=RESET;
		
		//enter bootloader
		if((aRxTempBuffer[0]==0x5A)&&(aRxTempBuffer[1]==0xA5)&&(aRxTempBuffer[2]==0x33)&&(aRxTempBuffer[3]==0x44))
		{
			if(Flash_CRC8(aRxTempBuffer,0,4)==aRxTempBuffer[4])
			{
				RecvCmdLine=0;
				recvUpdateCmd=SET;
			  aTxBuffer[0]=0x5A ;
				aTxBuffer[1]=0xA5 ;
				aTxBuffer[2]=0x00 ;
				aTxBuffer[3]=0xAA ;
				aTxBuffer[4]=0x55 ;
				aTxBuffer[5]=Flash_CRC8(aTxBuffer,0,5) ;
			  
				
				for(uint8_t i=0;i<4;i++)
				{
			  	aRxTempBuffer[i]=0;
				}
				HAL_UART_Transmit(&huart1,aTxBuffer,6,100);
				return;
			}
			
		}
		
	  //Start to program
		if((aRxTempBuffer[0]==0x5A)&&(aRxTempBuffer[1]==0xA5)&&(aRxTempBuffer[2]==0xFE)&&(aRxTempBuffer[3]==0xEF))
		{
    	if(Flash_CRC8(aRxTempBuffer,0,4)==aRxTempBuffer[4])
			{
				
				RecvCmdLine=0;
				recvUpdateCmd=SET;
		  	Flash_Erase_AppFlash();//Erase APP Flash
			  aTxBuffer[0]=0x5A ;
				aTxBuffer[1]=0xA5 ;
				aTxBuffer[2]=0x01 ;
				aTxBuffer[3]=0xAA ;
				aTxBuffer[4]=0x55 ;
				aTxBuffer[5]=Flash_CRC8(aTxBuffer,0,5) ;
				for(uint8_t i=0;i<4;i++)
				{
			  	aRxTempBuffer[i]=0;
				}
				HAL_UART_Transmit(&huart1,aTxBuffer,6,100);
				return;
			}

		}
		
		
		//jump to app
	  if((aRxTempBuffer[0]==0x5A)&&(aRxTempBuffer[1]==0xA5)&&(aRxTempBuffer[2]==0xFA)&&(aRxTempBuffer[3]==0xAF))
		{
			if(Flash_CRC8(aRxTempBuffer,0,4)==aRxTempBuffer[4])
			{
				RecvCmdLine=0;
			  aTxBuffer[0]=0x5A ;
				aTxBuffer[1]=0xA5 ;
				aTxBuffer[2]=0x05 ;
				aTxBuffer[3]=0xAA ;
				aTxBuffer[4]=0x55 ;
				aTxBuffer[5]=Flash_CRC8(aTxBuffer,0,5) ;
			  
				
				for(uint8_t i=0;i<4;i++)
				{
			  	aRxTempBuffer[i]=0;
				}
				HAL_UART_Transmit(&huart1,aTxBuffer,6,100);
				
				Flash_Program_WriteFlag(0xAAAA);
				Jump();
				return;
			}
		}
		
		//program line by line
		if((aRxTempBuffer[0]==0x5A)&&(aRxTempBuffer[1]==0xA5)&&(((aRxTempBuffer[2]<<8)+aRxTempBuffer[3])==RecvCmdLine))//First Line
		{

		  tempCrc=Flash_CRC8(aRxTempBuffer,4,aRxTempBuffer[4]+4);
			
			if(tempCrc!=0)
			{
				tempCrc=0x100-tempCrc;
			}
			
			if(tempCrc==aRxTempBuffer[aRxTempBuffer[4]+8])
			{
				if((RecvCmdLine==0)&&aRxTempBuffer[7]==04)//FirstLine
				{
					AppAddressBase=(aRxTempBuffer[8]<<24)+(aRxTempBuffer[9]<<16);
					aTxBuffer[0]=0x5A ;
					aTxBuffer[1]=0xA5 ;
					aTxBuffer[2]=0x02 ;
					aTxBuffer[3]=aRxTempBuffer[2] ;
					aTxBuffer[4]=aRxTempBuffer[3] ;
					aTxBuffer[5]=Flash_CRC8(aTxBuffer,0,5) ;
					for(uint8_t i=0;i<aRxTempBuffer[5]+8;i++)
					{
						aRxTempBuffer[i]=0;
					}
					RecvCmdLine++;	
					HAL_UART_Transmit(&huart1,aTxBuffer,6,100);
					return;
				}
				if(aRxTempBuffer[7]==00)//Data Line
				{
					if(NORMAL==	Flash_Program_Oneline((AppAddressBase+(aRxTempBuffer[5]<<8)+aRxTempBuffer[6]),aRxTempBuffer,8,aRxTempBuffer[4]))
					{
						aTxBuffer[0]=0x5A ;
						aTxBuffer[1]=0xA5 ;
						aTxBuffer[2]=0x02 ;
						aTxBuffer[3]=aRxTempBuffer[2] ;
						aTxBuffer[4]=aRxTempBuffer[3] ;
					}
					else
          {
						aTxBuffer[0]=0x5A ;
						aTxBuffer[1]=0xA5 ;
						aTxBuffer[2]=0x04 ;
						aTxBuffer[3]=0x00 ;
						aTxBuffer[4]=ADDRESS_ERROR ;
					}
					aTxBuffer[5]=Flash_CRC8(aTxBuffer,0,5) ;

					for(uint8_t i=0;i<aRxTempBuffer[5]+8;i++)
					{
						aRxTempBuffer[i]=0;
					}
					RecvCmdLine++;	
					HAL_UART_Transmit(&huart1,aTxBuffer,6,100);
					return;
				}	
				if(aRxTempBuffer[7]==05)//
				{
					//Flash_Program_Oneline((AppAddressBase+(aRxTempBuffer[5]<<8)+aRxTempBuffer[6]),aRxTempBuffer,8,aRxTempBuffer[4]);
					aTxBuffer[0]=0x5A ;
					aTxBuffer[1]=0xA5 ;
					aTxBuffer[2]=0x02 ;
					aTxBuffer[3]=aRxTempBuffer[2] ;
					aTxBuffer[4]=aRxTempBuffer[3] ;
					aTxBuffer[5]=Flash_CRC8(aTxBuffer,0,5) ;

					for(uint8_t i=0;i<aRxTempBuffer[5]+8;i++)
					{
						aRxTempBuffer[i]=0;
					}
					RecvCmdLine++;	
					HAL_UART_Transmit(&huart1,aTxBuffer,6,100);
					return;
				}	
				if(aRxTempBuffer[7]==01)//the end of the hex
				{
					//Flash_Program_Oneline((AppAddressBase+(aRxTempBuffer[5]<<8)+aRxTempBuffer[6]),aRxTempBuffer,8,aRxTempBuffer[4]);
					aTxBuffer[0]=0x5A ;
					aTxBuffer[1]=0xA5 ;
					aTxBuffer[2]=0x03 ;
					aTxBuffer[3]=aRxTempBuffer[2] ;
					aTxBuffer[4]=aRxTempBuffer[3] ;
					aTxBuffer[5]=Flash_CRC8(aTxBuffer,0,5) ;

					for(uint8_t i=0;i<aRxTempBuffer[5]+8;i++)
					{
						aRxTempBuffer[i]=0;
					}
					RecvCmdLine++;	
					HAL_UART_Transmit(&huart1,aTxBuffer,6,100);
					return;
				}	
			}
		}
	}
}



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim2);
  //HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_SET);
	HAL_UART_Receive_IT(&huart1,aRxBuffer,1);
	
	//HAL_UART_Transmit(&huart1,aTxBuffer,sizeof(aTxBuffer),100);
	Status=Flash_Program_ReadFlag();
	
	CheckBootStatus(Status);

	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		
    update();
		
		
		if(cnt_Timer2>=2)
		{
			  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_SET);
			  if((Status==0xAAAA)&&(recvUpdateCmd!=SET))
				{
					Jump();
				}
		}
		
		
		if(cnt_Timer2>=2)
		{
			  //HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_RESET);
		  	//cnt_Timer2=0;
			
//			  uint8_t aTxBuffer[] = "Jump to APP...";
		  	//HAL_UART_Transmit(&huart1,aTxBuffer,sizeof(aTxBuffer),100);
		  	//Jump();
		}
		
		
	 /* if(usartRecvFlag==SET)
		{
			uint8_t aTxBuffer[] = "Erasing please wait...";
			HAL_UART_Transmit(&huart1,aTxBuffer,sizeof(aTxBuffer),100);
			usartRecvFlag=RESET;
			
			Flash_Erase_AppFlash();
			uint8_t aTxBuffer1[] = "Erase finished...";
			HAL_UART_Transmit(&huart1,aTxBuffer1,sizeof(aTxBuffer1),100);
		}
      

		if(cnt_Timer2==5)
		{
			cnt_Timer2=6;
			if(firstJumpToFlag==RESET)
			{
				uint8_t aTxBuffer1[] = "JumpToApp...";
				HAL_UART_Transmit(&huart1,aTxBuffer1,sizeof(aTxBuffer1),100);
				
				Jump();
				firstJumpToFlag=SET;
			}
			{
				uint8_t aTxBuffer2[] = "NoValidApp...";
				HAL_UART_Transmit(&huart1,aTxBuffer2,sizeof(aTxBuffer2),100);
			}
		}
		*/
		

  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
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

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 800-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(cnt_Timer2<10000)
	{
    cnt_Timer2++;
	}
	else
	{
		cnt_Timer2=0;
	}
}




void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance==USART1)
	{
		aRxTempBuffer[25]=aRxBuffer[0];
		
		for(uint8_t i=0;i<25;i++)
		{
			aRxTempBuffer[i]=aRxTempBuffer[i+1];
		}
	
    HAL_UART_Receive_IT(&huart1,aRxBuffer,1);

		if((aRxTempBuffer[0]==0x5A)&&(aRxTempBuffer[1]==0xA5))
		{
			usartRecvFlag=SET;
		}
	}
}






/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
