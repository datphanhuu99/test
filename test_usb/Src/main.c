/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"
#include "usb_host.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
//#include <stdio.h>
//#include <stdlib.h>
#include <stdbool.h>	
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
extern ApplicationTypeDef Appli_state;
FATFS myUsbFatFS;
char USBH_Path[4];

FIL myFile;
FRESULT res;
UINT byteswritten, bytesread;
 char rwtext[10000];  //Read/Write buf

bool Usb_Write_float(float data_s[100], float data_duty[100], float data_count[100]);//+++
float data_c[200],data_s[200],data_duty[200],data_count[200];//+++++
char data0[10],data1[10],data2[10],data3[100],text[100];
int i=0;
int gui(float data_s[100], float data_duty[100], float data_count[100]);//+++
void noichuoi(char s1[100],char s2[100]);//+++++
//2. USB test Read function
bool UsbTest_Read(void);

int n=0;
int data_i=0;
//float data_s=3.14215;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */

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
  MX_FATFS_Init(); 
  MX_USB_HOST_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    MX_USB_HOST_Process();
		
    /* USER CODE BEGIN 3 */
		switch(Appli_state)
		{
			case APPLICATION_IDLE:
				n=10;
				break;
			
			case APPLICATION_START:
				n=2;
				if(f_mount(&myUsbFatFS, (TCHAR const*)USBH_Path, 0) == FR_OK) 
				{
					n=2;
					HAL_GPIO_WritePin(GPIOG,GPIO_PIN_13, 1);
				}
				break;
			
			case APPLICATION_READY:
		
				n=3;
				if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0) == 1)
				{
					for( i=0;i<100;i++)
					{
						data_s[i]=i;
						data_duty[i]=i;
						data_count[i]=i;
					}
//					gui(data_s,data_duty,data_count);
					Usb_Write_float(data_s, data_duty,data_count) ;
						
				}
				n=0;
				HAL_GPIO_WritePin(GPIOG,GPIO_PIN_13, 0);
		
				break;
			
			case APPLICATION_DISCONNECT:
				n=4;
				//Turn Green LED OFF
				//HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
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

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PC4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PG13 PG14 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
bool Usb_Write_float(float data_s[100], float data_duty[100], float data_count[100])
{
	
	if(f_open(&myFile, "TEST2.txt", FA_WRITE| FA_CREATE_ALWAYS) !=FR_OK)
	{
		return 0;
	}
//	for( i=0; i<1000; i++)
//	{
//		res = f_read(&myFile, (uint8_t*)&rwtext[i], 1, &bytesread);
//		if(rwtext[i] == 0x00) // NULL string
//		{
//			bytesread = i;
//			break;
//		}
//	}
	
	for(i=0;i<100;i++)
	{
		sprintf(data0,"%.3f",data_s[i]);	
		sprintf(data1,"%.3f",data_duty[i]);
		sprintf(data2,"%.3f",data_count[i]);
		
		
		snprintf(data3,100,"%s%s%s%s%s%s", data0,"\t",data1,"\t",data2,"\n");
		strcat(rwtext,data3);
		
//		noichuoi(rwtext,data);
//		strncat(rwtext,data,6);
//		strncat(rwtext,"\t",6);
//		
//		sprintf(data,"%.3f",data_count[i]);
//		strncat(rwtext,data,6);
//		strncat(rwtext,"\t",6);
//		
//		sprintf(data,"%.3f",data_duty[i]);
//		strncat(rwtext,data,6);
//		strncat(rwtext,"\n",6); 
		
		HAL_Delay(1);
	}
	
	res = f_write(&myFile,(const void *)rwtext, strlen(rwtext),&byteswritten);
		if(res!= FR_OK||(byteswritten==0))
	{
		return 0;
	}
	sprintf(rwtext, "");
	f_close(&myFile);
	return 1;
}
//int gui(float data_s[100], float data_duty[100], float data_count[100])
//	{
//		for(int i=0;i<50;i++)
//		{
//			Usb_Write_float(data_s[i],1,0);
//			HAL_Delay(1);
//			Usb_Write_float(data_duty[i],1,0);
//			HAL_Delay(1);
//			Usb_Write_float(data_count[i],0,1);
//			HAL_Delay(1);
//		}
//		return 0;
//	}
//bool Usb_Write_float(float data_s)
//{
//	char data[10];
//	//Open or Create file for writing
//	if(f_open(&myFile, "TEST2.TXT", FA_WRITE) != FR_OK)
//	{
//		return 0;
//	}
//	//read file
//	char data_receve[100];
//	for(uint8_t i=0; i<100; i++)
//	{
//		res = f_read(&myFile, (uint8_t*)&rwtext[i], 1, &bytesread);
//		if(rwtext[i] == 0x00) // NULL string
//		{
//			data_receve[i]=rwtext[i];
//			bytesread = i;
//			break;
//		}
//	}
//	HAL_Delay(1);
//	//if(bytesread==0) return 0;
//	char text[100];
//	int number_read=strlen(rwtext);
//	
//	sprintf(data,"%.3f",data_s);
//	
//	//Copy test Text to my temporary read/write buffer
//	sprintf(text, " \n Hello! data you is: 111: ");
//	//Write to text file
//	strcat(rwtext,data_receve);
//	strcat(rwtext,text);
//	strcat(rwtext,data);
//	res = f_write(&myFile, (const void *)rwtext, strlen(rwtext), &byteswritten);
//	
//	if((res != FR_OK) || (byteswritten == 0))
//	{
//		return 0;
//	}
//	f_close(&myFile);
//	return 1; //Success
//}

//2. USB test Read function
bool UsbTest_Read(void)
{
	//Open file for reading 
	if(f_open(&myFile, "TEST2.TXT", FA_READ) != FR_OK)
	{
		return 0;
	}
	
	//Read text from files until NULL
	for(uint8_t i=0; i<100; i++)
	{
		res = f_read(&myFile, (uint8_t*)&rwtext[i], 1, &bytesread);
		if(rwtext[i] == 0x00) // NULL string
		{
			
			bytesread = i;
			break;
		}
	}
	//Reading error handling
	if(bytesread==0) return 0;
	
	
	//Close file
	f_close(&myFile);
	return 1;  // success
	
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
