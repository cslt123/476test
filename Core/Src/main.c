/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2024 STMicroelectronics.
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
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
const char stringMode1[8]="mode1#";//���ڽ��պ�Ա�ָ��	
const char stringMode2[8]="mode2#";//���ڽ��պ�Ա�ָ��	
const char stringStop[8]="stop#";//���ڽ��պ�Ա�ָ��	
int8_t ledMode = -1;//ģʽֵ
//����2����	
#define Uart2RxBuffSIZE  256     //�������ֽ���	
char Uart2RxBuffer[Uart2RxBuffSIZE]= {0};   //������������	
uint8_t aUart2RxBuff;			//�����жϻ���	
uint8_t Uart2RxCnt = 0;//�������ݼ���
//����3����	
#define Uart3RxBuffSIZE  256     //�������ֽ���	
char Uart3RxBuffer[Uart3RxBuffSIZE]= {0};   //������������	
uint8_t aUart3RxBuff;			//�����жϻ���	
uint8_t Uart3RxCnt = 0;//�������ݼ���

uint8_t adc_mq4;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  uint8_t TxData[20]= "hello world!";//��ʼ��
  char stringAuto[20]="Auto\r\n";//���ڽ��պ�Ա�ָ��	
  char stringHand[20]="Hand\r\n";//���ڽ��պ�Ա�ָ��	
  char stringRead[20]="Read\r\n";//���ڽ��պ�Ա�ָ��
	HAL_ADCEx_Calibration_Start(&hadc1,ADC_SINGLE_ENDED);
	
  /***�Ƚ�printf��HAL_UART_Transmit�÷����������ͬ***/	
  printf("Hello STM32L476RG!\r\n");	
  HAL_UART_Transmit(&huart2,TxData,sizeof(TxData),100);	
  HAL_UART_Transmit(&huart2,(uint8_t *)"\r\n",2,100);
	//HAL_UART_Transmit(&huart3,(uint8_t *)stringHand,sizeof(stringHand),100);
  HAL_UART_Receive_IT(&huart2, (uint8_t *)&aUart2RxBuff, 1);   //���������ж�  
  HAL_UART_Receive_IT(&huart3, (uint8_t *)&aUart3RxBuff, 1);   //���������ж�
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
			if(!HAL_GPIO_ReadPin(B1_GPIO_Port,B1_Pin))
			{
				 HAL_Delay(20);
				 if(!HAL_GPIO_ReadPin(B1_GPIO_Port,B1_Pin)){
						 while(!HAL_GPIO_ReadPin(B1_GPIO_Port,B1_Pin));
						 HAL_GPIO_TogglePin(LD2_GPIO_Port,LD2_Pin);
						 HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_0);
						 HAL_UART_Transmit(&huart3,(uint8_t *)stringRead,sizeof(stringRead),100);
						 adc_mq4 = ADC_IN_1();
						 printf("CH4:%d",adc_mq4);
				 }			
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){  
	UNUSED(huart);
  if(huart == &huart2){
    if(Uart2RxCnt >= 255){		
		 Uart2RxCnt = 0;		
		 memset(Uart2RxBuffer,0x00,sizeof(Uart2RxBuffer));		
		 HAL_UART_Transmit(&huart2, (uint8_t *)"�������", 10,0xFFFF); 		
	  }	else{		
		 Uart2RxBuffer[Uart2RxCnt++] = aUart2RxBuff;   //��������ת��			
		 if((Uart2RxBuffer[Uart2RxCnt-1] == 0x0A)
      &&(Uart2RxBuffer[Uart2RxCnt-2] == 0x0D)
      &&(Uart2RxBuffer[Uart2RxCnt-3] == '#')) //�жϽ���λ			
		  {	
			 if(strstr((const char *)Uart2RxBuffer,stringMode1)!=NULL){				
				 printf("I'm in mode_1!\r\n");				
				 ledMode = 1;				
				 HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin,GPIO_PIN_SET);			
			  }			
			  else if(strstr((const char *)Uart2RxBuffer,stringMode2)!=NULL)	{				
				 printf("I'm in mode_2!\r\n");				
				 ledMode = 2;				
				 HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin,GPIO_PIN_RESET);			
			  }			
			  else if(strstr((const char *)Uart2RxBuffer,stringStop)!=NULL)	{				
				 printf("I'm stop!\r\n");				
				 ledMode = 0;			
			  }			
			  HAL_UART_Transmit(&huart2, (uint8_t *)&Uart2RxBuffer, Uart2RxCnt,0xFFFF); //���յ�����Ϣ���ͳ�ȥ			
			  printf("hello\r\n");            
			  while(HAL_UART_GetState(&huart2) == HAL_UART_STATE_BUSY_TX);//���UART���ͽ���			
			  Uart2RxCnt = 0;			
			  memset(Uart2RxBuffer,0x00,sizeof(Uart2RxBuffer)); //�������		
		  }	
	  }	
    HAL_UART_Receive_IT(&huart2, (uint8_t *)&aUart2RxBuff, 1);   //�ٿ��������ж�}
  }else if(huart == &huart3){
    if(Uart3RxCnt >= 255){		
		 Uart3RxCnt = 0;		
		 memset(Uart3RxBuffer,0x00,sizeof(Uart3RxBuffer));		
		 HAL_UART_Transmit(&huart3, (uint8_t *)"�������", 10,0xFFFF); 		
	  }	else{		
		 Uart3RxBuffer[Uart3RxCnt++] = aUart3RxBuff;   //��������ת��			
		 if((Uart3RxBuffer[Uart3RxCnt-1] == 0x0A)
      &&(Uart3RxBuffer[Uart3RxCnt-2] == 0x0D)){
        HAL_UART_Transmit(&huart2, (uint8_t *)&Uart3RxBuffer, Uart3RxCnt,0xFFFF); //���յ�����Ϣ���ͳ�ȥ     
			  while(HAL_UART_GetState(&huart2) == HAL_UART_STATE_BUSY_TX);//���UART���ͽ���			
			  Uart3RxCnt = 0;			
			  memset(Uart3RxBuffer,0x00,sizeof(Uart3RxBuffer)); //�������		
		  }	
	  }	
    HAL_UART_Receive_IT(&huart3, (uint8_t *)&aUart3RxBuff, 1);   //�ٿ��������ж�}
      }
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/