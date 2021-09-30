/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "adc.h"
#include "rtc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//main.h

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <DHT11.h>
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
DHT11_Data_TypeDef dht11_data;
static float temp_static = 0.0;
static float humi_static = 0.0;
static int fengsu_static = 0;
static int fengxiang_static = 0;
static 
void Displey_Temp()
{
	int  i = 0;
	float temp = 0.00;
	float temp_sum = 0.00;
	float humi = 0.00;
	float humi_sum = 0.00;
	uint8_t res = dht11_read_data(&dht11_data);
	if(res == SUCCESS)
	{
		for(i= 0;i<20;i++){
			temp = dht11_data.temperature;
			humi = dht11_data.humidity;
			temp_sum = temp_sum + temp;
			humi_sum = humi_sum + humi;
			HAL_Delay(100);
			
		}
temp_static  = temp_sum/20;
humi_static  = humi_sum/20;
	}else{
//        printf("read ERROR ! \r\n");
	}
}
void WKUP_EnterStandby(void)
{
	__HAL_RCC_APB2_FORCE_RESET();
	__HAL_RCC_PWR_CLK_ENABLE();

	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
	HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN2);
	HAL_PWR_EnterSTANDBYMode();
}
void ADC_collect_2s(void){
	int i = 0;
	int j = 0;
	int adcBuf[2] = {0};
	int fengsu = 0;
	int fengxiang = 0;
	long fengsu_sum = 0;
	long fengxiang_sum = 0;
	for(j=0;j<100;j++){
for(i=0;i<2;i++)
{
HAL_ADC_Start(&hadc1);
HAL_ADC_PollForConversion(&hadc1,0xffff);
adcBuf[i]=HAL_ADC_GetValue(&hadc1);
}
fengsu = adcBuf[0];
fengxiang = adcBuf[1];
HAL_ADC_Stop(&hadc1);
fengsu_sum = fengsu+fengsu_sum;
fengxiang_sum = fengxiang+fengxiang_sum;
HAL_Delay(600);
}
fengsu_static = fengsu_sum/100;
fengxiang_static = fengxiang_sum/100;
}

void lora_send(void){
uint16_t addr;
uint8_t chn;
uint8_t data[30] = {0};
uint32_t obj_addr;
uint16_t obj_chn;
obj_chn = 0;
obj_addr = 0;
chn = obj_chn;
addr = (uint16_t)obj_addr;
int i = 0;
data[i++] = (addr>>8) & 0xff;
data[i++] = addr & 0xff;
data[i] = chn;
for(i = 0; i<3;i++){
	printf("%c",data[i]);
	HAL_Delay(1);
}
float Sum = 0.0;
float Sum1 = 0.0;
Sum  = (float)fengxiang_static*3.3/4096;
Sum1 = (float)fengsu_static*3.3/4096*30;
            if(Sum>=0.0625&&Sum<=0.1875){
printf("AABBmod1%4.1f %4.1f %5.2f 00CCDD",temp_static,humi_static,Sum1);
                              }
                              else if(Sum>=0.1875&&Sum<=0.3125){
printf("AABBmod1%4.1f %4.1f %5.2f 01CCDD",temp_static,humi_static,Sum1);
                              }
                              else if(Sum>=0.3125&&Sum<=0.4375){
printf("AABBmod1%4.1f %4.1f %5.2f 02CCDD",temp_static,humi_static,Sum1);
                                     }
                              else if(Sum>=0.4375&&Sum<=0.5625){
printf("AABBmod1%4.1f %4.1f %5.2f 03CCDD",temp_static,humi_static,Sum1);
                                     }
                              else if(Sum>=0.5625&&Sum<=0.6875){
printf("AABBmod1%4.1f %4.1f %5.2f 04CCDD",temp_static,humi_static,Sum1);
                                     }
                              else if(Sum>=0.6875&&Sum<=0.8125){
printf("AABBmod1%4.1f %4.1f %5.2f 05CCDD",temp_static,humi_static,Sum1);
                                     }
                              else if(Sum>=0.8125&&Sum<=0.9375){
printf("AABBmod1%4.1f %4.1f %5.2f 06CCDD",temp_static,humi_static,Sum1);
                                     }
                              else if(Sum>=0.9375&&Sum<=1.0625){
printf("AABBmod1%4.1f %4.1f %5.2f 07CCDD",temp_static,humi_static,Sum1);
                                             }
                              else if(Sum>=1.0625&&Sum<=1.1875){
printf("AABBmod1%4.1f %4.1f %5.2f 08CCDD",temp_static,humi_static,Sum1);
                                             }
                              else if(Sum>=1.1875&&Sum<=1.3125){
printf("AABBmod1%4.1f %4.1f %5.2f 09CCDD",temp_static,humi_static,Sum1);
                                             }
                              else if(Sum>=1.3125&&Sum<=1.4375){
printf("AABBmod1%4.1f %4.1f %5.2f 10CCDD",temp_static,humi_static,Sum1);
                                             }
                              else if(Sum>=1.4375&&Sum<=1.5625){
printf("AABBmod1%4.1f %4.1f %5.2f 11CCDD",temp_static,humi_static,Sum1);
                                             }
                              else if(Sum>=1.5625&&Sum<=1.6875){
printf("AABBmod1%4.1f %4.1f %5.2f 12CCDD",temp_static,humi_static,Sum1);
                                             }
                              else if(Sum>=1.6875&&Sum<=1.8125){
printf("AABBmod1%4.1f %4.1f %5.2f 13CCDD",temp_static,humi_static,Sum1);
                                             }
                              else if(Sum>=1.8125&&Sum<=1.9375){
printf("AABBmod1%4.1f %4.1f %5.2f 14CCDD",temp_static,humi_static,Sum1);
                                             }
                              else{
printf("AABBmod1%4.1f %4.1f %5.2f 15CCDD",temp_static,humi_static,Sum1);
                                             }
}
	
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
  MX_RTC_Init();
  MX_TIM7_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		ADC_collect_2s();
		Displey_Temp();
		lora_send();
		WKUP_EnterStandby();
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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USART1
                              |RCC_PERIPHCLK_ADC;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 16;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
