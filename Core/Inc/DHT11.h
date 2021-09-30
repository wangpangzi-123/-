#ifndef __DHT11_H__
#define __DHT11_H__
 
#include "main.h"
#include "gpio.h"
 
/************************ DHT11 ??????******************************/
typedef struct
{
	uint8_t  humi_high8bit;                //????:???8?
	uint8_t  humi_low8bit;                 //????:???8?
	uint8_t  temp_high8bit;                //????:???8?
	uint8_t  temp_low8bit;                 //????:???8?
	uint8_t  check_sum;                     //???
  float    humidity;        //????
  float    temperature;     //????  
} DHT11_Data_TypeDef;
 
#define DTH11_Clr() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET)
#define DTH11_Set() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET)	 
#define DTH11_IN	HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15)
 
void dth11_input(void);
void dth11_output(void);
void dth11_reset(void);
unsigned short dht11_read_bit(void);
unsigned short dht11_read_byte(void);
uint8_t dht11_read_data(DHT11_Data_TypeDef * DTH_Data);
 
#endif