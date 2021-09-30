#include <DHT11.h>
#include "tim.h"
 
// ?????
void dth11_input(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	__HAL_RCC_GPIOA_CLK_ENABLE();
	
	/*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}
 
// ?????
void dth11_output(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	__HAL_RCC_GPIOA_CLK_ENABLE();
	
	/*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}
 
// ??
void dth11_reset(void)
{
	dth11_output();
	DTH11_Clr();
	delay_us(20000);
	DTH11_Set();
	delay_us(20);
	dth11_input();
}
 
// ????bit??
unsigned short dht11_read_bit(void)
{
	while(DTH11_IN == GPIO_PIN_RESET);
	delay_us(40);
	if(DTH11_IN == GPIO_PIN_SET)
	{
		while(DTH11_IN == GPIO_PIN_SET);
		return 1;
	}else {
		return 0;
	}
}
 
// ?????????
unsigned short dht11_read_byte(void)
{
	unsigned char i;
	unsigned short data = 0;
	for(i=0;i<8;i++)
	{
		data <<= 1;
		data |= dht11_read_bit();
	}
	return data;
}
 
// ????
uint8_t dht11_read_data(DHT11_Data_TypeDef* DTH_Data)
{
	uint8_t checksum;
	uint16_t data_temp;
	
	dth11_reset(); // ??
	if(DTH11_IN == GPIO_PIN_RESET)
	{
		// ???DTH11??
		while(DTH11_IN == GPIO_PIN_RESET);
		while(DTH11_IN == GPIO_PIN_SET);
		// ????????,??????
		DTH_Data->humi_high8bit = dht11_read_byte();
		DTH_Data->humi_low8bit = dht11_read_byte();
		DTH_Data->temp_high8bit = dht11_read_byte();
		DTH_Data->temp_low8bit = dht11_read_byte();
		DTH_Data->check_sum = dht11_read_byte();
		while(DTH11_IN == GPIO_PIN_RESET);
		// ????,?????
		dth11_output();
		DTH11_Set();
		/* ??????? */
    data_temp=DTH_Data->humi_high8bit*100+DTH_Data->humi_low8bit;
    DTH_Data->humidity =(float)data_temp/100;
    
    data_temp=DTH_Data->temp_high8bit*100+DTH_Data->temp_low8bit;
    DTH_Data->temperature=(float)data_temp/100;   
		
		// ????
		checksum = DTH_Data->humi_high8bit + DTH_Data->humi_low8bit + DTH_Data->temp_high8bit + DTH_Data->temp_low8bit;
		if(checksum != DTH_Data->check_sum){
			return ERROR;
		}else{
			return SUCCESS;
		}
	}else {
		return ERROR;
	}
}
