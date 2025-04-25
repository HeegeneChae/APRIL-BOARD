#include "adc.h"


#define OFF 1000
#define ON 0

void alertLED(uint16_t number) 
{
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	if(number == 0) {
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, ON); 
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, OFF);    
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, OFF); 
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, OFF); 
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, OFF);    
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, OFF); 
				HAL_Delay(150);
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, ON); 
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, OFF);    
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, OFF); 
	}
	
	
		else if (number <= 20) {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, ON); 
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, OFF);    
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, OFF); 
    }
    else if (number <= 40) {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 300);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, ON);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, OFF);
    }
		else if (number <= 60) {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, OFF);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, ON);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, OFF);
    }
		else if (number <= 80) {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, OFF);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, ON);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 300);
    }
		else if(number <=95) {
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, OFF);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, OFF);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, ON); 
		}
    else {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, OFF);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, OFF);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, ON); 
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, OFF);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, OFF);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, OFF); 
				HAL_Delay(150);
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, OFF);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, OFF);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, ON); 
    }
		
	
	
}

uint16_t adcValue()
{
			uint32_t led_last_tick; 
			GPIOC->BSRR = LED1_Pin << 16;
			sprintf(seg,"LED:%d,ON\n", 4);
			HAL_UART_Transmit_IT(&huart1,(uint8_t*)seg,strlen(seg)); 	

	while(adc_flag ==1) {
		uint32_t current_tick = HAL_GetTick(); 
		uint32_t led_tick = HAL_GetTick();
		
	for (uint16_t i = 0 ; i<256; i++)
		{
			HAL_ADC_Start(&hadc1);			
			adc_value += HAL_ADC_GetValue(&hadc1);
			
		}
		adc_value /= 256;	
		number = adc_value/40;
		
		if (number >100) number = 100; 
		HAL_Delay(10);
		alertLED(number);
		sprintf(msg,"SEG:%d\n", number);
		HAL_UART_Transmit_IT(&huart1,(uint8_t*)msg,strlen(msg)); 
		HAL_Delay(10);
		sprintf(msg,"ADC:%d\n", number);
		HAL_UART_Transmit_IT(&huart1,(uint8_t*)msg,strlen(msg)); 
		HAL_Delay(10);
		
	if(HAL_GPIO_ReadPin(SW4_GPIO_Port,SW4_Pin) == GPIO_PIN_RESET) 
			{	
				if(sw4_flag ==1)
				{
					sw4_flag = 0; 
					number = 0000; 
					GPIOC->BSRR = LED1_Pin;
					sprintf(seg,"LED:%d,OFF\n", 4);
					HAL_UART_Transmit(&huart1,(uint8_t*)seg,strlen(seg), HAL_MAX_DELAY); 
					HAL_Delay(10);
					sprintf(seg, "SEG:%d\n", 0);
					HAL_UART_Transmit(&huart1, (uint8_t*)seg, strlen(seg),HAL_MAX_DELAY);
					BUZZER();
					break;
				}
			}
			
			else
					sw4_flag =1; 
		}	///while
		return number; 	
	}

	
	
void start_adc_mode(void)
{
	
	  adc_flag1 = 1;
		sendStartTick = HAL_GetTick();  
}
void adc_from_uart(void)
{
	
	adc_value =0; 
	adc_flag =0; 
	
	if(adc_flag1 ==1) 
	{
		
	for (uint8_t i = 0; i < 10; i++)
		{
			
				HAL_ADC_Start(&hadc1);
				adc_value += HAL_ADC_GetValue(&hadc1);
			
		}
				number = adc_value/400;
				if (number >100) number = 100; 
		
		sprintf(msg,"SEG:%d\n", number);
		HAL_UART_Transmit(&huart1,(uint8_t*)msg,strlen(msg),1); 
		
		sprintf(msg,"ADC:%d\n", number);
		HAL_UART_Transmit(&huart1,(uint8_t*)msg,strlen(msg),1); 
		
		
		alertLED(number);
		
		HAL_Delay(100);

		if(stop_flag ==1)
						{
							adc_flag1 = 0;
							number = 0; 
							sprintf(seg, "SEG:%d\n", 0);
						HAL_UART_Transmit(&huart1, (uint8_t*)seg, strlen(seg), HAL_MAX_DELAY);
							return ; 
						}		
						
		}
}




