#include "adc.h"

/*

RGB LED도 합칠까? 
adc의 값으로 밝기를 조절하는 케이스
-> 이정도는 그냥 pwm 제어 함수를 호출해서 사용하는정도도 충분함 
즉, 역할을 섞지 않고 각자의 기능만 가능하게 

추후에 main.c에서 이 둘을 조합해서 ADC값으로 LED 밝기 조절 이런 로직 작성 추천 

아, sw제어때문에 애매한데 

1.ADC읽기(Raw)값 평균내기 
2.ADC->number 변환
3.SW4 처리하기로 하는편이 rgb 켜기도 좋음 


현재 끈끈한 함수로 만들어놨음 분리불가 

*/
#define OFF 1000
#define ON 0
//근데 얘들은 왜 h파일에서 안되는거야? 
//아 알았다 값을 변화 시켜야겠어 






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
				HAL_Delay(100);
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
				HAL_Delay(100);
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, OFF);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, OFF);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, ON); 
    }
		
		HAL_Delay(10);
	
	
}

uint16_t adcValue()
{
	while(adc_flag ==1) {
		
	for (uint8_t i = 0 ; i<10; i++)
		{
			HAL_ADC_Start(&hadc1);			
			adc_value += HAL_ADC_GetValue(&hadc1);
			HAL_Delay(30);
		}
		adc_value /= 10;	
		number = adc_value/40;
		if (number >100) number = 100; 
		alertLED(number);
		
		sprintf(msg,"SEG:%d\n", number);
		HAL_UART_Transmit(&huart1,(uint8_t*)msg,strlen(msg),1); 
		
		sprintf(msg,"ADC:%d\n", number);
		HAL_UART_Transmit(&huart1,(uint8_t*)msg,strlen(msg),1); 
		
		
	if(HAL_GPIO_ReadPin(SW4_GPIO_Port,SW4_Pin) == GPIO_PIN_RESET) 
			{	
				if(sw4_flag ==1)
				{
					sw4_flag = 0; 
					number = 0000; 
					HAL_Delay(50);
					break;
				}
			}
			
			else
					sw4_flag =1; 
		}
		return number; 	
	}

	
	
	uint16_t adc_from_uart(void)
{
	
	///다시 봐야댐 
    uint32_t adc_accum = 0;
    uint16_t result = 0;

    for (uint8_t i = 0; i < 10; i++)
    {
        HAL_ADC_Start(&hadc1);
        adc_accum += HAL_ADC_GetValue(&hadc1);
        HAL_Delay(30);
    }

    result = adc_accum / 10;
    uint16_t number = result / 40;
    if (number > 100) number = 100;

    alertLED(number);

    sprintf(msg, "SEG:%d\n", number);
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    sprintf(msg, "ADC:%d\n", number);
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    return number;
}




