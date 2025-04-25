#ifndef __ADC_H
#define __ADC_H

#include "main.h"
#include "uart.h"

 extern TIM_HandleTypeDef htim3;  
extern UART_HandleTypeDef huart1; 
extern ADC_HandleTypeDef hadc1; 

extern uint8_t BUZZER();
extern char msg[64];
extern char seg[12];
extern uint8_t stop_flag; 

//아 근데 이거 공용으로 쓰지말자 
extern uint8_t sendTimeFlag; 
extern uint32_t sendStartTick; 
extern uint32_t led_tick; 


extern uint16_t number ; 
extern uint8_t adc_flag; 
extern uint8_t adc_flag1; 
extern uint8_t test_flag; 
extern uint8_t sw4_flag; 
extern uint32_t adc_value; 





uint16_t adcValue(); 

void alertLED( uint16_t number); 
void adc_from_uart(void); 
void start_adc_mode(void); 


#endif /*__ADC_H*/

