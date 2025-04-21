#include "uart.h"
extern  UART_HandleTypeDef huart1;
/*
	현재 문제인지 아닌지, 
	rx tx가 같이 들어있는데 일반적인 케이스가 아님 
	
	TX모드 일 때만 sw_routine()이 발생 
	
	TX모드: 
	RX모드: GUI에서 명령이 들어오면 그걸 파싱해서 sw_routine()의 하나를 실행해 
	
*/

/*
	int *p;	p는 int형 변수를 가리키는 포인터다
	*p	포인터가 가리키는 값
	void *	타입 없는 포인터. 뭐든 가리킬 수 있음 (캐스팅 필요)
	(int *)data	data를 int 포인터로 변환
*/
const uint16_t SEGMENT_MAP[10] = 
{
  0x3F,  
  0x06, 
  0x5B, 
  0x4F,
  0x66, 
  0x6D,  
  0x7D,  
  0x07,  
  0x7F,  
  0x6F   
};

void display_7SEG(uint8_t num) 
{
	uint16_t pattern = SEGMENT_MAP[num]; 
	
	GPIOA->ODR &= ~(0x7F);  	
  GPIOA->ODR |= (pattern & 0x7F); 
	
}

// UART 메시지 전송 함수: 숫자와 문자 모두 보낼 수 있는 범용 함수로 변경하자 ! 
/*
	IT모드는 버퍼가 덮이면 안된다 
	msg[]가 바뀌면 값이 엉뚱한 문자열이 나가 나중에 큐나 더블 버퍼 
*/
void UART_TransmitMessage(const void *data, uint8_t type)
{
	
	if(type == TYPE_STRING)
	{
		const char *msg = (const char *)data; 
		HAL_UART_Transmit_IT(&huart1, (uint8_t *)msg, strlen(msg)); 
	}
	else if(type == TYPE_INT) 
	{
		int num = *((int *)data);
		sprintf(msg, "%d", num); 
		HAL_UART_Transmit_IT(&huart1, (uint8_t *)msg, strlen(msg)); 
	}

}


void BootMessage()
{
	HAL_UART_Transmit_IT(&huart1, (uint8_t *)"\r\n[System Started]",18); 
	HAL_UART_Receive_IT(&huart1, rx_buffer,1); 
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
	{	//SEGMENT_MAP[]이 있음:0~9까지 매핑 
	
		static uint8_t digit = 0; 
	
		digits[0] = (number / 1000) % 10;
		digits[1] = (number / 100) % 10;
		digits[2] = (number / 10) % 10;
		digits[3] = number % 10;
	
	if(htim-> Instance == TIM6) 
	{
//				blink_counter += 1;
//        if (blink_counter >= 50) {  
//            blink_flag = !blink_flag;
//            blink_counter = 0;
//        }
		
		HAL_GPIO_WritePin(QWE1_GPIO_Port,QWE1_Pin,GPIO_PIN_RESET); 
		HAL_GPIO_WritePin(QWE2_GPIO_Port,QWE2_Pin,GPIO_PIN_RESET); 
		HAL_GPIO_WritePin(QWE3_GPIO_Port,QWE3_Pin,GPIO_PIN_RESET); 
		HAL_GPIO_WritePin(QWE4_GPIO_Port,QWE4_Pin,GPIO_PIN_RESET); 
		
		
//		if (!(adc_mode && digit == 0))
//        {
					//여기 자동으로 첫번째 자리 꺼주는거는 내일 다시 해야겠다 
					
            // 세그먼트에 숫자 표시
            display_7SEG(digits[digit]);

            // 해당 자리 선택
            switch (digit) {
                case 0: HAL_GPIO_WritePin(QWE1_GPIO_Port, QWE1_Pin, GPIO_PIN_SET); break;
                case 1: HAL_GPIO_WritePin(QWE2_GPIO_Port, QWE2_Pin, GPIO_PIN_SET); break;
                case 2: HAL_GPIO_WritePin(QWE3_GPIO_Port, QWE3_Pin, GPIO_PIN_SET); break;
                case 3: HAL_GPIO_WritePin(QWE4_GPIO_Port, QWE4_Pin, GPIO_PIN_SET); break;
            }
     //  }

        // 다음 자리로 이동
        digit = (digit + 1) % 4;

				
    }
	
}
	

	

/*
HAL_UART_Transmit(&huart1, (uint8_t *)"Hello", 6, HAL_MAX_DELAY);
: "Hello"는 실제로는 "Hello\0"로 6bit
	근데 strlen()은 null은 빼니까 주의해야하고 
	헷갈리니까 그냥 직접세거나 sizeof(msg)-1으로 계산하면 편함

아, interrupt로 변경 

-uart command parser
	형식: $CMD01;
       ↑  ↑
      헤더  명령값
문자열 3~5자리 num1234->

-rx/tx 이사 
*/


//uart를 먼저 분리하는게 제일 편한거 같음 
//함수 하나만 더 해보고 c쪽 구조는 이제 구조 변경은 필요없는거같아 
//boot메세지와 led토글 확인 완료 
