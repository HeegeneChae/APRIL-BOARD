/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
	*	@author					: hana-3
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2025 STMicroelectronics.
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
#include "segment.h"
#include "spi_flash.h"
#include "uart.h"
#include "rgb.h"
#include "adc.h"


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdint.h>

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
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_RTC_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM8_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN PFP */
 uint8_t sw1_flag; 
 uint8_t sw2_flag; 
 uint8_t sw3_flag; 
 uint8_t sw4_flag; 
 uint8_t sw5_flag; 

volatile uint8_t adc_mode = 0;  
uint8_t displayCompact = 0;



uint8_t adc_flag; 
uint8_t adc_flag1; 
uint8_t test_flag; 
uint8_t timer_flag; 
uint8_t timer_flag1;
uint8_t realTime_flag= 0;

uint8_t readID_flag; 



uint8_t mode_flag; 

uint8_t stop_flag= 0;

uint8_t sendTimeFlag; 
uint32_t sendStartTick; 

/*
typedef enum {
  MODE_SWITCH = 0,
  MODE_SERIAL = 1
} ModeFlag;		나중에 이렇게 변경하려고 현재는 !switchMode로 되어있음 

나머지 플래그들도 변경했으면 좋겠다 
*/

//이거는 시간 남으면 하고 아니면 말고 
uint8_t blink_flag = 0;     // 깜빡임 토글용
uint8_t blink_mode = 0;     // 0: 숫자모드 / 1: 박스모드 / 2: 깜빡임박스모드 등등
uint32_t blink_counter = 0; // 시간 세기용 (ms 기준)
uint32_t last_tick = 0; 
uint32_t adc_value =0;
uint8_t digits[4];
uint16_t number = 0; 

char msg[64];
char seg[12];


#define PCF8523_ADDR  (0x68 << 1 )           // Control 레지스터 1
#define REG_CONTROL_1 0x00
#define REG_SECONDS 0x03          // Seconds 레지스터 주소
#define REG_MINUTES 0x04          // Minutes 레지스터 주소
#define REG_HOURS   0x05          // Hours 레지스터 주소

#define SPI_DUMMY_BYTE                      0x00

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t mode_flag = 0;           // 0: SWITCH_MODE, 1: SERIAL_MODE
uint8_t sw4_prev_state = 1;

char received_cmd;
uint8_t serial_flag = 0;
//uint32_t timer_cnt = 0; 같이 내가 모르는 새에 변경될 수 있는 것들은 static 안하는게 좋다  
//static은 우리집 반경 뭐 어딜가든 상관없는데 암튼 우리집에 두는거임 우리집현관문에 두는건 안됨 그거는 밖이니까 
/*
Debugging Error
HardFault
- 기본 폴트 익셉션이고 우선순위가 가장 높습니다. 다른 폴트가 Disable 된 경우에도 발생으로 하고 우선순위가 높기 때문에 다른 익셉션에서도 우선 진입합니다. 
MemManage
- MPU로 설정한 메모리에 대해서 잘못 접근한 경우 발생함
BusFault
- 잘못된 메모리 Read/Write에 의해서 발생함 
UsageFault
- 정렬되지 않은 Lead/Store나 0으로 나누기 등에 의해 발생함 

typedef struct :  사실상 cpu 와 가까이 배치하기 위해서 padding 

*/



const uint16_t SEGMENT_BOX[4] = {
  0x21, 
  0x03, 
  0x0A, 
  0x28  
	
	/*0x77, // 좌상단 ┌
    0x3E, // 우상단 ┐
    0x6D, // 좌하단 └
    0x7C  // 우하단 ┘*/
};

/* 만약에 포트도 따로 조금 있었다면 
void enable_digit(uint8_t digit) 
{
    GPIOD->ODR &= ~(0x0F); // 모든 디지트 끄기
    GPIOD->ODR |= (1 << digit); // 해당 디지트만 켜기
}

*/


//void test_segment_direct(void)
//{
//    // 숫자 8 전체 점등 테스트용
//    GPIOA->ODR &= ~(0x7F);               // 먼저 꺼주고
//    GPIOA->ODR |= SEGMENT_MAP[8] & 0x7F; // 숫자 8 표시
//}
//=====================7-SEGMENT===========================//

/* Private SPI code ---------------------------------------------------------*/
//4.6 리셋 (/RESET)(1) SOIC-16 및 TFBGA 패키지에서 전용 하드웨어 /RESET 핀이 제공
//이 핀이 최소 약 1µS 동안 낮게 유지되면, 이 장치는 모든 외부 또는 내부 작업을 종료하고 전원 켜진 상태로 돌아감

void stop_mode(void) 
{
	stop_flag = 1; 
	
}




void BUZZER(void) 
{
					HAL_TIM_OC_Start(&htim8,  TIM_CHANNEL_4);
					HAL_Delay(100);
					HAL_TIM_OC_Stop(&htim8, TIM_CHANNEL_4);
}



void apMain(void) 
{
	//평소와 다른 스타일 나중에 활용 test code 
	uint32_t pre_time; 
	
	pre_time = HAL_GetTick();
	while(1) 
	{	//심지어 50ms하에서는 동작을 안한다 
		if(HAL_GetTick() - pre_time >= 500)	//이게 HAL_Delay(500); 과 같은 효과면서 non blocking 
		{
			pre_time = HAL_GetTick(); 
			HAL_GPIO_TogglePin(LED1_GPIO_Port,LED1_Pin);
			
		}
		
			HAL_GPIO_TogglePin(LED2_GPIO_Port,LED2_Pin); 
	}	
}


void RTC_Init(void) 
{
    HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);

    uint8_t init[3] = {0x00, 0x35, 0x14};
    HAL_StatusTypeDef status;

// 시간 초기화: 근데 여기 왜 1435 안돼? 
    status = HAL_I2C_Mem_Write(&hi2c1, PCF8523_ADDR, REG_SECONDS, I2C_MEMADD_SIZE_8BIT, init, 3, HAL_MAX_DELAY);
    if (status != HAL_OK) {
        number = 7777; // I2C 쓰기 실패 디버깅
        return;
    }

    // Control_1 레지스터 읽기
    uint8_t ctrl1;
    status = HAL_I2C_Mem_Read(&hi2c1, PCF8523_ADDR, REG_CONTROL_1, I2C_MEMADD_SIZE_8BIT, &ctrl1, 1, 100);
    if (status != HAL_OK) {
        number = 8888; // I2C 읽기 실패 디버깅
        return;
    }

    // STOP 비트 확인 후 해제
    if (ctrl1 & (1 << 5)) {
        ctrl1 &= ~(1 << 5);  // STOP 비트 해제
        status = HAL_I2C_Mem_Write(&hi2c1, PCF8523_ADDR, REG_CONTROL_1, I2C_MEMADD_SIZE_8BIT, &ctrl1, 1, 100);
        if (status != HAL_OK) {
            number = 9999; // STOP 비트 해제 실패 디버깅
            return;
        }
    }
}


uint32_t REALTIME(void)	
{	
	/*HAL_I2C_MEM_Read->number계산까지 다 때려박은 구조라서 가독성이 떨어짐 
	decode_bcd_to_decimal()과 같은 헬퍼함수 작성 
	RTOS가 없는 구조에서 CPU잡아먹는 루프야 
	if(realTime_flag) { … realTime_flag = 0; }와 같은 방식으로 변경 
	*/
	// os 이상data[0] &= 0x7F; -> 아니 이거 진짜 부품 갈았는데 왜 또 STOP BIT 선언 해야하는거지? 부품이 default동작을 안해 
		uint32_t lastSendTick = 0;
		HAL_StatusTypeDef status;
    static uint8_t data[3];  
	
		while(realTime_flag == 1 ) {

    status = HAL_I2C_Mem_Read(&hi2c1, PCF8523_ADDR, REG_SECONDS, I2C_MEMADD_SIZE_8BIT, data, 3, 100);
	
    if (status != HAL_OK) {
        number = 6666;
        return 0xFFFF; 
    }
		
		data[0] &= 0x7F; // bit 7을 0으로 만들고 나머지는 1 : 127 값 대입 근데 크리스탈 바꿔도 안되네 
		
    uint8_t hours = ((data[2] >> 4) * 10) + (data[2] & 0x0F);
    uint8_t minutes = ((data[1] >> 4) * 10) + (data[1] & 0x0F);
		uint8_t seconds = ((data[0] >> 4) * 10) + (data[0] & 0x0F);
		number =( minutes * 100 )+ seconds;

		
		
		if(HAL_GPIO_ReadPin(SW1_GPIO_Port,SW1_Pin) == GPIO_PIN_RESET) 
		{
			if(sw1_flag ==1)
			{
				sw1_flag = 0; 
				number = 0; 
				HAL_Delay(50);
				break;
			}
		}
		
		else
				sw1_flag =1; 
		
		if(HAL_GetTick() - lastSendTick >=1000)
		{
			lastSendTick = HAL_GetTick(); 
			char buffer[52]; 
			sprintf(seg,"TIM:%d\n", number);
			HAL_UART_Transmit(&huart1,(uint8_t*)seg,strlen(seg), HAL_MAX_DELAY); 
			sprintf(buffer, "\n\r[Boot] System started at RTC Time: %02d:%02d:%02d", hours, minutes, seconds);
			HAL_UART_Transmit(&huart1, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
			
	
		}

	HAL_Delay(100);
	
	}	//while
		
}



void start_realtime_mode(void)
{
	sendTimeFlag = 1;
  sendStartTick = HAL_GetTick();  
    
}

void realTime_from_uart(void) 
{
    if (sendTimeFlag == 1) 
    {
        
            HAL_StatusTypeDef status;
            uint8_t data[3];

            status = HAL_I2C_Mem_Read(&hi2c1, PCF8523_ADDR, REG_SECONDS, I2C_MEMADD_SIZE_8BIT, data, 3, 100);
            data[0] &= 0x7F;

            uint8_t hours = ((data[2] >> 4) * 10) + (data[2] & 0x0F);
            uint8_t minutes = ((data[1] >> 4) * 10) + (data[1] & 0x0F);
            uint8_t seconds = ((data[0] >> 4) * 10) + (data[0] & 0x0F);
            number = (minutes * 100) + seconds;

            sprintf(seg, "TIM:%d\n", number);
            HAL_UART_Transmit(&huart1, (uint8_t*)seg, strlen(seg), HAL_MAX_DELAY);
						HAL_Delay(500);
			
			
						if(stop_flag ==1)
						{
							sendTimeFlag = 0;
							return ;
						}
		}
		
        
    
}

/*
                elif cmd_type == "SEG":
                    self.segment_display.set_value(cmd_data)

                elif cmd_type =="TIM":
                    self.segment_display.set_value(cmd_data)
*/



//여기 근데 중복이 많아가지고 나중에 받는 데이터 크기 다른것들 len들을 활용해 합치는걸로 리팩토링 

///오늘 이거 추가 
void timer()
{
	timer_flag =1; 
	while( timer_flag == 1) 
	{
		uint32_t current_tick = HAL_GetTick(); 
		if(current_tick - last_tick >=300) 
		{
			last_tick = current_tick; 
			number++;
			
			if(number >9999)
				
				number = 0; 
			
			sprintf(msg,"SEG:%d\n", number);
		HAL_UART_Transmit(&huart1,(uint8_t*)msg,strlen(msg),HAL_MAX_DELAY); 
			
		}
		
		
		
		if(HAL_GPIO_ReadPin(SW2_GPIO_Port,SW2_Pin) == GPIO_PIN_RESET) 
		{
			if(sw2_flag ==1)
			{
				
				
				sw2_flag = 0; 
				number = 0000; 
				HAL_Delay(50);
				break;
			}
		}
		
		else
				sw2_flag =1; 
		
		
	}
	
} 


void start_timer_mode(void)
{
	timer_flag1 = 1;
  sendStartTick = HAL_GetTick();  
	
}
void timer_from_uart() 
{
	while( timer_flag1 == 1) 
	{
		uint32_t current_tick = HAL_GetTick(); 
		if(current_tick - last_tick >=300) 
		{
			last_tick = current_tick; 
			number++;
			
			if(number >9999)
				
				number = 0; 
			
			sprintf(msg,"SEG:%d\n", number);
		HAL_UART_Transmit(&huart1,(uint8_t*)msg,strlen(msg),HAL_MAX_DELAY); 
			
		}
		
		
		
		if(stop_flag ==1)
						{
							timer_flag1 = 0;
							return ;
						}
	}
	
	
}


//스위치를 체크하고  플래깅 
//strncmp, strcmp 활용 
//if( strcmp( str1, str2 ) == 0 )
//if( strncmp( str1, str2, 5 ) == 0 )
void sw1()
	{

	if(HAL_GPIO_ReadPin(SW1_GPIO_Port,SW1_Pin) == GPIO_PIN_RESET) 
	{
		BUZZER();
		if(sw1_flag ==1)
		{
			sw1_flag = 0; 
			
			realTime_flag = 1; 
			REALTIME();
			
			
		}
	}
		else 
		{
			
				sw1_flag =1; 
			
		}
	
}
void sw2()
	{
		
	if(HAL_GPIO_ReadPin(SW2_GPIO_Port,SW2_Pin) == GPIO_PIN_RESET) 
	{
		BUZZER();
		if(sw2_flag ==1)
		{
			sw2_flag = 0; 
			
			timer_flag = 1; 
			timer();
			
			
		}
	}
		else 
		{
			
				sw2_flag =1; 
		}
		
			
}
void sw3()
	{
	if(HAL_GPIO_ReadPin(SW3_GPIO_Port,SW3_Pin) == GPIO_PIN_RESET) 
	{
		BUZZER();
		if(sw3_flag ==1)
		{
			sw3_flag = 0; 
			
			readManufacturer();
			
		}
	}
		else 
		{
			
				sw3_flag =1; 
			
		}
	
	
}
	

void sw4()
	{
	
	if(HAL_GPIO_ReadPin(SW4_GPIO_Port,SW4_Pin) == GPIO_PIN_RESET) 
	{
		BUZZER();
		if(sw4_flag ==1)
		{

			sw4_flag = 0; 
			
			HAL_Delay(30);
			
			adc_flag =1; 
			test_flag =1;
			adc_mode = !adc_mode;
			adcValue(); 
			
			
		}
	}
		else 
		{
			
				sw4_flag =1; 
			
		}
	
}
void sw5()
	{
	if(HAL_GPIO_ReadPin(SW5_GPIO_Port,SW5_Pin) == GPIO_PIN_RESET) 
	{
		BUZZER();
		if(sw5_flag ==1)
		{
			sw5_flag = 0; 
			
			number =1122; 
			HAL_GPIO_TogglePin(LED3_GPIO_Port,LED3_Pin);
			if(HAL_GPIO_ReadPin(LED3_GPIO_Port,LED3_Pin) ==GPIO_PIN_SET) 
			{
			sprintf(seg,"LED:%d,OFF\n", 3);
			HAL_UART_Transmit_IT(&huart1,(uint8_t*)seg,strlen(seg)); 
			}
			else if(HAL_GPIO_ReadPin(LED3_GPIO_Port,LED3_Pin) ==GPIO_PIN_RESET) 
			{
			sprintf(seg,"LED:%d,ON\n", 3);
			HAL_UART_Transmit_IT(&huart1,(uint8_t*)seg,strlen(seg)); 
			}
			HAL_Delay(10);
			sprintf(msg,"SEG:%d\n", number);
			HAL_UART_Transmit_IT(&huart1,(uint8_t*)msg,strlen(msg)); 
			
			
			
			switchMode = !switchMode; // 토글 어근데 흠 진입로가 여러개일 뿐 같은 함수로 들어가야하잖아 
			if(switchMode)
			{
				
				UART_TransmitMessage("\r\nTX MODE\r\n", TYPE_STRING); 

			}
			else
				
				UART_TransmitMessage("\r\nRX MODE\r\n", TYPE_STRING); 
				HAL_UART_Receive_IT(&huart1, rx_buffer, 6);
			
			
			}
			}
			else 
			{
				
					sw5_flag =1; 
				
			}
		
}
void sw4_logic() 
{
	
	
	adc_flag = 0; 
	stop_flag = !stop_flag; 
	start_adc_mode();
	
}
void sw3_logic() 
{
	readID_flag =0; 
	stop_flag = !stop_flag; 
	start_read();

}
void sw2_logic() 
{
	sendTimeFlag = 0;
	timer_flag = 0; 
	
	stop_flag = !stop_flag; 
	start_timer_mode();

	
}
void sw1_logic() 
{
	sendTimeFlag = 0;
	stop_flag = !stop_flag; 
	start_realtime_mode(); 
}



void start_mode() 
{
	mode_flag = 1;
  sendStartTick = HAL_GetTick();  
	
}
void mode()
{
	
	if(mode_flag ==1)
	{
		
		number = 5555; 
		HAL_GPIO_TogglePin(LED3_GPIO_Port,LED3_Pin);
		
		sprintf(msg,"SEG:%d\n", number);
		HAL_UART_Transmit(&huart1,(uint8_t*)msg,strlen(msg),1); 
		
		
			if(HAL_GPIO_ReadPin(LED3_GPIO_Port,LED3_Pin) ==GPIO_PIN_SET) 
			{
			sprintf(seg,"LED:%d,OFF\n", 3);
			HAL_UART_Transmit_IT(&huart1,(uint8_t*)seg,strlen(seg)); 
			}
			else if(HAL_GPIO_ReadPin(LED3_GPIO_Port,LED3_Pin) ==GPIO_PIN_RESET) 
			{
			sprintf(seg,"LED:%d,ON\n", 3);
			HAL_UART_Transmit_IT(&huart1,(uint8_t*)seg,strlen(seg)); 
			}
			mode_flag=0; 
HAL_Delay(100);
			if(stop_flag ==1)
						{
							mode_flag = 0;
							return ;
						}
						
		
	}
	
	
}



void sw5_logic() 
{
	mode_flag = 0;
	
	stop_flag = !stop_flag; 
	start_mode(); 

}

//LED만 보내주면 끝 !


	
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	
	if (huart->Instance == USART1)
    {
		
			        if (rx_buffer[0] == 'B' && rx_buffer[1] == 'T' && rx_buffer[2] == 'N')
							{

                    char btn_num_char = rx_buffer[3];  
                    int btn_num = btn_num_char - '0';  

                    switch (btn_num)
                    {
                        case 1:
                            sw1_logic();
                            break;

                        case 2:
                            sw2_logic();
                            break;

                        case 3:
                            sw3_logic();
                            break;

                        case 4:
														test_flag =1;
														adc_flag =1; 
                            sw4_logic();
                            break;
												case 5:
                            sw5_logic();
												
                            break;

                        default:
                            break;
                    }
										
                }
            
        		        HAL_UART_Receive_IT(&huart1, rx_buffer, 4);
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
	//RTC_Init(); 

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_RTC_Init();
  MX_SPI2_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_TIM8_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
//	uint8_t send_data = 0x9A; //송신할 데이터 
//	uint8_t received_data; 

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	number = 8888;
	HAL_TIM_Base_Start_IT(&htim6);
  HAL_UART_Receive_IT(&huart1, rx_buffer, 4);
	BootMessage();
	
	while (1)
  {
		HAL_Delay(100); 
		
		stop_flag=0; 

		sw1();
		sw2();
		sw3();
		sw4();
		sw5();
		realTime_from_uart();
		adc_from_uart();
		timer_from_uart();  
		readID_form_uart();
		mode();

		
		
	}

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
		
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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_ADC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_HSE_DIV128;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only 
  */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 72-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000;	//period 255->1000
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 500;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 1000;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 72-1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 2500-1;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 72-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 2500;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 72-1;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 10000-1;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 72-1;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 370-1;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, A_Pin|B_Pin|C_Pin|D_Pin 
                          |E_Pin|F_Pin|G_Pin|DP_Pin 
                          |LED4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, QWE1_Pin|QWE2_Pin|QWE3_Pin|QWE4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED3_Pin|LED2_Pin|LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SW5_Pin SW1_Pin SW2_Pin SW3_Pin 
                           SW4_Pin */
  GPIO_InitStruct.Pin = SW5_Pin|SW1_Pin|SW2_Pin|SW3_Pin 
                          |SW4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : A_Pin B_Pin C_Pin D_Pin 
                           E_Pin F_Pin G_Pin DP_Pin 
                           LED4_Pin */
  GPIO_InitStruct.Pin = A_Pin|B_Pin|C_Pin|D_Pin 
                          |E_Pin|F_Pin|G_Pin|DP_Pin 
                          |LED4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : QWE1_Pin QWE2_Pin QWE3_Pin QWE4_Pin 
                           SPI2_CS_Pin */
  GPIO_InitStruct.Pin = QWE1_Pin|QWE2_Pin|QWE3_Pin|QWE4_Pin 
                          |SPI2_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LED3_Pin LED2_Pin LED1_Pin */
  GPIO_InitStruct.Pin = LED3_Pin|LED2_Pin|LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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
