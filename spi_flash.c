/* 
spi_flash.c는 SPI2를 "사용"만 하면 됨 
spi2를 쓰기 위한 주체는 main.c/ spi.c에서 초기화해놓고, 
*/
//======================COMMUNICATION==========================//
/*
// SPI Flash라면
SPIFlash_Write();
SPIFlash_Read();

// RTC면
RTC_SetTime();
RTC_GetTime();

HAL->IRQ Handler
millis()/ TIM Interrupt
HAL_Delay()는 Systick기반이라 블로킹

int __io_putchar(int ch) {
    HAL_UART_Transmit(&huart1, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

-->printf("Hello\n"); 이렇게 디버깅 가능 

*/


/*	SPI_TransmitReceive
select함수들을 사용해 슬레이브 디바이스를 선택한다 
hal_SPI_TransmitReceive()를 사용하여 데이터를 송수신을 해 
이 함수는 송신할 데이터와 수신할 버퍼를 인수로 받으며, 설정된 SPI를 통해 통신을 수행 
main() 함수에서 초기화 작업을 수행한 후 SPI_TransmitReceive()함수를 이용해 데이터를 송신하고 응답 

추후에: winbond 제조 ID같은거 확인키 알려주는 정도면 된다 
*/


#include "spi_flash.h"

// 외부에서 선언한 SPI 핸들러를 사용한다고 알림
extern SPI_HandleTypeDef hspi2;
extern UART_HandleTypeDef huart1; 

// 칩 선택 (CS 핀 LOW)
void CS_Select(void) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
}

// 칩 해제 (CS 핀 HIGH)
void CS_Deselect(void) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
}

// SPI 송수신 함수
void SPI_TransmitReceive(uint8_t *tx_buf, uint8_t *rx_buf, uint16_t len) {
    CS_Select();
    HAL_SPI_TransmitReceive(&hspi2, tx_buf, rx_buf, len, HAL_MAX_DELAY);
    CS_Deselect();
}

// 제조사 ID 읽기
void readManufacturer(void) {
    uint8_t tx_buf[6] = {0x90, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint8_t rx_buf[6] = {0};
		
		HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_12);

    SPI_TransmitReceive(tx_buf, rx_buf, 6);

    snprintf(msg, sizeof(msg)-1, "\r\n0x90 ID - Manufacturer: 0x%02X, Device: 0x%02X",
             rx_buf[4], rx_buf[5]);

    HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
		
		//it 끊겨서 체크: 버퍼 크기 안맞았음 
}
