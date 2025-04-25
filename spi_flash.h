#ifndef __SPI_FLASH_H__
#define __SPI_FLASH_H__

#include "main.h"
#include "uart.h"

#include "stm32f1xx_hal.h"

extern char msg[64];
extern uint8_t stop_flag; 




extern uint8_t readID_flag; 

// 외부에서 쓸 수 있는 함수들
// gpio도 분리해도 되는데 그건 나중에 
void CS_Select(void);
void CS_Deselect(void);
void SPI_TransmitReceive(uint8_t *tx_buf, uint8_t *rx_buf, uint16_t len);
void readManufacturer(void);
void start_read(void); 
void readID_form_uart(void); 

#endif
