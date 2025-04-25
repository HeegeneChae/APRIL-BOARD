#ifndef __UART_H
#define __UART_H

#include "main.h"
#include "stm32f1xx_hal.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>

extern uint8_t digits[4];
extern uint16_t number ; 
extern char msg[64];


#define TYPE_STRING 0 
#define TYPE_INT 1 


static uint8_t switchMode =1; 
static uint8_t rx_buffer[6];

void display_7SEG(uint8_t num); 
void UART_TransmitMessage(const void *data, uint8_t type);
void BootMessage();



#endif /* __UART_H */
