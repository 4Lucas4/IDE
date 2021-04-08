/*
 * File:        uart.h
 * Purpose:     Provide UART declares for serial IO
 *
 * Notes:		
 *
 */

#define BAUD_RATE 9600      //default baud rate 
#define SYS_CLOCK 20485760 //default system clock (see DEFAULT_SYSTEM_CLOCK  in system_MK64F12.c)
#include <stdint.h>

void uart_init(void);
uint8_t uart_getchar(void);
void uart_putchar(char ch);
void put(char *ptr_str);

