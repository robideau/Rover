/*
 * serial.h
 *
 * Created: 3/24/2015 12:33:57 PM
 *  Author: robideau
 */ 

void USART_Init( unsigned int ubrr );

void USART_Transmit( unsigned char data );

unsigned char USART_Receive( void );

void serial_putString(char toPrint[], int length);

char serial_getc(void);