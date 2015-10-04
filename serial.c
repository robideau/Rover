/*
 * serial.c
 *
 * Created: 3/24/2015 12:33:49 PM
 *  Author: robideau
 */ 
#include <avr/io.h>
#include <avr/interrupt.h>
#include "util.h"

/// Initialized the USART protocol
/**
 * Initializes all necessary USART data, sets baud rate, enables receivers and transmitters, and sets frame format
 * @param ubrr the given baud rate value
 */
void USART_Init( unsigned int ubrr )
{
	/* Set baud rate */
	UBRR0H = (unsigned char)(ubrr>>8);
	UBRR0L = (unsigned char)ubrr;
	/* Enable receiver and transmitter */
	UCSR0B = (1<<RXEN)|(1<<TXEN);
	/* Set frame format: 8data, 2stop bit */
	UCSR0C = (1<<USBS)|(3<<UCSZ0);
}

/// Transmits a single character of data using USART
/**
 * Uses a data buffer to send a single character of data
 * @param data the character to transmit
 */
void USART_Transmit( unsigned char data )
{
	/* Wait for empty transmit buffer */
	while ( !( UCSR0A & (1<<UDRE)) )
	;
	/* Put data into buffer, sends the data */
	UDR0 = data;
}

/// Receives a single character of data using USART
/**
 * Uses a data buffer to receive a single character of data
 * 
 */
unsigned char USART_Receive( void )
{
	/* Wait for data to be received */
	while ( !(UCSR0A & (1<<RXC)) )
	;
	/* Get and return received data from buffer */
	wait_ms(1000);
	return UDR0;
}

/// Transmit a full string using USART
/**
 * Uses the USART_Transmit method to send a string with USART
 * @param toPrint the string to transmit
 * @param length the length of the given string
 */
void serial_putString(char toPrint[], int length) {
	for (int k = 0; k < length; k++) {
		USART_Transmit(toPrint[k]);
	}
}

/// Receives a character using USART - alternative to USART_Receive
/**
 * Uses a data buffer to receive a character from USART
 * @return UDR0 the received character
 */
char serial_getc() {
	while ((UCSR0A & 0b10000000) == 0);
	return UDR0;
}