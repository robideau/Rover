/*
 * Rover.c
 *
 * Created: 4/9/2015 10:54:56 AM
 *  Author: robideau
 */ 

/*
*CURRENT CONFIGURATION FOR ROBOT 4
*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include "util.h"
#include "lcd.h"
#include <math.h>
#include <stdio.h>
#include "irsensor.h"
#include "serial.h"
#include "ping.h"
#include "servo.h"
#include "open_interface.h"
#include "movement.h"
#include "remoteControl.h"
#include "audio.h"


#define CLOCK_COUNT 16000000
#define FOSC 16000000// Clock speed - oscillation (unchanged)
#define BAUD 57600//Baud rate (bluetooth)
#define MYUBRR (FOSC/(16*BAUD))-1 //set ubrr for serial init


Object currentObjects[20]; //must empty before each use to prevent residual objects


int main() {
	
	//initialize all necessary sensors and utilities
	lcd_init();
	timer1_init();
	timer3_init();
	move_servo(90);
 	ADC_init();
	USART_Init(MYUBRR);
	init_push_buttons();
	
	oi_t *sensor_data = oi_alloc();
	oi_init(sensor_data);
	
	audioInit(sensor_data);
	//oi_play_song(1);
	
	while(1) {
		//empty currentObjects before proceeding by setting all stored objects to "invalid" - ignored by later checks
		for (int i = 0; i < 20; i++) {
			currentObjects[i].isValid = 0;
		}
		char received = serial_getc(); //take keyboard input from putty
		takeDirectionInput(received, currentObjects); //translate keyboard input into functionality
	}
	
	return 0;
}




