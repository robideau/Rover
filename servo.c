/*
 * servo.c
 *
 * Created: 3/24/2015 12:31:29 PM
 *  Author: robideau
 */ 
#include <avr/io.h>
#include <avr/interrupt.h>
#include "util.h"

unsigned int pulse_width;
unsigned pulse_interval = 128;
unsigned mid_point = 64;

/// Initializes timer 3 for use with the servo
/**
 * Initializes timer 3 for use with the servo, sets COM bits, WGM bits, prescalers
 * 
 */
void timer3_init() {
	OCR3A = pulse_interval-1;
	OCR3B = mid_point-1;
	TCCR3A = 0b10101011; //set COM bits (Ch A = 7,6; Ch B = 5,4; Ch C = 3,2;) and WGM bits 3 and 2 (1,0);
	TCCR3B = 0b10001100; //set WGM bits 1 and 0 (4,3) and clock prescaler
	TCCR3C = 0;
	
	DDRE |= _BV(4); //set port E pin 4 as output
}

/// Rotates the servo by a given number of degrees
/**
 * Converts a degree value to a pulse width, which is sent to the servo
 * @param degree the number of degrees to rotate the servo
 */
void move_servo(unsigned degree) {
	pulse_width = (((108*(degree+10))/180) + 29); //calculate pulse width
	OCR3B = pulse_width;
	wait_ms(5); //wait for servo to move - change as necessary
}