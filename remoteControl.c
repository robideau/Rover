/*
 * remoteControl.c
 *
 * Created: 4/9/2015 10:57:06 AM
 *  Author: robideau
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
#include <string.h>

struct oi_t {
	
	};
	
int degreeIntervals = 90; //intervals of rotation
int distanceIntervals = 10; //forward and backward motion intervals	
	
/// Takes keyboard inputs from putty
/**
 * Takes keyboard inputs from putty - allows the user to control the robot using the home computer's keyboard
 * @param received the key pressed by the operator
 * @param currentObjects any objects currently held by the program's container
 */
void takeDirectionInput(char received, Object currentObjects[]) {
	
	oi_t *sensor_data = oi_alloc();
	oi_init(sensor_data);
	
	oi_set_wheels(0,0);
	
	if (received == 'w') { // w = forward
		serial_putString("Moving forward...\n\r", 20);
		moveForward(sensor_data, distanceIntervals);
		oi_set_wheels(0, 0);
		wait_ms(50);
	}
	if (received == 's') { //s = backward
		serial_putString("Moving backward...\n\r", 21);
		moveBackward(sensor_data, -distanceIntervals);
		oi_set_wheels(0, 0);
		wait_ms(50);
	}
	if (received == 'a') { // a = counterclockwise
		serial_putString("Rotating counterclockwise 90 degrees...\n\r", 42);
		rotateCounterClockwise(sensor_data, degreeIntervals);
		oi_set_wheels(0, 0);
		wait_ms(50);
	}
	if (received == 'q') {
		serial_putString("Rotating counterclockwise 15 degrees...\n\r", 42);
		rotateCounterClockwiseFine(sensor_data, degreeIntervals);
		oi_set_wheels(0, 0);
		wait_ms(50);
	}
	if (received == 'd') { // d = clockwise
		serial_putString("Rotating clockwise 90 degrees...\n\r", 35);
		rotateClockwise(sensor_data, -degreeIntervals);
		oi_set_wheels(0, 0);
		wait_ms(50);
	}
	if (received == 'e') {
		serial_putString("Rotating clockwise 15 degrees...\n\r", 35);
		rotateClockwiseFine(sensor_data, -degreeIntervals);
		oi_set_wheels(0, 0);
		wait_ms(50);
	}
	if (received == 'r') { // r = scan for objects
		serial_putString("Scanning...\n\r", 14);
		sweepScan(currentObjects);
		for (int i = 0; i < 20; i++) {
			if (currentObjects[i].isValid) {
				char* scanString = "*";
				sprintf(scanString, "Object at %d degrees, %d cm away, %d cm wide\n\r", currentObjects[i].degreePosition, currentObjects[i].cmDistance, currentObjects[i].cmWidth);
				wait_ms(10);		
				serial_putString(scanString, 47);
			}
		}
		moveBackward(sensor_data, 0);
	}
	if (received == 'c') { //c = scan for colors -- used for calibration
		char colorString[40];
		sprintf(colorString, "FL: %d   L: %d    R: %d   FR: %d\n\r", sensor_data->cliff_frontleft_signal, sensor_data->cliff_left_signal, sensor_data->cliff_right_signal, sensor_data->cliff_frontright_signal);
		serial_putString(colorString, 40);
		colorCheck(sensor_data->cliff_frontleft_signal, sensor_data->cliff_left_signal, sensor_data->cliff_right_signal, sensor_data->cliff_frontright_signal);
	}
	if (received == 't') { //t = play song
		serial_putString("Playing song...\n\r", 18);
		oi_play_song(0);
	}
}
