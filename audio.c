/*
 * audio.c
 *
 * Created: 4/23/2015 10:31:31 AM
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
#include "remoteControl.h"

/// Prepares the robot to play audio files
/**
 * Creates and loads all necessary audio files onto the robot
 * @param sensor_data the struct holding the open interface sensor data
 */
void audioInit(oi_t *sensor_data) {
	oi_update(sensor_data); //struct must be holding sensor data before music can be loaded
	
	unsigned char testNotes[] = {60, 65, 69, 72, 69, 72}; //audio debug use
	unsigned char testDurations[] = {8, 8, 8, 16, 8, 16};
		
	unsigned char startNotes[] = {16, 32, 64}; //play when program starts
	unsigned char startDurations[] = {16, 16, 16}; 
		
	/*unsigned char bumpNotes[] = {}; //play when bumpers are triggered
	unsigned char bumpDurations[] = {};
		
	unsigned char cliffNotes[] = {}; //play when cliff sensor is triggered
	unsigned char cliffDurations[] = {};
		
	unsigned char tapeNotes[] = {}; //play when tape is detected
	unsigned char tapeDurations[] = {};
		
	unsigned char finishNotes[] = {}; //play when destination is reached
	unsigned char finishDurations[] = {};*/
	
	oi_load_song(0, 6, testNotes, testDurations);
	oi_load_song(1, 3, startNotes, startDurations);
	
	
}