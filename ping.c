/*
 * ping.c
 *
 * Created: 3/24/2015 12:31:43 PM
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
#include "servo.h"

typedef struct { //a struct that holds a scanned object
	int degreePosition; //the object's position relative to the servo rotation
	int cmDistance; //distance from sensor in cm
	int cmWidth; //object's actual width in cm
	int scannedDegrees; //number of degrees for which the object was detected
	int isValid; //1 or 0 - determines whether or not object is a remainder from previous scans
} Object;

//variables used by the interrupt to determine time between pulses
volatile unsigned long rising_time = 0;
volatile unsigned long falling_time = 0;
volatile unsigned overflows = 0;
volatile unsigned new_overflows = 0;
volatile unsigned long delta = 0;

//various data used to keep track of ISR behavior
int ISRruns = 0;
int pulses = 0;
volatile unsigned long pingDistance = 0;
int quantization; //quanitzation factor between 0 and 1023 read from ADC
int IRdistance = 0; //quantization factor converted to distance using function


/// Keep track of timer overflows
/**
 * Keeps track of total timer1 overflows
 * @param TIMER1_OVF_vect the vector tracking timer overflows
 * 
 */
ISR (TIMER1_OVF_vect) {
	new_overflows++; //count total overflows
}

/// Keep track of timer capture events
/**
 * Keeps track of timer capture events and creates the delta value used to calculate actual distance
 * @param TIMER1_CAPT_vect the vector monitoring timer capture events
 */
ISR (TIMER1_CAPT_vect) {
	ISRruns++;
	if ((TCCR1B & 0b01000000) == 0b01000000) {
		rising_time = ICR1; //catch rising time
		TCCR1B &= 0b10111111; //switch to react on falling edge
	}
	while(ICR1 == rising_time) {
		wait_ms(1); //wait until ICR1 value detects falling edge
	}
	if ((TCCR1B & 0b01000000) == 0b00000000) {
		falling_time = ICR1;  //catch falling edge
		delta = falling_time - rising_time; //calculate time between high and low
		TCCR1B |= 0b01000000; //switch to react on rising edge
	}
	TCCR1B |= 0b01000000; //ensure that rising edge is detected
}

/// Sends a single pulse from the ping sensor
/**
 * Sends one single pulse from the ping sensor, enabling and disabling interrupt flags as necessary
 * 
 */
void send_pulse() {
	TIMSK &= 0xDB; //disable IC interrupt
	DDRD |= 0x10; //PD4 to output
	PORTD |= 0x10; //PD4 to high
	wait_ms(1); //wait
	PORTD &= 0xEF; //PD4 to low
	wait_ms(1);
	DDRD &= 0xEF; //PD4 to input
	TIFR |= 0x20; //Clear IC flag
	TIMSK |= 0x24; //re-enable IC interrupt
	wait_ms(1);
}

/// Converts raw delta value to real distance
/**
 * Takes the delta value determined by capture event interrupts and converts it to distance in cm
 * @param delta the raw value determined by the ISR
 * @return distance the delta value converted to a distance in cm
 */
int timeToDist(int delta) {
	float distance = (delta*0.06972973)+3.6481622; //factor determined by calibration (robot 4)
	int roundingError = distance; //avoid float rounding errors
	if (distance - roundingError > 0.5) {
		return distance+1; //if rounded down, add 1 to compensate
	}
	return distance;
}

/// A helper method that sends a pulse and measures distance
/**
 * Sends a pulse from the ping sensor, determines delta, and converts into a distance in cm
 * @param delta the raw delta value from the ISR
 * @return distance the distance from the sensor in cm
 */
unsigned long ping_read(unsigned long delta) {
	send_pulse(); //send a pulse and calculate delta
	unsigned long distance = timeToDist(delta); //convert delta to cm
	return distance;
}

/// Initializes the timer for use with the ping sensor
/**
 * Initializes timer 1, setting prescalers and interrupt flag enables
 * 
 */
void timer1_init() {
	TCCR1A = 0x00;
	TCCR1B = 0xC3; //noise canceler on, rising edge selected (bit 6), 64 prescaler
	TCCR1C = 0x00; //do not use force output compare
	TIMSK |= 0x20; //use interrupts - bit 2 high is timer 1 value is overflowed
}

/// Scans a 180 degree radius and determines the smallest object in sight
/**
 * Takes 180 data measurements from the ping sensor, converts them to cm values, and determines the smallest object's index, width, and distance.
 * 
 */
void scanSmallestObj() {
	int degrees = 0;
	Object objects[10]; //holds scanned objects for later analysis
	int objectCount = 0; //number of objects already scanned
	int prevDetectionStatus = 0; //previous state of object detection
	move_servo(0); //move servo to starting position
	wait_ms(1500); //wait for initializations to finish
	int finalValuesCalculated = 1;
	
	while(degrees < 180) { //for one full rotation
		move_servo(degrees); //sweep servo
		wait_ms(10);
		degrees++; //increments of 2 degrees
		
		ping_read(delta); //take ping sensor data
		wait_ms(50); //wait for return pulse
		int foundDelta = delta;
		pingDistance = timeToDist(foundDelta); //convert ping data to cm
		
		quantization = avgSensorResults(); //read from ADC channel 2 (IR sensor)
		IRdistance = 2364.5*(pow(quantization, -0.888));	//convert quantization to distance in cm
		
		int objectDetected = 0; //whether or not an object is being detected
		
		//int distDifference = abs(IRdistance-pingDistance); //determine the absolute value of the difference between sensor values
		if (IRdistance < 85 && prevDetectionStatus == 0) { //if an object is near and was not previously being detected
			objectDetected = 1; //an object is near
			Object scannedObject; //create a new object struct
			scannedObject.degreePosition = degrees; //set degrees to current servo position
			scannedObject.scannedDegrees = 1; //currently been scanned for one degree
			scannedObject.cmDistance = 0; //distance according to ping sensor
			objects[objectCount] = scannedObject; //add object to objects array
			objectCount++;	//increment object count by 1
			prevDetectionStatus = objectDetected;
			finalValuesCalculated = 0;
		}
		else if (IRdistance < 150 && prevDetectionStatus == 1) { //if an object is still being detected
			objectDetected = 1;

		}
		else if (IRdistance > 150 && prevDetectionStatus == 1) { //if a large change in IRdistance has occurred (noise) but an object was being scanned
			objectDetected = 1;	//ignore IRdistance and continue scanning object
			prevDetectionStatus = 0; //if the large gap persists, assume object is no longer being scanned
		}
		if (objectDetected) { //if currently scanning an object
			objects[objectCount-1].scannedDegrees++; //increase number of degrees scanned for each servo rotation
			objects[objectCount-1].cmDistance += pingDistance;
		}
		if (objectDetected == 0 && finalValuesCalculated == 0) { //if the object is no longer being detected, perform final calculations
			objects[objectCount-1].cmDistance = (objects[objectCount-1].cmDistance/objects[objectCount-1].scannedDegrees);
			objects[objectCount-1].cmWidth = ((2*objects[objectCount-1].cmDistance) * tan(((objects[objectCount-1].scannedDegrees)*3.14)/360)); //calculate width using angular diameter formula
			finalValuesCalculated = 1;
		}
		lprintf("Objects: %d\nDegrees: %d\nWidth: %d", objectCount, objects[objectCount-1].scannedDegrees, objects[objectCount-1].cmWidth); //FOR DEBUG ONLY
		
		
		char toPrint[31]; //contains string to pass to putty
		toPrint[0] = ' ';
		sprintf(toPrint, "%d      %d      %lu     %d\n\r", degrees, IRdistance, pingDistance, objectDetected);
		serial_putString(toPrint, 29); //send string to putty
	}
	
	int smallestWidth = 1023; //used to determine smallest object
	int index = 0; //current object index
	int removedObjects = 0; //running count of objects thrown out due to size or distance
	int prevRemovedObjects = 0; //objects removed prior to indicated index
	int totalRemovedObjects = 0;
	int loopRuns = 0;
	for (int i = 0; i < objectCount; i++) {
		if (objects[i].cmWidth <= 3 || objects[i].cmDistance > 100) { //if the object is very small or very far away, throw it out
			removedObjects++;
			loopRuns++;
			totalRemovedObjects++;
		}
		else if ((objects[i].cmWidth < smallestWidth) && objects[i].cmWidth > 3) { //check if current object has new smallest width
			smallestWidth = objects[i].cmWidth; //replace smallest width with new value
			index = i; //lock onto index
			prevRemovedObjects = removedObjects; //use for index compensation before final print statement
			removedObjects = 0; //reset value for previous removed objects
		}
	}
	lprintf("Index: %d of %d\nDist (cm): %d\nAngular width: %d\nWidth (cm): %d\n", (index-prevRemovedObjects+1), (objectCount-removedObjects+prevRemovedObjects), objects[index].cmDistance, objects[index].scannedDegrees, objects[index].cmWidth); //final results
	move_servo(objects[index].degreePosition); //point to smallest object
}

/// Scans a 180 degree radius, adding all detected objects to the given container
/**
 * Takes 180 data points from the ping sensor, converts all values into cm, and puts all objects into the given container
 * @param objects an array of objects used to store any data collected by sweepScan
 */
void sweepScan(Object objects[]) {
	int degrees = 0;
	int objectCount = 0; //number of objects already scanned
	int prevDetectionStatus = 0; //previous state of object detection
	move_servo(0); //move servo to starting position
	wait_ms(1000); //wait for initializations to finish
	int finalValuesCalculated = 1;
	
	while(degrees < 180) { //for one full rotation
		move_servo(degrees); //sweep servo
		degrees++; //increments of 2 degrees
		
		ping_read(delta); //take ping sensor data
		wait_ms(10); //wait for return pulse
		int foundDelta = delta;
		pingDistance = timeToDist(foundDelta); //convert ping data to cm
		
		quantization = avgSensorResults(); //read from ADC channel 2 (IR sensor)
		IRdistance = 2364.5*(pow(quantization, -0.888));	//convert quantization to distance in cm
		
		int objectDetected = 0; //whether or not an object is being detected
		
		//int distDifference = abs(IRdistance-pingDistance); //determine the absolute value of the difference between sensor values
		if (IRdistance < 85 && prevDetectionStatus == 0) { //if an object is near and was not previously being detected
			objectDetected = 1; //an object is near
			Object scannedObject; //create a new object struct
			scannedObject.degreePosition = degrees; //set degrees to current servo position
			scannedObject.scannedDegrees = 1; //currently been scanned for one degree
			scannedObject.cmDistance = 0; //distance according to ping sensor
			scannedObject.isValid = 1;
			objects[objectCount] = scannedObject; //add object to objects array
			objectCount++;	//increment object count by 1
			prevDetectionStatus = objectDetected;
			finalValuesCalculated = 0;
		}
		else if (IRdistance < 150 && prevDetectionStatus == 1) { //if an object is still being detected
			objectDetected = 1;

		}
		else if (IRdistance > 150 && prevDetectionStatus == 1) { //if a large change in IRdistance has occurred (noise) but an object was being scanned
			objectDetected = 1;	//ignore IRdistance and continue scanning object
			prevDetectionStatus = 0; //if the large gap persists, assume object is no longer being scanned
		}
		if (objectDetected) { //if currently scanning an object
			objects[objectCount-1].scannedDegrees++; //increase number of degrees scanned for each servo rotation
			objects[objectCount-1].cmDistance += pingDistance;
		}
		if (objectDetected == 0 && finalValuesCalculated == 0) { //if the object is no longer being detected, perform final calculations
			objects[objectCount-1].cmDistance = (objects[objectCount-1].cmDistance/objects[objectCount-1].scannedDegrees);
			objects[objectCount-1].cmWidth = ((2*objects[objectCount-1].cmDistance) * tan(((objects[objectCount-1].scannedDegrees)*3.14)/360)); //calculate width using angular diameter formula
			finalValuesCalculated = 1;
		}
		//lprintf("Objects: %d\nDegrees: %d\nWidth: %d", objectCount, objects[objectCount-1].scannedDegrees, objects[objectCount-1].cmWidth); //FOR DEBUG ONLY
		
		
		/*char toPrint[31]; //contains string to pass to putty
		toPrint[0] = ' ';
		sprintf(toPrint, "%d      %d      %lu     %d\n\r", degrees, IRdistance, pingDistance, objectDetected);
		serial_putString(toPrint, 29); //send string to putty for debug
		*/
		
	}
}
