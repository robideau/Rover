/*
 * movement.c
 *
 * Created: 1/27/2015 12:34:08 PM
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

int rotationCalibration = 13; //calibration for the rotation values - robot #4 specifically
int colorFlag = 0; //whether or not colored tape has been detected
int bumperFlag = 0;
int cliffFlag = 0;


/// Checks cliff sensors 
/**
 * Checks to ensure that no cliff is currently being detected by cliff sensors
 * @param *sensor_data the struct holding the robot's sensor data
 */
void checkSensors(oi_t *sensor_data) { //TODO
	if (!sensor_data->cliff_frontleft && !sensor_data->cliff_left && !sensor_data->cliff_right && !sensor_data->cliff_frontright) { //no cliffs detected
		//do nothing
		cliffFlag = 0;
	}
	if (sensor_data->cliff_frontleft) { //front left cliff
		colorFlag = 0; //color data becomes unreliable, remove color flags
		cliffFlag = 2;
	}
	if (sensor_data->cliff_left) { //left cliff
		colorFlag = 0;
		cliffFlag = 1;
	}
	if (sensor_data->cliff_right) { //right cliff
		colorFlag = 0;
		cliffFlag = 4;
	}
	if (sensor_data->cliff_frontright) { //front right cliff
		colorFlag = 0;
		cliffFlag = 3;
	}
	
}

/// Move the robot backwards one distance increment
/**
 * Sets the robot's wheels to move backwards for a specified distance, updating sensors after each degree of wheel rotation
 * @param *sensor the struct holding the robot's sensor data
 * @param cm the distance to move
 */
void moveBackward(oi_t *sensor, int cm) {
	if (cm != 0) {
		oi_set_wheels(-100, -100); //set wheels to move backwards
	
		int totalDistance = 0; //keep track of total distance traveled
	
		while(totalDistance >= cm*10) {
			oi_update(sensor); //update all sensors
			totalDistance += sensor->distance;
		}
	}
	oi_set_wheels(0,0); //stop motion
	colorFlag = 0;
	bumperFlag = 0;
	cliffFlag = 0;
}

/// Checks for any colored tape below cliff sensors
/**
 * Checks to see what the color of the ground is using the cliff sensors (Black, Grey, or White)
 * @param frontLeft the second cliff sensor from the left's sensor signal
 * @param left the first cliff sensor from the left's sensor signal
 * @param right the fourth cliff sensor from the left's sensor signal
 * @param frontRight the third cliff sensor from the left's sensor signal
 */
void colorCheck(int frontLeft, int left, int right, int frontRight) {
	if (frontLeft > 125 || left > 645 || right > 1324 || frontRight > 1030) { //white tape values
		colorFlag = 1;
	}
	else if (frontLeft < 22 || left < 66 || right < 152 || frontRight < 118) { //black circle values
		colorFlag = 2;
		//play celebration song
	}
}

/// Checks for any objects detected by bump sensors
/**
 * Checks bump sensors to ensure that the robot has not collided with any objects in the field
 * @param *sensor_data the struct holding the robot's sensor data
 * @param leftBumper the status of the left bump sensor
 * @param rightBumper the status of the right bump sensor
 */
void bumperCheck(oi_t *sensor_data, int leftBumper, int rightBumper) { //TODO
	bumperFlag = 0;
	if (leftBumper == 1 && rightBumper == 0) { //just the left bumper is triggered
		bumperFlag = 1;
	}
	else if (rightBumper == 1 && leftBumper == 0) { //just the right bumper is triggered
		bumperFlag = 2;
	}
	else if (rightBumper == 1 && leftBumper == 1) { //both bumpers are triggered
		bumperFlag = 3;
	}
}


/// Move the robot forward one distance increment
/**
 * Sets the robot's wheels to move forward for a specified distance, updating sensors after each degree of wheel rotation
 * @param *sensor the struct holding the robot's sensor data
 * @param cm the distance to move
 */
void moveForward(oi_t *sensor, int cm) {
	oi_set_wheels(200, 200); //set wheels in motion
	oi_update(sensor); //check sensors
	bumperCheck(sensor, sensor->bumper_left, sensor->bumper_right);
	colorCheck(sensor->cliff_frontleft_signal, sensor->cliff_left_signal, sensor->cliff_right_signal, sensor->cliff_frontright_signal); //ensure no tape is detected
	checkSensors(sensor);
	int totalDistance = 0;
	
	while(totalDistance <= cm*10 && //for given distance increment
			cliffFlag == 0 && //while no cliffs detected
			colorFlag == 0 && //while no white or black tape detected
			bumperFlag == 0) {  //while no bumpers detected
		oi_update(sensor);
		bumperCheck(sensor, sensor->bumper_left, sensor->bumper_right);
		checkSensors(sensor);
		colorCheck(sensor->cliff_frontleft_signal, sensor->cliff_left_signal, sensor->cliff_right_signal, sensor->cliff_frontright_signal);
		totalDistance += sensor->distance;
	}
	
	oi_set_wheels(0,0); //stop motion
	checkSensors(sensor);
	
	if (colorFlag == 1) {
		serial_putString("White tape detected.\n\r", 23);
	}
	if (colorFlag == 2) {
		serial_putString("Black tape detected!\n\r", 23);
	}
	if (bumperFlag == 1) {
		serial_putString("Left bumper triggered.\n\r", 25);
	}
	if (bumperFlag == 2) {
		serial_putString("Right bumper triggered.\n\r", 26);
	}
	if (bumperFlag == 3) {
		serial_putString("Both bumpers triggered.\n\r", 26);
	}
	if (cliffFlag == 1) {
		serial_putString("Left cliff sensor triggered.\n\r", 31);
	}
	if (cliffFlag == 2) {
		serial_putString("Front left cliff sensor triggered.\n\r", 37);
	}
	if (cliffFlag == 3) {
		serial_putString("Front right cliff sensor triggered.\n\r", 38);
	}
	if (cliffFlag == 4) {
		serial_putString("Right cliff sensor triggered.\n\r", 32);
	}
}

/// Rotate the robot clockwise one degree increment
/**
 * Sets the robot's wheels to rotate clockwise for a specified degree amount, updating sensors after each degree of wheel rotation
 * @param *sensor the struct holding the robot's sensor data
 * @param degrees the angle to rotate
 */
void rotateClockwise(oi_t *sensor, int degrees) {
	int totalRotation = 0;
	
	oi_set_wheels(-200, 200); //begin rotation
	oi_update(sensor); //check sensors
	colorCheck(sensor->cliff_frontleft_signal, sensor->cliff_left_signal, sensor->cliff_right_signal, sensor->cliff_frontright_signal); //check for colored tape
	checkSensors(sensor);
	bumperCheck(sensor, sensor->bumper_left, sensor->bumper_right);
	
	while (totalRotation >= degrees+rotationCalibration && //while still rotating
			cliffFlag == 0 && //while no cliffs detected
			colorFlag == 0 &&
			bumperFlag == 0) { //while no colored tape detected
		oi_update(sensor);
		bumperCheck(sensor, sensor->bumper_left, sensor->bumper_right);
		colorCheck(sensor->cliff_frontleft_signal, sensor->cliff_left_signal, sensor->cliff_right_signal, sensor->cliff_frontright_signal);
		checkSensors(sensor);
		totalRotation += sensor->angle;
	}
	
	oi_set_wheels(0,0); //stop rotation
	checkSensors(sensor);
}

/// Rotate the robot clockwise one sixth of a degree increment
/**
 * Sets the robot's wheels to rotate clockwise for a specified degree amount, updating sensors after each degree of wheel rotation
 * @param *sensor the struct holding the robot's sensor data
 * @param degrees the angle to rotate
 */
void rotateClockwiseFine(oi_t *sensor, int degrees) {
	int totalRotation = 0;
	
	oi_set_wheels(-200, 200); //begin rotation
	oi_update(sensor); //check sensors
	colorCheck(sensor->cliff_frontleft_signal, sensor->cliff_left_signal, sensor->cliff_right_signal, sensor->cliff_frontright_signal); //check for colored tape
	checkSensors(sensor);
	bumperCheck(sensor, sensor->bumper_left, sensor->bumper_right);
	
	while (totalRotation >= (degrees+rotationCalibration)/6 && //while still rotating
	cliffFlag == 0 && //while no cliffs detected
	colorFlag == 0 &&
	bumperFlag == 0) { //while no colored tape detected
		oi_update(sensor);
		bumperCheck(sensor, sensor->bumper_left, sensor->bumper_right);
		colorCheck(sensor->cliff_frontleft_signal, sensor->cliff_left_signal, sensor->cliff_right_signal, sensor->cliff_frontright_signal);
		checkSensors(sensor);
		totalRotation += sensor->angle;
	}
	
	oi_set_wheels(0,0); //stop rotation
	checkSensors(sensor);
}

/// Rotate the robot counterclockwise one degree increment
/**
 * Sets the robot's wheels to rotate counterclockwise for a specified degree amount, updating sensors after each degree of wheel rotation
 * @param *sensor the struct holding the robot's sensor data
 * @param degrees the angle to rotate
 */
void rotateCounterClockwise(oi_t *sensor, int degrees) {
	int totalRotation = 0;
	
	oi_set_wheels(200, -200); //begin rotation
	oi_update(sensor); //check sensors
	colorCheck(sensor->cliff_frontleft_signal, sensor->cliff_left_signal, sensor->cliff_right_signal, sensor->cliff_frontright_signal); //check for colored tape
	checkSensors(sensor);
	while (totalRotation <= (degrees-rotationCalibration) && //while still rotating
			cliffFlag == 0 && //while no cliffs detected
			colorFlag == 0 &&
			bumperFlag == 0) { //while no colored tape detected
		oi_update(sensor);
		bumperCheck(sensor, sensor->bumper_left, sensor->bumper_right);
		colorCheck(sensor->cliff_frontleft_signal, sensor->cliff_left_signal, sensor->cliff_right_signal, sensor->cliff_frontright_signal);
		checkSensors(sensor);
		totalRotation += sensor->angle;
	}
	
	oi_set_wheels(0,0); //stop motion
	checkSensors(sensor);
}

/// Rotate the robot counterclockwise one sixth of a degree increment
/**
 * Sets the robot's wheels to rotate counterclockwise for a specified degree amount, updating sensors after each degree of wheel rotation
 * @param *sensor the struct holding the robot's sensor data
 * @param degrees the angle to rotate
 */
void rotateCounterClockwiseFine(oi_t *sensor, int degrees) {
	int totalRotation = 0;
	
	oi_set_wheels(200, -200); //begin rotation
	oi_update(sensor); //check sensors
	colorCheck(sensor->cliff_frontleft_signal, sensor->cliff_left_signal, sensor->cliff_right_signal, sensor->cliff_frontright_signal); //check for colored tape
	checkSensors(sensor);
	while (totalRotation <= (degrees-rotationCalibration)/6 && //while still rotating
	cliffFlag == 0 && //while no cliffs detected
	colorFlag == 0 &&
	bumperFlag == 0) { //while no colored tape detected
		oi_update(sensor);
		bumperCheck(sensor, sensor->bumper_left, sensor->bumper_right);
		colorCheck(sensor->cliff_frontleft_signal, sensor->cliff_left_signal, sensor->cliff_right_signal, sensor->cliff_frontright_signal);
		checkSensors(sensor);
		totalRotation += sensor->angle;
	}
	
	oi_set_wheels(0,0); //stop motion
	checkSensors(sensor);
}

