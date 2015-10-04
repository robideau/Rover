/*
 * ping.h
 *
 * Created: 3/24/2015 12:32:14 PM
 *  Author: robideau
 */ 

typedef struct { //a struct that holds a scanned object
	int degreePosition; //the object's position relative to the servo rotation
	int cmDistance; //distance from sensor in cm
	int cmWidth; //object's actual width in cm
	int scannedDegrees; //number of degrees for which the object was detected
	int isValid;
} Object;

void send_pulse(void);

int timeToDist(int delta);

unsigned long ping_read(void);

void timer1_init(void);

void sweepScan(Object objects[]);

