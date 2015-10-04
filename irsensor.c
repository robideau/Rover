/*
 * irsensor.c
 *
 * Created: 3/24/2015 12:31:53 PM
 *  Author: robideau
 */ 
#include <avr/io.h>
#include <avr/interrupt.h>
#include "util.h"

/// Reads one set of data from the ADC
/**
 * Takes data from the ADC using the given channel.
 * @param channel the channel from which to read the ADC data
 */
int ADC_read(char channel) {
	ADMUX |= (channel & 0x1F); //select channel to read from ADC
	ADCSRA |= _BV(ADSC); //start ADC read transfer
	while (ADCSRA & _BV(ADSC)); //while transfer is available
	return ADC; //read from ADC
}

/// Gets the average of 30 sensor results
/** 
 * Takes 30 data points from the sensor and averages them, eliminating outliers within a specific tolerance
 * @return the average of 30 results as an int
 */
int avgSensorResults() { //get the average of 30 sensor readings to reduce "jitter"
	int resultTotal = 0;
	int finalTotal = 0;
	int results[30];
	int toleranceH = 0;
	int toleranceL = 0;
	for (int i = 0; i < 30; i++) {
		results[i] = ADC_read(2); //read from ADC channel 2 (IR sensor)
		resultTotal += ADC_read(2);
	}
	toleranceH = (resultTotal/30)+10;
	toleranceL = (resultTotal/30)-10;
	for (int j = 0; j < 30; j++) {
		if (results[j] > toleranceH || results[j] < toleranceL) {
			results[j] = (resultTotal/30);			
		}
		finalTotal += results[j];
	}
	return finalTotal/30;
}

/// Initialize the ADC
/**
 * Prepares the ADC for use by the program, setting prescalar values, transfer modes, etc.
 * 
 */
void ADC_init() {
	ADMUX = _BV(REFS1) | _BV(REFS0);
	ADCSRA = _BV(ADEN) | (7<<ADPS0);
	//ADCSRA = 0b10100111; - last three bits divide frequency by 128 - results in 125kHz
}

