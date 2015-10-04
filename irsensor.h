/*
 * irsensor.h
 *
 * Created: 3/24/2015 12:32:04 PM
 *  Author: robideau
 */ 

int avgSensorResults(void);

void ADC_init(void);

int ADC_read(char channel);