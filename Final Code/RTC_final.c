/*
 * Final Project - Pet Food and Water Dispenser.c
 *
 * Created: 11/26/2021 8:01:54 PM
 * Author : gopik
 */ 
#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include "uart.h"
#include "ds1307.h"
#include "i2cmaster.h"

char String[10];

//ADC init
void ADC_init(){
	PRR &= ~(1<<PRADC);
	ADMUX |= (1<<REFS0);
	ADMUX &= ~(1<<REFS1);
	ADMUX &= ~(1<<MUX0);
	ADMUX &= ~(1<<MUX1);
	ADMUX &= ~(1<<MUX2);
	ADMUX &= ~(1<<MUX3);
	ADCSRB &= ~(1<<ADTS0);
	ADCSRB &= ~(1<<ADTS1);
	ADCSRB &= ~(1<<ADTS2);
	DIDR0 |= (1<<ADC0D);
	ADCSRA = 0xE7 ;  // 7th en, 6th - start conversion, 5th auto trigger enable, 4th interrupt flag, 3rd- interrupt enable ,210-> prescaling we will by 128 so all 1
}

//Liquid Level Sensor
void depth(){
	while(!(ADCSRA * (1<<ADIF)));
	uint16_t val = ADC;
	sprintf(String," %d ",val);
	uartPrint(String);
	ADCSRA &= ~(1<<ADIF);
}

//Timer - Motor - PWM
void motorInit(){
	TCCR1A|=(1<<COM1A1)|(1<<COM1B1)|(1<<WGM11); //NON Inverted PWM
	TCCR1B|=(1<<WGM13)|(1<<WGM12)|(0<<CS11)|(1<<CS10); //PRESCALER=1 MODE 14(FAST PWM)
	
	ICR1=19999; //fPWM=50Hz (Period = 20ms Standard).
	DDRD|=(1<<PORTD4)|(1<<PORTD5);   //PWM Pins as Output
}

//main
void Initialize(){
	//PORTC &= ~(1<<PORTC0);
	//motorInit();
	uartInit(9600);
	i2c_init();
}

int main(void)
{
	Initialize();
	uint8_t year = 0;
	uint8_t month = 0;
	uint8_t day = 0;
	uint8_t hour = 0;
	uint8_t minute = 0;
	uint8_t second = 0;

	//check set date
	//ds1307_setdate(21, 11, 30, 13, 52, 35);


	while(1)
	{
		//get date
		ds1307_getdate(&year, &month, &day, &hour, &minute, &second);
		sprintf(String, "%d/%d/%d %d:%d:%d", year, month, day, hour, minute, second);
		uartPrint(String);	
		uartPrint("\r\n");
		_delay_ms(1000);
	}
}

//Notes:
//ADC values for Water level Sensor = ~ around 420-430

