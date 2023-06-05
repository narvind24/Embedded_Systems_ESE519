
#define F_CPU 16000000UL


#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>
#include "uart.h"

int edge = 0;
int period = 0;
int echo_pulse_uS;
int distance_cm;


int distance;
int d;
char String[10];


void Initialize(){
	cli(); //clear prior interrupts
	
	DDRB |= (1<<DDB3); //set PD6 as output
	DDRB &= ~(1<<DDB0); //PB0 as input (ICP1)
	DDRD |= (1<<DDD2);
	
	//Fast PWM Configuration/
	TCCR2A |= (1<<COM2A1);
	TCCR2A |= (1<<COM2A0);
	TCCR2A |= (1<<WGM21);
	TCCR2A |= (1<<WGM20);
	
	//prescaler = 8 for timer 2
	TCCR2B &= ~(1<<CS22);
	TCCR2B |= (1<<CS21);
	TCCR2B &= ~(1<<CS20);
	
	OCR2A = 235; //10uS trigger pulse, 118uS off-time (128uS repetition rate)
	
	//Input Capture configuration/
	//Timer 1 running in normal mode
	TCCR1A &= ~(COM1A1);
	TCCR1A &= ~(COM1A0);
	
	//positive edge detection for input capture
	TCCR1B |= (1<<ICES1);
	
	//prescaler = 8 for timer 1
	TCCR1B &= ~(1<<CS12);
	TCCR1B |= (1<<CS11);
	TCCR1B &= ~(1<<CS10);
	
	TIMSK1 |= (1<<ICIE1); //enable timer1 input capture interrupt
	
	sei();//enable global interrupts
}

int calculate_distance(){
	//32768uS = 65536 clock ticks for Timer 1 with prescaler = 8
	echo_pulse_uS = (float)period * 32768 / 65536;
	distance_cm = echo_pulse_uS * 0.034 / 2;
	return distance_cm;
}

ISR(TIMER1_CAPT_vect){
	if ((TCCR1B & (1<<ICES1)) == (1<<ICES1)){
		edge = ICR1;
	}
	else{
		period = ICR1 - edge;
		edge = 0;
	}
	TCCR1B ^= (1<<ICES1); //toggle edge detection bit
	TIFR1 = (1<<ICF1);//clear Input Capture Flag
}

int main(){

	Initialize();
	uartInit(9600);
	while (1){
		distance = calculate_distance();
		sprintf(String,"%d", distance);
		uartPrint(String);
		//_delay_ms(500);
		if (distance < 10 && distance >0)
		{
			PORTD |= (1<<PORTD2);
			_delay_ms(500);
		}
		PORTD &= ~(1<<PORTD2);
	}
	
}