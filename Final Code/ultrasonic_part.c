
#define F_CPU 16000000UL
#define BAUD_RATE 9600
#define BAUD_PRESCALER (((F_CPU / (BAUD_RATE * 16UL))) - 1)

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
	UART_init(BAUD_PRESCALER);
	while (1){
		distance = calculate_distance();
// 		sprintf(String,"%d", distance);
// 		strcat(String, " cm   ");	/* Concat unit i.e.cm */
// 		UART_putstring(String);
// 		_delay_ms(500);
		if (distance < 20)
		{
			//sprintf(String,"%d", distance);
			//sprintf(String, " Thirsty Cat ");	/* Concat unit i.e.cm */
			UART_putstring(" Thirsty Cat ");
			_delay_ms(500);
		}
		else if (distance >20) 
		{
			//sprintf(String,"%d", distance);
			//strcat(String, " Get lost ");	/* Concat unit i.e.cm */
			UART_putstring(" Get lost, you hooman ");
			_delay_ms(500);
		}
	}
	
}