#define F_CPU 16000000UL
// #define BAUD_RATE 9600
// #define BAUD_PRESCALER (((F_CPU / (BAUD_RATE * 16UL))) - 1)
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>
#include "uart.h"
#include "ds1307.h"
#include "i2cmaster.h"


int edge = 0;
int period = 0;
int echo_pulse_uS;
int distance_cm;

uint8_t year = 0;
uint8_t month = 0;
uint8_t day = 0;
uint8_t hour = 0;
uint8_t minute = 0;
uint8_t second = 0;

char String[10];
int distance;
int d;

//----------------ADC INITIALIZATION------------------------//

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


//----------------LIQUID LEVEL SENSOR------------------------//

void liquid_level_init()
{
	PORTC &= ~(1<<PORTC0);
}

void depth()
{
	while(!(ADCSRA * (1<<ADIF)));
	uint16_t val = ADC;
	sprintf(String," %d ",val);
	uartPrint(String);
	ADCSRA &= ~(1<<ADIF);
}

//----------------------BUZZER----------------------//
void buzzerInit()
{
	DDRD |= (1<<(DDD6));
	
	//Prescale: 256
	TCCR0B |= (1<<CS02);
	TCCR0B &= ~(1<<CS01);
	TCCR0B &= ~(1<<CS00);
	
	//Mode: Phase Correct PWM
	TCCR0A |= (1<<WGM00);
	TCCR0A &= ~(1<<WGM01);
	TCCR0B |= (1<<WGM02);
	
	//Toggle OC0A
	TCCR0A |= (1<<COM0A0);
	TCCR0A &= ~(1<<COM0A1);
	
	OCR0A = 0;
}

//----------------------Servo Motor - Using Fast PWM (Prescaler = 1024, Frequency = 50Hz)----------------------//
void motorInit()
{
	DDRB |= (1<<DDB2);
	//FOR TIMER1
	TCCR1A |= (1<<WGM10); // FAST PWM - OCR1A
	TCCR1A |= (1<<WGM11);
	TCCR1B |= (1<<WGM12);
	TCCR1B |= (1<<WGM13);
	
	TCCR1B |= (1<<CS12);   // setting a prescaler of 1024
	TCCR1B &= ~(1<<CS11);
	TCCR1B|= (1<<CS10);
	
	// compare match
	TCCR1A |= (1<<COM1B1);
	
	
	OCR1A = 312; // for 50Hz (20ms)
	
	TIFR1 |= (1<<OCF1B);
}

void food_dispense()
{
	uartPrint(" Food Dispenser ");
	//Servo Motor Start
	OCR1B = 32;                //90 degree
	_delay_ms(3000);			//Remember to check
	OCR1B= 23;                 // 50 degree angle approximate
	_delay_ms(500);
}

//----------------------WATER PUMP PORTION---------------------------//
void water_dispense_init()
{
	DDRD |= (1<<DDD2);
	PORTD &=~ (1<<PORTD2);
}

void water_dispense()
{
	uartPrint(" Water Dispenser ");
	// Pump Starts
	PORTD |= (1<<PORTD2);
	_delay_ms(3000);
	PORTD &= ~(1<<PORTD2);
	_delay_ms(500);
}

//----------------------ULTRASONIC SENSOR PORTION---------------------------//
void ultrasonic_init()
{
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
}

int calculate_distance()
{
	//32768uS = 65536 clock ticks for Timer 1 with prescaler = 8
	echo_pulse_uS = (float)period * 32768 / 65536;
	distance_cm = echo_pulse_uS * 0.034 / 2;
	return distance_cm;
}

ISR(TIMER1_CAPT_vect)
{
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


//----------------------MAIN---------------------------//
void Initialize()
{
	cli();
	uartInit(9600);
	ADC_init();
	//RTC Starts
	i2c_init();
	//Liquid Level Sensor Starts
	liquid_level_init();
	//Servo Motor Starts
	motorInit();
	// Water Dispenser Starts
	water_dispense_init();
	//ultrasonic_init();
	buzzerInit();
	sei();
}


int main(void)
{
	Initialize();
	//check set date
	ds1307_setdate(21, 12, 2, 18, 42, 0);
	DDRD &= ~(1<<DDD3);
	DDRD &= ~(1<<DDD4);
	while(1)
	{
		//get date
		ds1307_getdate(&year, &month, &day, &hour, &minute, &second);
		if (hour == 18 && minute == 42 && second == 5)
		{
			OCR0A = 35;
			food_dispense();
			water_dispense();
			_delay_ms(3000);
			OCR0A = 0;
		}
		depth();
		//Ultrasonic Sensor Starts
		//distance = calculate_distance();
		//sprintf(String," %d \n",distance);
		//if (distance < 20)
		//{
		//uartPrint(" TC ");
		//_delay_ms(500);
		//}
		//else if (distance >20)
		//{
		//uartPrint(" GL ");
		//_delay_ms(500);
		//}
		if (PIND & (1<<PIND3))
		{
			OCR0A = 35;
			water_dispense();
			_delay_ms(500);
			OCR0A = 0;
		}
		if (PIND & (1<<PIND4)){
			OCR0A = 35;
			food_dispense();
			_delay_ms(500);
			OCR0A = 0;
		}
		_delay_ms(1000);
		
	}
}

//Notes:
//ADC values for Water level Sensor = ~ around 420-430