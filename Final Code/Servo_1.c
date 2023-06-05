#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

void Initialize(){
	
	cli();

	DDRB |= (1<<DDB1);
	//PORTB &= ~(1<<(PORTB1));
// 	DDRB |= (1<<DDB2);
// 	PORTB &= ~(1<<(PORTB2));
	//FOR TIMER1
	TCCR1A |= (1<<WGM10); // FAST PWM - OCR1A
	TCCR1A |= (1<<WGM11);
	TCCR1B |= (1<<WGM12);
	TCCR1B |= (1<<WGM13);
	
	TCCR1B |= (1<<CS12);   // setting a prescaler of 256
	TCCR1B &= ~(1<<CS11);
	TCCR1B &= ~(1<<CS10);
	
	TCCR1A |= (1<<COM1A1);  // compare match
	TCCR1A |= (1<<COM1B1);
	
	
	OCR1A = 1249; // for 50Hz (20ms)

	sei();
}


int main()
{
	Initialize();
	while(1)
	{
		OCR1B=79;                 //90 degree
		_delay_ms(1000);
		OCR1B=24;                //0 degree
		_delay_ms(1000);
		OCR1B=134;               //180 degree
		_delay_ms(1000);
	}
}

// Part 2

#define F_CPU 8000000UL
#include <avr/io.h>
#include <stdio.h>
#include <util/delay.h>

int main(void)
{
	DDRD |= (1<<PORTD5);
	ICR1 = 2499;

	
	TCCR1B &= ~(1<<CS12);   // setting a prescaler of 64
	TCCR1B |= (1<<CS11);
	TCCR1B |= (1<<CS10);
	
	TCCR1A &= ~(1<<WGM10); // FAST PWM - ICR1
	TCCR1A |= (1<<WGM11);
	TCCR1B |= (1<<WGM12);
	TCCR1B |= (1<<WGM13);
	
	TCCR1A |= (1<<COM1A1);  // compare match
	TCCR1A |= (1<<COM1B1);
	
	
	while(1)
	{
		OCR1A = 65;
		_delay_ms(1500);
		OCR1A = 175;	/* Set servo shaft at 0° position */
		_delay_ms(1500);
		OCR1A = 300;	/* Set servo at +90° position */
		_delay_ms(1500);
	}
}


#define F_CPU 8000000UL // 8 MHz clock speed
#include <avr/io.h>
#include <util/delay.h>

int main(void)
{
	DDRC |= (1<<DDC0); //Makes RC0 output pin
	PORTC &= ~(1<<PORTC0);
	while(1)
	{
		//Rotate Motor to 0 degree
		PORTC = 0x01;
		_delay_us(1000);
		PORTC = 0x00;

		//_delay_ms(2000);

		//Rotate Motor to 90 degree
		// 		PORTC = 0x01;
		// 		_delay_us(1500);
		// 		PORTC = 0x00;
		//
		// 		_delay_ms(2000);
		//
		// 		//Rotate Motor to 180 degree
		// 		PORTC = 0x01;
		// 		_delay_us(2000);
		// 		PORTC = 0x00;

		//_delay_ms(2000);
	}
}



#define F_CPU 8000000UL
#include <avr/io.h>
#include <stdio.h>
#include <util/delay.h>

int main(void)
{
	DDRD |= (1<<PORTD5);
	ICR1 = 2499;

	
	TCCR1B &= ~(1<<CS12);   // setting a prescaler of 64
	TCCR1B |= (1<<CS11);
	TCCR1B |= (1<<CS10);
	
	TCCR1A &= ~(1<<WGM10); // FAST PWM - ICR1
	TCCR1A |= (1<<WGM11);
	TCCR1B |= (1<<WGM12);
	TCCR1B |= (1<<WGM13);
	
	TCCR1A |= (1<<COM1A1);  // compare match
	TCCR1A |= (1<<COM1B1);
	
	
	while(1)
	{
		OCR1A = 65;
		_delay_ms(1500);
		OCR1A = 175;	/* Set servo shaft at 0° position */
		_delay_ms(1500);
		OCR1A = 300;	/* Set servo at +90° position */
		_delay_ms(1500);
	}
}