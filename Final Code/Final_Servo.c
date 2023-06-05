
#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

void Initialize(){
	
	cli();

	//DDRD |= (1<<DDD5);

	//PORTB &= ~(1<<(PORTB1));
	DDRB |= (1<<DDB2);
	// 	PORTB &= ~(1<<(PORTB2));
	//FOR TIMER1
	TCCR1A |= (1<<WGM10); // FAST PWM - OCR1A
	TCCR1A |= (1<<WGM11);
	TCCR1B |= (1<<WGM12);
	TCCR1B |= (1<<WGM13);
	
	TCCR1B |= (1<<CS12);   // setting a prescaler of 1024
	TCCR1B &= ~(1<<CS11);
	TCCR1B|= (1<<CS10);
	
	//TCCR1A |= (1<<COM1A1);  // compare match
	TCCR1A |= (1<<COM1B1);
	
	
	OCR1A = 312; // for 50Hz (20ms)
	
	TIFR1 |= (1<<OCF1B);
	sei();
}


int main()
{
	Initialize();
	
	while(1)
	{
		OCR1B = 32;                 //90 degree
		_delay_ms(1000);
		OCR1B= 16;                //-90 degree
		_delay_ms(1000);
	}
}
