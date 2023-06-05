#include <avr/io.h>
#include <stdio.h>

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <util/delay.h>


void uartInit(int BAUD_RATE) {
	int baudPrescaled = (((F_CPU / (BAUD_RATE * 16UL))) - 1);
	UBRR0H = (unsigned char)(baudPrescaled>>8);
	UBRR0L = (unsigned char)baudPrescaled;
	UCSR0B |= (1<<TXEN0);
	UCSR0B |= (1<<RXEN0);
	UCSR0C |= (1<<UCSZ01);
	UCSR0C |= (1<<UCSZ00);
}

void uartPrint(char* string) {
	while(*string != 0x00)
	{
		while(!(UCSR0A & (1<<UDRE0)));
		UDR0 = *string;
		string++;
	}
}