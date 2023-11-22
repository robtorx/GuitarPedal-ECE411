/*
 * GuitarPedal.c
 *
 * Created: 11/13/23 12:47:54
 * Author : Rob
 */ 

#ifndef F_CPU
#define F_CPU 16000000UL	// 16MHz Clock Speed
#endif

#include <avr/io.h>
#include <util/delay.h>

int main(void)
{
    DDRD = 0x80;	// Makes PD7 output, PIN21
    while (1) 
    {
		PORTD = 0x80;	//Turn on PD7
		_delay_ms(1000);
		PORTD = 0x00;	// Turn off PD7
		_delay_ms(1000);
    }
}

