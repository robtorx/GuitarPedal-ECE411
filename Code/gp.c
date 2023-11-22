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

#define PWM_FREQ 0x00FF   // pwm frequency - 31.3KHz
#define PWM_MODE 0    // Fast (1) or Phase Correct (0)
#define PWM_QTY 2   // 2 PWMs in parallel
int main(void)
{
  // SETUP
    //PD1 PD2 as output with PWM
      DDRD = 0x80;    //PD7 (pin21 - led) output. 
      //DDRD = 0x06  // Make just PD1 and PD2 output.
    // setup PWM
      TCCR1A = (((PWM_QTY - 1) << 5) | 0x80 | (PWM_MODE << 1)); //
      TCCR1B = ((PWM_MODE << 3) | 0x11);    // ck/1
      TIMSK1 = 0x20;    // interrupt on capture interrupt
      ICR1H = (PWM_FREQ >> 8);
      ICR1L = (PWM_FREQ & 0xff);
      DDRD |= 0x06;   // turn on PWM outputs PD1 and PD2
      sei();      // turn on interrupts  


    //PA0 PA1 as input both with ADC
      //PA0 Input (Pin40)
      ADMUX = 0x60;   // left adjust, adc0, internal vcc
      ADCSRA = 0xe5;  // turn on adc, ck/32, auto trigger
      ADCSRB = 0x07;  // t1 capture for trigger
      DIDR0 = 0x01;   // turn off digital inputs for adc0

                                                    
    }



    while (1) 
    {
		PORTD = 0x80;	//Turn on PD7
		_delay_ms(1000);
		PORTD = 0x00;	// Turn off PD7
		_delay_ms(1000);
    }
}
