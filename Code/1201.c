/*
 * GuitarPedal.c
 *
 * Created: 12/01/2023
 * Author : Lynn
 * 
 * NOTE : ADC operates in single conversion, manual mode.
 */ 


#ifndef F_CPU
#define F_CPU 16000000UL    // 16MHz Clock Speed
#endif

#include <avr/io.h>
#include <util/delay.h>

#define PWM_FREQ 0x00FF     // pwm frequency - 31.3KHz
#define PWM_MODE 0          // Fast (1) or Phase Correct (0)
#define PWM_QTY 2           // 2 PWMs in parallel

unsigned int ADC_low, ADC_high;
int input;
int counter = 0;

// Function definitions
long map(long x, long in_min, long in_max, long out_min, long out_max);
void switch_adc(void);

int main(void) {
    int counter = 0;

    cli();
    adc_setup();
    sei(); // Enable global interrupts

    while (1) {}; // prevent program from ending
}
////////////////////////////////////////////////////////////////////////////
//                      Interrupt Service Routines                        //
////////////////////////////////////////////////////////////////////////////

// ADC Conversion Complete
ISR(ADC_vect) {
    ADC_low = 0; // Always 0 to save space
    ADC_high = ADCH;
    
    input = ((ADC_high << 8) | ADC_low) + 0x8000; // make a signed 16b value

    OCR1AL = ((input + 0x8000) >> 8);       // convert to unsigned, send out high byte
    OCR1BL = input; // send out low byte    // PortD for the ATMega1284
}

////////////////////////////////////////////////////////////////////////////
//                          HELPER FUNCTIONS                              //
////////////////////////////////////////////////////////////////////////////

void pin_setup(void){
    // Pins PD1 PD2 as output with PWM
    DDRD |= ((1<<DDB1) | (1<<DDB2));
}

void adc_setup(void){
    ADCSRA |= (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);     // CLK/128 = 125 KHz ADC frequency
    ADMUX |= (0<<REFS1) | (1<<REFS0);                   // AVCC with external capacitor at AREF pin
    ADMUX |= (1<<ADLAR);                                // Left aligned for 8 bit resolution
	ADMUX |= (0<<MUX0) | (0<<MUX1) | (0<<MUX2) | (0<<MUX3) | (0<<MUX4); // Select port ADC1
    PRR &= ~(1<<PRADC);     // Clear power reduction bit
    ADCSRA |= (1<<ADEN);    // Enable ADC
	ADCSRA |= (1<<ADSC);    // Start first conversion
	// SFIOR |= (0<<ADTS0) | (0<<ADTS1) | (0<<ADTS2); // Free running mode
}

// Toggles LSB of ADMUX.MUX (switches between ADC0 and ADC1)
void switch_adc(void) {
    ADMUX ^= (1 << MUX0);
}

long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// // Bit manipulation -- reference
// DDRB |= (1 << DDB3);     // set pin 3 of Port B as output
// PORTB |= (1 << PB3);     // set pin 3 of Port B high
// PORTB &= ~(1 << PB3);    // set pin 3 of Port B low
// PORTB ^= (1 << PB3);   // toggles the state of the bit (XOR)